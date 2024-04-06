//! This module contains code related to broadcasting messages over CAN.

use crate::{
    crc::TransferCrc,
    dsdl::{
        AhrsSolution, GetSetResponse, HardwareVersion, IdAllocationData, NodeHealth, NodeMode,
        NodeStatus, SoftwareVersion,
    },
    f16, find_tail_byte_index,
    gnss::{FixDronecan, GlobalNavSolution, GnssAuxiliary},
    make_tail_byte,
    messages::MsgType,
    protocol::{CanId, FrameType, RequestResponse, ServiceData, TransferComponent},
    CanError,
};
use bitvec::prelude::*;
use embedded_can;
use heapless::{Deque, Entry, FnvIndexMap, Vec};
use log::*;
use packed_struct::PackedStruct;

// Note: These are only capable of handling one message at a time. This is especially notable
// for reception.
static mut MULTI_FRAME_BUFS_TX: [[u8; 64]; 20] = [[0; 64]; 20];

pub(crate) const DATA_FRAME_MAX_LEN_FD: usize = 64;
pub(crate) const DATA_FRAME_MAX_LEN_LEGACY: usize = 8;

// Static buffers, to ensure they live long enough through transmission. Note; This all include room for a tail byte,
// based on payload len. We also assume no `certificate_of_authority` for hardware size.
// static mut BUF_NODE_INFO: [u8; 64] = [0; MsgType::GetNodeInfo.buf_size()]; // todo: This approach would be better, but not working.

// These ID allocation buffers accomodate the length including full 16-bit unique id, and tail byte.
static mut BUF_ID_ALLOCATION: [u8; 20] = [0; 20];
// static mut BUF_ID_RESP: [u8; 17] = [0; 17];
// This node info buffer is padded to accomodate a 20-character name.
static mut BUF_NODE_INFO: [u8; 64] = [0; 64];
static mut BUF_NODE_STATUS: [u8; 8] = [0; 8];
static mut BUF_TIME_SYNC: [u8; 8] = [0; 8];
static mut BUF_TRANSPORT_STATS: [u8; 20] = [0; 20];
// Rough size that includes enough room for i64 on most values, and a 40-len name field.
// Also long enough to support padding to the tail byte of 32-len for a 2-frame FD transfer.
static mut BUF_GET_SET: [u8; 90] = [0; 90];

// static mut BUF_MAGNETIC_FIELD_STRENGTH2: [u8; 8] = [0; 8]; // Note: No covariance.
// Potentially need size 12 for mag strength in FD mode, even with no cov.
static mut BUF_MAGNETIC_FIELD_STRENGTH2: [u8; 12] = [0; 12]; // Note: No covariance.
static mut BUF_RAW_IMU: [u8; 64] = [0; 64]; // Note: No covariance.
static mut BUF_PRESSURE: [u8; 8] = [0; 8];
static mut BUF_TEMPERATURE: [u8; 8] = [0; 8];
static mut BUF_GNSS_AUX: [u8; 20] = [0; 20]; // 16 bytes, but needs a tail byte, so 20.
static mut BUF_FIX2: [u8; 64] = [0; 64]; // 48-byte payload; pad to 64.
static mut BUF_GLOBAL_NAVIGATION_SOLUTION: [u8; 90] = [0; 90];

const ACTUATOR_COMMAND_SIZE: usize = 4;
// Hard-coded to no more than 4 commands, for now.
static mut BUF_ACTUATOR_ARRAY_COMMAND: [u8; ACTUATOR_COMMAND_SIZE * 4] =
    [0; ACTUATOR_COMMAND_SIZE * 4];

static mut BUF_ARDUPILOT_GNSS_STATUS: [u8; 8] = [0; 8];
static mut BUF_CIRCUIT_STATUS: [u8; 8] = [0; 8];
static mut BUF_POWER_SUPPLY_STATUS: [u8; 8] = [0; 8];

// Per DC spec.
pub const NODE_ID_MIN_VALUE: u8 = 1;
pub const NODE_ID_MAX_VALUE: u8 = 127;

/// The key for a map entry to `TransferDesc`
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub enum TransferDescKey {
    /// datatype, source_id
    Message(u16, u8),
    /// datatype
    MessageAnon(u16),
    /// datatype, source_id, dest_id
    Service(u16, u8, u8),
}

/// The `TransferDesc` contains state for a given transfer
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct TransferDesc {
    transfer_id: u8,
    last_txfer: u64,
}

impl TransferDesc {
    /// Get the current transfer_id
    pub fn transfer_id(&mut self) -> u8 {
        self.transfer_id
    }
    /// Get the current transfer_id, then increment it
    ///
    /// transfer_id's are 5 bits in length and roll over when
    pub fn transfer_id_then_incr(&mut self) -> u8 {
        let id = self.transfer_id;
        self.transfer_id += 1;
        self.transfer_id &= 0b1_1111;
        id
    }
}

/// Broadcast struct for Uavcan on CAN interface
pub struct Uavcan<CAN, FRAME>
where
    CAN: embedded_can::blocking::Can<Frame = FRAME>,
    //    FRAME: embedded_can::Frame,
{
    iface: Vec<CAN, 2>,
    tx_queue: Deque<FRAME, 10>,
    txf_map: FnvIndexMap<TransferDescKey, TransferDesc, 20>,
}

impl<CAN, FRAME, FE> Uavcan<CAN, FRAME>
where
    CAN: embedded_can::blocking::Can<Frame = FRAME, Error = FE>,
    FRAME: embedded_can::Frame,
{
    /// Write out all pending transmit frames to all interfaces
    pub fn can_send(&mut self) -> Result<(), FE> {
        loop {
            if let Some(frame) = self.tx_queue.pop_front() {
                for iface in self.iface.iter_mut() {
                    iface.transmit(&frame)?;
                }
            } else {
                break;
            }
        }
        Ok(())
    }

    /// Handles splitting a payload into multiple frames
    /// when it is larger than a single frame
    fn queue_multiple_frames(
        &mut self,
        payload: &[u8],
        can_id: u32,
        transfer_id: u8,
        fd_mode: bool,
        base_crc: u16,
    ) -> Result<(), CanError> {
        let frame_payload_len = if fd_mode {
            DATA_FRAME_MAX_LEN_FD as usize
        } else {
            DATA_FRAME_MAX_LEN_LEGACY as usize
        };

        let id = embedded_can::Id::Extended(embedded_can::ExtendedId::new(can_id).unwrap());

        let mut crc = TransferCrc::new(base_crc);
        crc.add_payload(payload, payload.len());

        // We use slices of the FD buf, even for legacy frames, to keep code simple.
        let bufs = unsafe { &mut MULTI_FRAME_BUFS_TX };

        // See https://dronecan.github.io/Specification/4._CAN_bus_transport_layer/,
        // "Multi-frame transfer" re requirements such as CRC and Toggle bit.
        let mut component = TransferComponent::MultiStart;
        let mut active_frame = 0;
        // This tracks the index of the payload we sent in the previous frame.

        // Populate the first frame. This is different from the others due to the CRC.
        let mut tail_byte = make_tail_byte(component, transfer_id);
        let mut payload_len_this_frame = DATA_FRAME_MAX_LEN_LEGACY - 3; // -3 for CRC and tail byte.

        // Add 2 for the CRC.
        let tail_byte_i = find_tail_byte_index(payload_len_this_frame as u8 + 2);

        bufs[active_frame][..2].clone_from_slice(&crc.value.to_le_bytes());
        bufs[active_frame][2..DATA_FRAME_MAX_LEN_LEGACY - 1]
            .clone_from_slice(&payload[..payload_len_this_frame]);
        bufs[active_frame][tail_byte_i] = tail_byte.value();

        let f = FRAME::new(id, &bufs[active_frame][..payload_len_this_frame]).unwrap();
        self.tx_queue
            .push_back(f)
            .map_err(|_| CanError::TransmitQueueFull)?;

        let mut latest_i_sent = payload_len_this_frame - 1;

        active_frame += 1;

        // Populate subsequent frames.
        while latest_i_sent < payload.len() - 1 {
            let payload_i = latest_i_sent + 1;
            if payload_i + frame_payload_len <= payload.len() {
                // Not the last frame.
                payload_len_this_frame = frame_payload_len - 1;
                component = TransferComponent::MultiMid(tail_byte.toggle);
            } else {
                // Last frame.
                payload_len_this_frame = payload.len() - payload_i;
                component = TransferComponent::MultiEnd(tail_byte.toggle);
            }

            tail_byte = make_tail_byte(component, transfer_id);

            bufs[active_frame][0..payload_len_this_frame]
                .clone_from_slice(&payload[payload_i..payload_i + payload_len_this_frame]);

            let tail_byte_i = find_tail_byte_index(payload_len_this_frame as u8);
            bufs[active_frame][tail_byte_i] = tail_byte.value();

            let f = FRAME::new(id, &bufs[active_frame][..payload_len_this_frame]).unwrap();
            self.tx_queue
                .push_back(f)
                .map_err(|_| CanError::TransmitQueueFull)?;

            latest_i_sent += payload_len_this_frame; // 8 byte frame size, - 1 for each frame's tail byte.
            active_frame += 1;
        }

        Ok(())
    }

    /// Send a DroneCAN "broadcast" message. See [The DroneCAN spec, transport layer page](https://dronecan.github.io/Specification/4._CAN_bus_transport_layer/)
    /// Should be broadcast at interval between 2 and 1000ms.
    /// Note: The payload must be static, and include space for the tail byte.
    pub fn broadcast(
        &mut self,
        frame_type: FrameType,
        msg_type: MsgType,
        source_node_id: u8,
        payload: &[u8],
        fd_mode: bool,
    ) -> Result<(), CanError> {
        // This saves some if logic in node firmware re decision to broadcast.
        if source_node_id == 0 && frame_type != FrameType::MessageAnon {
            return Err(CanError::PayloadData);
        }

        // get the transfer descriptor
        // first construct the key from the info provided, then
        // we can get the descriptor from the hashmap
        let txf_key = match frame_type {
            FrameType::Message => TransferDescKey::Message(msg_type.id(), source_node_id),
            FrameType::MessageAnon => TransferDescKey::MessageAnon(msg_type.id()),
            FrameType::Service(s) => {
                TransferDescKey::Service(msg_type.id(), source_node_id, s.dest_node_id)
            }
        };
        let mut txf_desc = match self.txf_map.entry(txf_key) {
            Entry::Occupied(v) => *v.get(),
            Entry::Vacant(v) => {
                let desc = TransferDesc {
                    transfer_id: 0,
                    last_txfer: 0,
                };
                v.insert(desc).map_err(|_| CanError::TransferMapFull)?;
                desc
            }
        };
        // if this is a service reply, use the existing transfer id, otherwise, increment and
        // use that.
        let transfer_id = match frame_type {
            FrameType::Service(s) => {
                if s.req_or_resp == RequestResponse::Response {
                    txf_desc.transfer_id()
                } else {
                    txf_desc.transfer_id_then_incr()
                }
            }
            _ => txf_desc.transfer_id_then_incr(),
        };
        txf_desc.last_txfer += 1;
        let can_id = CanId {
            priority: msg_type.priority(),
            type_id: msg_type.id(),
            source_node_id,
            frame_type,
        };

        let frame_payload_len = if fd_mode {
            DATA_FRAME_MAX_LEN_FD
        } else {
            DATA_FRAME_MAX_LEN_LEGACY
        };

        // The transfer payload is up to 7 bytes for non-FD DRONECAN.
        // If data is longer than a single frame, set up a multi-frame transfer.
        if payload.len() >= frame_payload_len {
            return self.queue_multiple_frames(
                payload,
                can_id.value(),
                transfer_id,
                fd_mode,
                msg_type.base_crc(),
            );
        }

        // make a new buffer to construct the data for a single frame, we need to append one byte
        // to the payload for the tail byte
        let mut buf: Vec<u8, DATA_FRAME_MAX_LEN_FD> = Vec::new();
        buf.copy_from_slice(payload);
        let tail_byte = make_tail_byte(TransferComponent::SingleFrame, transfer_id);
        buf.push(tail_byte.value()).ok();

        let id = embedded_can::Id::Extended(embedded_can::ExtendedId::new(can_id.value()).unwrap());
        self.tx_queue
            .push_back(FRAME::new(id, buf.as_slice()).unwrap())
            .map_err(|_| CanError::TransmitQueueFull)?;
        Ok(())
    }
}
// todo: You need a fn to get a payload from multiple frames

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/341.NodeStatus.uavcan
/// Standard data type: uavcan.protocol.NodeStatus
/// Must be broadcast at intervals between 2 and 1000ms. FC firmware should
/// consider the node to be faulty if this is not received for 3s.
pub fn publish_node_status<CAN, FRAME>(
    can: &mut Uavcan<CAN, FRAME>,
    health: NodeHealth,
    mode: NodeMode,
    vendor_specific_status_code: u16,
    uptime_sec: u32,
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError>
where
    CAN: embedded_can::blocking::Can<Frame = FRAME>,
    FRAME: embedded_can::Frame,
{
    let status = NodeStatus {
        uptime_sec,
        health,
        mode,
        vendor_specific_status_code,
    };

    let m_type = MsgType::NodeStatus;

    // let mut buf = [0; crate::find_tail_byte_index(PAYLOAD_SIZE_NODE_STATUS as u8) + 1];
    let buf = unsafe { &mut BUF_NODE_STATUS };

    buf[..m_type.payload_size() as usize].clone_from_slice(&status.to_bytes());

    can.broadcast(
        FrameType::Message,
        m_type,
        node_id,
        &status.to_bytes(),
        fd_mode,
    )
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/1.GetNodeInfo.uavcan
/// A composite type sent in response to a request.
pub fn publish_node_info<CAN, FRAME>(
    can: &mut Uavcan<CAN, FRAME>,
    status: &NodeStatus,
    software_version: &SoftwareVersion,
    hardware_version: &HardwareVersion,
    node_name: &[u8],
    fd_mode: bool,
    node_id: u8,
    requester_node_id: u8,
) -> Result<(), CanError>
where
    CAN: embedded_can::blocking::Can<Frame = FRAME>,
    FRAME: embedded_can::Frame,
{
    let m_type = MsgType::GetNodeInfo;
    let buf = unsafe { &mut BUF_NODE_INFO };

    if node_name.len() > buf.len() - m_type.payload_size() as usize {
        return Err(CanError::PayloadData);
    }

    // println!("Node name: {:?}", node_name);

    buf[..7].clone_from_slice(&status.to_bytes());
    buf[7..22].clone_from_slice(&software_version.to_bytes());
    buf[22..41].clone_from_slice(&hardware_version.to_bytes());

    // Important: In legacy mode, there is no node len field for name, due to tail array
    // optimization; We jump right into the byte representation.
    // In FD mode, a 7-bit node len field is required.

    // From experiments, doesn't seem to need 1 added for FD with currently tested
    // inputs?
    let mut payload_size = m_type.payload_size() as usize + node_name.len();

    if fd_mode {
        let bits = buf.view_bits_mut::<Msb0>();

        let mut i_bit = 41 * 8;

        bits[i_bit..i_bit + 7].store_le(node_name.len() as u8);
        i_bit += 7;

        for char in node_name {
            // Why big endian? Not sure, but by trial+error, this is it.
            // todo: Why BE here? Confirm in non-FD it's the same
            bits[i_bit..i_bit + 8].store_be(*char);
            i_bit += 8;
        }

        payload_size += 1;
    } else {
        buf[41..41 + node_name.len()].clone_from_slice(node_name);
    }

    let frame_type = FrameType::Service(ServiceData {
        dest_node_id: requester_node_id,
        req_or_resp: RequestResponse::Response,
    });

    can.broadcast(frame_type, m_type, node_id, buf, fd_mode)
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/4.GetTransportStats.uavcan
/// Standard data type: uavcan.protocol.GetTransportStats
/// This is published in response to a requested.
/// todo: What is the data type ID? 4 is in conflict.
pub fn publish_transport_stats<CAN, FRAME>(
    can: &mut Uavcan<CAN, FRAME>,
    num_transmitted: u64,
    num_received: u64,
    num_errors: u64,
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError>
where
    CAN: embedded_can::blocking::Can<Frame = FRAME>,
    FRAME: embedded_can::Frame,
{
    let m_type = MsgType::TransportStats;

    let buf = unsafe { &mut BUF_TRANSPORT_STATS };

    buf[..6].clone_from_slice(&num_transmitted.to_le_bytes()[..6]);
    buf[6..12].clone_from_slice(&num_received.to_le_bytes()[..6]);
    buf[12..18].clone_from_slice(&num_errors.to_le_bytes()[..6]);

    can.broadcast(FrameType::Message, m_type, node_id, buf, fd_mode)
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/5.Panic.uavcan
/// "Nodes that are expected to react to this message should wait for at least MIN_MESSAGES subsequent messages
/// with any reason text from any sender published with the interval no higher than MAX_INTERVAL_MS before
/// undertaking any emergency actions." (Min messages: 3. Max interval: 500ms)
pub fn publish_panic<CAN, FRAME>(
    can: &mut Uavcan<CAN, FRAME>,
    reason_text: &mut [u8],
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError>
where
    CAN: embedded_can::blocking::Can<Frame = FRAME>,
    FRAME: embedded_can::Frame,
{
    if reason_text.len() > 7 {
        return Err(CanError::PayloadSize);
    }

    let m_type = MsgType::Panic;

    can.broadcast(FrameType::Message, m_type, node_id, reason_text, fd_mode)
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/4.GlobalTimeSync.uavcan
pub fn publish_time_sync<CAN, FRAME>(
    can: &mut Uavcan<CAN, FRAME>,
    previous_transmission_timestamp_usec: u64,
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError>
where
    CAN: embedded_can::blocking::Can<Frame = FRAME>,
    FRAME: embedded_can::Frame,
{
    let m_type = MsgType::GlobalTimeSync;

    let buf = unsafe { &mut BUF_TIME_SYNC };

    buf[..7].clone_from_slice(&previous_transmission_timestamp_usec.to_le_bytes());

    can.broadcast(FrameType::Message, m_type, node_id, buf, fd_mode)?;

    Ok(())
}

// todo: Publish air data

/// https://github.com/dronecan/DSDL/blob/master/uavcan/equipment/air_data/1028.StaticPressure.uavcan
pub fn publish_static_pressure<CAN, FRAME>(
    can: &mut Uavcan<CAN, FRAME>,
    pressure: f32,          // Pascal
    pressure_variance: f32, // Pascal^2. 16-bit
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError>
where
    CAN: embedded_can::blocking::Can<Frame = FRAME>,
    FRAME: embedded_can::Frame,
{
    let m_type = MsgType::StaticPressure;

    let buf = unsafe { &mut BUF_PRESSURE };

    buf[..4].copy_from_slice(&pressure.to_le_bytes());

    let pressure_variance = f16::from_f32(pressure_variance);
    buf[4..6].copy_from_slice(&pressure_variance.to_le_bytes());

    can.broadcast(FrameType::Message, m_type, node_id, buf, fd_mode)
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/equipment/air_data/1029.StaticTemperature.uavcan
pub fn publish_temperature<CAN, FRAME>(
    can: &mut Uavcan<CAN, FRAME>,
    temperature: f32,          // Kelvin. 16-bit.
    temperature_variance: f32, // Kelvin^2. 16-bit
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError>
where
    CAN: embedded_can::blocking::Can<Frame = FRAME>,
    FRAME: embedded_can::Frame,
{
    let m_type = MsgType::StaticTemperature;

    let buf = unsafe { &mut BUF_TEMPERATURE };

    let temperature = f16::from_f32(temperature);
    buf[..2].clone_from_slice(&temperature.to_le_bytes());

    let temperature_variance = f16::from_f32(temperature_variance);
    buf[2..4].clone_from_slice(&temperature_variance.to_le_bytes());

    can.broadcast(FrameType::Message, m_type, node_id, buf, fd_mode)
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/equipment/ahrs/1000.Solution.uavcan
pub fn publish_ahrs_solution<CAN, FRAME>(
    can: &mut Uavcan<CAN, FRAME>,
    solution: AhrsSolution,
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError>
where
    CAN: embedded_can::blocking::Can<Frame = FRAME>,
    FRAME: embedded_can::Frame,
{
    let mut buf = Vec::new();
    solution.to_bytes(&mut buf);

    can.broadcast(
        FrameType::Message,
        MsgType::AhrsSolution,
        node_id,
        buf.as_mut_slice(),
        fd_mode,
    )
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/equipment/ahrs/1002.MagneticFieldStrength2.uavcan
pub fn publish_mag_field_strength<CAN, FRAME>(
    can: &mut Uavcan<CAN, FRAME>,
    sensor_id: u8,
    magnetic_field: &[f32; 3],          // f16. Gauss; X, Y, Z.
    _magnetic_field_covariance: &[f32], // f16
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError>
where
    CAN: embedded_can::blocking::Can<Frame = FRAME>,
    FRAME: embedded_can::Frame,
{
    let m_type = MsgType::MagneticFieldStrength2;

    // let mut buf = [0; crate::find_tail_byte_index(PAYLOAD_SIZE_MAGNETIC_FIELD_STRENGTH2 as u8) + 1];
    let buf = unsafe { &mut BUF_MAGNETIC_FIELD_STRENGTH2 };

    let field_x = f16::from_f32(magnetic_field[0]);
    let field_y = f16::from_f32(magnetic_field[1]);
    let field_z = f16::from_f32(magnetic_field[2]);

    buf[0] = sensor_id;
    buf[1..3].clone_from_slice(&field_x.to_le_bytes());
    buf[3..5].clone_from_slice(&field_y.to_le_bytes());
    buf[5..7].clone_from_slice(&field_z.to_le_bytes());

    // Must specify covariance length if on FD mode.
    let payload_size = if fd_mode {
        buf[7] = 0;
        Some(8)
    } else {
        None
    };
    // todo: Covariance as-required.

    can.broadcast(FrameType::Message, m_type, node_id, buf, fd_mode)
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/equipment/ahrs/1003.RawIMU.uavcan
pub fn publish_raw_imu<CAN, FRAME>(
    can: &mut Uavcan<CAN, FRAME>,
    timestamp: u64,  // 7-bytes, us.
    gyro: [f32; 3],  // f16. x, y, z. rad/s (Roll, pitch, yaw)
    accel: [f32; 3], // f16. x, y, z. m/s^2
    // todo: integration?
    // todo: Covariances?
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError>
where
    CAN: embedded_can::blocking::Can<Frame = FRAME>,
    FRAME: embedded_can::Frame,
{
    let m_type = MsgType::RawImu;

    let buf = unsafe { &mut BUF_RAW_IMU };

    buf[..7].clone_from_slice(&timestamp.to_le_bytes()[0..7]);
    // integration interval: 0 here.

    buf[11..13].clone_from_slice(&f16::from_f32(gyro[0]).to_le_bytes());
    buf[13..15].clone_from_slice(&f16::from_f32(gyro[1]).to_le_bytes());
    buf[15..17].clone_from_slice(&f16::from_f32(gyro[2]).to_le_bytes());

    // 0s for integral of gyro and accel.

    buf[29..31].clone_from_slice(&f16::from_f32(accel[0]).to_le_bytes());
    buf[31..33].clone_from_slice(&f16::from_f32(accel[1]).to_le_bytes());
    buf[33..35].clone_from_slice(&f16::from_f32(accel[2]).to_le_bytes());

    // todo: Covariance, and integration as-required.

    let payload_size = if fd_mode {
        m_type.payload_size() + 1 // Due to no TCO on final cov array.
    } else {
        m_type.payload_size()
    };

    can.broadcast(FrameType::Message, m_type, node_id, buf, fd_mode)
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/navigation/2000.GlobalNavigationSolution.uavcan
pub fn publish_global_navigation_solution<CAN, FRAME>(
    can: &mut Uavcan<CAN, FRAME>,
    data: &GlobalNavSolution,
    // todo: Covariances?
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError>
where
    CAN: embedded_can::blocking::Can<Frame = FRAME>,
    FRAME: embedded_can::Frame,
{
    let m_type = MsgType::GlobalNavigationSolution;

    let buf = unsafe { &mut BUF_GLOBAL_NAVIGATION_SOLUTION };

    buf[..m_type.payload_size() as usize].clone_from_slice(&data.pack().unwrap());

    let payload_size = if fd_mode {
        m_type.payload_size() + 1 // Due to no TCO on final cov array.
    } else {
        m_type.payload_size()
    };

    can.broadcast(FrameType::Message, m_type, node_id, buf, fd_mode)
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/equipment/gnss/1061.Auxiliary.uavcan
pub fn publish_gnss_aux<CAN, FRAME>(
    can: &mut Uavcan<CAN, FRAME>,
    data: &GnssAuxiliary,
    // todo: Covariances?
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError>
where
    CAN: embedded_can::blocking::Can<Frame = FRAME>,
    FRAME: embedded_can::Frame,
{
    // let mut buf = [0; crate::find_tail_byte_index(PAYLOAD_SIZE_MAGNETIC_FIELD_STRENGTH2 as u8) + 1];
    let buf = unsafe { &mut BUF_GNSS_AUX };

    let m_type = MsgType::GnssAux;

    buf[..m_type.payload_size() as usize].clone_from_slice(&data.to_bytes());

    can.broadcast(FrameType::Message, m_type, node_id, buf, fd_mode)
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/equipment/gnss/1063.Fix2.uavcan
pub fn publish_fix2<CAN, FRAME>(
    can: &mut Uavcan<CAN, FRAME>,
    data: &FixDronecan,
    // todo: Covariances?
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError>
where
    CAN: embedded_can::blocking::Can<Frame = FRAME>,
    FRAME: embedded_can::Frame,
{
    let buf = unsafe { &mut BUF_FIX2 };

    let m_type = MsgType::Fix2;

    buf[..m_type.payload_size() as usize].clone_from_slice(&data.pack().unwrap());

    can.broadcast(FrameType::Message, m_type, node_id, buf, fd_mode)
}

/// https://github.com/dronecan/DSDL/blob/master/ardupilot/gnss/20003.Status.uavcan
/// Standard data type: uavcan.protocol.NodeStatus
/// Must be broadcast at intervals between 2 and 1000ms. FC firmware should
/// consider the node to be faulty if this is not received for 3s.
pub fn publish_ardupilot_gnss_status<CAN, FRAME>(
    can: &mut Uavcan<CAN, FRAME>,
    error_codes: u32,
    _healthy: bool,
    _status: u32,
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError>
where
    CAN: embedded_can::blocking::Can<Frame = FRAME>,
    FRAME: embedded_can::Frame,
{
    let m_type = MsgType::ArdupilotGnssStatus;

    let buf = unsafe { &mut BUF_ARDUPILOT_GNSS_STATUS };

    buf[0..4].copy_from_slice(&error_codes.to_le_bytes());

    // let status = status & 0b111_1111_1111_1111_1111_1111;

    // todo: Sort this out.
    // buf[4..8].copy_from_slice(&((healthy as u32) << 23 | status).to_le_bytes());

    buf[4] = 0x81; // armable and status 0. Having trouble with bit masks.

    can.broadcast(FrameType::Message, m_type, node_id, buf, fd_mode)
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/dynamic_node_id/1.Allocation.uavcan
/// Send while the node is anonymous; requests an ID.
pub fn request_id_allocation_req<CAN, FRAME>(
    can: &mut Uavcan<CAN, FRAME>,
    data: &IdAllocationData,
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError>
where
    CAN: embedded_can::blocking::Can<Frame = FRAME>,
    FRAME: embedded_can::Frame,
{
    let buf = unsafe { &mut BUF_ID_ALLOCATION };

    let m_type = MsgType::IdAllocation;

    buf[0..m_type.payload_size() as usize].clone_from_slice(&data.to_bytes(fd_mode));

    // 6 bytes of unique_id unless in the final stage; then 4.
    let len = if fd_mode {
        m_type.payload_size() as usize
    } else if data.stage == 2 {
        5
    } else {
        7
    };

    can.broadcast(FrameType::MessageAnon, m_type, node_id, buf, fd_mode)
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/param/11.GetSet.uavcan
/// We send this after receiving an (empty) GetSet request. We respond to the parameter
/// associated with the index requested. If the requested index doesn't match with a parameter
/// we have, we reply with an empty response. This indicates that we have passed all parameters.
/// The requester increments the index starting at 0 to poll parameters available.
pub fn publish_getset_resp<CAN, FRAME>(
    can: &mut Uavcan<CAN, FRAME>,
    data: &GetSetResponse,
    fd_mode: bool,
    node_id: u8,
    requester_node_id: u8,
) -> Result<(), CanError>
where
    CAN: embedded_can::blocking::Can<Frame = FRAME>,
    FRAME: embedded_can::Frame,
{
    let buf = unsafe { &mut BUF_GET_SET };

    // //Empty the buffer in case this message is shorter than the previous one; variable length.
    // *buf = [0; unsafe { BUF_GET_SET.len() }];

    let m_type = MsgType::GetSet;

    let len = data.to_bytes(buf, fd_mode);

    let frame_type = FrameType::Service(ServiceData {
        dest_node_id: requester_node_id,
        req_or_resp: RequestResponse::Response,
    });

    can.broadcast(frame_type, m_type, node_id, buf, fd_mode)
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/4.GlobalTimeSync.uavcan
pub fn _handle_time_sync<CAN, FRAME>(
    _can: &mut Uavcan<CAN, FRAME>,
    payload: &[u8],
    _fd_mode: bool,
    _node_id: u8,
) -> Result<(), CanError>
where
    CAN: embedded_can::blocking::Can<Frame = FRAME>,
    FRAME: embedded_can::Frame,
{
    let mut buf = [0; 8];
    buf[..7].clone_from_slice(payload);
    let _previous_transmission_timestamp_usec = u64::from_le_bytes(buf);

    // todo: Handle.

    Ok(())
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/5.RestartNode.uavcan
pub fn handle_restart_request<CAN, FRAME>(
    can: &mut Uavcan<CAN, FRAME>,
    payload: &[u8],
    fd_mode: bool,
    node_id: u8,
    requester_node_id: u8,
) -> Result<(), CanError>
where
    CAN: embedded_can::blocking::Can<Frame = FRAME>,
    FRAME: embedded_can::Frame,
{
    let m_type = MsgType::Restart;

    let mut num_bytes = [0; 8];
    num_bytes[..5].clone_from_slice(payload);
    let magic_number = u64::from_le_bytes(num_bytes);

    if magic_number != 0xAC_CE55_1B1E {
        let frame_type = FrameType::Service(ServiceData {
            dest_node_id: requester_node_id,
            req_or_resp: RequestResponse::Response,
        });

        can.broadcast(
            frame_type,
            m_type,
            node_id,
            &mut [0], // ie false; error
            fd_mode,
        )?;

        return Err(CanError::PayloadData);
    }

    let frame_type = FrameType::Service(ServiceData {
        dest_node_id: requester_node_id,
        req_or_resp: RequestResponse::Response,
    });

    can.broadcast(
        frame_type,
        m_type,
        node_id,
        &mut [1], // ie true; success
        // 1,
        fd_mode,
    )?;

    // let cp = unsafe { cortex_m::Peripherals::steal() };
    // todo: Not working.
    // cp.SCB.sys_reset();

    Ok(())
}

// fn message_pending_handler(mailbox: Mailbox, header: TxFrameHeader, buf: &[u32]) {
//     println!("Mailbox overflow!");
// }

// /// Function to help parse the nested result from CAN rx results
// pub fn get_frame_info(
//     rx_result: Result<ReceiveOverrun<RxFrameInfo>, nb::Error<Infallible>>,
// ) -> Result<RxFrameInfo, CanError> {
//     // todo: This masks overruns currently.

//     match rx_result {
//         Ok(r) => match r {
//             ReceiveOverrun::NoOverrun(frame_info) => Ok(frame_info),
//             ReceiveOverrun::Overrun(frame_info) => Ok(frame_info),
//         },
//         Err(_) => Err(CanError::Hardware),
//     }
// }

#[derive(Clone, Copy)]
#[repr(u8)]
pub enum ActuatorCommandType {
    Unitless = 0,
    Position = 1,
    Force = 2,
    Speed = 3,
    Pwm = 4,
}

pub struct ActuatorCommand {
    pub actuator_id: u8,
    pub type_: ActuatorCommandType,
    pub value: f32,
}

impl ActuatorCommand {
    pub fn from_buf(data: &[u8]) -> Self {
        let actuator_id = data[0];
        let type_ = match data[1] {
            // todo: DRY with definitions above; use num_enum.
            0 => ActuatorCommandType::Unitless,
            1 => ActuatorCommandType::Position,
            2 => ActuatorCommandType::Force,
            3 => ActuatorCommandType::Speed,
            4 => ActuatorCommandType::Pwm,
            _ => ActuatorCommandType::Unitless, // fallthrough. Fail?
        };

        // todo: Handle different command types; currently assuming PWM.

        let value = f16::from_le_bytes([data[2], data[3]]).to_f32(); // todo: QC order.

        Self {
            actuator_id,
            type_,
            value,
        }
    }
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/equipment/actuator/Command.uavcan
/// https://github.com/dronecan/DSDL/blob/master/uavcan/equipment/actuator/1010.ArrayCommand.uavcan
pub fn publish_actuator_commands<CAN, FRAME>(
    can: &mut Uavcan<CAN, FRAME>,
    commands: &[ActuatorCommand],
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError>
where
    CAN: embedded_can::blocking::Can<Frame = FRAME>,
    FRAME: embedded_can::Frame,
{
    if commands.len() > 4 {
        warn!("Exceeded present limit of 4 actuator commands");
        return Err(CanError::PayloadData);
    }

    let buf = unsafe { &mut BUF_ACTUATOR_ARRAY_COMMAND };

    let m_type = MsgType::ActuatorArrayCommand;

    let mut payload_len = ACTUATOR_COMMAND_SIZE * commands.len();

    if fd_mode {
        let bits = buf.view_bits_mut::<Msb0>();

        bits[0..4].store_le(commands.len());
        let mut i_bit = 4;

        for command in commands {
            bits[i_bit..i_bit + 8].store_be(command.actuator_id);
            bits[i_bit + 8..i_bit + 16].store_be(command.type_ as u8);

            let f16_bytes = f16::from_f32(command.value).to_le_bytes();

            bits[i_bit + 16..i_bit + 24].store_be(f16_bytes[0]);
            bits[i_bit + 24..i_bit + 32].store_be(f16_bytes[1]);

            i_bit += ACTUATOR_COMMAND_SIZE * 8;

            // todo: This may not be strictly always true; could cause CRC failures.(?)
            payload_len += 1;
        }
    } else {
        let mut i = 0;
        for command in commands {
            buf[i] = command.actuator_id;
            buf[i + 1] = command.type_ as u8;

            let f16_bytes = f16::from_f32(command.value).to_le_bytes();
            buf[i + 2] = f16_bytes[0];
            buf[i + 3] = f16_bytes[1];

            i += ACTUATOR_COMMAND_SIZE
        }
    }

    can.broadcast(FrameType::Message, m_type, node_id, buf, fd_mode)
}

/// [DSDL 1091](https://github.com/dronecan/DSDL/blob/master/uavcan/equipment/power/1091.CircuitStatus.uavcan)
pub fn publish_circuit_status<CAN, FRAME>(
    can: &mut Uavcan<CAN, FRAME>,
    circuit_id: u16,
    voltage: f32,
    current: f32,
    // error_flags: &[CircuitStatusErrorFlag],
    error_flags: u8,
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError>
where
    CAN: embedded_can::blocking::Can<Frame = FRAME>,
    FRAME: embedded_can::Frame,
{
    let buf = unsafe { &mut BUF_CIRCUIT_STATUS };

    let m_type = MsgType::CircuitStatus;

    buf[0..2].clone_from_slice(&circuit_id.to_le_bytes());
    buf[2..4].clone_from_slice(&f16::from_f32(voltage).to_le_bytes());
    buf[4..6].clone_from_slice(&f16::from_f32(current).to_le_bytes());

    // Flags handled in firmware for now due to not having dynamic arrays.
    buf[6] = error_flags;
    // buf[6] = 0;
    // for flag in error_flags {
    //     buf[6] |= *flag as u8;
    // }

    can.broadcast(FrameType::Message, m_type, node_id, buf, fd_mode)
}

/// [DSDL 1090](https://github.com/dronecan/DSDL/blob/master/uavcan/equipment/power/1090.PrimaryPowerSupplyStatus.uavcan)
pub fn publish_power_supply_status<CAN, FRAME>(
    can: &mut Uavcan<CAN, FRAME>,
    hours_to_empty: f32,
    hours_to_empty_variance: f32,
    external_power_avail: bool,
    remaining_energy_pct: u8,
    _remaining_energy_ct_std: u8,
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError>
where
    CAN: embedded_can::blocking::Can<Frame = FRAME>,
    FRAME: embedded_can::Frame,
{
    let buf = unsafe { &mut BUF_POWER_SUPPLY_STATUS };

    let m_type = MsgType::PowerSupplyStatus;

    buf[0..2].clone_from_slice(&f16::from_f32(hours_to_empty).to_le_bytes());
    buf[2..4].clone_from_slice(&f16::from_f32(hours_to_empty_variance).to_le_bytes());

    buf[4] = (external_power_avail as u8) << 7 | (remaining_energy_pct & 0b111_1111);
    // Note: Skipping remaining energy pct std for now.

    can.broadcast(FrameType::Message, m_type, node_id, buf, fd_mode)
}
