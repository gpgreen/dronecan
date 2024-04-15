//! This module contains code related to broadcasting messages over CAN.

use crate::{
    dsdl::{
        AhrsSolution, GetNodeInfoResponse, GetSetResponse, IdAllocationData, NodeHealth, NodeMode,
        NodeStatus,
    },
    f16,
    gnss::{FixDronecan, GlobalNavSolution, GnssAuxiliary},
    interface::UavcanInterface,
    messages::MsgType,
    protocol::{FrameType, RequestResponse, ServiceData},
    CanError,
};
use embedded_can;
use log::*;
use packed_struct::PackedStruct;

// Static buffers, to ensure they live long enough through transmission. Note; This all include room for a tail byte,
// based on payload len. We also assume no `certificate_of_authority` for hardware size.

// These ID allocation buffers accomodate the length including full 16-bit unique id, and tail byte.
static mut BUF_ID_ALLOCATION: [u8; 20] = [0; 20];
// static mut BUF_ID_RESP: [u8; 17] = [0; 17];
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

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/341.NodeStatus.uavcan
/// Standard data type: uavcan.protocol.NodeStatus
/// Must be broadcast at intervals between 2 and 1000ms. FC firmware should
/// consider the node to be faulty if this is not received for 3s.
pub fn publish_node_status<CAN, FRAME>(
    can: &mut UavcanInterface<CAN, FRAME>,
    health: NodeHealth,
    mode: NodeMode,
    vendor_specific_status_code: u16,
    uptime_sec: u32,
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

    can.broadcast(FrameType::Message, m_type, node_id, &status.to_bytes())
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/1.GetNodeInfo.uavcan
/// A composite type sent in response to a request.
pub fn publish_node_info<CAN, FRAME>(
    can: &mut UavcanInterface<CAN, FRAME>,
    node_info: &GetNodeInfoResponse,
    node_id: u8,
    requester_node_id: u8,
) -> Result<(), CanError>
where
    CAN: embedded_can::blocking::Can<Frame = FRAME>,
    FRAME: embedded_can::Frame,
{
    let m_type = MsgType::GetNodeInfo;

    let frame_type = FrameType::Service(ServiceData {
        dest_node_id: requester_node_id,
        req_or_resp: RequestResponse::Response,
    });

    can.broadcast(frame_type, m_type, node_id, &node_info.to_bytes())
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/4.GetTransportStats.uavcan
/// Standard data type: uavcan.protocol.GetTransportStats
/// This is published in response to a requested.
/// todo: What is the data type ID? 4 is in conflict.
pub fn publish_transport_stats<CAN, FRAME>(
    can: &mut UavcanInterface<CAN, FRAME>,
    num_transmitted: u64,
    num_received: u64,
    num_errors: u64,
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

    can.broadcast(FrameType::Message, m_type, node_id, buf)
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/5.Panic.uavcan
/// "Nodes that are expected to react to this message should wait for at least MIN_MESSAGES subsequent messages
/// with any reason text from any sender published with the interval no higher than MAX_INTERVAL_MS before
/// undertaking any emergency actions." (Min messages: 3. Max interval: 500ms)
pub fn publish_panic<CAN, FRAME>(
    can: &mut UavcanInterface<CAN, FRAME>,
    reason_text: &mut [u8],
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

    can.broadcast(FrameType::Message, m_type, node_id, reason_text)
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/4.GlobalTimeSync.uavcan
pub fn publish_time_sync<CAN, FRAME>(
    can: &mut UavcanInterface<CAN, FRAME>,
    previous_transmission_timestamp_usec: u64,
    node_id: u8,
) -> Result<(), CanError>
where
    CAN: embedded_can::blocking::Can<Frame = FRAME>,
    FRAME: embedded_can::Frame,
{
    let m_type = MsgType::GlobalTimeSync;

    let buf = unsafe { &mut BUF_TIME_SYNC };

    buf[..7].clone_from_slice(&previous_transmission_timestamp_usec.to_le_bytes());

    can.broadcast(FrameType::Message, m_type, node_id, buf)?;

    Ok(())
}

// todo: Publish air data

/// https://github.com/dronecan/DSDL/blob/master/uavcan/equipment/air_data/1028.StaticPressure.uavcan
pub fn publish_static_pressure<CAN, FRAME>(
    can: &mut UavcanInterface<CAN, FRAME>,
    pressure: f32,          // Pascal
    pressure_variance: f32, // Pascal^2. 16-bit
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

    can.broadcast(FrameType::Message, m_type, node_id, buf)
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/equipment/air_data/1029.StaticTemperature.uavcan
pub fn publish_temperature<CAN, FRAME>(
    can: &mut UavcanInterface<CAN, FRAME>,
    temperature: f32,          // Kelvin. 16-bit.
    temperature_variance: f32, // Kelvin^2. 16-bit
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

    can.broadcast(FrameType::Message, m_type, node_id, buf)
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/equipment/ahrs/1000.Solution.uavcan
pub fn publish_ahrs_solution<CAN, FRAME>(
    can: &mut UavcanInterface<CAN, FRAME>,
    solution: AhrsSolution,
    node_id: u8,
) -> Result<(), CanError>
where
    CAN: embedded_can::blocking::Can<Frame = FRAME>,
    FRAME: embedded_can::Frame,
{
    let mut buf = solution.to_bytes();

    can.broadcast(
        FrameType::Message,
        MsgType::AhrsSolution,
        node_id,
        buf.as_mut_slice(),
    )
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/equipment/ahrs/1002.MagneticFieldStrength2.uavcan
pub fn publish_mag_field_strength<CAN, FRAME>(
    can: &mut UavcanInterface<CAN, FRAME>,
    sensor_id: u8,
    magnetic_field: &[f32; 3],          // f16. Gauss; X, Y, Z.
    _magnetic_field_covariance: &[f32], // f16
    node_id: u8,
) -> Result<(), CanError>
where
    CAN: embedded_can::blocking::Can<Frame = FRAME>,
    FRAME: embedded_can::Frame,
{
    let m_type = MsgType::MagneticFieldStrength2;

    let buf = unsafe { &mut BUF_MAGNETIC_FIELD_STRENGTH2 };

    let field_x = f16::from_f32(magnetic_field[0]);
    let field_y = f16::from_f32(magnetic_field[1]);
    let field_z = f16::from_f32(magnetic_field[2]);

    buf[0] = sensor_id;
    buf[1..3].clone_from_slice(&field_x.to_le_bytes());
    buf[3..5].clone_from_slice(&field_y.to_le_bytes());
    buf[5..7].clone_from_slice(&field_z.to_le_bytes());

    // todo: Covariance as-required.

    can.broadcast(FrameType::Message, m_type, node_id, buf)
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/equipment/ahrs/1003.RawIMU.uavcan
pub fn publish_raw_imu<CAN, FRAME>(
    can: &mut UavcanInterface<CAN, FRAME>,
    timestamp: u64,  // 7-bytes, us.
    gyro: [f32; 3],  // f16. x, y, z. rad/s (Roll, pitch, yaw)
    accel: [f32; 3], // f16. x, y, z. m/s^2
    // todo: integration?
    // todo: Covariances?
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

    can.broadcast(FrameType::Message, m_type, node_id, buf)
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/navigation/2000.GlobalNavigationSolution.uavcan
pub fn publish_global_navigation_solution<CAN, FRAME>(
    can: &mut UavcanInterface<CAN, FRAME>,
    data: &GlobalNavSolution,
    // todo: Covariances?
    node_id: u8,
) -> Result<(), CanError>
where
    CAN: embedded_can::blocking::Can<Frame = FRAME>,
    FRAME: embedded_can::Frame,
{
    let m_type = MsgType::GlobalNavigationSolution;

    let buf = unsafe { &mut BUF_GLOBAL_NAVIGATION_SOLUTION };

    buf[..m_type.payload_size() as usize].clone_from_slice(&data.pack().unwrap());

    can.broadcast(FrameType::Message, m_type, node_id, buf)
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/equipment/gnss/1061.Auxiliary.uavcan
pub fn publish_gnss_aux<CAN, FRAME>(
    can: &mut UavcanInterface<CAN, FRAME>,
    data: &GnssAuxiliary,
    // todo: Covariances?
    node_id: u8,
) -> Result<(), CanError>
where
    CAN: embedded_can::blocking::Can<Frame = FRAME>,
    FRAME: embedded_can::Frame,
{
    let buf = unsafe { &mut BUF_GNSS_AUX };

    let m_type = MsgType::GnssAux;

    buf[..m_type.payload_size() as usize].clone_from_slice(&data.to_bytes());

    can.broadcast(FrameType::Message, m_type, node_id, buf)
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/equipment/gnss/1063.Fix2.uavcan
pub fn publish_fix2<CAN, FRAME>(
    can: &mut UavcanInterface<CAN, FRAME>,
    data: &FixDronecan,
    // todo: Covariances?
    node_id: u8,
) -> Result<(), CanError>
where
    CAN: embedded_can::blocking::Can<Frame = FRAME>,
    FRAME: embedded_can::Frame,
{
    let buf = unsafe { &mut BUF_FIX2 };

    let m_type = MsgType::Fix2;

    buf[..m_type.payload_size() as usize].clone_from_slice(&data.pack().unwrap());

    can.broadcast(FrameType::Message, m_type, node_id, buf)
}

/// https://github.com/dronecan/DSDL/blob/master/ardupilot/gnss/20003.Status.uavcan
/// Standard data type: uavcan.protocol.NodeStatus
/// Must be broadcast at intervals between 2 and 1000ms. FC firmware should
/// consider the node to be faulty if this is not received for 3s.
pub fn publish_ardupilot_gnss_status<CAN, FRAME>(
    can: &mut UavcanInterface<CAN, FRAME>,
    error_codes: u32,
    _healthy: bool,
    _status: u32,
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

    can.broadcast(FrameType::Message, m_type, node_id, buf)
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/dynamic_node_id/1.Allocation.uavcan
/// Send while the node is anonymous; requests an ID.
pub fn request_id_allocation_req<CAN, FRAME>(
    can: &mut UavcanInterface<CAN, FRAME>,
    data: &IdAllocationData,
    node_id: u8,
) -> Result<(), CanError>
where
    CAN: embedded_can::blocking::Can<Frame = FRAME>,
    FRAME: embedded_can::Frame,
{
    let buf = unsafe { &mut BUF_ID_ALLOCATION };

    let m_type = MsgType::IdAllocation;

    buf[0..m_type.payload_size() as usize].clone_from_slice(&data.to_bytes(false));

    can.broadcast(FrameType::MessageAnon, m_type, node_id, buf)
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/param/11.GetSet.uavcan
/// We send this after receiving an (empty) GetSet request. We respond to the parameter
/// associated with the index requested. If the requested index doesn't match with a parameter
/// we have, we reply with an empty response. This indicates that we have passed all parameters.
/// The requester increments the index starting at 0 to poll parameters available.
pub fn publish_getset_resp<CAN, FRAME>(
    can: &mut UavcanInterface<CAN, FRAME>,
    _data: &GetSetResponse,
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

    let frame_type = FrameType::Service(ServiceData {
        dest_node_id: requester_node_id,
        req_or_resp: RequestResponse::Response,
    });

    can.broadcast(frame_type, m_type, node_id, buf)
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/4.GlobalTimeSync.uavcan
pub fn _handle_time_sync<CAN, FRAME>(
    _can: &mut UavcanInterface<CAN, FRAME>,
    payload: &[u8],
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
    can: &mut UavcanInterface<CAN, FRAME>,
    payload: &[u8],
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
    can: &mut UavcanInterface<CAN, FRAME>,
    commands: &[ActuatorCommand],
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

    let _payload_len = ACTUATOR_COMMAND_SIZE * commands.len();

    let mut i = 0;
    for command in commands {
        buf[i] = command.actuator_id;
        buf[i + 1] = command.type_ as u8;

        let f16_bytes = f16::from_f32(command.value).to_le_bytes();
        buf[i + 2] = f16_bytes[0];
        buf[i + 3] = f16_bytes[1];

        i += ACTUATOR_COMMAND_SIZE
    }

    can.broadcast(FrameType::Message, m_type, node_id, buf)
}

/// [DSDL 1091](https://github.com/dronecan/DSDL/blob/master/uavcan/equipment/power/1091.CircuitStatus.uavcan)
pub fn publish_circuit_status<CAN, FRAME>(
    can: &mut UavcanInterface<CAN, FRAME>,
    circuit_id: u16,
    voltage: f32,
    current: f32,
    // error_flags: &[CircuitStatusErrorFlag],
    error_flags: u8,
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

    can.broadcast(FrameType::Message, m_type, node_id, buf)
}

/// [DSDL 1090](https://github.com/dronecan/DSDL/blob/master/uavcan/equipment/power/1090.PrimaryPowerSupplyStatus.uavcan)
pub fn publish_power_supply_status<CAN, FRAME>(
    can: &mut UavcanInterface<CAN, FRAME>,
    hours_to_empty: f32,
    hours_to_empty_variance: f32,
    external_power_avail: bool,
    remaining_energy_pct: u8,
    _remaining_energy_ct_std: u8,
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

    can.broadcast(FrameType::Message, m_type, node_id, buf)
}
