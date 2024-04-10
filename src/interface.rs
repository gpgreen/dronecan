//! This module contains code related to broadcasting messages over CAN.

use crate::{
    crc::TransferCrc,
    find_tail_byte_index,
    messages::MsgType,
    protocol::{CanId, FrameType, RequestResponse, TailByte, TransferComponent},
    CanError,
};
use embedded_can;
use heapless::{Deque, Entry, FnvIndexMap, Vec};
use log::*;

// Note: These are only capable of handling one message at a time. This is especially notable
// for reception.
static mut MULTI_FRAME_BUFS_TX: [[u8; 64]; 20] = [[0; 64]; 20];

pub(crate) const DATA_FRAME_MAX_LEN_LEGACY: usize = 8;

pub(crate) const MAX_RX_FRAMES_LEGACY: usize = 24;
pub(crate) const RX_BUFFER_MAX_SIZE: usize = MAX_RX_FRAMES_LEGACY * DATA_FRAME_MAX_LEN_LEGACY;

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

impl TransferDescKey {
    pub fn new(frame_type: FrameType, type_id: u16, source_node_id: u8) -> Self {
        match frame_type {
            FrameType::Message => TransferDescKey::Message(type_id, source_node_id),
            FrameType::MessageAnon => TransferDescKey::MessageAnon(type_id),
            FrameType::Service(s) => {
                TransferDescKey::Service(type_id, source_node_id, s.dest_node_id)
            }
        }
    }
}

/// The `TransferDesc` contains state for a given transfer
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug, Clone, PartialEq, Default)]
pub struct TransferDesc {
    transfer_id: u8,
    next_toggle_bit: bool,
    ts: u64,
    iface_index: usize,
    payload: Vec<u8, RX_BUFFER_MAX_SIZE>,
}

impl TransferDesc {
    pub const TRANSFER_ID_HALF_RANGE: u8 = 16;

    /// Get the forward distance for a transfer id
    pub fn compute_forward_distance(a: u8, b: u8) -> u8 {
        const MAX_VALUE: u8 = (1 << 5) - 1;
        if a > MAX_VALUE || b > MAX_VALUE {
            panic!();
        }
        if a > b {
            MAX_VALUE - a + b + 1
        } else {
            b - a
        }
    }

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
pub struct Uavcan<CAN, FRAME> {
    iface: Vec<CAN, 2>,
    tx_queue: Deque<FRAME, 10>,
    txf_map: FnvIndexMap<TransferDescKey, TransferDesc, 16>, // size must be power of 2
    initialized: bool,
    transfer_timeout: u64,
    interface_switch_delay: u64,
}

impl<CAN, FRAME, FE> Uavcan<CAN, FRAME>
where
    CAN: embedded_can::blocking::Can<Frame = FRAME, Error = FE>,
    FRAME: embedded_can::Frame,
{
    /// Create new `Uavcan`
    pub fn new(transfer_timeout: u64, interface_switch_delay: u64) -> Self {
        Self {
            iface: Vec::new(),
            tx_queue: Deque::new(),
            txf_map: FnvIndexMap::new(),
            initialized: false,
            transfer_timeout,
            interface_switch_delay,
        }
    }

    /// Add a `CAN` interface, maximum of 2 is allowed
    pub fn add_interface(&mut self, interface: CAN) {
        if self.iface.len() == 2 {
            panic!("Too many CAN interfaces");
        }
        self.iface.push(interface).ok();
    }

    /// Write out all pending transmit frames to all interfaces
    pub fn can_send(&mut self) -> Result<(), FE> {
        while let Some(frame) = self.tx_queue.pop_front() {
            for iface in self.iface.iter_mut() {
                iface.transmit(&frame)?;
            }
        }
        Ok(())
    }

    /// Send a DroneCAN "broadcast" message. See [The DroneCAN spec, transport layer page](https://dronecan.github.io/Specification/4._CAN_bus_transport_layer/)
    /// Should be broadcast at interval between 2 and 1000ms.
    pub fn broadcast(
        &mut self,
        frame_type: FrameType,
        msg_type: MsgType,
        source_node_id: u8,
        payload: &[u8],
    ) -> Result<(), CanError> {
        debug!(
            "broadcast frame:{:?} msg_type:{:?} source node:{} payload len: {}",
            frame_type,
            msg_type,
            source_node_id,
            payload.len(),
        );

        // This saves some if logic in node firmware re decision to broadcast.
        if source_node_id == 0 && frame_type != FrameType::MessageAnon {
            return Err(CanError::PayloadData);
        }

        // get the transfer descriptor
        // first construct the key from the info provided, then
        // we can get the descriptor from the hashmap
        let txf_key = TransferDescKey::new(frame_type, msg_type.id(), source_node_id);
        trace!("txf_key:{:?}", txf_key);
        loop {
            match self.txf_map.entry(txf_key) {
                Entry::Occupied(_v) => break,
                Entry::Vacant(v) => {
                    let desc = TransferDesc::default();
                    v.insert(desc).map_err(|_| CanError::TransferMapFull)?;
                }
            }
        }
        let txf_desc = match self.txf_map.get_mut(&txf_key) {
            Some(t) => Ok(t),
            None => panic!(),
        }?;
        trace!("descriptor start state: {:?}", txf_desc);

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
        // reset the tx buffer
        txf_desc.payload.clear();

        let can_id = CanId {
            priority: msg_type.priority(),
            type_id: msg_type.id(),
            source_node_id,
            frame_type,
        };
        trace!("can id:{:?}", can_id);

        // The transfer payload is up to 7 bytes for non-FD DRONECAN.
        // If data is longer than a single frame, set up a multi-frame transfer.
        if payload.len() >= DATA_FRAME_MAX_LEN_LEGACY {
            trace!("multiple frames payload");
            return self.queue_multiple_frames(
                payload,
                can_id.value(),
                transfer_id,
                msg_type.base_crc(),
            );
        }

        // make a new buffer to construct the data for a single frame, we need to append one byte
        // to the payload for the tail byte
        txf_desc
            .payload
            .extend_from_slice(payload)
            .map_err(|_| CanError::TxPayloadBufferFull)?;
        let tail_byte = TailByte::new(TransferComponent::SingleFrame, transfer_id);
        trace!("tail_byte:{:?}", tail_byte);
        txf_desc
            .payload
            .push(tail_byte.value())
            .map_err(|_| CanError::TxPayloadBufferFull)?;

        let id = embedded_can::Id::Extended(embedded_can::ExtendedId::new(can_id.value()).unwrap());
        self.tx_queue
            .push_back(FRAME::new(id, txf_desc.payload.as_slice()).unwrap())
            .map_err(|_| CanError::TransmitQueueFull)?;
        Ok(())
    }

    /// Handles splitting a payload into multiple frames
    /// when it is larger than a single frame
    fn queue_multiple_frames(
        &mut self,
        payload: &[u8],
        can_id: u32,
        transfer_id: u8,
        base_crc: u16,
    ) -> Result<(), CanError> {
        let frame_payload_len = DATA_FRAME_MAX_LEN_LEGACY;

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
        let mut tail_byte = TailByte::new(component, transfer_id);
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

            tail_byte = TailByte::new(component, transfer_id);

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

    /// Handle a new received frame. This returns the TransferDesc with the assembled payload
    /// if this is the last frame in a multi-frame transfer, or if it's a single-frame transfer.
    /// Returns None if frame is ignored for state reasons, or in the middle of a multi-frame payload
    /// see algorithm at [The DroneCAN spec, transport layer page](https://dronecan.github.io/Specification/4._CAN_bus_transport_layer/)
    pub fn handle_frame_rx(
        &mut self,
        iface_index: usize,
        frame: FRAME,
        frame_ts: u64,
    ) -> Result<Option<&TransferDesc>, CanError> {
        // get the CanId
        let can_id = match frame.id() {
            embedded_can::Id::Extended(eid) => CanId::from_value(eid.as_raw()),
            _ => return Ok(None),
        };
        debug!(
            "handle_frame_rx: Frame Id: {:?} dlc:{}",
            can_id,
            frame.dlc()
        );

        // get the tail byte
        let tail_byte = TailByte::from_value(frame.data()[frame.dlc() - 1]);
        trace!("tail_byte: {:?}", tail_byte);

        // get the transfer descriptor
        // first construct the key from the info provided, then
        // we can get the descriptor from the hashmap
        let txf_key =
            TransferDescKey::new(can_id.frame_type, can_id.type_id, can_id.source_node_id);
        trace!("txf_key: {:?}", txf_key);
        loop {
            match self.txf_map.entry(txf_key) {
                Entry::Occupied(_v) => break,
                Entry::Vacant(v) => {
                    let desc = TransferDesc::default();
                    v.insert(desc).map_err(|_| CanError::TransferMapFull)?;
                }
            }
        }
        let txf_desc = match self.txf_map.get_mut(&txf_key) {
            Some(t) => Ok(t),
            None => panic!(),
        }?;
        trace!("descriptor start state: {:?}", txf_desc);

        // update state flags for restart check
        let tid_timed_out = (frame_ts - txf_desc.ts) > self.transfer_timeout;
        let iface_switch_allowed = (frame_ts - txf_desc.ts) > self.interface_switch_delay;
        let same_iface = iface_index == txf_desc.iface_index;
        let non_wrapped_tid =
            TransferDesc::compute_forward_distance(txf_desc.transfer_id, tail_byte.transfer_id)
                < TransferDesc::TRANSFER_ID_HALF_RANGE;
        let not_previous_tid =
            TransferDesc::compute_forward_distance(tail_byte.transfer_id, txf_desc.transfer_id) > 1;
        trace!("tid_timed_out:{} iface_switch_allowed:{} same_iface:{} non_wrapped_tid:{} not_previous_tid:{}", tid_timed_out, iface_switch_allowed, same_iface, non_wrapped_tid, not_previous_tid);

        // check the state flags, deciding whether to restart
        if !self.initialized
            || tid_timed_out
            || (same_iface && tail_byte.start_of_transfer && not_previous_tid)
            || (iface_switch_allowed && tail_byte.start_of_transfer && non_wrapped_tid)
        {
            debug!("needs a restart");
            // needs a restart
            self.initialized = true;
            txf_desc.iface_index = iface_index;
            txf_desc.transfer_id = tail_byte.transfer_id;
            txf_desc.payload.clear();
            txf_desc.next_toggle_bit = false;
            // ignore this frame, if start of transfer was missed
            if !tail_byte.start_of_transfer {
                txf_desc.transfer_id += 1;
                return Ok(None);
            }
        }

        if iface_index != txf_desc.iface_index {
            trace!("wrong interface");
            // wrong interface, ignore
            return Ok(None);
        }
        if tail_byte.toggle != txf_desc.next_toggle_bit {
            trace!("unexpected toggle bit");
            // unexpected toggle bit, ignore
            return Ok(None);
        }
        if tail_byte.transfer_id != txf_desc.transfer_id {
            trace!("unexpected transfer id");
            // unexpected transfer ID, ignore
            return Ok(None);
        }
        if tail_byte.start_of_transfer {
            txf_desc.ts = frame_ts;
        }

        // update the toggle bit
        txf_desc.next_toggle_bit = !txf_desc.next_toggle_bit;
        // copy the frame data to the buffer
        txf_desc
            .payload
            .extend_from_slice(frame.data())
            .map_err(|_| CanError::RxPayloadBufferFull)?;
        trace!("descriptor final state: {:?}", txf_desc);

        if tail_byte.end_of_transfer {
            // do something with the payload
            txf_desc.transfer_id += 1;
            txf_desc.next_toggle_bit = false;
            Ok(Some(txf_desc))
        } else {
            Ok(None)
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use core::convert::Infallible;
    use embedded_can::{blocking::Can, ExtendedId, Frame, Id};
    use test_log::test;

    /// MockFrame
    #[derive(Debug, Clone, PartialEq, Eq)]
    struct MockFrame {
        id: ExtendedId,
        data: Vec<u8, 8>,
    }

    impl MockFrame {
        fn new(id: ExtendedId, data: &[u8]) -> Self {
            let mut d = Vec::new();
            d.extend_from_slice(data).unwrap();
            Self { id, data: d }
        }
    }

    impl Frame for MockFrame {
        fn new(id: impl Into<Id>, data: &[u8]) -> Option<Self> {
            if let Id::Extended(can_id) = id.into() {
                Some(MockFrame::new(can_id, data))
            } else {
                None
            }
        }

        fn new_remote(_id: impl Into<Id>, _dlc: usize) -> Option<Self> {
            unimplemented!();
        }

        fn is_extended(&self) -> bool {
            true
        }

        fn is_remote_frame(&self) -> bool {
            false
        }

        fn id(&self) -> Id {
            Id::Extended(self.id)
        }

        fn dlc(&self) -> usize {
            self.data.len()
        }

        fn data(&self) -> &[u8] {
            self.data.as_slice()
        }
    }

    /// MockCan
    /// TODO: implement errors
    #[derive(Debug)]
    struct MockCan {
        expected_tx: Vec<MockFrame, 10>,
        expected_rx: Vec<MockFrame, 10>,
        tx_index: usize,
        rx_index: usize,
    }

    impl MockCan {
        /// New MockCan with expected tx and rx frames
        pub fn new(expected_tx: Vec<MockFrame, 10>, expected_rx: Vec<MockFrame, 10>) -> Self {
            Self {
                expected_tx,
                expected_rx,
                tx_index: 0,
                rx_index: 0,
            }
        }
    }

    impl Can for MockCan {
        type Frame = MockFrame;
        type Error = Infallible;

        fn transmit(&mut self, frame: &Self::Frame) -> Result<(), Self::Error> {
            assert_eq!(*frame, self.expected_tx[self.tx_index]);
            self.tx_index += 1;
            Ok(())
        }

        fn receive(&mut self) -> Result<Self::Frame, Self::Error> {
            let frame = self.expected_rx[self.rx_index].clone();
            self.rx_index += 1;
            Ok(frame)
        }
    }

    #[test]
    fn test_default() {
        let txfr = TransferDesc::default();
        assert_eq!(0, txfr.transfer_id);
        assert_eq!(false, txfr.next_toggle_bit);
        assert_eq!(0, txfr.ts);
        assert_eq!(0, txfr.iface_index);
        assert_eq!(0, txfr.payload.len());
    }

    #[test]
    fn test_compute_forward_distance() {
        assert_eq!(0, TransferDesc::compute_forward_distance(0, 0));
        assert_eq!(1, TransferDesc::compute_forward_distance(0, 1));
        assert_eq!(7, TransferDesc::compute_forward_distance(0, 7));
        assert_eq!(0, TransferDesc::compute_forward_distance(7, 7));
        assert_eq!(31, TransferDesc::compute_forward_distance(31, 30));
        assert_eq!(1, TransferDesc::compute_forward_distance(31, 0));
    }

    #[test]
    fn test_broadcast1() {
        let expected_tx = Vec::from_slice(&[
            MockFrame::new(
                ExtendedId::new(0x6015501).unwrap(),
                &[0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0xc0],
            ),
            MockFrame::new(
                ExtendedId::new(0x6015501).unwrap(),
                &[0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0xc1],
            ),
        ])
        .unwrap();
        let rx_exp = Vec::new();
        let intf = MockCan::new(expected_tx, rx_exp);
        let mut uav = Uavcan::new(2000, 300);
        uav.add_interface(intf);

        // now send 2 frames
        let frame_type = FrameType::Message;
        let msg_type = MsgType::NodeStatus;
        let source_node_id = 1;
        let payload = [0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7];
        uav.broadcast(frame_type, msg_type, source_node_id, &payload)
            .unwrap();
        uav.broadcast(frame_type, msg_type, source_node_id, &payload)
            .unwrap();
        uav.can_send().unwrap();
    }
}
