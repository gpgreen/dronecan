//! This module contains code related to sending and receiving DroneCAN frames over CAN.

use crate::{
    crc::TransferCrc,
    messages::MsgType,
    protocol::{CanId, FrameType, RequestResponse, TailByte, TransferComponent},
    CanError,
};
use bitvec::prelude::*;
use embedded_can;
use heapless::{Deque, FnvIndexMap, Vec};
use log::*;

pub(crate) const DATA_FRAME_MAX_LEN_LEGACY: usize = 8;
pub(crate) const MAX_RX_FRAMES_LEGACY: usize = 24;
pub(crate) const RX_TX_BUFFER_MAX_SIZE: usize = MAX_RX_FRAMES_LEGACY * DATA_FRAME_MAX_LEN_LEGACY;

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
    payload: Vec<u8, RX_TX_BUFFER_MAX_SIZE>,
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

    /// Get the transfer_id
    pub fn transfer_id(&mut self) -> u8 {
        self.transfer_id
    }

    /// Increment the transfer_id
    ///
    /// transfer_id's are 5 bits in length and roll over when
    pub fn transfer_id_incr(&mut self) -> u8 {
        let id = self.transfer_id;
        self.transfer_id += 1;
        self.transfer_id &= 0b1_1111;
        id
    }
}

/// The `RxPayload` contains state for a completed transfer
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug, Clone, PartialEq)]
pub struct RxPayload {
    /// Transfer number for this payload
    pub transfer_id: u8,
    /// timestamp at beginning of payload
    pub ts: u64,
    /// If a multi-frame payload, this is the crc
    pub payload_crc: Option<u16>,
    /// the payload in bytes
    pub payload: &'static Vec<u8, RX_TX_BUFFER_MAX_SIZE>,
}

/// Broadcast struct for Uavcan on CAN interface
pub struct UavcanInterface<CAN, FRAME> {
    pub iface: Vec<CAN, 2>,
    tx_queue: Deque<FRAME, 10>,
    txf_map: FnvIndexMap<TransferDescKey, TransferDesc, 16>, // size must be power of 2
    initialized: bool,
    transfer_timeout: u64,
    interface_switch_delay: u64,
}

impl<CAN, FRAME, FE> UavcanInterface<CAN, FRAME>
where
    CAN: embedded_can::blocking::Can<Frame = FRAME, Error = FE>,
    FRAME: embedded_can::Frame,
{
    /// Create new `UavcanInterface`
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
            "broadcast frame:{:?} msg_type:{:?} source node:{} payload len:{}",
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
        let txf_key = TransferDescKey::new(frame_type, msg_type.id(), source_node_id);
        trace!("txf_key:{:?}", txf_key);
        if !self.txf_map.contains_key(&txf_key) {
            self.txf_map
                .insert(txf_key, TransferDesc::default())
                .map_err(|_| CanError::TransferMapFull)?;
        }
        let txf_desc = match self.txf_map.get_mut(&txf_key) {
            Some(t) => Ok(t),
            None => panic!(),
        }?;
        // TODO: update descriptor with timestamp
        trace!("descriptor start state:{:?}", txf_desc);

        // if this is a service reply, use the existing transfer id, otherwise, increment and
        // use that.
        let transfer_id = match frame_type {
            FrameType::Service(s) => {
                if s.req_or_resp == RequestResponse::Response {
                    txf_desc.transfer_id()
                } else {
                    txf_desc.transfer_id_incr()
                }
            }
            _ => txf_desc.transfer_id_incr(),
        };
        trace!("descriptor updated state:{:?}", txf_desc);

        let can_id = CanId {
            priority: msg_type.priority(),
            type_id: msg_type.id(),
            source_node_id,
            frame_type,
        };
        trace!("can id: {:?}", can_id);

        // The transfer payload is up to 7 bytes for non-FD DRONECAN.
        // If data is longer than a single frame, set up a multi-frame transfer.
        if payload.len() >= DATA_FRAME_MAX_LEN_LEGACY {
            trace!("multiple frames payload");
            self.queue_multiple_frames(
                txf_key,
                payload,
                can_id.value(),
                transfer_id,
                msg_type.base_crc(),
            )
        } else {
            trace!("single frame payload");
            // copy the data
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

            let id =
                embedded_can::Id::Extended(embedded_can::ExtendedId::new(can_id.value()).unwrap());
            trace!("frame can id: {:?}", id);
            self.tx_queue
                .push_back(FRAME::new(id, txf_desc.payload.as_slice()).unwrap())
                .map_err(|_| CanError::TransmitQueueFull)?;
            // reset the tx buffer
            txf_desc.payload.clear();
            Ok(())
        }
    }

    /// Handles splitting a payload into multiple frames
    /// when it is larger than a single frame
    fn queue_multiple_frames(
        &mut self,
        txf_key: TransferDescKey,
        payload: &[u8],
        can_id: u32,
        transfer_id: u8,
        base_crc: u16,
    ) -> Result<(), CanError> {
        debug!("queue multiple frames");
        let id = embedded_can::Id::Extended(embedded_can::ExtendedId::new(can_id).unwrap());
        trace!("canid:{:?}", id);

        let txf_desc = match self.txf_map.get_mut(&txf_key) {
            Some(t) => Ok(t),
            None => panic!(),
        }?;

        let mut crc = TransferCrc::new(base_crc);
        crc.add_payload(payload, payload.len());
        trace!("payload crc:{:?}", crc);
        // See https://dronecan.github.io/Specification/4._CAN_bus_transport_layer/,
        // "Multi-frame transfer" re requirements such as CRC and Toggle bit.
        let mut component = TransferComponent::MultiStart;

        // Populate the first frame. This is different from the others due to the CRC.
        let mut payload_len_this_frame = DATA_FRAME_MAX_LEN_LEGACY - 3; // -3 for CRC and tail byte.

        txf_desc
            .payload
            .extend_from_slice(&crc.value.to_le_bytes())
            .map_err(|_| CanError::TxPayloadBufferFull)?;
        txf_desc
            .payload
            .extend_from_slice(&payload[..payload_len_this_frame])
            .map_err(|_| CanError::TxPayloadBufferFull)?;
        let tail_byte = TailByte::new(component, transfer_id);
        trace!("tail_byte:{:?}", tail_byte);
        txf_desc
            .payload
            .push(tail_byte.value())
            .map_err(|_| CanError::TxPayloadBufferFull)?;

        let f = FRAME::new(id, &txf_desc.payload).unwrap();
        trace!("frame queued");
        self.tx_queue
            .push_back(f)
            .map_err(|_| CanError::TransmitQueueFull)?;

        let mut latest_i_sent = payload_len_this_frame - 1;

        // Populate subsequent frames.
        while latest_i_sent < payload.len() - 1 {
            trace!("latest_i_sent:{}", latest_i_sent);
            txf_desc.payload.clear();
            let payload_i = latest_i_sent + 1;
            if payload_i + DATA_FRAME_MAX_LEN_LEGACY <= payload.len() {
                // Not the last frame.
                payload_len_this_frame = DATA_FRAME_MAX_LEN_LEGACY - 1;
                component = TransferComponent::MultiMid(tail_byte.toggle);
            } else {
                // Last frame.
                payload_len_this_frame = payload.len() - payload_i;
                component = TransferComponent::MultiEnd(tail_byte.toggle);
            }

            txf_desc
                .payload
                .extend_from_slice(&payload[payload_i..payload_i + payload_len_this_frame])
                .map_err(|_| CanError::TxPayloadBufferFull)?;

            let tail_byte = TailByte::new(component, transfer_id);
            trace!("tail_byte:{:?}", tail_byte);
            txf_desc
                .payload
                .push(tail_byte.value())
                .map_err(|_| CanError::TxPayloadBufferFull)?;

            let f = FRAME::new(id, &txf_desc.payload).unwrap();
            trace!("next frame queued");
            self.tx_queue
                .push_back(f)
                .map_err(|_| CanError::TransmitQueueFull)?;

            latest_i_sent += payload_len_this_frame; // 8 byte frame size, - 1 for each frame's tail byte.
        }
        // clear buffer
        txf_desc.payload.clear();

        Ok(())
    }

    /// Handle a new received frame. This returns the TransferDesc with the assembled payload
    /// if this is the last frame in a multi-frame transfer, or if it's a single-frame transfer.
    /// Returns None if frame is ignored for state reasons, or in the middle of a multi-frame payload
    ///
    /// SAFETY: unsafe due to use of static buffer reference in `RxPayload`. If the static buffer is not
    /// read and copied before this method is called again, the contents of this receive transfer
    /// will be lost.
    ///
    /// see algorithm at [The DroneCAN spec, transport layer page](https://dronecan.github.io/Specification/4._CAN_bus_transport_layer/)
    pub unsafe fn handle_frame_rx(
        &mut self,
        iface_index: usize,
        can_id: &CanId,
        frame: &FRAME,
        frame_ts: u64,
        //        f: impl FnOnce() -> RxPayload,
    ) -> Result<Option<RxPayload>, CanError> {
        // static buffer to use in RxPayload
        static mut PAYLOAD_BUFFER: Vec<u8, RX_TX_BUFFER_MAX_SIZE> = Vec::new();

        // get the tail byte
        let tail_byte = TailByte::from_value(frame.data()[frame.dlc() - 1]);
        trace!("tail_byte: {:?}", tail_byte);

        // get the transfer descriptor
        // first construct the key from the info provided, then
        // we can get the descriptor from the hashmap
        let txf_key =
            TransferDescKey::new(can_id.frame_type, can_id.type_id, can_id.source_node_id);
        trace!("txf_key: {:?}", txf_key);
        if !self.txf_map.contains_key(&txf_key) {
            self.txf_map
                .insert(txf_key, TransferDesc::default())
                .map_err(|_| CanError::TransferMapFull)?;
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
        trace!("initialized:{} tid_timed_out:{} iface_switch_allowed:{} same_iface:{} non_wrapped_tid:{} not_previous_tid:{}",
               self.initialized, tid_timed_out, iface_switch_allowed, same_iface, non_wrapped_tid, not_previous_tid);

        // check the state flags, deciding whether to restart
        if !self.initialized
            || tid_timed_out
            || (same_iface && tail_byte.start_of_transfer && not_previous_tid)
            || (iface_switch_allowed && tail_byte.start_of_transfer && non_wrapped_tid)
        {
            debug!("needs a restart from txfid:{}", txf_desc.transfer_id);
            // needs a restart
            self.initialized = true;
            txf_desc.iface_index = iface_index;
            txf_desc.transfer_id = tail_byte.transfer_id;
            txf_desc.payload.clear();
            txf_desc.next_toggle_bit = false;
            // ignore this frame, if start of transfer was missed
            if !tail_byte.start_of_transfer {
                txf_desc.transfer_id_incr();
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
            txf_desc.iface_index = iface_index;
        }

        // update the toggle bit
        txf_desc.next_toggle_bit = !txf_desc.next_toggle_bit;
        // copy the frame data to the buffer, minus the tail byte
        txf_desc
            .payload
            .extend_from_slice(&frame.data()[0..frame.dlc() - 1])
            .map_err(|_| CanError::RxPayloadBufferFull)?;
        trace!("descriptor final state: {:?}", txf_desc);

        if tail_byte.end_of_transfer {
            // do something with the payload
            PAYLOAD_BUFFER.clear();
            let rx_completed = if tail_byte.start_of_transfer {
                PAYLOAD_BUFFER
                    .extend_from_slice(txf_desc.payload.as_slice())
                    .unwrap();
                // single frame payload
                RxPayload {
                    transfer_id: txf_desc.transfer_id,
                    ts: txf_desc.ts,
                    payload_crc: None,
                    payload: &PAYLOAD_BUFFER,
                }
            } else {
                // multi-frame payload
                let bits = txf_desc.payload.as_slice().view_bits::<Msb0>();
                let payload_crc = Some(bits[0..16].load_le());
                // copy the payload, minus the CRC bytes
                PAYLOAD_BUFFER
                    .extend_from_slice(&txf_desc.payload.as_slice()[2..])
                    .unwrap();
                RxPayload {
                    transfer_id: txf_desc.transfer_id,
                    ts: txf_desc.ts,
                    payload_crc,
                    payload: &PAYLOAD_BUFFER,
                }
            };

            // setup for the next one
            txf_desc.transfer_id_incr();
            txf_desc.next_toggle_bit = false;
            txf_desc.payload.clear();
            return Ok(Some(rx_completed));
        }
        Ok(None)
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
    fn test_txfr_desc_default() {
        let txfr = TransferDesc::default();
        assert_eq!(0, txfr.transfer_id);
        assert_eq!(false, txfr.next_toggle_bit);
        assert_eq!(0, txfr.ts);
        assert_eq!(0, txfr.iface_index);
        assert_eq!(0, txfr.payload.len());
    }

    #[test]
    fn test_txfr_desc_compute_forward_distance() {
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
        let expected_rx = Vec::new();
        let intf = MockCan::new(expected_tx, expected_rx);
        let mut uav = UavcanInterface::new(2000, 300);
        uav.add_interface(intf);

        // now send 2 frames each a message of 7 bytes
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

    #[test]
    fn test_broadcast2() {
        let expected_tx = Vec::from_slice(&[
            MockFrame::new(
                ExtendedId::new(0x6015501).unwrap(),
                &[0x9e, 0x33, 0x01, 0x02, 0x03, 0x04, 0x05, 0x80],
            ),
            MockFrame::new(
                ExtendedId::new(0x6015501).unwrap(),
                &[0x06, 0x07, 0x8, 0x60],
            ),
            MockFrame::new(
                ExtendedId::new(0x6015501).unwrap(),
                &[0x9e, 0x33, 0x01, 0x02, 0x03, 0x04, 0x05, 0x81],
            ),
            MockFrame::new(
                ExtendedId::new(0x6015501).unwrap(),
                &[0x06, 0x07, 0x8, 0x61],
            ),
        ])
        .unwrap();
        let expected_rx = Vec::new();
        let intf = MockCan::new(expected_tx, expected_rx);
        let mut uav = UavcanInterface::new(2000, 300);
        uav.add_interface(intf);

        // now send 4 frames, consisting of 2 messages of 8 bytes
        let frame_type = FrameType::Message;
        let msg_type = MsgType::NodeStatus;
        let source_node_id = 1;
        let payload = [0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8];
        uav.broadcast(frame_type, msg_type, source_node_id, &payload)
            .unwrap();
        uav.broadcast(frame_type, msg_type, source_node_id, &payload)
            .unwrap();
        uav.can_send().unwrap();
    }

    #[test]
    fn test_rx1() {
        let expected_rx = Vec::from_slice(&[
            MockFrame::new(
                ExtendedId::new(0x6015501).unwrap(),
                &[0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0xc0],
            ),
            MockFrame::new(
                ExtendedId::new(0x6015501).unwrap(),
                &[0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0xc1],
            ),
            MockFrame::new(
                ExtendedId::new(0x6015501).unwrap(),
                &[0x9e, 0x33, 0x01, 0x02, 0x03, 0x04, 0x05, 0x82],
            ),
            MockFrame::new(
                ExtendedId::new(0x6015501).unwrap(),
                &[0x06, 0x07, 0x8, 0x62],
            ),
            MockFrame::new(
                ExtendedId::new(0x6015501).unwrap(),
                &[0x9e, 0x33, 0x01, 0x02, 0x03, 0x04, 0x05, 0x83],
            ),
            MockFrame::new(
                ExtendedId::new(0x6015501).unwrap(),
                &[0x06, 0x07, 0x8, 0x63],
            ),
        ])
        .unwrap();
        let expected_tx = Vec::new();
        let intf = MockCan::new(expected_tx, expected_rx);
        let mut uav = UavcanInterface::new(2000, 300);
        uav.add_interface(intf);

        // now get 4 frames, first is (2) 7 byte messages, followed by (2) messages of 8 bytes
        let frame = uav.iface[0].receive().unwrap();
        let can_id = match frame.id() {
            embedded_can::Id::Extended(eid) => Some(CanId::from_value(eid.as_raw())),
            _ => None,
        }
        .unwrap();
        let rx_res = unsafe { uav.handle_frame_rx(0, &can_id, &frame, 10) };
        if let Ok(Some(rx)) = rx_res {
            assert_eq!(rx.transfer_id, 0);
            assert_eq!(rx.ts, 10);
            assert!(rx.payload_crc.is_none());
            assert_eq!(rx.payload, &[0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07]);
        } else {
            assert!(false);
        }
        let frame = uav.iface[0].receive().unwrap();
        let can_id = match frame.id() {
            embedded_can::Id::Extended(eid) => Some(CanId::from_value(eid.as_raw())),
            _ => None,
        }
        .unwrap();
        let rx_res = unsafe { uav.handle_frame_rx(0, &can_id, &frame, 20) };
        if let Ok(Some(rx)) = rx_res {
            assert_eq!(rx.transfer_id, 1);
            assert_eq!(rx.ts, 20);
            assert!(rx.payload_crc.is_none());
            assert_eq!(rx.payload, &[0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07]);
        } else {
            assert!(false);
        }

        // the multi frame messages now
        let frame = uav.iface[0].receive().unwrap();
        let can_id = match frame.id() {
            embedded_can::Id::Extended(eid) => Some(CanId::from_value(eid.as_raw())),
            _ => None,
        }
        .unwrap();
        let rx_res = unsafe { uav.handle_frame_rx(0, &can_id, &frame, 40) };
        if let Ok(Some(_rx)) = rx_res {
            assert!(false);
        };
        let frame = uav.iface[0].receive().unwrap();
        let can_id = match frame.id() {
            embedded_can::Id::Extended(eid) => Some(CanId::from_value(eid.as_raw())),
            _ => None,
        }
        .unwrap();
        let rx_res = unsafe { uav.handle_frame_rx(0, &can_id, &frame, 45) };
        if let Ok(Some(rx)) = rx_res {
            assert_eq!(rx.transfer_id, 2);
            assert_eq!(rx.ts, 40);
            assert_eq!(13214, rx.payload_crc.unwrap());
            assert_eq!(
                rx.payload,
                &[0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08]
            );
        } else {
            assert!(false);
        }

        let frame = uav.iface[0].receive().unwrap();
        let can_id = match frame.id() {
            embedded_can::Id::Extended(eid) => Some(CanId::from_value(eid.as_raw())),
            _ => None,
        }
        .unwrap();
        let rx_res = unsafe { uav.handle_frame_rx(0, &can_id, &frame, 50) };
        if let Ok(Some(_rx)) = rx_res {
            assert!(false);
        };
        let frame = uav.iface[0].receive().unwrap();
        let can_id = match frame.id() {
            embedded_can::Id::Extended(eid) => Some(CanId::from_value(eid.as_raw())),
            _ => None,
        }
        .unwrap();
        let rx_res = unsafe { uav.handle_frame_rx(0, &can_id, &frame, 55) };
        if let Ok(Some(rx)) = rx_res {
            assert_eq!(rx.transfer_id, 3);
            assert_eq!(rx.ts, 50);
            assert_eq!(13214, rx.payload_crc.unwrap());
            assert_eq!(
                rx.payload,
                &[0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08]
            );
        } else {
            assert!(false);
        }
    }

    // #[derive(Debug)]
    // struct FooBar {
    //     foo: i32,
    //     bar: i32,
    // }
    // impl FooBar {
    //     fn woot(&mut self, x: i32) {
    //         self.foo += x;
    //         self.bar -= x
    //     }
    // }

    // fn call(foobar: &mut FooBar, functor: fn(&mut FooBar, x: i32), v: i32) {
    //     functor(foobar, v)
    // }

    // fn main() {
    //     let mut foobar = FooBar { foo: 123, bar: 123 };
    //     call(&mut foobar, FooBar::woot, 234);
    //     println!("{:?}", foobar);
    // }
}
