use crate::{
    CanError, DATA_FRAME_MAX_LEN_LEGACY, RxPayload,
    crc::TransferCrc,
    interface::{TransferDesc, TransferDescKey, UavcanInterface},
    messages::MsgType,
    protocol::{CanId, FrameType, RequestResponse, TailByte, TransferComponent},
};
use bitvec::prelude::*;
#[cfg(feature = "defmt")]
use defmt::*;
use embedded_can;
use heapless::{Vec, index_map::FnvIndexMap};

/// Type to represent a Uavcan message
/// for broadcasting. A return value in `UavcanHandler`
/// which is used in Uavcan::broadcast to convert to
/// CAN frames
#[derive(Debug, Clone)]
pub struct BroadcastMessage<const BROADCAST_PAYLOAD_SIZE_MAX: usize = 72> {
    pub frame_type: FrameType,
    pub msg_type: MsgType,
    pub payload: Vec<u8, BROADCAST_PAYLOAD_SIZE_MAX>,
}

impl<const BROADCAST_PAYLOAD_SIZE_MAX: usize> BroadcastMessage<BROADCAST_PAYLOAD_SIZE_MAX> {
    pub fn new(frame_type: FrameType, msg_type: MsgType) -> Self {
        BroadcastMessage {
            frame_type,
            msg_type,
            payload: Vec::new(),
        }
    }
}

/// Trait for application specific frame handler
/// This allows different handlers for different Uavcan frame types
/// The main categories of frame types, are Services, Messages, and Anonymous Messages
pub trait UavcanHandler<CAN, FRAME, E, const BROADCAST_PAYLOAD_SIZE_MAX: usize>
where
    CAN: embedded_can::nb::Can<Frame = FRAME, Error = E>,
    FRAME: embedded_can::Frame,
    E: embedded_can::Error,
{
    /// Handle a Uavcan message
    fn handle_message(
        &self,
        node_id: u8,
        payload: &RxPayload,
        can_id: &CanId,
        interface: &mut UavcanInterface<CAN>,
    ) -> Result<Option<Vec<BroadcastMessage<BROADCAST_PAYLOAD_SIZE_MAX>, 10>>, CanError<E>>;
}

/// Uavcan This type allows handling of received CAN frames for the
/// dronecode CAN stack. This type will also assemble CAN frames to send
/// using broadcast. To send or receive CAN frames, use `UavcanInterface`
///
/// TXQUEUELEN is the maximum size of the transmit queue for CAN
/// frames. This should be sized for the number of frames that the
/// largest transmitted Uavcan message contains
/// TXFMAPLEN is the size of the `FnvIndexMap` for `TransferDesc`
/// for this application. (power of 2 size)
pub struct Uavcan<const TXQUEUELEN: usize = 10, const TXFMAPLEN: usize = 16> {
    node_id: u8,
    txf_map: FnvIndexMap<TransferDescKey, TransferDesc, TXFMAPLEN>,
    initialized: bool,
    transfer_timeout: u64,
    interface_switch_delay: u64,
}

impl<const TXQUEUELEN: usize, const TXFMAPLEN: usize> Uavcan<TXQUEUELEN, TXFMAPLEN> {
    /// Create new `Uavcan`
    pub fn new(node_id: u8, transfer_timeout: u64, interface_switch_delay: u64) -> Self {
        Self {
            node_id,
            txf_map: FnvIndexMap::new(),
            initialized: false,
            transfer_timeout,
            interface_switch_delay,
        }
    }

    #[inline]
    fn ensure_node_id(node_id: u8) -> bool {
        (1..=127).contains(&node_id)
    }

    /// Get the node Id
    pub fn node_id(&self) -> u8 {
        self.node_id
    }

    /// function to set the node id
    /// The node_id can be set to '0' in Uavcan::new as it isn't checked. This is required
    /// if dynamic node allocation is being used. We have no code for this right now
    /// This function will not allow a node id of zero.
    pub fn set_node_id(&mut self, node_id: u8) {
        if Self::ensure_node_id(node_id) {
            self.node_id = node_id;
        } else {
            #[cfg(feature = "defmt")]
            error!(
                "Trying to set Node Id to value:{} outside of range 1 <= node id <= 127",
                node_id
            );
        }
    }

    /// Handle a CAN frame received on the interface `iface`
    ///
    /// If the frame is a or completes a uavcan message, this is passed to the
    /// handler.  If the frame is not a data frame, or doesn't have an
    /// extended id, it is ignored. If the frame is a service request
    /// or response for different node id, it is ignored.
    pub fn handle_rx_frame<CAN, FRAME, E, FH, const BROADCAST_PAYLOAD_SIZE_MAX: usize>(
        &mut self,
        iface: &mut UavcanInterface<CAN>,
        frame: &FRAME,
        ts: &u64,
        handler: Option<&FH>,
    ) -> Result<Option<Vec<BroadcastMessage<BROADCAST_PAYLOAD_SIZE_MAX>, 10>>, CanError<E>>
    where
        CAN: embedded_can::nb::Can<Frame = FRAME, Error = E>,
        FRAME: embedded_can::Frame,
        E: embedded_can::Error,
        FH: UavcanHandler<CAN, FRAME, E, BROADCAST_PAYLOAD_SIZE_MAX>,
    {
        // get the CanId, ignore non-extended CAN id's
        let can_id = match frame.id() {
            embedded_can::Id::Extended(eid) => CanId::from_value(eid.as_raw()),
            _ => return Ok(None),
        };
        #[cfg(feature = "defmt")]
        debug!(
            "==> rx_frame {:x?} dlc:{} data:{:x?}",
            can_id,
            frame.dlc(),
            frame.data()
        );
        if frame.is_data_frame() {
            // if this frame is Service Response or a Request with different node id, skip
            if let FrameType::Service(service_data) = can_id.frame_type {
                if service_data.dest_node_id != self.node_id {
                    #[cfg(feature = "defmt")]
                    debug!("service request/response is not for this node");
                    return Ok(None);
                }
            }
            // otherwise, process the data frame
            if let Some(rx_payload) = self._handle_rx_frame(iface, &can_id, frame, ts)? {
                // got a completed transfer, pass to handler
                if let Some(handler) = handler {
                    let msgs = handler.handle_message(self.node_id, &rx_payload, &can_id, iface)?;
                    return Ok(msgs);
                }
            }
        }
        #[cfg(feature = "defmt")]
        debug!("<== rx_frame {:x?}", can_id.type_id);
        Ok(None)
    }

    /// Queue a DroneCAN "broadcast" message.
    ///
    /// See [The DroneCAN spec, transport layer page](https://dronecan.github.io/Specification/4._CAN_bus_transport_layer/)
    /// When done, a series of CAN frames will be in the transmit queue, ready to be sent via the CAN controller
    /// Should be broadcast at interval between 2 and 1000ms.
    pub fn broadcast<
        FRAME: embedded_can::Frame,
        E: embedded_can::Error,
        const BROADCAST_PAYLOAD_SIZE_MAX: usize,
    >(
        &mut self,
        msg: BroadcastMessage<BROADCAST_PAYLOAD_SIZE_MAX>,
    ) -> Result<Vec<FRAME, 10>, CanError<E>> {
        #[cfg(feature = "defmt")]
        debug!(
            "broadcast frame:{:?} msg_type:{:?} source node:{} payload len:{}",
            frame_type,
            msg_type,
            self.node_id,
            payload.len(),
        );

        // get the transfer descriptor
        let txf_key = TransferDescKey::new(msg.frame_type, msg.msg_type.id(), self.node_id);
        #[cfg(feature = "defmt")]
        trace!("txf_key:{:?}", txf_key);
        if !self.txf_map.contains_key(&txf_key) {
            self.txf_map
                .insert(txf_key, TransferDesc::default())
                .map_err(|_| CanError::TransferMapFull)?;
        }
        let txf_desc = match self.txf_map.get_mut(&txf_key) {
            Some(t) => Ok::<&mut TransferDesc, E>(t),
            None => panic!(),
        }?;
        // TODO: update descriptor with timestamp
        #[cfg(feature = "defmt")]
        trace!("descriptor start state:{:?}", txf_desc);

        // if this is a service reply, use the existing transfer id, otherwise, increment and
        // use that.
        let transfer_id = match msg.frame_type {
            FrameType::Service(s) if s.req_or_resp == RequestResponse::Response => {
                txf_desc.transfer_id()
            }
            _ => txf_desc.transfer_id_incr(),
        };
        #[cfg(feature = "defmt")]
        trace!("descriptor updated state:{:?}", txf_desc);

        let can_id = CanId {
            priority: msg.msg_type.priority(),
            type_id: msg.msg_type.id(),
            source_node_id: self.node_id,
            frame_type: msg.frame_type,
        };
        // unwrap always works because our can_id's can't be invalid
        let frame_id =
            embedded_can::Id::Extended(embedded_can::ExtendedId::new(can_id.value()).unwrap());
        #[cfg(feature = "defmt")]
        trace!("can id: {:?} frame id {:?}", can_id, frame_id);

        // The transfer payload is up to 7 bytes for non-FD DRONECAN.
        // If data is longer than a single frame, set up a multi-frame transfer.
        if msg.payload.len() >= DATA_FRAME_MAX_LEN_LEGACY {
            #[cfg(feature = "defmt")]
            trace!("multiple frames payload");
            self.queue_multiple_frames::<FRAME, E>(
                txf_key,
                msg.payload.as_slice(),
                frame_id,
                transfer_id,
                msg.msg_type.base_crc(),
            )
        } else {
            #[cfg(feature = "defmt")]
            trace!("single frame payload");
            // copy the data
            txf_desc
                .payload
                .extend_from_slice(msg.payload.as_slice())
                .map_err(|_| CanError::TxPayloadBufferFull)?;
            let tail_byte = TailByte::new(TransferComponent::SingleFrame, transfer_id);
            #[cfg(feature = "defmt")]
            trace!("tail_byte:{:?}", tail_byte);
            txf_desc
                .payload
                .push(tail_byte.value())
                .map_err(|_| CanError::TxPayloadBufferFull)?;

            #[cfg(feature = "defmt")]
            trace!("frame can id: {:?}", id);
            // create the return vec
            let mut tx_vec = Vec::new();

            if let Some(frame) = FRAME::new(frame_id, txf_desc.payload.as_slice()) {
                tx_vec
                    .push(frame)
                    .map_err(|_| CanError::TransmitQueueFull)?;
            }
            // reset the tx buffer
            txf_desc.payload.clear();
            Ok(tx_vec)
        }
    }

    /// Handles splitting a payload into multiple frames
    /// when it is larger than a single frame
    fn queue_multiple_frames<FRAME: embedded_can::Frame, E: embedded_can::Error>(
        &mut self,
        txf_key: TransferDescKey,
        payload: &[u8],
        frame_id: embedded_can::Id,
        transfer_id: u8,
        base_crc: u16,
    ) -> Result<Vec<FRAME, 10>, CanError<E>> {
        #[cfg(feature = "defmt")]
        debug!("queue multiple frames");
        #[cfg(feature = "defmt")]
        trace!("canid:{:?}", frame_id);

        let txf_desc = match self.txf_map.get_mut(&txf_key) {
            Some(t) => Ok::<&mut TransferDesc, E>(t),
            None => panic!(),
        }?;

        let mut crc = TransferCrc::new(base_crc);
        crc.add_payload(payload);
        #[cfg(feature = "defmt")]
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
        #[cfg(feature = "defmt")]
        trace!("tail_byte:{:?}", tail_byte);
        txf_desc
            .payload
            .push(tail_byte.value())
            .map_err(|_| CanError::TxPayloadBufferFull)?;
        #[cfg(feature = "defmt")]
        trace!("frame queued");

        // create the return vector
        let mut tx_vec = Vec::new();

        if let Some(f) = FRAME::new(frame_id, &txf_desc.payload) {
            tx_vec.push(f).map_err(|_| CanError::TransmitQueueFull)?;
        }
        let mut latest_i_sent = payload_len_this_frame - 1;

        // Populate subsequent frames.
        while latest_i_sent < payload.len() - 1 {
            #[cfg(feature = "defmt")]
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
            #[cfg(feature = "defmt")]
            trace!("tail_byte:{:?}", tail_byte);
            txf_desc
                .payload
                .push(tail_byte.value())
                .map_err(|_| CanError::TxPayloadBufferFull)?;

            if let Some(f) = FRAME::new(frame_id, &txf_desc.payload) {
                tx_vec.push(f).map_err(|_| CanError::TransmitQueueFull)?;
            }

            #[cfg(feature = "defmt")]
            trace!("next frame queued");
            latest_i_sent += payload_len_this_frame; // 8 byte frame size, - 1 for each frame's tail byte.
        }
        // reset the tx buffer
        txf_desc.payload.clear();
        Ok(tx_vec)
    }

    /// Handle a new received frame.
    ///
    /// This returns a `RxPayload` containing data relevant to the
    /// assembled payload if this is the last frame in a multi-frame
    /// transfer, or if it's a single-frame transfer.
    ///
    /// Returns
    /// None if frame is ignored for state reasons, or in the middle of a
    /// multi-frame payload.
    /// error if the CRC doesn't match after the last frame is received.
    /// error if the TransferMap is full, no more slots for TransferDescriptors
    /// error if the payload buffer is full
    /// error if there is no MsgType for this frame
    ///
    /// Panics
    ///
    /// see algorithm at [The DroneCAN spec, transport layer page](https://dronecan.github.io/Specification/4._CAN_bus_transport_layer/)
    fn _handle_rx_frame<CAN, FRAME, E>(
        &mut self,
        iface: &mut UavcanInterface<CAN>,
        can_id: &CanId,
        frame: &FRAME,
        frame_ts: &u64,
    ) -> Result<Option<RxPayload>, CanError<E>>
    where
        CAN: embedded_can::nb::Can<Frame = FRAME, Error = E>,
        FRAME: embedded_can::Frame,
        E: embedded_can::Error,
    {
        // get the tail byte
        let tail_byte = TailByte::from_value(frame.data()[frame.dlc() - 1]);
        #[cfg(feature = "defmt")]
        trace!("tail_byte: {:?}", tail_byte);

        // get the transfer descriptor
        // first construct the key from the info provided, then
        // we can get the descriptor from the hashmap
        let txf_key =
            TransferDescKey::new(can_id.frame_type, can_id.type_id, can_id.source_node_id);
        #[cfg(feature = "defmt")]
        trace!("txf_key: {:?}", txf_key);
        if !self.txf_map.contains_key(&txf_key) {
            self.txf_map
                .insert(txf_key, TransferDesc::default())
                .map_err(|_| CanError::TransferMapFull)?;
        }
        let txf_desc = match self.txf_map.get_mut(&txf_key) {
            Some(t) => Ok::<&mut TransferDesc, E>(t),
            None => panic!(),
        }?;
        #[cfg(feature = "defmt")]
        trace!("descriptor start state: {:?}", txf_desc);

        // update state flags for restart check
        let tid_timed_out = (frame_ts - txf_desc.ts()) > self.transfer_timeout;
        let iface_switch_allowed = (frame_ts - txf_desc.ts()) > self.interface_switch_delay;
        let same_iface = iface.index() == txf_desc.index();
        let non_wrapped_tid =
            TransferDesc::compute_forward_distance(txf_desc.transfer_id(), tail_byte.transfer_id)
                < TransferDesc::TRANSFER_ID_HALF_RANGE;
        let not_previous_tid =
            TransferDesc::compute_forward_distance(tail_byte.transfer_id, txf_desc.transfer_id())
                > 1;
        #[cfg(feature = "defmt")]
        trace!(
            "initialized:{} tid_timed_out:{} iface_switch_allowed:{} same_iface:{} non_wrapped_tid:{} not_previous_tid:{}",
            self.initialized,
            tid_timed_out,
            iface_switch_allowed,
            same_iface,
            non_wrapped_tid,
            not_previous_tid
        );

        // check the state flags, deciding whether to restart
        if !self.initialized
            || tid_timed_out
            || (same_iface && tail_byte.start_of_transfer && not_previous_tid)
            || (iface_switch_allowed && tail_byte.start_of_transfer && non_wrapped_tid)
        {
            #[cfg(feature = "defmt")]
            debug!("needs a restart from txfid:{}", txf_desc.transfer_id());
            // needs a restart
            self.initialized = true;
            txf_desc.restart(iface.index(), tail_byte.transfer_id);
            // ignore this frame, if start of transfer was missed
            if !tail_byte.start_of_transfer {
                txf_desc.transfer_id_incr();
                return Ok(None);
            }
        }

        if iface.index() != txf_desc.index() {
            #[cfg(feature = "defmt")]
            trace!("wrong interface");
            // wrong interface, ignore
            return Ok(None);
        }
        if tail_byte.toggle != txf_desc.next_toggle() {
            #[cfg(feature = "defmt")]
            trace!("unexpected toggle bit");
            // unexpected toggle bit, ignore
            return Ok(None);
        }
        if tail_byte.transfer_id != txf_desc.transfer_id() {
            #[cfg(feature = "defmt")]
            trace!("unexpected transfer id");
            // unexpected transfer ID, ignore
            return Ok(None);
        }
        if tail_byte.start_of_transfer {
            txf_desc.start(frame_ts, iface.index());
        }

        // update the toggle bit
        txf_desc.toggle();
        // copy the frame data to the buffer, minus the tail byte
        txf_desc
            .payload
            .extend_from_slice(&frame.data()[0..frame.dlc() - 1])
            .map_err(|_| CanError::RxPayloadBufferFull)?;
        #[cfg(feature = "defmt")]
        trace!("descriptor final state: {:?}", txf_desc);

        if tail_byte.end_of_transfer {
            // do something with the payload
            let mut payload_buffer = Vec::new();
            let rx_completed = if tail_byte.start_of_transfer {
                payload_buffer
                    .extend_from_slice(txf_desc.payload.as_slice())
                    .map_err(|_| CanError::RxPayloadBufferFull)?;
                // single frame payload
                RxPayload::new(txf_desc, None, payload_buffer)
            } else {
                // multi-frame payload
                let bits = txf_desc.payload.as_slice().view_bits::<Msb0>();
                let payload_crc = Some(bits[0..16].load_le());
                // copy the payload, minus the CRC bytes
                payload_buffer
                    .extend_from_slice(&txf_desc.payload.as_slice()[2..])
                    .map_err(|_| CanError::RxPayloadBufferFull)?;
                // check the crc
                if let Some(payload_crc) = payload_crc {
                    let msg_type = MsgType::from_id(can_id.type_id)?;
                    let mut crc_calc = TransferCrc::new(msg_type.base_crc());
                    crc_calc.add_payload(&payload_buffer);
                    if crc_calc.value != payload_crc {
                        return Err(CanError::MultiFrameCrcMismatch);
                    }
                }
                RxPayload::new(txf_desc, payload_crc, payload_buffer)
            };

            // setup for the next one
            txf_desc.setup_next();
            return Ok(Some(rx_completed));
        }
        Ok(None)
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use core::convert::Infallible;
    use embedded_can::{ExtendedId, Frame, Id, nb::Can};
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

        fn transmit(
            &mut self,
            frame: &Self::Frame,
        ) -> nb::Result<Option<Self::Frame>, Self::Error> {
            assert_eq!(*frame, self.expected_tx[self.tx_index]);
            self.tx_index += 1;
            Ok(None)
        }

        fn receive(&mut self) -> nb::Result<Self::Frame, Self::Error> {
            let frame = self.expected_rx[self.rx_index].clone();
            self.rx_index += 1;
            Ok(frame)
        }
    }

    /// MockHandler
    /// TODO: implement returning Frame Vec, checking node_id, can_id
    #[derive(Debug)]
    struct MockHandler {
        expected: Vec<RxPayload, 10>,
        index: core::cell::RefCell<usize>,
    }

    impl MockHandler {
        pub fn new(expected: Vec<RxPayload, 10>) -> Self {
            Self {
                expected,
                index: core::cell::RefCell::new(0),
            }
        }
    }

    impl UavcanHandler<MockCan, MockFrame, Infallible, 72> for MockHandler {
        fn handle_message(
            &self,
            _node_id: u8,
            payload: &RxPayload,
            _can_id: &CanId,
            _interface: &mut UavcanInterface<MockCan>,
        ) -> Result<Option<Vec<BroadcastMessage, 10>>, CanError<Infallible>> {
            let idx = *self.index.borrow();
            assert_eq!(*payload, self.expected[idx]);
            self.index.replace(idx + 1);
            Ok(None)
        }
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
        let mut intf = UavcanInterface::new(MockCan::new(expected_tx, expected_rx), 0);
        let mut uav: Uavcan = Uavcan::new(1, 2000, 300);

        // now send 2 frames each a message of 7 bytes
        let payload = [0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7];
        let mut msg1 = BroadcastMessage::new(FrameType::Message, MsgType::NodeStatus);
        msg1.payload.extend_from_slice(&payload).unwrap();
        let tx_frames = uav.broadcast::<MockFrame, Infallible, 72>(msg1).unwrap();
        intf.send(tx_frames).unwrap();
        let mut msg2 = BroadcastMessage::new(FrameType::Message, MsgType::NodeStatus);
        msg2.payload.extend_from_slice(&payload).unwrap();
        let tx_frames = uav.broadcast::<MockFrame, Infallible, 72>(msg2).unwrap();
        intf.send(tx_frames).unwrap();
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
        let mut intf = UavcanInterface::new(MockCan::new(expected_tx, expected_rx), 0);
        let mut uav: Uavcan = Uavcan::new(1, 2000, 300);

        // now send 4 frames, consisting of 2 messages of 8 bytes
        let payload = [0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8];
        let mut msg1 = BroadcastMessage::new(FrameType::Message, MsgType::NodeStatus);
        msg1.payload.extend_from_slice(&payload).unwrap();
        let tx_frames = uav.broadcast::<MockFrame, Infallible, 72>(msg1).unwrap();
        intf.send(tx_frames).unwrap();
        let mut msg2 = BroadcastMessage::new(FrameType::Message, MsgType::NodeStatus);
        msg2.payload.extend_from_slice(&payload).unwrap();
        let tx_frames = uav.broadcast::<MockFrame, Infallible, 72>(msg2).unwrap();
        intf.send(tx_frames).unwrap();
    }

    #[test]
    fn test_rx1() {
        // interface expected rx frames
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
        // interface expected tx frames
        let expected_tx = Vec::new();
        let mut intf = UavcanInterface::new(MockCan::new(expected_tx, expected_rx), 0);

        let mut uav: Uavcan = Uavcan::new(1, 2000, 300);

        // handler expected results
        let expected = Vec::from_slice(&[
            RxPayload {
                transfer_id: 0,
                ts: 10,
                payload_crc: None,
                payload: Vec::from_slice(&[0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7]).unwrap(),
            },
            RxPayload {
                transfer_id: 1,
                ts: 20,
                payload_crc: None,
                payload: Vec::from_slice(&[0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7]).unwrap(),
            },
            RxPayload {
                transfer_id: 2,
                ts: 40,
                payload_crc: Some(13214),
                payload: Vec::from_slice(&[0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8]).unwrap(),
            },
            RxPayload {
                transfer_id: 3,
                ts: 50,
                payload_crc: Some(13214),
                payload: Vec::from_slice(&[0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8]).unwrap(),
            },
        ])
        .unwrap();
        let handler = MockHandler::new(expected);

        // now get 6 frames, first is (2) 7 byte messages, followed by (2) messages of 8 bytes
        let frame = intf.receive().unwrap();
        uav.handle_rx_frame(&mut intf, &frame, &10, Some(&handler))
            .unwrap();

        let frame = intf.receive().unwrap();
        uav.handle_rx_frame(&mut intf, &frame, &20, Some(&handler))
            .unwrap();

        // the multi frame messages now
        let frame = intf.receive().unwrap();
        uav.handle_rx_frame(&mut intf, &frame, &40, Some(&handler))
            .unwrap();

        let frame = intf.receive().unwrap();
        uav.handle_rx_frame(&mut intf, &frame, &45, Some(&handler))
            .unwrap();

        let frame = intf.receive().unwrap();
        uav.handle_rx_frame(&mut intf, &frame, &50, Some(&handler))
            .unwrap();

        let frame = intf.receive().unwrap();
        uav.handle_rx_frame(&mut intf, &frame, &55, Some(&handler))
            .unwrap();
    }
}
