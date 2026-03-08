//! This module contains code related to sending and receiving DroneCAN frames over CAN.

use crate::CanError;
use crate::protocol::FrameType;
use embedded_can;
use heapless::Vec;
use nb;

/// the maximum size of a legacy data frame
pub(crate) const DATA_FRAME_MAX_LEN_LEGACY: usize = 8;
/// the maximum number of legacy data frames in a uavcan message
pub(crate) const MAX_RX_FRAMES_LEGACY: usize = 24;
/// the maximum size in bytes of a buffer for frames of a uavcan message
pub(crate) const RX_TX_BUFFER_MAX_SIZE: usize = MAX_RX_FRAMES_LEGACY * DATA_FRAME_MAX_LEN_LEGACY;

/// The key for a map entry to `TransferDesc`
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, PartialEq, Default)]
pub struct TransferDesc {
    transfer_id: u8,
    next_toggle_bit: bool,
    ts: u64,
    iface_index: usize,
    pub(crate) payload: Vec<u8, RX_TX_BUFFER_MAX_SIZE>,
}

impl TransferDesc {
    pub const TRANSFER_ID_HALF_RANGE: u8 = 16;

    /// Get the forward distance for a transfer id
    pub fn compute_forward_distance(a: u8, b: u8) -> u8 {
        const MAX_VALUE: u8 = (1 << 5) - 1;
        if a > MAX_VALUE || b > MAX_VALUE {
            panic!();
        }
        if a > b { MAX_VALUE - a + b + 1 } else { b - a }
    }

    /// Get the transfer_id
    pub fn transfer_id(&self) -> u8 {
        self.transfer_id
    }

    /// Get the timestamp
    pub fn ts(&self) -> u64 {
        self.ts
    }

    /// Get the interface index
    pub fn index(&self) -> usize {
        self.iface_index
    }

    /// Get the next toggle bit
    pub fn next_toggle(&self) -> bool {
        self.next_toggle_bit
    }

    /// Set the timestamp
    pub fn set_ts(&mut self, ts: u64) {
        self.ts = ts;
    }

    /// Start of a transfer
    pub fn start(&mut self, ts: &u64, iface_index: usize) {
        self.iface_index = iface_index;
        self.ts = *ts;
    }

    /// Restart a transfer
    pub fn restart(&mut self, iface_index: usize, tid: u8) {
        self.iface_index = iface_index;
        self.transfer_id = tid;
        self.next_toggle_bit = false;
        self.payload.clear();
    }

    /// Setup for next transfer
    pub fn setup_next(&mut self) {
        self.transfer_id_incr();
        self.next_toggle_bit = false;
        self.payload.clear();
    }

    /// Toggle the bit
    pub fn toggle(&mut self) {
        self.next_toggle_bit = !self.next_toggle_bit;
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
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone)]
pub struct RxPayload {
    /// Transfer number for this payload
    pub transfer_id: u8,
    /// timestamp at beginning of payload
    pub ts: u64,
    /// If a multi-frame payload, this is the crc
    pub payload_crc: Option<u16>,
    /// the payload in bytes
    pub payload: Vec<u8, RX_TX_BUFFER_MAX_SIZE>,
}

impl RxPayload {
    pub fn new(
        txf: &TransferDesc,
        payload_crc: Option<u16>,
        payload: Vec<u8, RX_TX_BUFFER_MAX_SIZE>,
    ) -> Self {
        Self {
            transfer_id: txf.transfer_id(),
            ts: txf.ts(),
            payload_crc,
            payload,
        }
    }
}

// implement PartialEq to compare the payload vectors
impl core::cmp::PartialEq for RxPayload {
    fn eq(&self, other: &Self) -> bool {
        if self.transfer_id == other.transfer_id
            && self.ts == other.ts
            && self.payload_crc == other.payload_crc
        {
            for (b1, b2) in core::iter::zip(&self.payload, &other.payload) {
                if b1 != b2 {
                    return false;
                }
            }
            return self.payload.len() == other.payload.len();
        }
        false
    }
}

/// Wrapper for Uavcan on CAN interface, associates an index
/// with a embedded_can interface
pub struct UavcanInterface<CAN> {
    iface: CAN,
    n: usize,
}

impl<CAN, FRAME, E> UavcanInterface<CAN>
where
    CAN: embedded_can::nb::Can<Frame = FRAME, Error = E>,
    FRAME: embedded_can::Frame,
    E: embedded_can::Error,
{
    /// Create new `UavcanInterface` with an index
    pub fn new(iface: CAN, n: usize) -> Self {
        assert!(n == 0 || n == 1);
        Self { iface, n }
    }

    /// Get the index of this interface
    pub fn index(&self) -> usize {
        self.n
    }

    /// Transmit a set of frames, blocking
    pub fn send(&mut self, frames: Vec<FRAME, 10>) -> Result<(), CanError<E>> {
        for f in frames {
            match nb::block!(self.iface.transmit(&f))? {
                // TODO: we could get a lower priority frame, need to resend it
                Some(_lower_priority_frame) => {}
                None => {}
            }
        }
        Ok(())
    }
}

impl<CAN, FRAME, E> core::ops::Deref for UavcanInterface<CAN>
where
    CAN: embedded_can::nb::Can<Frame = FRAME, Error = E>,
    FRAME: embedded_can::Frame,
    E: embedded_can::Error,
{
    type Target = CAN;

    fn deref(&self) -> &Self::Target {
        &self.iface
    }
}

impl<CAN, FRAME, E> core::ops::DerefMut for UavcanInterface<CAN>
where
    CAN: embedded_can::nb::Can<Frame = FRAME, Error = E>,
    FRAME: embedded_can::Frame,
    E: embedded_can::Error,
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.iface
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use test_log::test;

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
}
