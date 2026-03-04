//! This module contains code that prepares a payload and ID in accordance with the DroneCAN
//! specification.

//! [Relevant section of DroneCAN specification](https://dronecan.github.io/Specification/4._CAN_bus_transport_layer/)

#![no_std]

pub mod crc;
pub mod dsdl;
pub mod gnss;
pub mod interface;
pub mod messages;
pub mod protocol;
pub mod uavcan_reactor;

pub use interface::*;
pub use messages::*;
pub use protocol::*;
pub use uavcan_reactor::*;

pub const PAYLOAD_SIZE_CONFIG_COMMON: usize = 4;

pub const NODE_STATUS_BROADCAST_PERIOD: f32 = 1.; // In s. Between 2 and 1000.

/// Calculate the size in bytes needed to store a certain number of bits.
pub fn bit_size_to_byte_size(len_bits: usize) -> usize {
    let base_size = len_bits / 8;

    if len_bits % 8 > 0 {
        base_size + 1
    } else {
        base_size
    }
}

/// Random number generator, used in Anonymous messages, for now we are returning
/// a constant
pub fn random_u32() -> u32 {
    // TODO: a random number generator
    // 14-bit discriminator. Discriminator should be random. We use the RNG peripheral.
    // Once set, we keep it.
    //#[cfg(feature = "hal")]
    //let discriminator = (rng::read() & 0b11_1111_1111_1111) as u32;
    335
}

/// Errors that can occur in dronecan
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone)]
pub enum CanError<E: embedded_can::Error + core::fmt::Debug> {
    Hardware(E),
    PayloadSize,
    PayloadData,
    TransmitQueueFull,
    TransferMapFull,
    RxPayloadBufferFull,
    TxPayloadBufferFull,
    /// multi frame payload CRC doesn't match calculated value
    MultiFrameCrcMismatch,
    /// Library is not aware of this Id found in can frame
    UnimplementedUavCanId,
}

impl<E: embedded_can::Error> From<E> for CanError<E> {
    fn from(e: E) -> CanError<E> {
        CanError::Hardware(e)
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use test_log::test;

    #[test]
    fn test_bit_size() {
        assert_eq!(bit_size_to_byte_size(0), 0);
        assert_eq!(bit_size_to_byte_size(1), 1);
        assert_eq!(bit_size_to_byte_size(2), 1);
        assert_eq!(bit_size_to_byte_size(3), 1);
        assert_eq!(bit_size_to_byte_size(4), 1);
        assert_eq!(bit_size_to_byte_size(5), 1);
        assert_eq!(bit_size_to_byte_size(6), 1);
        assert_eq!(bit_size_to_byte_size(7), 1);
        assert_eq!(bit_size_to_byte_size(8), 1);
        assert_eq!(bit_size_to_byte_size(9), 2);
    }
}
