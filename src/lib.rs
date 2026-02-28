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

pub use interface::*;
pub use messages::*;
pub use protocol::*;

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

/// Errors that can occur in dronecan
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug, Clone)]
pub enum CanError<E: embedded_can::Error + core::fmt::Debug> {
    Hardware(E),
    PayloadSize,
    PayloadData,
    TransmitQueueFull,
    TransferMapFull,
    RxPayloadBufferFull,
    TxPayloadBufferFull,
}

impl<E: embedded_can::Error> From<E> for CanError<E> {
    fn from(e: E) -> CanError<E> {
        CanError::Hardware(e)
    }
}

#[cfg(test)]
mod test {
    //    use super::*;
    use test_log::test;

    #[test]
    fn test_it() {
        assert!(true);
    }
}
