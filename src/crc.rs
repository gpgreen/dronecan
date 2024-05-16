//! Code for the CRC and data type signature, used in multi-frame transfers.

const CRC_POLY: u16 = 0x1021;

/// Code for computing CRC for multi-frame transfers:
/// Adapted from https://dronecan.github.io/Specification/4._CAN_bus_transport_layer/
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub struct TransferCrc {
    pub value: u16,
}

impl TransferCrc {
    /// Use `pydronecan` to find each message type's base CRC:
    /// `dronecan.DATATYPES[(message_type_id, 1 or 0)].base_crc`
    pub fn new(base: u16) -> Self {
        Self { value: base }
    }

    /// Constructor doesn't use the message type's base CRC,
    /// This requires the use of the 'Data Type Signature':
    /// https://dronecan.github.io/Specification/4.1_CAN_bus_transport_layer/#payload-decomposition
    /// The result of this can be used as the 'base_crc' calculated in python.
    pub fn new_with_signature(sig: u64) -> Self {
        let mut calc = Self { value: 0xFFFF };
        calc.add_payload(&sig.to_le_bytes());
        calc
    }

    /// See https://github.com/dronecan/pydronecan/blob/master/dronecan/dsdl/common.py#L50
    /// Verified against the Python lib with a few examples.
    fn add_byte(&mut self, byte: u8) {
        self.value ^= (byte as u16) << 8;

        for _bit in 0..8 {
            if (self.value & 0x8000) != 0 {
                self.value = (self.value << 1) ^ CRC_POLY;
            } else {
                self.value <<= 1;
            }
        }
    }

    /// Add a buffer to the CRC calculation
    pub fn add_payload(&mut self, payload: &[u8]) {
        for b in payload.iter() {
            self.add_byte(*b);
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_new_with_sig() {
        // the uavcan_protocol_Panic data type signature
        let crc_fn = TransferCrc::new_with_signature(0x8B79B4101811C1D7);
        assert_eq!(crc_fn.value, 64_606);
    }
}
