//! This module contains code that prepares a payload and ID in accordance with the DroneCAN
//! specification.

//! [Relevant section of DroneCAN specification](https://dronecan.github.io/Specification/4._CAN_bus_transport_layer/)

#![no_std]

use core::mem;

pub mod broadcast;

mod crc;
pub mod dsdl;
pub mod gnss;
pub mod interface;
pub mod messages;
pub mod protocol;

pub use broadcast::*;
pub use dsdl::*;
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
#[derive(Debug, Clone, Copy)]
pub enum CanError {
    Hardware,
    PayloadSize,
    FrameSize,
    PayloadData,
    TransmitQueueFull,
    TransferMapFull,
    RxPayloadBufferFull,
    TxPayloadBufferFull,
}

/// 16-bit floating point
/// Alternative to `half` lib, without bringing in a dep.
///
#[allow(non_camel_case_types)]
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug)]
/// We use this instead of the `half` lib due to binary size issues when using the lib.
pub struct f16 {
    bits: u16,
}

/// from `half`'s impl: https://github.com/starkat99/half-rs/blob/main/src/leading_zeros.rs
#[inline]
const fn leading_zeros_u16(mut x: u16) -> u32 {
    use crunchy::unroll;
    let mut c = 0;
    let msb = 1 << 15;
    unroll! { for i in 0 .. 16 {
        if x & msb == 0 {
            c += 1;
        } else {
            return c;
        }
        #[allow(unused_assignments)]
        if i < 15 {
            x <<= 1;
        }
    }}
    c
}

impl f16 {
    pub const fn from_f32(value: f32) -> Self {
        // half's implementation
        // https://github.com/starkat99/half-rs/blob/main/src/binary16/arch.rs
        // TODO: Replace mem::transmute with to_bits() once to_bits is const-stabilized
        // Convert to raw bytes
        let x: u32 = unsafe { mem::transmute(value) };

        // Extract IEEE754 components
        let sign = x & 0x8000_0000u32;
        let exp = x & 0x7F80_0000u32;
        let man = x & 0x007F_FFFFu32;

        // Check for all exponent bits being set, which is Infinity or NaN
        if exp == 0x7F80_0000u32 {
            // Set mantissa MSB for NaN (and also keep shifted mantissa bits)
            let nan_bit = if man == 0 { 0 } else { 0x0200u32 };
            return Self {
                bits: ((sign >> 16) | 0x7C00u32 | nan_bit | (man >> 13)) as u16,
            };
        }

        // The number is normalized, start assembling half precision version
        let half_sign = sign >> 16;
        // Unbias the exponent, then bias for half precision
        let unbiased_exp = ((exp >> 23) as i32) - 127;
        let half_exp = unbiased_exp + 15;

        // Check for exponent overflow, return +infinity
        if half_exp >= 0x1F {
            return Self {
                bits: (half_sign | 0x7C00u32) as u16,
            };
        }

        // Check for underflow
        if half_exp <= 0 {
            // Check mantissa for what we can do
            if 14 - half_exp > 24 {
                // No rounding possibility, so this is a full underflow, return signed zero
                return Self {
                    bits: half_sign as u16,
                };
            }
            // Don't forget about hidden leading mantissa bit when assembling mantissa
            let man = man | 0x0080_0000u32;
            let mut half_man = man >> (14 - half_exp);
            // Check for rounding (see comment above functions)
            let round_bit = 1 << (13 - half_exp);
            if (man & round_bit) != 0 && (man & (3 * round_bit - 1)) != 0 {
                half_man += 1;
            }
            // No exponent for subnormals
            return Self {
                bits: (half_sign | half_man) as u16,
            };
        }

        // Rebias the exponent
        let half_exp = (half_exp as u32) << 10;
        let half_man = man >> 13;
        // Check for rounding (see comment above functions)
        let round_bit = 0x0000_1000u32;

        let bits = if (man & round_bit) != 0 && (man & (3 * round_bit - 1)) != 0 {
            // Round it
            ((half_sign | half_exp | half_man) + 1) as u16
        } else {
            (half_sign | half_exp | half_man) as u16
        };

        Self { bits }
    }

    pub fn to_f32(self) -> f32 {
        // half's implementation
        // https://github.com/starkat99/half-rs/blob/main/src/binary16/arch.rs

        let i = self.bits;

        // Check for signed zero
        // TODO: Replace mem::transmute with from_bits() once from_bits is const-stabilized
        if i & 0x7FFFu16 == 0 {
            return unsafe { mem::transmute((i as u32) << 16) };
        }

        let half_sign = (i & 0x8000u16) as u32;
        let half_exp = (i & 0x7C00u16) as u32;
        let half_man = (i & 0x03FFu16) as u32;

        // Check for an infinity or NaN when all exponent bits set
        if half_exp == 0x7C00u32 {
            // Check for signed infinity if mantissa is zero
            if half_man == 0 {
                return unsafe { mem::transmute((half_sign << 16) | 0x7F80_0000u32) };
            } else {
                // NaN, keep current mantissa but also set most significiant mantissa bit
                return unsafe {
                    mem::transmute((half_sign << 16) | 0x7FC0_0000u32 | (half_man << 13))
                };
            }
        }

        // Calculate single-precision components with adjusted exponent
        let sign = half_sign << 16;
        // Unbias exponent
        let unbiased_exp = ((half_exp as i32) >> 10) - 15;

        // Check for subnormals, which will be normalized by adjusting exponent
        if half_exp == 0 {
            // Calculate how much to adjust the exponent by
            let e = leading_zeros_u16(half_man as u16) - 6;

            // Rebias and adjust exponent
            let exp = (127 - 15 - e) << 23;
            let man = (half_man << (14 + e)) & 0x7F_FF_FFu32;
            return unsafe { mem::transmute(sign | exp | man) };
        }

        // Rebias exponent for a normalized normal
        let exp = ((unbiased_exp + 127) as u32) << 23;
        let man = (half_man & 0x03FFu32) << 13;

        unsafe { mem::transmute(sign | exp | man) }
    }

    pub fn from_le_bytes(bytes: [u8; 2]) -> Self {
        Self {
            bits: u16::from_le_bytes(bytes),
        }
    }

    pub fn to_le_bytes(&self) -> [u8; 2] {
        self.bits.to_le_bytes()
    }

    pub fn to_be_bytes(&self) -> [u8; 2] {
        self.bits.to_be_bytes()
    }

    pub fn as_u16(&self) -> u16 {
        self.bits
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
