//! This module contains types associated with specific Dronecan messages.

use crate::{f16, protocol::ConfigCommon, CanError, MsgType};
use bitvec::prelude::*;
use heapless::{String, Vec};

#[cfg(feature = "defmt")]
use defmt::*;
//#[cfg(not(feature = "defmt"))]
//use log::*;

// To simplify some logic re trailing values, we only match the first few characters of the param name.
pub const NAME_CUTOFF: usize = 6;

pub const PARAM_NAME_NODE_ID: &[u8] = "Node ID (desired if dynamic allocation is set)".as_bytes();
pub const PARAM_NAME_DYNAMIC_ID: &[u8] = "Dynamic ID allocation".as_bytes();
pub const PARAM_NAME_FD_MODE: &[u8] = "CAN FD enabled".as_bytes();
pub const PARAM_NAME_BITRATE: &[u8] = "CAN bitrate (see datasheet)".as_bytes();

// This message is sent by Ardupilot; respond with 0 unless you wish to limit
// or inhibit GPS.
pub const PARAM_NAME_GPS_TYPE: &[u8] = "GPS_TYPE".as_bytes();

////////////////////////////////////////////////////////////////////////////////

/// Reference: https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/341.NodeStatus.uavcan
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum NodeHealth {
    Ok = 0,
    Warning = 1,
    Error = 2,
    Critical = 3,
}

impl From<u8> for NodeHealth {
    fn from(val: u8) -> Self {
        match val {
            0 => NodeHealth::Ok,
            1 => NodeHealth::Warning,
            2 => NodeHealth::Error,
            3 => NodeHealth::Critical,
            _ => panic!(),
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

/// Reference: https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/341.NodeStatus.uavcan
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum NodeMode {
    Operational = 0,
    Initialization = 1,
    Maintenance = 2,
    SoftwareUpdate = 3,
    Offline = 7,
}

impl From<u8> for NodeMode {
    fn from(val: u8) -> Self {
        match val {
            0 => NodeMode::Operational,
            1 => NodeMode::Initialization,
            2 => NodeMode::Maintenance,
            3 => NodeMode::SoftwareUpdate,
            7 => NodeMode::Offline,
            _ => panic!(),
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

/// Size of Payload for NodeStatus message (fixed)
pub const PAYLOAD_SIZE_NODE_STATUS: usize = 7;

/// Broadcast periodically, and sent as part of the Node Status message.
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NodeStatus {
    pub uptime_sec: u32,
    pub health: NodeHealth,
    pub mode: NodeMode,
    pub vendor_specific_status_code: u16,
}

impl NodeStatus {
    pub fn to_bytes(&self) -> [u8; PAYLOAD_SIZE_NODE_STATUS] {
        let mut result = [0; PAYLOAD_SIZE_NODE_STATUS];

        result[..4].clone_from_slice(&self.uptime_sec.to_le_bytes());

        // Health and mode. Submode is reserved by the spec for future use,
        // but is currently not used.
        result[4] = ((self.health as u8) << 6) | ((self.mode as u8) << 3);

        result[5..7].clone_from_slice(&self.vendor_specific_status_code.to_le_bytes());

        result
    }
    pub fn from_bytes(buf: &[u8]) -> Self {
        let bits = buf.view_bits::<Msb0>();
        let uptime_sec = bits[0..32].load_le();
        let health = NodeHealth::from(bits[32..34].load::<u8>());
        let mode = NodeMode::from(bits[34..37].load::<u8>());
        let vendor_specific_status_code = bits[40..56].load_le();
        Self {
            uptime_sec,
            health,
            mode,
            vendor_specific_status_code,
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

/// Maximum Size of Payload for HardwareVersion, (variable)
pub const HARDWARE_VERSION_MAX_SIZE: usize = 19 + 255;

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/HardwareVersion.uavcan
/// Generic hardware version information.
/// These values should remain unchanged for the device's lifetime.
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HardwareVersion {
    pub major: u8,
    pub minor: u8,
    /// Unique ID is a 128 bit long sequence that is globally unique for each node.
    /// All zeros is not a valid UID.
    /// If filled with zeros, assume that the value is undefined.
    /// on linux we can get one via:
    /// sudo cat /sys/class/dmi/id/product_uuid
    pub unique_id: [u8; 16],
    // We currently don't use certificate of authority.
    // /// Certificate of authenticity (COA) of the hardware, 255 bytes max.
    pub certificate_of_authority: Option<Vec<u8, 255>>,
}

impl HardwareVersion {
    /// serialize `HardwareVersion` to buffer
    pub fn to_bytes(&self) -> Vec<u8, HARDWARE_VERSION_MAX_SIZE> {
        let mut buf = Vec::new();
        let _ = buf.resize_default(HARDWARE_VERSION_MAX_SIZE);
        buf[0] = self.major;
        buf[1] = self.minor;
        buf[2..18].clone_from_slice(&self.unique_id);
        // The final index is our 8-bit length field for COA, which we hard-set to 0.
        if let Some(coa) = &self.certificate_of_authority {
            buf[18] = coa.len() as u8;
            buf[19..19 + coa.len()].copy_from_slice(coa.as_slice());
            buf.truncate(19 + coa.len());
        } else {
            buf[18] = 0;
            buf.truncate(19);
        }
        buf
    }
    /// construct `HardwareVersion` from buffer
    pub fn from_bytes(buf: &[u8]) -> Self {
        let mut unique_id = [0; 16];
        unique_id.copy_from_slice(&buf[2..18]);
        let coa_len = buf[18];
        let coa = if coa_len > 0 {
            let mut v: Vec<u8, 255> = Vec::new();
            v.extend_from_slice(&buf[19..19 + coa_len as usize])
                .unwrap();
            Some(v)
        } else {
            None
        };
        Self {
            major: buf[0],
            minor: buf[1],
            unique_id,
            certificate_of_authority: coa,
        }
    }
    /// the size of the type in a buffer
    pub fn buffer_size(&self) -> usize {
        if let Some(coa) = &self.certificate_of_authority {
            19 + coa.len()
        } else {
            19
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

/// Size of payload for SoftwareVersion (fixed)
pub const PAYLOAD_SIZE_SOFTWARE_VERSION: usize = 15;

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/SoftwareVersion.uavcan
/// Generic software version information.
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct SoftwareVersion {
    pub major: u8,
    pub minor: u8,
    /// This mask indicates which optional fields (see below) are set.
    /// u8 OPTIONAL_FIELD_FLAG_VCS_COMMIT = 1
    /// u8 OPTIONAL_FIELD_FLAG_IMAGE_CRC = 2
    pub optional_field_flags: u8,
    /// VCS commit hash or revision number, e.g. git short commit hash. Optional.
    pub vcs_commit: u32,
    /// The value of an arbitrary hash function applied to the firmware image.
    pub image_crc: u64,
}

impl SoftwareVersion {
    /// serialize `SoftwareVersion` to a buffer
    pub fn to_bytes(&self) -> [u8; PAYLOAD_SIZE_SOFTWARE_VERSION] {
        let mut result = [0; 15];

        result[0] = self.major;
        result[1] = self.minor;
        result[2] = self.optional_field_flags;
        result[3..7].clone_from_slice(&self.vcs_commit.to_le_bytes());
        result[7..15].clone_from_slice(&self.image_crc.to_le_bytes());

        result
    }

    /// read `SoftwareVersion` from buffer
    pub fn from_bytes(buf: &[u8]) -> Self {
        let bits = buf.view_bits::<Msb0>();
        let vcs_commit = bits[24..56].load_le();
        let image_crc = bits[56..120].load_le();
        Self {
            major: buf[0],
            minor: buf[1],
            optional_field_flags: buf[2],
            vcs_commit,
            image_crc,
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

/// Maximum size of the Vec for GetNodeInfoResponse
pub const UAVCAN_PROTOCOL_GET_NODE_INFO_RESPONSE_MAX_SIZE: usize = 377;

///
/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/GetNodeInfoResponse.uavcan
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(Format))]
pub struct GetNodeInfoResponse {
    pub node_status: NodeStatus,
    pub sw_version: SoftwareVersion,
    pub hw_version: HardwareVersion,
    /// Human readable non-empty ASCII node name.
    /// Node name shall not be changed while the node is running.
    /// Empty string is not a valid node name.
    /// Allowed characters are: a-z (lowercase ASCII letters) 0-9 (decimal digits) . (dot) - (dash) _ (underscore).
    /// Node name is a reversed internet domain name (like Java packages), e.g. "com.manufacturer.project.product".
    pub name: String<80>, // max length of 80
}

impl GetNodeInfoResponse {
    /// serialize `GetNodeInfoResponse` to buffer
    pub fn to_bytes(&self) -> Vec<u8, UAVCAN_PROTOCOL_GET_NODE_INFO_RESPONSE_MAX_SIZE> {
        let mut buf = Vec::new();
        buf.resize_default(UAVCAN_PROTOCOL_GET_NODE_INFO_RESPONSE_MAX_SIZE)
            .unwrap();
        // copy node status
        let ns = self.node_status.to_bytes();
        buf[0..ns.len()].clone_from_slice(&ns);
        let mut start = ns.len();

        // copy sw version
        let sw = self.sw_version.to_bytes();
        buf[start..start + sw.len()].clone_from_slice(&sw);
        start += sw.len();

        // copy hw version
        let hw = self.hw_version.to_bytes();
        buf[start..start + hw.len()].clone_from_slice(&hw);
        start += hw.len();
        if self.name.is_empty() || self.name.len() > 80 {
            panic!();
        }
        buf[start] = self.name.len() as u8;
        start += 1;
        buf[start..start + self.name.len()].clone_from_slice(self.name.as_bytes());
        buf.truncate(start + self.name.len());
        buf
    }

    /// construct `GetNodeInfoResponse` from buffer
    pub fn from_bytes(buf: &[u8]) -> Self {
        let mut start = 0;
        let node_status = NodeStatus::from_bytes(&buf[start..PAYLOAD_SIZE_NODE_STATUS]);
        start = PAYLOAD_SIZE_NODE_STATUS;
        let sw_version =
            SoftwareVersion::from_bytes(&buf[start..start + PAYLOAD_SIZE_SOFTWARE_VERSION]);
        start += PAYLOAD_SIZE_SOFTWARE_VERSION;
        let hw_version = HardwareVersion::from_bytes(&buf[start..]);
        start += hw_version.buffer_size();
        let name_len = buf[start];
        start += 1;
        let mut v = Vec::new();
        let _ = v.extend_from_slice(&buf[start..start + name_len as usize]);
        let name = String::from_utf8(v).unwrap();
        Self {
            node_status,
            sw_version,
            hw_version,
            name,
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

/// Maximum size of the Vec for GetNodeInfoResponse
pub const UAVCAN_PROTOCOL_EXECUTEOPCODE_REQUEST_SIZE: usize = 7;
pub const UAVCAN_PROTOCOL_EXECUTEOPCODE_RESPONSE_SIZE: usize = 7;

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/param/10.ExecuteOpcode.uavcan
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum OpcodeType {
    Save = 0,
    Erase = 1,
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/param/10.ExecuteOpcode.uavcan
/// Pertains to saving or erasing config to/from flash
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExecuteOpcodeRequest {
    pub opcode: OpcodeType,
    pub argument: i64,
}

impl ExecuteOpcodeRequest {
    /// serialize `ExecuteOpcodeRequest` to buffer
    pub fn to_bytes(&self) -> Vec<u8, UAVCAN_PROTOCOL_EXECUTEOPCODE_REQUEST_SIZE> {
        let mut buf = Vec::new();
        buf.push(match self.opcode {
            OpcodeType::Save => 0,
            OpcodeType::Erase => 1,
        })
        .ok();
        buf.extend_from_slice(&self.argument.to_le_bytes()[..6])
            .unwrap();
        buf
    }

    /// construct `ExecuteOpcodeRequest` from buffer
    pub fn from_bytes(buf: &[u8]) -> Self {
        let opcode = if buf[0] == 1 {
            OpcodeType::Erase
        } else {
            OpcodeType::Save
        };

        // argument is stored as 6 bytes, the final 2 bytes for a i64 need to be sign-extended from the high bit
        let mut argbuf: [u8; 8] = [0; 8];
        argbuf[..6].clone_from_slice(&buf[1..7]);
        // sign-extend if needed
        if argbuf[5] & 0x80 == 0x80 {
            argbuf[6] = 0xff;
            argbuf[7] = 0xff;
        }
        let arg_val = i64::from_le_bytes(argbuf);

        Self {
            opcode,
            argument: arg_val,
        }
    }
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/param/10.ExecuteOpcode.uavcan
/// Pertains to saving or erasing config to/from flash
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExecuteOpcodeResponse {
    pub error_code: Option<i64>, // 48 bits.
    pub ok: bool,
}

impl ExecuteOpcodeResponse {
    /// serialize `ExecuteOpcodeResponse` to buffer
    pub fn to_bytes(&self) -> Vec<u8, UAVCAN_PROTOCOL_EXECUTEOPCODE_RESPONSE_SIZE> {
        let mut buf = Vec::new();
        let val = self.error_code.unwrap_or(0);
        buf.extend_from_slice(&val.to_le_bytes()[..6]).unwrap();
        let val = match self.ok {
            true => 1,
            false => 0,
        };
        buf.push(val).ok();
        buf
    }

    /// construct `ExecuteOpcodeResponse` from buffer
    pub fn from_bytes(buf: &[u8]) -> Self {
        // error code is stored as 6 bytes, the final 2 bytes for a i64 need to be sign-extended from the high bit
        let mut ecbuf: [u8; 8] = [0; 8];
        ecbuf[..6].clone_from_slice(&buf[..6]);
        // sign-extend if needed
        if ecbuf[5] & 0x80 == 0x80 {
            ecbuf[6] = 0xff;
            ecbuf[7] = 0xff;
        }
        let error_code_val = i64::from_le_bytes(ecbuf);
        let error_code = match error_code_val {
            0 => None,
            _ => Some(error_code_val),
        };

        Self {
            error_code,
            ok: buf[6] != 0,
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

// Used to determine which enum (union) variant is used.
// "Tag is 3 bit long, so outer structure has 5-bit prefix to ensure proper alignment"
const VALUE_TAG_BIT_LEN: usize = 3;
const VALUE_NUMERIC_TAG_BIT_LEN: usize = 2;
// For use in `GetSet`
const NAME_LEN_BIT_SIZE: usize = 7; // round_up(log2(92+1));

const MAX_GET_SET_NAME_LEN: usize = 50; // not overal max; max we use to keep buf size down

// Size in bits of the value string's size byte (leading byte)
const VALUE_STRING_LEN_SIZE: usize = 8; // round_up(log2(128+1));

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/param/NumericValue.uavcan
/// `uavcan.protocol.param.NumericValue`
/// 2-bit tag.
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug, Clone, Copy)]
pub enum NumericValue {
    Empty,
    Integer(i64),
    Real(f32),
}

impl Default for NumericValue {
    fn default() -> Self {
        Self::Empty
    }
}

impl NumericValue {
    fn tag(&self) -> u8 {
        match self {
            Self::Empty => 0,
            Self::Integer(_) => 1,
            Self::Real(_) => 2,
        }
    }

    /// Similar to `Value.to_bits()`.
    pub fn to_bits(&self, bits: &mut BitSlice<u8, Msb0>, tag_start_i: usize) -> usize {
        let val_start_i = tag_start_i + VALUE_NUMERIC_TAG_BIT_LEN; // bits
        bits[tag_start_i..val_start_i].store(self.tag());

        match self {
            Self::Empty => val_start_i,
            Self::Integer(v) => {
                bits[val_start_i..val_start_i + 64].store_le(*v);
                val_start_i + 64
            }
            Self::Real(v) => {
                // bitvec doesn't support floats.
                let v_u32 = u32::from_le_bytes(v.to_le_bytes());
                bits[val_start_i..val_start_i + 32].store_le(v_u32);
                val_start_i + 32
            }
        }
    }
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/param/Value.uavcan
/// `uavcan.protocol.param.Value`
/// 3-bit tag with 5-bit prefix for alignment.
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug, Clone, Copy)]
pub enum Value<'a> {
    Empty,
    Integer(i64),
    Real(f32),
    Boolean(bool), // u8 repr
    /// Max length of 128 bytes.
    String(&'a [u8]),
}

impl<'a> Default for Value<'a> {
    fn default() -> Self {
        Self::Empty
    }
}

impl<'a> Value<'a> {
    fn tag(&self) -> u8 {
        match self {
            Self::Empty => 0,
            Self::Integer(_) => 1,
            Self::Real(_) => 2,
            Self::Boolean(_) => 3,
            Self::String(_) => 4,
        }
    }

    /// Modifies a bit array in place, with content from this value.
    /// Returns current bit index.
    pub fn to_bits(&self, bits: &mut BitSlice<u8, Msb0>, tag_start_i: usize) -> usize {
        let val_start_i = tag_start_i + VALUE_TAG_BIT_LEN; // bits
        bits[tag_start_i..val_start_i].store(self.tag());

        // success value of pdc msg for node id w min and max
        // [78, 111, 100, 101, 32, 73, 68, 67]
        // or is it this: [31, 15, 1, 70, 0, 0, 0, 131, 0]

        match self {
            Self::Empty => val_start_i,
            Self::Integer(v) => {
                bits[val_start_i..val_start_i + 64].store_le(*v);
                val_start_i + 64
            }
            Self::Real(v) => {
                // bitvec doesn't support floats.
                let v_u32 = u32::from_le_bytes(v.to_le_bytes());
                bits[val_start_i..val_start_i + 32].store_le(v_u32);
                val_start_i + 32
            }
            Self::Boolean(v) => {
                bits[val_start_i..val_start_i + 8].store_le(*v as u8);
                val_start_i + 8
            }
            Self::String(v) => {
                let mut i = val_start_i;
                bits[i..VALUE_STRING_LEN_SIZE].store_le(v.len());
                i += VALUE_STRING_LEN_SIZE;
                for char in *v {
                    bits[i..i + 8].store_le(*char);
                    i += 8;
                }
                i
            }
        }
    }

    /// Converts from a bit array, eg one of a larger message. Anchors using bit indexes
    /// passed as arguments.
    /// Returns self, and current bit index.
    pub fn from_bits<E: embedded_can::Error>(
        bits: &BitSlice<u8, Msb0>,
        tag_start_i: usize,
    ) -> Result<(Self, usize), CanError<E>> {
        let val_start_i = tag_start_i + VALUE_TAG_BIT_LEN;

        Ok(match bits[tag_start_i..val_start_i].load_le::<u8>() {
            0 => (Self::Empty, val_start_i),
            1 => (
                Self::Integer(bits[val_start_i..val_start_i + 64].load_le::<i64>()),
                val_start_i + 64,
            ),
            2 => {
                // No support for floats in bitvec.
                let as_u32 = bits[val_start_i..val_start_i + 32].load_le::<u32>();
                (
                    Self::Real(f32::from_le_bytes(as_u32.to_le_bytes())),
                    val_start_i + 32,
                )
            }
            3 => (
                Self::Boolean(bits[val_start_i..val_start_i + 8].load_le::<u8>() != 0),
                val_start_i + 8,
            ),
            4 => {
                // todo: Handle non-FD mode that uses TCO
                let current_i = val_start_i + VALUE_STRING_LEN_SIZE;
                let _str_len: u8 = bits[val_start_i..current_i].load_le();

                // todo: WTH?
                (Self::Integer(69), val_start_i + 64)
                // unimplemented!()
                // todo: Need to convert bitslice to byte slice.
                // (Self::String(bits[current_i..current_i + str_len as usize * 8]), current_i)
            }
            _ => return Err(CanError::PayloadData),
        })
    }
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/param/11.GetSet.uavcan
pub struct GetSetRequest<'a> {
    pub index: u16, // 13 bits
    /// If set - parameter will be assigned this value, then the new value will be returned.
    /// If not set - current parameter value will be returned.
    pub value: Value<'a>,
    // pub name: &'a [u8], // up to 92 bytes.
    /// Name of the parameter; always preferred over index if nonempty.
    pub name: [u8; MAX_GET_SET_NAME_LEN], // large enough for many uses
    pub name_len: usize,
}

impl<'a> GetSetRequest<'a> {
    // pub fn to_bytes(buf: &mut [u8]) -> Self {
    //     let index
    //     Self {
    //         index,
    //         value,
    //         name,
    //     }
    // }

    pub fn from_bytes<E: embedded_can::Error>(
        buf: &[u8],
        fd_mode: bool,
    ) -> Result<Self, CanError<E>> {
        let bits = buf.view_bits::<Msb0>();

        let tag_start_i = 13;
        let index: u16 = bits[0..tag_start_i].load_le();

        // `i` in this function is in bits, not bytes.
        let (value, mut current_i) = Value::from_bits(bits, tag_start_i)?;

        let name_len = if fd_mode {
            let v = bits[current_i..current_i + NAME_LEN_BIT_SIZE].load_le::<u8>() as usize;
            current_i += NAME_LEN_BIT_SIZE;
            v
        } else {
            // todo: Figure it out from message len, or infer from 0s.
            MAX_GET_SET_NAME_LEN // max name len we use
        };

        let mut name = [0; MAX_GET_SET_NAME_LEN];

        if name_len as usize > name.len() {
            return Err(CanError::PayloadData);
        }

        for char_i in 0..name_len {
            // todo: Why BE here? confirm this is the same for non-FD mode.
            name[char_i] = bits[current_i..current_i + 8].load_be::<u8>();
            current_i += 8;
        }

        Ok(Self {
            index,
            value,
            name,
            name_len,
        })
    }
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/param/11.GetSet.uavcan
pub struct GetSetResponse<'a> {
    /// For set requests, it should contain the actual parameter value after the set request was
    /// executed. The objective is to let the client know if the value could not be updated, e.g.
    /// due to its range violation, etc.
    pub value: Value<'a>,
    pub default_value: Value<'a>,
    pub max_value: NumericValue,
    pub min_value: NumericValue,
    /// Empty name (and/or empty value) in response indicates that there is no such parameter.
    // pub name: [u8; MAX_GET_SET_NAME_LEN], // large enough for many uses
    pub name: [u8; MAX_GET_SET_NAME_LEN], // large enough for many uses
    pub name_len: usize,
    // pub name: &'a [u8], // up to 92 bytes.
}

impl<'a> GetSetResponse<'a> {
    /// Returns array length in bytes.
    pub fn to_bytes(&self, buf: &mut [u8], fd_mode: bool) -> usize {
        let bits = buf.view_bits_mut::<Msb0>();

        let val_tag_start_i = 5; // bits.

        let current_i = self.value.to_bits(bits, val_tag_start_i);

        // 5 is the pad between `value` and `default_value`.
        let default_value_i = current_i + 5;

        let current_i = self.default_value.to_bits(bits, default_value_i);
        let max_value_i = current_i + 6;

        let current_i = self.max_value.to_bits(bits, max_value_i);
        let min_value_i = current_i + 6;

        let current_i = self.min_value.to_bits(bits, min_value_i);

        // In FD mode, we need the len field of name.
        let mut i_bit = if fd_mode {
            let mut i_bit = current_i; // bits.

            bits[i_bit..i_bit + NAME_LEN_BIT_SIZE].store_le(self.name_len);
            i_bit += NAME_LEN_BIT_SIZE;

            i_bit
        } else {
            current_i
        };

        for char in &self.name[..self.name_len] {
            bits[i_bit..i_bit + 8].store_be(*char);
            i_bit += 8;
        }

        crate::bit_size_to_byte_size(i_bit)
    }

    pub fn from_bytes<E: embedded_can::Error>(buf: &[u8]) -> Result<Self, CanError<E>> {
        let _bits = buf.view_bits::<Msb0>();

        unimplemented!() // todo: You just need to work thorugh it like with related.
                         //
                         // let val_tag_start_i = 5;
                         // let (value, current_i) = Value::from_bits(bits, val_tag_start_i, &mut [])?; // todo: t str buf
                         //
                         //
                         // // todo: Max, min and default values
                         // let default_value = Value::Empty;
                         //
                         // let max_value_i = default_value_i + VALUE_TAG_BIT_LEN + 6; // Includes pad.
                         //
                         // let max_value = NumericValue::Empty;
                         // let max_value_size = 0; // todo
                         //
                         // let min_value = NumericValue::Empty;
                         // let min_value_size = 0; // todo
                         // // 2 is tag size of numeric value.
                         // let min_value_i = max_value_i + 2 + max_value_size + 6;
                         //
                         // // todo: Update once you include default values.
                         // let name_len_i = min_value_i + 2 + min_value_size + 6;
                         //
                         // // todo: Name section here is DRY with request.
                         // let name_start_i = name_len_i + NAME_LEN_BIT_SIZE;
                         //
                         // let name_len = bits[name_len_i..name_start_i].load_le::<u8>() as usize;
                         //
                         // let mut name = [0; MAX_GET_SET_NAME_LEN];
                         //
                         // let mut i = name_start_i; // bits.
                         //
                         // i += VALUE_STRING_LEN_SIZE;
                         //
                         // if name_len as usize > name.len() {
                         //     return Err(CanError::PayloadData);
                         // }
                         //
                         // for char_i in 0..name_len {
                         //     name[char_i] = bits[i..i + 8].load_le::<u8>();
                         //     i += 8;
                         // }
                         //
                         // Ok(Self {
                         //     value,
                         //     default_value,
                         //     max_value,
                         //     min_value,
                         //     name,
                         //     name_len,
                         // })
    }
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/DataTypeKind.uavcan
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum DataTypeKind {
    Service = 0,
    Message = 1,
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/dynamic_node_id/1.Allocation.uavcan
pub struct IdAllocationData {
    pub node_id: u8, // 7 bytes
    pub stage: u8,   // 0, 1, or 3.
    pub unique_id: [u8; 16],
}

impl IdAllocationData {
    pub fn to_bytes(&self, fd_mode: bool) -> [u8; MsgType::IdAllocation.payload_size() as usize] {
        let mut result = [0; MsgType::IdAllocation.payload_size() as usize];

        result[0] = (self.node_id << 1) | ((self.stage == 0) as u8);

        // unique id. Split across payloads if not on FD mode.
        if fd_mode {
            // Must include the 5-bit unique_id len field in FD mode.
            let bits = result.view_bits_mut::<Msb0>();

            let mut i_bit = 8;
            bits[i_bit..i_bit + 5].store_le(16_u8);

            i_bit += 5;

            for val in self.unique_id {
                bits[i_bit..i_bit + 8].store_le(val);
                i_bit += 8;
            }
        } else {
            match self.stage {
                0 => {
                    result[1..7].copy_from_slice(&self.unique_id[0..6]);
                }
                1 => {
                    result[1..7].copy_from_slice(&self.unique_id[6..12]);
                }
                2 => {
                    result[1..5].copy_from_slice(&self.unique_id[12..16]);
                }
                _ => (),
            };
        }

        result
    }

    pub fn from_bytes(buf: &[u8; MsgType::IdAllocation.payload_size() as usize]) -> Self {
        let stage = if (buf[0] & 1) != 0 { 1 } else { 0 };

        Self {
            // todo: QC order
            node_id: (buf[0] << 1) & 0b111_1111,
            stage,
            unique_id: buf[1..17].try_into().unwrap(),
        }
    }
}

/// Make a GetSet response from common config items. This reduces repetition in node firmware.
pub fn make_getset_response_common<'a>(
    config: &ConfigCommon,
    index: u8,
) -> Option<GetSetResponse<'a>> {
    // We load the default config to determine default values.
    let cfg_default = ConfigCommon::default();

    let mut name = [0; MAX_GET_SET_NAME_LEN];

    match index {
        0 => {
            let text = PARAM_NAME_NODE_ID;
            name[0..text.len()].copy_from_slice(text);

            Some(GetSetResponse {
                value: Value::Integer(config.node_id as i64),
                default_value: Value::Integer(cfg_default.node_id as i64),
                max_value: NumericValue::Integer(127),
                min_value: NumericValue::Integer(0),
                name,
                name_len: text.len(),
            })
        }
        1 => {
            let text = PARAM_NAME_DYNAMIC_ID;
            name[0..text.len()].copy_from_slice(text);

            Some(GetSetResponse {
                value: Value::Boolean(config.dynamic_id_allocation),
                default_value: Value::Boolean(cfg_default.dynamic_id_allocation),
                max_value: NumericValue::Empty,
                min_value: NumericValue::Empty,
                name,
                name_len: text.len(),
            })
        }
        2 => {
            let text = PARAM_NAME_FD_MODE;
            name[0..text.len()].copy_from_slice(text);

            Some(GetSetResponse {
                value: Value::Boolean(config.fd_mode),
                default_value: Value::Boolean(cfg_default.fd_mode),
                max_value: NumericValue::Empty,
                min_value: NumericValue::Empty,
                name,
                name_len: text.len(),
            })
        }
        // 3 => {
        //     let text = PARAM_NAME_BITRATE;
        //     name[0..text.len()].copy_from_slice(text);

        //     Some(GetSetResponse {
        //         value: Value::Integer(config.can_bitrate as i64),
        //         default_value: Value::Integer(cfg_default.can_bitrate as i64),
        //         max_value: NumericValue::Integer(6),
        //         min_value: NumericValue::Integer(0),
        //         name,
        //         name_len: text.len(),
        //     })
        // }
        _ => None,
    }
}

////////////////////////////////////////////////////////////////////////////////

/// Maximum size of the Vec for Covariance
pub const UAVCAN_EQUIPMENT_AHRS_SOLUTION_COVARIANCE_MAX_SIZE: usize = 9;

/// https://github.com/dronecan/DSDL/blob/master/uavcan/equipment/ahrs/1000.Solution.uavcan
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(Format))]
pub struct Covariance {
    pub data: Vec<f32, UAVCAN_EQUIPMENT_AHRS_SOLUTION_COVARIANCE_MAX_SIZE>,
}

impl Default for Covariance {
    fn default() -> Self {
        Self { data: Vec::new() }
    }
}

////////////////////////////////////////////////////////////////////////////////

/// Maximum size of the Vec for AhrsSolution (variable)
pub const UAVCAN_EQUIPMENT_AHRS_SOLUTION_MAX_SIZE: usize = 84;

/// https://github.com/dronecan/DSDL/blob/master/uavcan/equipment/ahrs/1000.Solution.uavcan
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(Format))]
pub struct AhrsSolution {
    pub uavcan_timestamp: u64,
    pub orientation_xyzw: [f32; 4],
    pub orientation_covariance: Covariance,
    pub angular_velocity: [f32; 3],
    pub angular_velocity_covariance: Covariance,
    pub linear_acceleration: [f32; 3],
    pub linear_acceleration_covariance: Covariance,
}

impl AhrsSolution {
    /// serialize `AhrsSolution` to buffer
    pub fn to_bytes(&self) -> Vec<u8, UAVCAN_EQUIPMENT_AHRS_SOLUTION_MAX_SIZE> {
        let mut buf = Vec::new();

        // fill the buffer with zero's to the max length
        buf.resize_default(UAVCAN_EQUIPMENT_AHRS_SOLUTION_MAX_SIZE)
            .ok();

        // copy the timestamp
        buf[..7].clone_from_slice(&self.uavcan_timestamp.to_le_bytes()[0..7]);

        let bits = buf.view_bits_mut::<Msb0>();

        let bit_index = 56; // bits

        let bit_index = self.orientation_xyzw.iter().fold(bit_index, |i, x| {
            let v = u16::from_le_bytes(f16::from_f32(*x).to_le_bytes());
            bits[i..i + 16].store_le(v);
            i + 16
        });

        // 4-bit pad and len covar
        bits[bit_index..bit_index + 4].store_le(0);
        let bit_index = bit_index + 4;
        bits[bit_index..bit_index + 4].store_le(self.orientation_covariance.data.len());
        let bit_index = bit_index + 4;
        let bit_index = self
            .orientation_covariance
            .data
            .iter()
            .fold(bit_index, |i, x| {
                let v = u16::from_le_bytes(f16::from_f32(*x).to_le_bytes());
                bits[i..i + 16].store_le(v);
                i + 16
            });

        let bit_index = self.angular_velocity.iter().fold(bit_index, |i, x| {
            let v = u16::from_le_bytes(f16::from_f32(*x).to_le_bytes());
            bits[i..i + 16].store_le(v);
            i + 16
        });

        // 4-bit pad and len covar
        bits[bit_index..bit_index + 4].store_le(0);
        let bit_index = bit_index + 4;
        bits[bit_index..bit_index + 4].store_le(self.angular_velocity_covariance.data.len());
        let bit_index = bit_index + 4;
        let bit_index = self
            .angular_velocity_covariance
            .data
            .iter()
            .fold(bit_index, |i, x| {
                let v = u16::from_le_bytes(f16::from_f32(*x).to_le_bytes());
                bits[i..i + 16].store_le(v);
                i + 16
            });

        let bit_index = self.linear_acceleration.iter().fold(bit_index, |i, x| {
            let v = u16::from_le_bytes(f16::from_f32(*x).to_le_bytes());
            bits[i..i + 16].store_le(v);
            i + 16
        });

        // tail array optimization, leave off if empty
        if !self.linear_acceleration_covariance.data.is_empty() {
            // 4-bit pad and len covar
            bits[bit_index..bit_index + 4].store_le(0);
            let bit_index = bit_index + 4;
            bits[bit_index..bit_index + 4].store_le(self.linear_acceleration_covariance.data.len());
            let bit_index = bit_index + 4;
            let bit_index =
                self.linear_acceleration_covariance
                    .data
                    .iter()
                    .fold(bit_index, |i, x| {
                        let v = u16::from_le_bytes(f16::from_f32(*x).to_le_bytes());
                        bits[i..i + 16].store_le(v);
                        i + 16
                    });

            // resize buf back to actual length
            buf.truncate(bit_index / 8);
        } else {
            buf.truncate(bit_index / 8);
        }
        buf
    }

    /// construct AhrsSolution from buffer
    pub fn from_bytes(buf: &[u8]) -> Self {
        // timestamp is stored as 7 bytes, the final byte for a u64 in le order is assumed to be zero
        let mut tsbuf: [u8; 8] = [0; 8];
        tsbuf[..7].clone_from_slice(&buf[..7]);
        let uavcan_timestamp = u64::from_le_bytes(tsbuf);
        let bits = buf.view_bits::<Msb0>();

        let mut bit_index = 56;

        // load orientation
        let mut orientation_xyzw = [0_f32; 4];
        let (_head, rest) = bits.split_at(bit_index);
        for (slot, f) in rest.chunks(16).zip(orientation_xyzw.iter_mut()) {
            let v: u16 = slot.load_le();
            let sf = f16 { bits: v };
            *f = sf.to_f32();
        }
        bit_index += 4 * 16;

        // covariance
        let (_head, rest) = bits.split_at(bit_index);
        let mut itr = rest.chunks(4);
        // skip pad
        itr.next();
        let len = itr.next().unwrap().load_le();
        bit_index += 8;
        let mut orientation_covariance = Covariance { data: Vec::new() };
        if len != 0 {
            let (_head, rest) = bits.split_at(bit_index);
            for (i, slot) in rest.chunks(16).enumerate() {
                if i == len {
                    break;
                }
                let v: u16 = slot.load_le();
                let sf = f16 { bits: v };
                orientation_covariance.data.push(sf.to_f32()).ok();
            }
        }
        bit_index += 16 * len;

        // load angular velocity
        let mut angular_velocity = [0_f32; 3];
        let (_head, rest) = bits.split_at(bit_index);
        for (slot, f) in rest.chunks(16).zip(angular_velocity.iter_mut()) {
            let v: u16 = slot.load_le();
            let sf = f16 { bits: v };
            *f = sf.to_f32();
        }
        bit_index += 3 * 16;

        // covariance
        let (_head, rest) = bits.split_at(bit_index);
        let mut itr = rest.chunks(4);
        // skip pad
        itr.next();
        let len = itr.next().unwrap().load_le();
        bit_index += 8;
        let mut angular_velocity_covariance = Covariance { data: Vec::new() };
        if len != 0 {
            let (_head, rest) = bits.split_at(bit_index);
            for (i, slot) in rest.chunks(16).enumerate() {
                if i == len {
                    break;
                }
                let v: u16 = slot.load_le();
                let sf = f16 { bits: v };
                angular_velocity_covariance.data.push(sf.to_f32()).ok();
            }
        }
        bit_index += 16 * len;

        // load linear acceleration
        let mut linear_acceleration = [0_f32; 3];
        let (_head, rest) = bits.split_at(bit_index);
        for (slot, f) in rest.chunks(16).zip(linear_acceleration.iter_mut()) {
            let v: u16 = slot.load_le();
            let sf = f16 { bits: v };
            *f = sf.to_f32();
        }
        bit_index += 3 * 16;

        // covariance (can be missing if tail-array optimization)
        let mut linear_acceleration_covariance = Covariance { data: Vec::new() };
        if buf.len() > bit_index / 8 {
            let (_head, rest) = bits.split_at(bit_index);
            let mut itr = rest.chunks(4);
            // skip pad
            itr.next();
            let len = itr.next().unwrap().load_le();
            bit_index += 8;
            if len != 0 {
                let (_head, rest) = bits.split_at(bit_index);
                for (i, slot) in rest.chunks(16).enumerate() {
                    if i == len {
                        break;
                    }
                    let v: u16 = slot.load_le();
                    let sf = f16 { bits: v };
                    linear_acceleration_covariance.data.push(sf.to_f32()).ok();
                }
            }
        }
        Self {
            uavcan_timestamp,
            orientation_xyzw,
            orientation_covariance,
            angular_velocity,
            angular_velocity_covariance,
            linear_acceleration,
            linear_acceleration_covariance,
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

/// Maximum size of the Vec for MagneticFieldStrength2
pub const UAVCAN_EQUIPMENT_AHRS_MAGNETIC_FIELD_STRENGTH2_MAX_SIZE: usize = 26;

///
/// https://github.com/dronecan/DSDL/blob/master/uavcan/equipment/ahrs/MagneticFieldStrength2.uavcan
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(Format))]
pub struct MagneticFieldStrength2 {
    pub sensor_id: u8,
    pub magnetic_field_ga: [f32; 3],
    pub magnetic_field_covariance: Covariance,
}

impl MagneticFieldStrength2 {
    /// serialize `MagneticFieldStrength2` to buffer
    pub fn to_bytes(&self) -> Vec<u8, UAVCAN_EQUIPMENT_AHRS_MAGNETIC_FIELD_STRENGTH2_MAX_SIZE> {
        let mut buf = Vec::new();
        buf.resize_default(UAVCAN_EQUIPMENT_AHRS_MAGNETIC_FIELD_STRENGTH2_MAX_SIZE)
            .unwrap();
        // copy sensor id
        buf[0] = self.sensor_id;

        let bits = buf.view_bits_mut::<Msb0>();

        let bit_index = 8; // starting at 8 bits, for rest of message

        let bit_index = self.magnetic_field_ga.iter().fold(bit_index, |i, x| {
            let v = u16::from_le_bytes(f16::from_f32(*x).to_le_bytes());
            bits[i..i + 16].store_le(v);
            i + 16
        });

        // tail array optimization, if not there, don't output anything
        if !self.magnetic_field_covariance.data.is_empty() {
            // 4-bit pad and len covar
            bits[bit_index..bit_index + 4].store_le(0);
            let bit_index = bit_index + 4;
            bits[bit_index..bit_index + 4].store_le(self.magnetic_field_covariance.data.len());
            let bit_index = bit_index + 4;
            let bit_index = self
                .magnetic_field_covariance
                .data
                .iter()
                .fold(bit_index, |i, x| {
                    let v = u16::from_le_bytes(f16::from_f32(*x).to_le_bytes());
                    bits[i..i + 16].store_le(v);
                    i + 16
                });
            // resize buf back to actual length
            buf.truncate(bit_index / 8);
        } else {
            // resize buf back to actual length
            buf.truncate(bit_index / 8);
        }

        buf
    }

    /// construct `MagneticFieldStrength2` from buffer
    pub fn from_bytes(buf: &[u8]) -> Self {
        // sensor id is 1 byte
        let sensor_id = buf[0];

        let bits = buf.view_bits::<Msb0>();

        let mut bit_index = 8;

        // load field
        let mut magnetic_field_ga = [0_f32; 3];
        let (_head, rest) = bits.split_at(bit_index);
        for (slot, f) in rest.chunks(16).zip(magnetic_field_ga.iter_mut()) {
            let v: u16 = slot.load_le();
            let sf = f16 { bits: v };
            *f = sf.to_f32();
        }
        bit_index += 3 * 16;

        // covariance (can be missing if tail-array optimization)
        let mut magnetic_field_covariance = Covariance { data: Vec::new() };
        if buf.len() > bit_index / 8 {
            let (_head, rest) = bits.split_at(bit_index);
            let mut itr = rest.chunks(4);
            // skip pad
            itr.next();
            let len = itr.next().unwrap().load_le();
            bit_index += 8;
            if len != 0 {
                let (_head, rest) = bits.split_at(bit_index);
                for (i, slot) in rest.chunks(16).enumerate() {
                    if i == len {
                        break;
                    }
                    let v: u16 = slot.load_le();
                    let sf = f16 { bits: v };
                    magnetic_field_covariance.data.push(sf.to_f32()).ok();
                }
            }
        }
        Self {
            sensor_id,
            magnetic_field_ga,
            magnetic_field_covariance,
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

/// Maximum size of the Vec for Rgb565
pub const UAVCAN_EQUIPMENT_INDICATION_RGB565_MAX_SIZE: usize = 2;

/// https://github.com/dronecan/DSDL/blob/master/uavcan/equipment/indication/RGB565.uavcan
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(Format))]
pub struct Rgb565 {
    pub red: u8,
    pub green: u8,
    pub blue: u8,
}

impl Rgb565 {
    /// serialize `Rgb565` to buffer
    pub fn to_bytes(&self) -> [u8; 2] {
        let mut buf = [0_u8; 2];
        let green = self.green & 0x3f;
        buf[0] = (self.red & 0x1f) << 3 | (green & 0x38) >> 3;
        buf[1] = (green & 0x7) << 5 | (self.blue & 0x1f);
        buf
    }

    /// extract `Rgb565` from buffer
    pub fn from_bytes(buf: &[u8]) -> Self {
        Self {
            red: (buf[0] & 0xf8) >> 3,
            green: ((buf[0] & 0x7) << 3) | (buf[1] >> 5),
            blue: (buf[1] & 0x1f),
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

/// https://github.com/dronecan/DSDL/blob/master/uavcan/equipment/indication/SingleLightCommand.uavcan
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(Format))]
#[repr(u8)]
pub enum LightId {
    Other(u8),
    AntiCollision = 246,
    RightOfWay = 247,
    Strobe = 248,
    Wing = 249,
    Logo = 250,
    Taxi = 251,
    TurnOff = 252,
    TakeOff = 253,
    Landing = 254,
    Formation = 255,
}

impl From<u8> for LightId {
    fn from(val: u8) -> Self {
        match val {
            246 => LightId::AntiCollision,
            247 => LightId::RightOfWay,
            248 => LightId::Strobe,
            249 => LightId::Wing,
            250 => LightId::Logo,
            251 => LightId::Taxi,
            252 => LightId::TurnOff,
            253 => LightId::TakeOff,
            254 => LightId::Landing,
            255 => LightId::Formation,
            0..=245 => LightId::Other(val),
        }
    }
}

impl From<LightId> for u8 {
    fn from(val: LightId) -> u8 {
        match val {
            LightId::AntiCollision => 246,
            LightId::RightOfWay => 247,
            LightId::Strobe => 248,
            LightId::Wing => 249,
            LightId::Logo => 250,
            LightId::Taxi => 251,
            LightId::TurnOff => 252,
            LightId::TakeOff => 253,
            LightId::Landing => 254,
            LightId::Formation => 255,
            LightId::Other(n) => n,
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

/// Maximum size of the Vec for SingleLightCommand
pub const UAVCAN_EQUIPMENT_INDICATION_SINGLELIGHTCOMMAND_SIZE: usize = 3;

/// https://github.com/dronecan/DSDL/blob/master/uavcan/equipment/indication/SingleLightCommand.uavcan
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(Format))]
pub struct SingleLightCommand {
    pub light_id: LightId,
    pub color: Rgb565,
}

impl SingleLightCommand {
    /// serialize `SingleLightCommand` to buffer
    pub fn to_bytes(&self) -> [u8; UAVCAN_EQUIPMENT_INDICATION_SINGLELIGHTCOMMAND_SIZE] {
        let mut buf = [0_u8; UAVCAN_EQUIPMENT_INDICATION_SINGLELIGHTCOMMAND_SIZE];
        buf[0] = self.light_id.clone().into();
        buf[1..].clone_from_slice(&self.color.to_bytes());
        buf
    }

    /// extract `SingleLightCommand` from buffer
    pub fn from_bytes(buf: &[u8]) -> Self {
        Self {
            light_id: buf[0].into(),
            color: Rgb565::from_bytes(&buf[1..UAVCAN_EQUIPMENT_INDICATION_SINGLELIGHTCOMMAND_SIZE]),
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

/// Maximum size of the Vec for LightsCommand
pub const UAVCAN_EQUIPMENT_INDICATION_LIGHTSCOMMAND_MAX_SIZE: usize =
    UAVCAN_EQUIPMENT_INDICATION_SINGLELIGHTCOMMAND_SIZE * 20;

///
/// https://github.com/dronecan/DSDL/blob/master/uavcan/equipment/indication/LightsCommand.uavcan
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(Format))]
pub struct LightsCommand {
    pub data: Vec<SingleLightCommand, 20>,
}

impl LightsCommand {
    /// serialize `LightsCommand` to buffer
    pub fn to_bytes(&self) -> Vec<u8, UAVCAN_EQUIPMENT_INDICATION_LIGHTSCOMMAND_MAX_SIZE> {
        let mut buf = Vec::new();
        for cmd in self.data.iter() {
            buf.extend_from_slice(&cmd.to_bytes()).ok();
        }
        buf
    }

    /// construct `LightsCommand` from buffer
    pub fn from_bytes(buf: &[u8]) -> Self {
        let mut data = Vec::new();
        for buf in buf.chunks(UAVCAN_EQUIPMENT_INDICATION_SINGLELIGHTCOMMAND_SIZE) {
            let cmd = SingleLightCommand::from_bytes(buf);
            data.push(cmd).ok();
        }
        Self { data }
    }
}

////////////////////////////////////////////////////////////////////////////////

#[cfg(test)]
mod test {
    use super::*;
    use test_log::test;

    #[test]
    fn test_ahrs_solution_no_covar() {
        let sol1 = AhrsSolution {
            uavcan_timestamp: 1,
            orientation_xyzw: [0.0, 1.0, 2.0, 3.0],
            orientation_covariance: Covariance::default(),
            angular_velocity: [0.0, 1.0, 2.0],
            angular_velocity_covariance: Covariance::default(),
            linear_acceleration: [0.0, 1.0, 2.0],
            linear_acceleration_covariance: Covariance::default(),
        };
        let sol1_buf = sol1.to_bytes();
        assert_eq!(sol1_buf.len(), 29);
        let sol2 = AhrsSolution::from_bytes(sol1_buf.as_slice());
        assert_eq!(sol2, sol1);
    }

    #[test]
    fn test_ahrs_solution_covar() {
        let sol1 = AhrsSolution {
            uavcan_timestamp: 1,
            orientation_xyzw: [0.0, 1.0, 2.0, 3.0],
            orientation_covariance: Covariance {
                data: Vec::from_slice(&[0.0, 1.0, 2.0]).unwrap(),
            },
            angular_velocity: [0.0, 1.0, 2.0],
            angular_velocity_covariance: Covariance {
                data: Vec::from_slice(&[0.0, 1.0, 2.0]).unwrap(),
            },
            linear_acceleration: [0.0, 1.0, 2.0],
            linear_acceleration_covariance: Covariance {
                data: Vec::from_slice(&[0.0, 1.0, 2.0]).unwrap(),
            },
        };
        let sol1_buf = sol1.to_bytes();
        assert_eq!(sol1_buf.len(), 48);
        let sol2 = AhrsSolution::from_bytes(sol1_buf.as_slice());
        assert_eq!(sol2, sol1);
    }

    #[test]
    fn test_software_version() {
        let sw1 = SoftwareVersion {
            major: 1,
            minor: 1,
            optional_field_flags: 0x3,
            vcs_commit: 2344,
            image_crc: u64::MAX,
        };
        let buf1 = sw1.to_bytes();
        let sw2 = SoftwareVersion::from_bytes(&buf1);
        assert_eq!(sw1, sw2);
    }

    #[test]
    fn test_hardware_version() {
        let hw1 = HardwareVersion {
            major: 2,
            minor: 2,
            unique_id: [
                0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9, 0xa, 0xb, 0xc, 0xd, 0xe, 0xf, 0x10,
            ],
            certificate_of_authority: None,
        };
        let buf1 = hw1.to_bytes();
        let hw2 = HardwareVersion::from_bytes(&buf1);
        assert_eq!(hw1, hw2);
        let mut coa = Vec::new();
        let _ = coa.resize_default(5);
        coa[0..5].copy_from_slice(&[0x1, 0x2, 0x3, 0x4, 0x5]);
        let hw2 = HardwareVersion {
            major: 2,
            minor: 2,
            unique_id: [
                0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9, 0xa, 0xb, 0xc, 0xd, 0xe, 0xf, 0x10,
            ],
            certificate_of_authority: Some(coa),
        };
        let buf1 = hw2.to_bytes();
        let hw3 = HardwareVersion::from_bytes(&buf1);
        assert_eq!(hw2, hw3);
    }

    #[test]
    fn test_magnetic_field_strength2() {
        let field = MagneticFieldStrength2 {
            sensor_id: 22,
            magnetic_field_ga: [0.0, 1.0, 2.0],
            magnetic_field_covariance: Covariance { data: Vec::new() },
        };
        let field_buf = field.to_bytes();
        assert_eq!(
            field_buf.len(),
            UAVCAN_EQUIPMENT_AHRS_MAGNETIC_FIELD_STRENGTH2_MAX_SIZE - 19
        );
        let field2 = MagneticFieldStrength2::from_bytes(field_buf.as_slice());
        assert_eq!(field2, field);
    }

    #[test]
    fn test_rgb565() {
        let col1 = Rgb565 {
            red: 0x15,
            green: 0x2a,
            blue: 0x15,
        };
        let buf = col1.to_bytes();
        assert_eq!(buf, [0xad, 0x55]);
        let col1back = Rgb565::from_bytes(&buf);
        assert_eq!(col1, col1back);
    }

    #[test]
    fn test_singlelightcommand() {
        let cmd1 = SingleLightCommand {
            light_id: LightId::Wing,
            color: Rgb565 {
                red: 0x15,
                green: 0x2a,
                blue: 0x15,
            },
        };
        let buf = cmd1.to_bytes();
        assert_eq!(buf, [249, 0xad, 0x55]);
        let cmd1back = SingleLightCommand::from_bytes(&buf);
        assert_eq!(cmd1, cmd1back);
    }

    #[test]
    fn test_lightscommand() {
        let mut cmd1 = LightsCommand { data: Vec::new() };
        let lc1 = SingleLightCommand {
            light_id: LightId::Wing,
            color: Rgb565 {
                red: 0x15,
                green: 0x2a,
                blue: 0x15,
            },
        };
        cmd1.data.push(lc1.clone()).unwrap();
        let buf = cmd1.to_bytes();
        assert_eq!(buf.len(), 3);
        assert_eq!(buf[0..3], [249, 0xad, 0x55]);
        let cmd1back = LightsCommand::from_bytes(&buf);
        assert_eq!(cmd1, cmd1back);
        assert_eq!(cmd1.data[0], lc1);
    }

    #[test]
    fn test_executeopcode_request() {
        let op1 = ExecuteOpcodeRequest {
            opcode: OpcodeType::Erase,
            argument: 224,
        };
        let buf = op1.to_bytes();
        assert!(buf.len() == UAVCAN_PROTOCOL_EXECUTEOPCODE_REQUEST_SIZE);
        let op1back = ExecuteOpcodeRequest::from_bytes(&buf);
        assert_eq!(op1, op1back);

        let op2 = ExecuteOpcodeRequest {
            opcode: OpcodeType::Erase,
            argument: -224,
        };
        let buf = op2.to_bytes();
        assert!(buf.len() == UAVCAN_PROTOCOL_EXECUTEOPCODE_REQUEST_SIZE);
        let op2back = ExecuteOpcodeRequest::from_bytes(&buf);
        assert_eq!(op2, op2back);
    }

    #[test]
    fn test_executeopcode_response() {
        let op1 = ExecuteOpcodeResponse {
            error_code: None,
            ok: true,
        };
        let buf = op1.to_bytes();
        assert!(buf.len() == UAVCAN_PROTOCOL_EXECUTEOPCODE_RESPONSE_SIZE);
        let op1back = ExecuteOpcodeResponse::from_bytes(&buf);
        assert_eq!(op1, op1back);
        let op2 = ExecuteOpcodeResponse {
            error_code: Some(-4),
            ok: false,
        };
        let buf = op2.to_bytes();
        assert!(buf.len() == UAVCAN_PROTOCOL_EXECUTEOPCODE_RESPONSE_SIZE);
        let op2back = ExecuteOpcodeResponse::from_bytes(&buf);
        assert_eq!(op2, op2back);
    }
}
