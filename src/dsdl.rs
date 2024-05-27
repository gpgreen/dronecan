//! This module contains types associated with specific Dronecan messages.

use crate::{f16, MsgType};
use bitflags::bitflags;
use bitvec::prelude::*;
use heapless::{String, Vec};

#[cfg(feature = "defmt")]
use defmt::*;
//#[cfg(not(feature = "defmt"))]
//use log::*;

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
            _ => panic!("max value of NodeHealth is 3"),
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
            _ => panic!("value out of range for NodeMode"),
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
    /// serialize `NodeStatus` to a buffer
    pub fn to_bytes(&self) -> [u8; PAYLOAD_SIZE_NODE_STATUS] {
        let mut result = [0; PAYLOAD_SIZE_NODE_STATUS];

        result[..4].clone_from_slice(&self.uptime_sec.to_le_bytes());

        // Health and mode. Submode is reserved by the spec for future use,
        // but is currently not used.
        result[4] = ((self.health as u8) << 6) | ((self.mode as u8) << 3);

        result[5..7].clone_from_slice(&self.vendor_specific_status_code.to_le_bytes());

        result
    }
    /// construct `NodeStatus` from a buffer
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
    /// Create a `HardwareVersion`
    pub fn new(major: u8, minor: u8, unique_id: &[u8; 16]) -> Self {
        HardwareVersion {
            major,
            minor,
            unique_id: *unique_id,
            certificate_of_authority: None,
        }
    }

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

bitflags! {
    /// Optional field flags in `SoftwareVersion`
    #[cfg_attr(feature = "defmt", derive(Format))]
    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
    pub struct FieldFlags: u8 {
        /// The value `VCS_COMMIT` at bit position `0`
        const VCS_COMMIT = 0b0000_0001;
        /// The value `IMAGE_CRC` at bit position `1`
        const IMAGE_CRC = 0b0000_0010;
    }
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/SoftwareVersion.uavcan
/// Generic software version information.
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct SoftwareVersion {
    pub major: u8,
    pub minor: u8,
    /// `FieldFlags` mask indicates which optional fields (see below) are set.
    pub optional_field_flags: FieldFlags,
    /// VCS commit hash or revision number, e.g. git short commit hash. Optional.
    pub vcs_commit: u32,
    /// The value of an arbitrary hash function applied to the firmware image.
    pub image_crc: u64,
}

impl SoftwareVersion {
    /// Create a `SoftwareVersion`
    pub fn new(major: u8, minor: u8) -> Self {
        SoftwareVersion {
            major,
            minor,
            optional_field_flags: FieldFlags::empty(),
            vcs_commit: 0,
            image_crc: 0,
        }
    }

    /// Set the vcs_commit and image crc
    /// if the value is zero, that member will be unset
    pub fn set_vcs_and_crc(&mut self, commit: u32, crc: u64) {
        if commit != 0 {
            self.optional_field_flags |= FieldFlags::VCS_COMMIT;
        } else {
            self.optional_field_flags &= !FieldFlags::VCS_COMMIT;
        }
        if crc != 0 {
            self.optional_field_flags |= FieldFlags::IMAGE_CRC;
        } else {
            self.optional_field_flags &= !FieldFlags::IMAGE_CRC;
        }
        self.image_crc = crc;
    }

    /// serialize `SoftwareVersion` to a buffer
    pub fn to_bytes(&self) -> [u8; PAYLOAD_SIZE_SOFTWARE_VERSION] {
        let mut result = [0; 15];

        result[0] = self.major;
        result[1] = self.minor;
        result[2] = self.optional_field_flags.bits();
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
            optional_field_flags: FieldFlags::from_bits_truncate(buf[2]),
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

/// Maximum size of the Vec for NumericValue (variable)
pub const UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_MAX_SIZE: usize = 9;
pub const VALUE_NUMERIC_TAG_BIT_LEN: usize = 2;

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/param/NumericValue.uavcan
/// `uavcan.protocol.param.NumericValue`
/// 2-bit tag with 6-bit prefix for alignment
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug, Clone, Copy, PartialEq)]
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

    /// Convert to bits in a buffer, return the number of bits written
    pub fn to_bits(&self, bits: &mut BitSlice<u8, Lsb0>, start_bit_pos: usize) -> usize {
        let val_start_i = start_bit_pos + VALUE_NUMERIC_TAG_BIT_LEN; // bits
        bits[start_bit_pos..val_start_i].store(self.tag());

        match self {
            Self::Empty => val_start_i,
            Self::Integer(v) => {
                bits[val_start_i..val_start_i + 64].store_le(*v);
                val_start_i + 64
            }
            Self::Real(v) => {
                // bitvec doesn't support floats.
                for (i, byte) in v.to_le_bytes().iter().enumerate() {
                    let start = val_start_i + i * 8;
                    bits[start..start + 8].store(*byte);
                }
                val_start_i + 32
            }
        }
    }

    /// Construct from a buffer starting from position, return Self and number of bits consumed
    pub fn from_bits(bits: &BitSlice<u8, Lsb0>, start_bit_pos: usize) -> (Self, usize) {
        let val_start_i = start_bit_pos + VALUE_NUMERIC_TAG_BIT_LEN; // bits
        let tag: u8 = bits[start_bit_pos..val_start_i].load();
        let (_head, rest) = bits.split_at(val_start_i);

        match tag {
            0 => (NumericValue::Empty, val_start_i),
            1 => {
                let (chunk, _rest) = rest.split_at(64);
                (NumericValue::Integer(chunk.load_le()), val_start_i + 64)
            }
            2 => {
                // bitvec doesn't support floats.
                let mut b = [0; 4];
                for (byte, chunk) in b.iter_mut().zip(rest.chunks(8)) {
                    *byte = chunk.load();
                }
                (NumericValue::Real(f32::from_le_bytes(b)), val_start_i + 32)
            }
            _ => panic!("tag has max value of 2, was {}", tag),
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

/// Maximum size of the Vec for Value (variable)
pub const UAVCAN_PROTOCOL_PARAM_VALUE_MAX_SIZE: usize = 130;
pub const VALUE_TAG_BIT_LEN: usize = 3;

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/param/Value.uavcan
/// `uavcan.protocol.param.Value`
/// 3-bit tag with 5-bit prefix for alignment.
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug, Clone, PartialEq)]
pub enum Value {
    Empty,
    Integer(i64),
    Real(f32),
    Boolean(bool), // u8 repr
    /// Max length of 128 bytes.
    String(heapless::String<128>),
}

impl Default for Value {
    fn default() -> Self {
        Self::Empty
    }
}

impl Value {
    fn tag(&self) -> u8 {
        match self {
            Self::Empty => 0,
            Self::Integer(_) => 1,
            Self::Real(_) => 2,
            Self::Boolean(_) => 3,
            Self::String(_) => 4,
        }
    }

    /// Convert to bits in a buffer, return the number of bits written
    pub fn to_bits(&self, bits: &mut BitSlice<u8, Lsb0>, start_bit_pos: usize) -> usize {
        let val_start_i = start_bit_pos + VALUE_TAG_BIT_LEN; // bits
        bits[start_bit_pos..val_start_i].store(self.tag());

        match self {
            Self::Empty => val_start_i,
            Self::Integer(v) => {
                bits[val_start_i..val_start_i + 64].store_le(*v);
                val_start_i + 64
            }
            Self::Real(v) => {
                // bitvec doesn't support floats.
                for (i, byte) in v.to_le_bytes().iter().enumerate() {
                    let start = val_start_i + i * 8;
                    bits[start..start + 8].store(*byte);
                }
                val_start_i + 32
            }
            Self::Boolean(v) => {
                let byte = if *v { 1 } else { 0 };
                bits[val_start_i..val_start_i + 8].store(byte);
                val_start_i + 8
            }
            Self::String(s) => {
                let (_head, rest) = bits.split_at_mut(val_start_i + 8);
                let mut slen = 0;
                for (byte, chunk) in s.as_bytes().iter().zip(rest.chunks_mut(8)) {
                    chunk.store(*byte);
                    slen += 1;
                    if slen == 128 {
                        break;
                    }
                }
                // now store the length
                bits[val_start_i..val_start_i + 8].store(slen);
                val_start_i + 8 + 8 * slen
            }
        }
    }

    /// Construct from a buffer starting from position, return Self and number of bits consumed
    pub fn from_bits(bits: &BitSlice<u8, Lsb0>, start_bit_pos: usize) -> (Self, usize) {
        let val_start_i = start_bit_pos + VALUE_TAG_BIT_LEN; // bits
        let tag: u8 = bits[start_bit_pos..val_start_i].load();

        match tag {
            0 => (Value::Empty, val_start_i),
            1 => {
                let (_head, rest) = bits.split_at(val_start_i);
                let (chunk, _rest) = rest.split_at(64);
                (Value::Integer(chunk.load_le()), val_start_i + 64)
            }
            2 => {
                // bitvec doesn't support floats.
                let mut b = [0; 4];
                let (_head, rest) = bits.split_at(val_start_i);
                for (byte, chunk) in b.iter_mut().zip(rest.chunks(8)) {
                    *byte = chunk.load();
                }
                (Value::Real(f32::from_le_bytes(b)), val_start_i + 32)
            }
            3 => {
                let byte: u8 = bits[val_start_i..val_start_i + 8].load();
                let v = byte != 0;
                (Value::Boolean(v), val_start_i + 8)
            }
            4 => {
                let mut buf: Vec<u8, 128> = Vec::new();
                let (_head, rest) = bits.split_at(val_start_i + 8);
                let slen = bits[val_start_i..val_start_i + 8].load();
                let mut count = 0;
                for ch in rest.chunks(8) {
                    let c: u8 = ch.load();
                    buf.push(c).unwrap();
                    count += 1;
                    if count == slen {
                        break;
                    }
                }
                let s = String::from_utf8(buf).unwrap();
                (Value::String(s), val_start_i + 8 + 8 * slen as usize)
            }
            _ => panic!("tag has max value of 4, was {}", tag),
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

pub const UAVCAN_PROTOCOL_GETSET_REQUEST_MAX_SIZE: usize = 224;

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/param/11.GetSet.uavcan
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug, Clone, PartialEq)]
pub struct GetSetRequest {
    pub index: u16, // 13 bits
    /// If set - parameter will be assigned this value, then the new value will be returned.
    /// If not set - current parameter value will be returned.
    pub value: Value,
    /// Name of the parameter; always preferred over index if nonempty.
    pub name: heapless::String<92>,
}

impl GetSetRequest {
    /// serialize `GetSetRequest` to buffer
    pub fn to_bytes(&self) -> Vec<u8, UAVCAN_PROTOCOL_GETSET_REQUEST_MAX_SIZE> {
        let mut buf = Vec::new();
        buf.resize_default(UAVCAN_PROTOCOL_GETSET_REQUEST_MAX_SIZE)
            .unwrap();
        let bits = buf.view_bits_mut::<Lsb0>();
        bits[0..13].store_le(self.index);
        let current_offset = self.value.to_bits(bits, 13);
        let name_vec = self.name.clone();
        let name_buf = name_vec.as_bytes();
        let slen = name_buf.len();
        let (_head, rest) = bits.split_at_mut(current_offset);
        for (byte, chunk) in name_buf.iter().zip(rest.chunks_mut(8)) {
            chunk.store(*byte);
        }
        buf.truncate(current_offset / 8 + slen);
        buf
    }

    /// construct `GetSetRequest` from buffer
    pub fn from_bytes(buf: &[u8]) -> Self {
        let bits = buf.view_bits::<Lsb0>();

        // the tag
        let tag_start_i = 13;
        let index: u16 = bits[0..tag_start_i].load_le();

        // the value
        let (value, current_i) = Value::from_bits(bits, tag_start_i);

        // the name, with tao
        let (_head, rest) = bits.split_at(current_i);
        let mut v = Vec::new();
        for ch in rest.chunks(8) {
            v.push(ch.load()).unwrap();
        }
        let name = String::from_utf8(v).unwrap();

        Self { index, value, name }
    }
}

pub const UAVCAN_PROTOCOL_GETSET_RESPONSE_MAX_SIZE: usize = 371;

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/param/11.GetSet.uavcan
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug, Clone, PartialEq)]
pub struct GetSetResponse {
    /// For set requests, it should contain the actual parameter value after the set request was
    /// executed. The objective is to let the client know if the value could not be updated, e.g.
    /// due to its range violation, etc.
    pub value: Value,
    pub default_value: Value,
    pub max_value: NumericValue,
    pub min_value: NumericValue,
    /// Empty name (and/or empty value) in response indicates that there is no such parameter.
    pub name: String<92>,
}

impl GetSetResponse {
    /// serialize `GetSetResponse` to buffer
    pub fn to_bytes(&self) -> Vec<u8, UAVCAN_PROTOCOL_GETSET_RESPONSE_MAX_SIZE> {
        let mut buf = Vec::new();
        buf.resize_default(UAVCAN_PROTOCOL_GETSET_RESPONSE_MAX_SIZE)
            .unwrap();
        let bits = buf.view_bits_mut::<Lsb0>();
        let bit_offset = self.value.to_bits(bits, 5);
        let bit_offset = self.default_value.to_bits(bits, bit_offset + 5);
        let bit_offset = self.max_value.to_bits(bits, bit_offset + 6);
        let bit_offset = self.min_value.to_bits(bits, bit_offset + 6);

        // store the name
        let name_vec = self.name.clone();
        let name_buf = name_vec.as_bytes();
        let slen = name_buf.len();
        let (_head, rest) = bits.split_at_mut(bit_offset);
        for (byte, chunk) in name_buf.iter().zip(rest.chunks_mut(8)) {
            chunk.store(*byte);
        }
        buf.truncate(bit_offset / 8 + slen);
        buf
    }

    /// construct `GetSetResponse` from buffer
    pub fn from_bytes(buf: &[u8]) -> Self {
        let bits = buf.view_bits::<Lsb0>();
        let (value, bit_offset) = Value::from_bits(bits, 5);
        let (default_value, bit_offset) = Value::from_bits(bits, 5 + bit_offset);
        let (max_value, bit_offset) = NumericValue::from_bits(bits, 6 + bit_offset);
        let (min_value, bit_offset) = NumericValue::from_bits(bits, 6 + bit_offset);
        // get the name, with tao
        let (_head, rest) = bits.split_at(bit_offset);
        let mut v = Vec::new();
        for ch in rest.chunks(8) {
            v.push(ch.load()).unwrap();
        }
        let name = String::from_utf8(v).unwrap();
        Self {
            value,
            default_value,
            max_value,
            min_value,
            name,
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/DataTypeKind.uavcan
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum DataTypeKind {
    Service = 0,
    Message = 1,
}

pub const UAVCAN_PROTOCOL_GETDATATYPEINFO_REQUEST_MAX_SIZE: usize = 84;

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/2.GetDataTypeInfo.uavcan
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug, Clone, PartialEq)]
pub struct GetDataTypeInfoRequest {
    pub id: u16,
    pub kind: DataTypeKind,
    pub name: String<80>,
}

impl GetDataTypeInfoRequest {
    /// serialize `GetDataTypeInfoRequest` to buffer
    pub fn to_bytes(&self) -> Vec<u8, UAVCAN_PROTOCOL_GETDATATYPEINFO_REQUEST_MAX_SIZE> {
        let mut buf = Vec::new();
        buf.extend_from_slice(&self.id.to_le_bytes()).unwrap();
        let kb = if self.kind == DataTypeKind::Service {
            0
        } else {
            1
        };
        buf.push(kb).unwrap();
        let name_vec = self.name.clone();
        let name_buf = name_vec.as_bytes();
        buf.extend_from_slice(name_buf).unwrap();
        buf
    }

    /// construct `GetDataTypeInfoRequest` from buffer
    pub fn from_bytes(buf: &[u8]) -> Self {
        let bits = buf.view_bits::<Lsb0>();

        let id = bits[0..16].load_le();
        let dtk: u8 = bits[16..24].load();
        let kind = if dtk == 0 {
            DataTypeKind::Service
        } else {
            DataTypeKind::Message
        };
        // get the name, with tao
        let (_head, rest) = bits.split_at(24);
        let mut v = Vec::new();
        for ch in rest.chunks(8) {
            v.push(ch.load()).unwrap();
        }
        let name = String::from_utf8(v).unwrap();

        Self { id, kind, name }
    }
}

pub const UAVCAN_PROTOCOL_GETDATATYPEINFO_RESPONSE_MAX_SIZE: usize = 93;
pub const UAVCAN_PROTOCOL_GETDATATYPEINFO_SIGNATURE: u64 = 0x1B283338A7BED2D8;

bitflags! {
    /// Data Type Info flags in `GetDataTypeInfoResponse`
    #[cfg_attr(feature = "defmt", derive(Format))]
    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
    pub struct DataTypeFlags: u8 {
        /// The value `KNOWN` at bit position `0`
        const KNOWN = 0b0000_0001;
        /// The value `SUBSCRIBED` at bit position `1`
        const SUBSCRIBED = 0b0000_0010;
        /// The value `PUBLISHING` at bit position `2`
        const PUBLISHING = 0b0000_0100;
        /// The value `SERVING` at bit position `3`
        const SERVING = 0b0000_1000;
    }
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/2.GetDataTypeInfo.uavcan
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug, Clone, PartialEq)]
pub struct GetDataTypeInfoResponse {
    /// UAVCAN signature of the Data Type
    pub signature: u64,
    /// The Id of the Data Type
    pub id: u16,
    /// What Kind of Data Type
    pub kind: DataTypeKind,
    /// Flags describing Data Type
    pub flags: DataTypeFlags,
    /// Name of the Data Type
    pub name: String<80>,
}

impl GetDataTypeInfoResponse {
    /// serialize `GetDataTypeInfoResponse` to buffer
    pub fn to_bytes(&self) -> Vec<u8, UAVCAN_PROTOCOL_GETDATATYPEINFO_REQUEST_MAX_SIZE> {
        let mut buf = Vec::new();
        buf.extend_from_slice(&self.signature.to_le_bytes())
            .unwrap();
        buf.extend_from_slice(&self.id.to_le_bytes()).unwrap();
        let kb = if self.kind == DataTypeKind::Service {
            0
        } else {
            1
        };
        buf.push(kb).unwrap();
        buf.push(self.flags.bits()).unwrap();
        let name_vec = self.name.clone();
        let name_buf = name_vec.as_bytes();
        buf.extend_from_slice(name_buf).unwrap();
        buf
    }

    /// construct `GetDataTypeInfoResponse` from buffer
    pub fn from_bytes(buf: &[u8]) -> Self {
        let bits = buf.view_bits::<Lsb0>();
        let signature = bits[0..64].load_le();
        let id = bits[64..80].load_le();
        let kind = if buf[10] == 0 {
            DataTypeKind::Service
        } else {
            DataTypeKind::Message
        };
        let flags = DataTypeFlags::from_bits_truncate(buf[11]);
        let mut v = Vec::new();
        v.extend_from_slice(&buf[12..]).unwrap();
        let name = String::from_utf8(v).unwrap();

        Self {
            signature,
            id,
            kind,
            flags,
            name,
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

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
            let bits = result.view_bits_mut::<Lsb0>();

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

        let bits = buf.view_bits_mut::<Lsb0>();

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
        let bits = buf.view_bits::<Lsb0>();

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

        let bits = buf.view_bits_mut::<Lsb0>();

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

        let bits = buf.view_bits::<Lsb0>();

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
            optional_field_flags: FieldFlags::VCS_COMMIT | FieldFlags::IMAGE_CRC,
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

    #[test]
    fn test_numeric_value() {
        // empty
        let nv1 = NumericValue::Empty;
        let mut buf = [0; UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_MAX_SIZE];
        let bits = buf.view_bits_mut::<Lsb0>();
        let offset = nv1.to_bits(bits, 0);
        assert_eq!(offset, 2);
        let (nv1back, new_offset) = NumericValue::from_bits(bits, 0);
        assert_eq!(nv1, nv1back);
        assert_eq!(new_offset, 2);

        // integer
        let nv2 = NumericValue::Integer(-2);
        let mut buf = [0; UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_MAX_SIZE];
        let bits = buf.view_bits_mut::<Lsb0>();
        let offset = nv2.to_bits(bits, 0);
        assert_eq!(offset, 2 + 64);
        let (nv2back, new_offset) = NumericValue::from_bits(bits, 0);
        assert_eq!(nv2, nv2back);
        assert_eq!(new_offset, 2 + 64);

        // real
        let nv3 = NumericValue::Real(-2.34);
        let mut buf = [0; UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_MAX_SIZE];
        let bits = buf.view_bits_mut::<Lsb0>();
        let offset = nv3.to_bits(bits, 0);
        assert_eq!(offset, 2 + 32);
        let (nv3back, new_offset) = NumericValue::from_bits(bits, 0);
        assert_eq!(nv3, nv3back);
        assert_eq!(new_offset, 2 + 32);
    }

    #[test]
    fn test_value() {
        // empty
        let v1 = Value::Empty;
        let mut buf = [0; UAVCAN_PROTOCOL_PARAM_VALUE_MAX_SIZE];
        let bits = buf.view_bits_mut::<Lsb0>();
        let offset = v1.to_bits(bits, 0);
        assert_eq!(offset, 3);
        let (v1back, new_offset) = Value::from_bits(bits, 0);
        assert_eq!(v1, v1back);
        assert_eq!(new_offset, 3);

        // integer
        let v2 = Value::Integer(-2);
        let mut buf = [0; UAVCAN_PROTOCOL_PARAM_VALUE_MAX_SIZE];
        let bits = buf.view_bits_mut::<Lsb0>();
        let offset = v2.to_bits(bits, 0);
        assert_eq!(offset, 3 + 64);
        let (v2back, new_offset) = Value::from_bits(bits, 0);
        assert_eq!(v2, v2back);
        assert_eq!(new_offset, 3 + 64);

        // integer
        let v2a = Value::Integer(i64::MIN);
        let mut buf = [0; UAVCAN_PROTOCOL_PARAM_VALUE_MAX_SIZE];
        let bits = buf.view_bits_mut::<Lsb0>();
        let offset = v2a.to_bits(bits, 0);
        assert_eq!(offset, 3 + 64);
        let (v2aback, new_offset) = Value::from_bits(bits, 0);
        assert_eq!(v2a, v2aback);
        assert_eq!(new_offset, 3 + 64);

        // real
        let v3 = Value::Real(-2.34);
        let mut buf = [0; UAVCAN_PROTOCOL_PARAM_VALUE_MAX_SIZE];
        let bits = buf.view_bits_mut::<Lsb0>();
        let offset = v3.to_bits(bits, 0);
        assert_eq!(offset, 3 + 32);
        let (v3back, new_offset) = Value::from_bits(bits, 0);
        assert_eq!(v3, v3back);
        assert_eq!(new_offset, 3 + 32);

        // boolean
        let v4 = Value::Boolean(true);
        let mut buf = [0; UAVCAN_PROTOCOL_PARAM_VALUE_MAX_SIZE];
        let bits = buf.view_bits_mut::<Lsb0>();
        let offset = v4.to_bits(bits, 0);
        assert_eq!(offset, 3 + 8);
        let (v4back, new_offset) = Value::from_bits(bits, 0);
        assert_eq!(v4, v4back);
        assert_eq!(new_offset, 3 + 8);

        // string
        let s: String<128> = String::try_from("this is a string").unwrap();
        let slen = s.len();
        let v5 = Value::String(s);
        let mut buf = [0; UAVCAN_PROTOCOL_PARAM_VALUE_MAX_SIZE];
        let bits = buf.view_bits_mut::<Lsb0>();
        let offset = v5.to_bits(bits, 0);
        assert_eq!(offset, 3 + 8 + 8 * slen);
        let (v5back, new_offset) = Value::from_bits(bits, 0);
        assert_eq!(v5, v5back);
        assert_eq!(new_offset, 3 + 8 + 8 * slen);
    }

    #[test]
    fn test_getset_request() {
        let gs1 = GetSetRequest {
            index: 0,
            value: Value::Empty,
            name: String::new(),
        };
        let buf = gs1.to_bytes();
        assert_eq!(buf.len(), 2);
        let gs1back = GetSetRequest::from_bytes(&buf);
        assert_eq!(gs1, gs1back);

        let gs2 = GetSetRequest {
            index: 1,
            value: Value::Integer(3),
            name: String::try_from("hello world").unwrap(),
        };
        let buf = gs2.to_bytes();
        assert_eq!(buf.len(), 21);
        let gs2back = GetSetRequest::from_bytes(&buf);
        assert_eq!(gs2, gs2back);

        let gs3 = GetSetRequest {
            index: 2,
            value: Value::Real(3.14),
            name: String::try_from("pi").unwrap(),
        };
        let buf = gs3.to_bytes();
        assert_eq!(buf.len(), 8);
        let gs3back = GetSetRequest::from_bytes(&buf);
        assert_eq!(gs3, gs3back);

        let gs4 = GetSetRequest {
            index: 3,
            value: Value::Boolean(false),
            name: String::try_from("pi is not equal to plancks constant").unwrap(),
        };
        let buf = gs4.to_bytes();
        assert_eq!(buf.len(), 38);
        let gs4back = GetSetRequest::from_bytes(&buf);
        assert_eq!(gs4, gs4back);
    }

    #[test]
    fn test_getset_response() {
        let gs1 = GetSetResponse {
            value: Value::Empty,
            default_value: Value::Empty,
            max_value: NumericValue::Empty,
            min_value: NumericValue::Empty,
            name: String::new(),
        };
        let buf = gs1.to_bytes();
        assert_eq!(buf.len(), 4);
        let gs1back = GetSetResponse::from_bytes(&buf);
        assert_eq!(gs1, gs1back);

        // max sizes of everything
        let nm = String::try_from("01234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678").unwrap();
        let nm2 = String::try_from(
            "01234567890123456789012345678901234567890123456789012345678901234567890123456789012",
        )
        .unwrap();
        let gs2 = GetSetResponse {
            value: Value::String(nm.clone()),
            default_value: Value::String(nm),
            max_value: NumericValue::Integer(i64::MAX),
            min_value: NumericValue::Integer(i64::MIN),
            name: nm2,
        };
        let buf = gs2.to_bytes();
        // BUGBUG: the max size doesn't seem correct
        //        assert_eq!(buf.len(), UAVCAN_PROTOCOL_GETSET_RESPONSE_MAX_SIZE);
        let gs2back = GetSetResponse::from_bytes(&buf);
        assert_eq!(gs2, gs2back);
    }

    #[test]
    fn test_getdatatypeinfo_request() {
        let gdt1 = GetDataTypeInfoRequest {
            id: 0,
            kind: DataTypeKind::Service,
            name: String::new(),
        };
        let buf = gdt1.to_bytes();
        assert_eq!(buf.len(), 3);
        let gdt1back = GetDataTypeInfoRequest::from_bytes(&buf);
        assert_eq!(gdt1, gdt1back);

        let gdt2 = GetDataTypeInfoRequest {
            id: 1,
            kind: DataTypeKind::Message,
            name: String::try_from("hello").unwrap(),
        };
        let buf = gdt2.to_bytes();
        assert_eq!(buf.len(), 8);
        let gdt2back = GetDataTypeInfoRequest::from_bytes(&buf);
        assert_eq!(gdt2, gdt2back);

        // max sizes of everything
        let gdt3 = GetDataTypeInfoRequest {
            id: u16::MAX,
            kind: DataTypeKind::Message,
            name: String::try_from(
                "01234567890123456789012345678901234567890123456789012345678901234567890123456789",
            )
            .unwrap(),
        };
        let buf = gdt3.to_bytes();
        assert_eq!(
            buf.len(),
            UAVCAN_PROTOCOL_GETDATATYPEINFO_REQUEST_MAX_SIZE - 1
        );
        let gdt3back = GetDataTypeInfoRequest::from_bytes(&buf);
        assert_eq!(gdt3, gdt3back);
    }

    #[test]
    fn test_getdatatypeinfo_response() {
        let gdt1 = GetDataTypeInfoResponse {
            signature: UAVCAN_PROTOCOL_GETDATATYPEINFO_SIGNATURE,
            id: 0,
            kind: DataTypeKind::Service,
            flags: DataTypeFlags::empty(),
            name: String::try_from("hello").unwrap(),
        };
        let buf = gdt1.to_bytes();
        assert_eq!(buf.len(), 17);
        let gdt1back = GetDataTypeInfoResponse::from_bytes(&buf);
        assert_eq!(gdt1, gdt1back);
    }
}
