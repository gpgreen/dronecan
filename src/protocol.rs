use crate::PAYLOAD_SIZE_CONFIG_COMMON;

// Per DC spec.
pub const NODE_ID_MIN_VALUE: u8 = 1;
pub const NODE_ID_MAX_VALUE: u8 = 127;

/// This includes configuration data that we use on all nodes, and is not part of the official
/// DroneCAN spec.
#[cfg_attr(feature = "defmt", derive(Format))]
pub struct ConfigCommon {
    /// Used to distinguish between multiple instances of this device. Stored in
    /// flash. If dyanmic id allocation is enabled, this is overwritten, but passed as
    /// the desired ID. If not, it is the node ID.
    /// Defaults to 69.
    pub node_id: u8,
    /// If true, the `node_id` field is a desired ID; get ID from an allocator.
    /// if false, hard-set the node id field. Defaults to `true`.
    pub dynamic_id_allocation: bool,
    /// Ie, capable of 64-byte frame lens, vice 8.
    pub fd_mode: bool,
    // /// Kbps. If less than 1_000, arbitration and data bit rate are the same.
    // /// If greater than 1_000, arbitration bit rate stays at 1_000 due to protocol limits
    // /// while data bit rate is this value.
    // pub can_bitrate: CanBitrate,
}

impl Default for ConfigCommon {
    fn default() -> Self {
        Self {
            // Between 1 and 127. Initialize to 0; this is expected by AP and
            // Px4, where id is assigned through a node ID server.
            node_id: 69,
            dynamic_id_allocation: true,
            fd_mode: false,
            //            can_bitrate: CanBitrate::default(),
        }
    }
}

impl ConfigCommon {
    pub fn from_bytes(buf: &[u8]) -> Self {
        Self {
            node_id: buf[0],
            dynamic_id_allocation: buf[1] != 0,
            fd_mode: buf[2] != 0,
            //            can_bitrate: buf[3].try_into().unwrap_or_default(),
        }
    }

    pub fn to_bytes(&self) -> [u8; PAYLOAD_SIZE_CONFIG_COMMON] {
        let mut result = [0; PAYLOAD_SIZE_CONFIG_COMMON];

        result[0] = self.node_id;
        result[1] = self.dynamic_id_allocation as u8;
        result[2] = self.fd_mode as u8;
        //        result[3] = self.can_bitrate as u8;

        result
    }
}

/// Distinguish single and multi-part transfers. The inner value is the toggle value of the
/// previous frame.
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug, Clone, Copy)]
pub enum TransferComponent {
    SingleFrame,
    MultiStart,
    MultiMid(bool),
    MultiEnd(bool),
}

/// https://dronecan.github.io/Specification/4._CAN_bus_transport_layer/
/// Valid values for priority range from 0 to 31, inclusively, where 0 corresponds to highest priority
/// (and 31 corresponds to lowest priority).
/// In multi-frame transfers, the value of the priority field must be identical for all frames of the transfer.
///
/// Cyphal: (Transfer Priority spec section)[https://opencyphal.org/specification/Cyphal_Specification.pdf]:
/// Valid values for transfer priority range from 0 to 7, inclusively, where 0 corresponds to the highest priority, and
/// 7 corresponds to the lowest priority (according to the CAN bus arbitration policy).
/// In multi-frame transfers, the value of the priority field shall be identical for all frames of the transfer.
///
/// We use the Cyphal specification, due to its specificity.
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MsgPriority {
    Exceptional,
    Immediate,
    Fast,
    High,
    Nominal,
    Low,
    Slow,
    Optional,
    Other(u8),
}

impl MsgPriority {
    pub fn val(&self) -> u8 {
        match self {
            Self::Exceptional => 0,
            Self::Immediate => 1,
            Self::Fast => 2,
            Self::High => 3,
            Self::Nominal => 4,
            Self::Low => 5,
            Self::Slow => 6,
            Self::Optional => 7,
            Self::Other(val) => *val,
        }
    }

    pub fn from_val(val: u8) -> Self {
        match val {
            0 => Self::Exceptional,
            1 => Self::Immediate,
            2 => Self::Fast,
            3 => Self::High,
            4 => Self::Nominal,
            5 => Self::Low,
            6 => Self::Slow,
            _ => Self::Other(val),
        }
    }
}

/// Differentiates between messages and services
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum FrameType {
    Message,
    MessageAnon,
    Service(ServiceData),
}

/// Data present in services, but not messages.
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug, PartialEq, Copy, Clone)]
pub struct ServiceData {
    pub dest_node_id: u8, // 7 bits
    pub req_or_resp: RequestResponse,
}

#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug, Copy, Clone)]
pub struct TailByte {
    pub start_of_transfer: bool,
    pub end_of_transfer: bool,
    pub toggle: bool,
    pub transfer_id: u8,
}

impl TailByte {
    pub fn value(&self) -> u8 {
        // (DroneCAN):
        // For single-frame transfers, the value of this field is always 1.
        // For multi-frame transfers, the value of this field is 1 if the current frame is the first
        // frame of the transfer, and 0 otherwise.
        ((self.start_of_transfer as u8) << 7)
            // For single-frame transfers, the value of this field is always 1.
            // For multi-frame transfers, the value of this field is 1 if the current frame is the last
            // frame of the transfer, and 0 otherwise.
            | ((self.end_of_transfer as u8) << 6)
            // For single-frame transfers, the value of this field is always 0.
            // For multi-frame transfers, this field contains the value of the toggle bit. As specified
            // above this will alternate value between frames, starting at 0 for the first frame.
            | ((self.toggle as u8) << 5)
            | (self.transfer_id & 0b1_1111)
    }

    /// Pull start_of_transfer, end_of_transfer, and toggle flags from the tail byte. We don't
    /// convert to TransferComponent due to ambiguities in Single vs multi-mid.
    /// https://dronecan.github.io/Specification/4._CAN_bus_transport_layer/
    /// todo: Currently hard-coded for DroneCAN.
    pub fn from_value(val: u8) -> Self {
        let transfer_id = val & 0b1_1111;
        let toggle = (val >> 5) & 1;
        let end = (val >> 6) & 1;
        let start = (val >> 7) & 1;

        Self {
            transfer_id,
            toggle: toggle != 0,
            end_of_transfer: end != 0,
            start_of_transfer: start != 0,
        }
    }

    /// Construct a tail byte. See DroneCAN Spec, CAN bus transport layer.
    /// https://dronecan.github.io/Specification/4._CAN_bus_transport_layer/
    /// "The Data field of the CAN frame is shared between the following fields:
    /// - Transfer payload
    /// - 0 tail byte, which contains the following fields. Start of transfer (1 bit), End of transfer (1 bit)
    /// toggle bit (1 bit), Transfer id (5 bits)."
    pub fn new(transfer_component: TransferComponent, transfer_id: u8) -> Self {
        let (start_of_transfer, end_of_transfer, toggle) = match transfer_component {
            TransferComponent::SingleFrame => (true, true, false),
            TransferComponent::MultiStart => (true, false, false),
            TransferComponent::MultiMid(toggle_prev) => (false, false, !toggle_prev),
            TransferComponent::MultiEnd(toggle_prev) => (false, true, !toggle_prev),
        };

        TailByte {
            start_of_transfer,
            end_of_transfer,
            toggle,
            transfer_id,
        }
    }
}

#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum RequestResponse {
    Request = 1,
    Response = 0,
}

/// Construct a CAN ID. See DroneCAN Spec, CAN bus transport layer doc, "ID field" section.
/// https://dronecan.github.io/Specification/4._CAN_bus_transport_layer/
/// "DroneCAN uses only CAN 2.0B frame format (29-bit identifiers).
/// DroneCAN can share the same bus with other protocols based on CAN 2.0A (11-bit identifiers)."
///  This means we always use extended Id.
///
/// "In the case of a message broadcast transfer, the CAN ID field of every frame of the transfer will contain the following fields:
/// - Priority (5 bits)
/// - Message type ID: Data type ID of the encoded message (16 bits)
/// - Service not message: Always 0. 1 bit.
/// - Source node ID.Can be 1-27. 7 bits.
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct CanId {
    // Valid values for priority range from 0 to 31, inclusively, where 0 corresponds to highest
    // priority (and 31 corresponds to lowest priority).
    // In multi-frame transfers, the value of the priority field must be identical for all frames of the transfer.
    // (We use the enum which constrains to Cyphal's values).
    pub priority: MsgPriority,
    /// Valid values of message type ID range from 0 to 65535, inclusively.
    /// Valid values of message type ID range for anonymous message transfers range from 0 to 3, inclusively.
    /// This limitation is due to the fact that only 2 lower bits of the message type ID are available in this case.\
    /// Note: This is `service_type_id` for services.
    pub type_id: u16, // Valid values of Node ID range from 1 to 127, inclusively.
    // Note that Node ID is represented by a 7-bit unsigned integer value and that zero is reserved,
    // to represent either an unknown node or all nodes, depending on the context.
    pub source_node_id: u8,
    pub frame_type: FrameType,
}

impl CanId {
    /// Get the raw u32 value of the CanId as used in the frame
    pub fn value(&self) -> u32 {
        let (priority_bits, priority_shift) = (self.priority.val() as u32 & 0b1_1111, 24);

        // The `&` operations are to enforce the smaller-bit-count allowed than the datatype allows.

        // let frame_type_val = if let FrameType::Service(ServiceData {
        //     dest_node_id: _,
        //     req_or_resp: RequestResponse::Request,
        // }) = self.frame_type
        // {
        //     0x800000
        // } else {
        //     0
        // };

        let mut result = (priority_bits << priority_shift)
//            | frame_type_val
            | ((self.source_node_id & 0b111_1111) as u32);

        // The middle 16 bits vary depending on frame type.
        match &self.frame_type {
            FrameType::Message => {
                result |= (self.type_id as u32) << 8;
            }
            // FrameType::MessageAnon(discriminator) => {
            FrameType::MessageAnon => {
                // 14-bit discriminator. Discriminator should be random. We use the RNG peripheral.
                // Once set, we keep it.
                #[cfg(feature = "hal")]
                let discriminator = (rng::read() & 0b11_1111_1111_1111) as u32;
                #[cfg(not(feature = "hal"))]
                let discriminator = 335;

                result |= (discriminator << 10) | ((self.type_id & 0b11) as u32) << 8;
            }
            FrameType::Service(service_data) => {
                result |= ((self.type_id as u32) << 16)
                    | ((service_data.req_or_resp as u8 as u32) << 15)
                    | 0x80
                    | (((service_data.dest_node_id & 0b111_1111) as u32) << 8)
            }
        }

        result
    }

    /// Pull priority, type id, and source node id from the CAN id.
    /// https://dronecan.github.io/Specification/4._CAN_bus_transport_layer/
    /// todo: Currently hard-coded for DroneCAN.
    pub fn from_value(val: u32) -> Self {
        let source_node_id = val as u8 & 0b111_1111;

        let priority = MsgPriority::from_val((val >> 24) as u8 & 0b1_1111);

        let mut frame_type = FrameType::Message;

        let type_id: u16 = match (val >> 7) & 1 {
            0 => match source_node_id {
                0 => {
                    frame_type = FrameType::MessageAnon;
                    ((val >> 8) & 0b11) as u16
                }
                _ => (val >> 8) as u16,
            },
            1 => {
                frame_type = FrameType::Service(ServiceData {
                    dest_node_id: ((val >> 8) & 0b111_1111) as u8,
                    req_or_resp: if (val & 0x8000) == 0x8000 {
                        RequestResponse::Request
                    } else {
                        RequestResponse::Response
                    },
                });
                ((val >> 16) & 0xff) as u16
            }
            _ => unreachable!(),
        };

        Self {
            priority,
            type_id,
            source_node_id,
            frame_type,
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_canid_msg_convert() {
        let id1 = 0b0000_0001_0000_0011_1110_1000_0000_0001; // 0x103E801
        let canid1 = CanId {
            priority: MsgPriority::Immediate,
            type_id: 1000,
            source_node_id: 1,
            frame_type: FrameType::Message,
        };
        let canid1_val = canid1.value();
        assert_eq!(canid1_val, id1);
        let reversed = CanId::from_value(canid1_val);
        assert_eq!(canid1, reversed);
    }

    #[test]
    fn test_canid_service_convert() {
        let id1 = 0b0000_0001_0110_0101_1111_1111_1000_0010; // 0x165FF82
        let canid1 = CanId {
            priority: MsgPriority::Immediate,
            type_id: 101,
            source_node_id: 2,
            frame_type: FrameType::Service(ServiceData {
                dest_node_id: 127,
                req_or_resp: RequestResponse::Request,
            }),
        };
        let canid1_val = canid1.value();
        assert_eq!(canid1_val, id1);
        let reversed = CanId::from_value(canid1_val);
        assert_eq!(canid1, reversed);
        let id2 = 0b0000_0001_0110_0101_0000_0010_1111_1111; // 0x16502FF
        let canid2 = CanId {
            priority: MsgPriority::Immediate,
            type_id: 101,
            source_node_id: 127,
            frame_type: FrameType::Service(ServiceData {
                dest_node_id: 2,
                req_or_resp: RequestResponse::Response,
            }),
        };
        let canid2_val = canid2.value();
        assert_eq!(canid2_val, id2);
        let reversed = CanId::from_value(canid2_val);
        assert_eq!(canid2, reversed);
    }

    // #[test]
    // fn test_canid_anon_msg_convert() {
    //     let id1 = 0b0000_0001_0110_0101_1111_1111_1000_0010; // 0x165FF82
    //     let canid1 = CanId {
    //         priority: MsgPriority::Immediate,
    //         type_id: 101,
    //         source_node_id: 2,
    //         frame_type: FrameType::Service(ServiceData {
    //             dest_node_id: 127,
    //             req_or_resp: RequestResponse::Request,
    //         }),
    //     };
    //     let canid1_val = canid1.value();
    //     assert_eq!(canid1_val, id1);
    //     let reversed = CanId::from_value(canid1_val);
    //     assert_eq!(canid1, reversed);
    // }
}
