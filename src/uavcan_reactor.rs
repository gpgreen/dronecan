use crate::{CanError, CanId, FrameType, RxPayload, interface::UavcanInterface};
#[cfg(feature = "defmt")]
use defmt::*;
use embedded_can;
use nb;

/// Trait for application specific frame handler
/// This allows different handlers for different Uavcan frame types
/// The main categories of frame types, are Services, Messages, and Anonymous Messages
pub trait UavcanHandler<CAN, FRAME, E>
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
        interface: &mut UavcanInterface<CAN, FRAME>,
    ) -> Result<(), CanError<E>>;
}

/// Uavcan
/// This type allows handling of received CAN frames for the dronecode CAN stack.
/// To send CAN frames, use the `UavcanInterface` which is part of this type
pub struct Uavcan<CAN, FRAME, E, FH>
where
    CAN: embedded_can::nb::Can<Frame = FRAME, Error = E>,
    FRAME: embedded_can::Frame,
    E: embedded_can::Error,
    FH: UavcanHandler<CAN, FRAME, E>,
{
    node_id: u8,
    interface: UavcanInterface<CAN, FRAME>,
    pub handler: Option<FH>,
}

impl<CAN, FRAME, E, FH> Uavcan<CAN, FRAME, E, FH>
where
    CAN: embedded_can::nb::Can<Frame = FRAME, Error = E>,
    FRAME: embedded_can::Frame,
    E: embedded_can::Error,
    FH: UavcanHandler<CAN, FRAME, E>,
{
    /// Create new `Uavcan`
    pub fn new(node_id: u8, interface: UavcanInterface<CAN, FRAME>) -> Self {
        Self {
            node_id,
            interface,
            handler: None,
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

    /// Get reference to `UavcanInterface`
    pub fn interface(&self) -> &UavcanInterface<CAN, FRAME> {
        &self.interface
    }

    /// Get mutable reference to `UavcanInterface`
    pub fn interface_mut(&mut self) -> &mut UavcanInterface<CAN, FRAME> {
        &mut self.interface
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

    /// function to handle Polling event
    /// ts: a `u64` representing milliseconds from start
    pub fn process_tx_rx_once(&mut self, iface: usize, ts: u64) -> Result<(), CanError<E>> {
        self.interface.can_send()?;
        let frame = nb::block!(self.interface.iface[iface].receive())?;
        self.handle_rx_frame(iface, frame, &ts)
    }

    /// Handle a CAN frame received on the interface at index `iface`
    fn handle_rx_frame(&mut self, iface: usize, frame: FRAME, ts: &u64) -> Result<(), CanError<E>> {
        // get the CanId, ignore non-extended CAN id's
        let can_id = match frame.id() {
            embedded_can::Id::Extended(eid) => CanId::from_value(eid.as_raw()),
            _ => return Ok(()),
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
                    return Ok(());
                }
            }
            // process the data frame
            if let Some(rx_payload) = self.interface.handle_rx_frame(iface, &can_id, &frame, ts)? {
                // got a completed transfer, pass to handler
                if let Some(handler) = &self.handler {
                    handler.handle_message(
                        self.node_id,
                        &rx_payload,
                        &can_id,
                        &mut self.interface,
                    )?;
                }
            }
        }
        #[cfg(feature = "defmt")]
        debug!("<== rx_frame {:x?}", can_id.type_id);
        Ok(())
    }
}
