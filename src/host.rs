//! Compatibility boundary for the upstream Embassy USB-host contracts.
//!
//! Class drivers in this crate use these types directly instead of defining a
//! second, subtly different pipe API. Keeping the re-exports in one module
//! isolates the rest of the crate from dependency-path churn while
//! preserving the exact `embassy_usb_driver::host::UsbPipe` semantics:
//! transport-owned NAK retry, DATA-toggle state, cancellation recovery and
//! transfer packetization.

pub use embassy_usb_driver::host::{
    DeviceEvent, HostError, PipeError, SplitInfo, SplitSpeed, TimeoutConfig, UsbHostAllocator,
    UsbHostController, UsbPipe, pipe,
};
pub use embassy_usb_driver::{Direction, EndpointAddress, EndpointInfo, EndpointType, Speed};
