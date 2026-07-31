# Using the RP2040 PIO USB host with CDC-ACM

This guide shows how to use `embassy-rp-pio-usb-host` from a new Rust/Embassy
application and expose a descriptor-compliant CDC-ACM device as an asynchronous
byte stream.

The verified RP2040 backend currently targets the **Adafruit Feather RP2040
with USB Type A Host**. It requires:

- exactly 120 MHz `clk_sys`;
- GPIO16 for D+ and GPIO17 for D−;
- PIO0, PIO1, DMA channel 0, and their IRQs exclusively;
- GPIO18 controlled by the application to enable the board's current-limited
  5 V VBUS switch;
- one directly attached full-speed CDC-ACM device.

`CdcAcmHost` is a generic, descriptor-driven class driver. Baud rate, DTR/RTS
policy, framing, command syntax, response parsing, and any TCP or SCPI bridge
belong to the application. The BleuIO example is therefore a product protocol
implemented above the CDC byte stream, not part of the class driver.

The RP2040 backend also supports directly attached low-speed devices, but
CDC-ACM uses bulk endpoints and therefore requires full speed.

## 1. Architecture at a glance

A complete application has four layers:

1. `Rp2040PioEngine` owns the fixed PIO, DMA, GPIO, and timing-critical packet
   engine resources.
2. `PioHostState` and `embassy_usb_host::bus` provide root-port events,
   enumeration, address allocation, and typed pipes.
3. `CdcAcmHost` discovers the CDC function, owns its control and bulk pipes,
   and implements `embedded-io-async::Read` and `Write`.
4. The application implements its device protocol and transports data to the
   rest of the system through bounded queues or other application-owned state.

The shortest working reference is [`src/main.rs`](../src/main.rs). It is a
product-neutral acceptance firmware: it enumerates the first valid CDC-ACM
function, exercises only controls advertised by that function, and retains the
session until detach.

For a busy integrated firmware, use the architecture proven by
`pico-io-bridge`: run PIO/DMA ownership, root-port monitoring, enumeration,
pipe scheduling, and class I/O on RP2040 core 1; keep networking and application
logic on core 0. Do not call the physical packet engine from both cores.

## 2. Add the dependency

The crate is not yet published on crates.io. Pin the Git dependency to a
revision that you have tested:

```toml
[dependencies]
embassy-rp-pio-usb-host = {
    git = "https://github.com/ulso/embassy-rp-pio-usb-host",
    rev = "<tested-commit-sha>",
    default-features = false,
    features = ["embassy-usb-host"],
}
embassy-usb-host = "=0.1.0"
embedded-io-async = "=0.7.0"

cortex-m = "0.7.7"
cortex-m-rt = "0.7.5"
defmt = "1.0.1"
defmt-rtt = "1.0.0"
embassy-executor = {
    version = "0.10.0",
    features = ["defmt", "executor-thread", "platform-cortex-m"],
}
embassy-futures = "=0.1.2"
embassy-rp = {
    version = "=0.10.0",
    features = [
        "critical-section-impl",
        "defmt",
        "rp2040",
        "time-driver",
        "unstable-pac",
    ],
}
embassy-time = {
    version = "0.5.1",
    features = ["defmt", "defmt-timestamp-uptime"],
}
panic-probe = { version = "1.0.0", features = ["print-defmt"] }

[profile.release]
debug = 2
opt-level = "s"
lto = "fat"
codegen-units = 1
```

During local development, replace the Git source with a path dependency:

```toml
embassy-rp-pio-usb-host = {
    path = "../embassy-rp-pio-usb-host",
    default-features = false,
    features = ["embassy-usb-host"],
}
```

Keep Embassy versions aligned with this repository's
[`Cargo.toml`](../Cargo.toml). Host traits from incompatible Embassy versions
are distinct Rust types even when their names are identical.

Dependency profiles are not inherited. The final firmware project must define
the release profile shown above; the PIO backend's verified timing paths assume
that optimized build configuration.

## 3. Own the target, linker, and memory layout

A dependency cannot select the consuming application's memory layout. The
final firmware project must own its `.cargo/config.toml`, `memory.x`, linker
arguments, toolchain selection, and runner.

For a new project, the corresponding files in this repository are useful
starting points:

```console
cargo new --bin my-cdc-host
cd my-cdc-host
rustup target add thumbv6m-none-eabi
cargo install elf2uf2-rs

export PIO_USB_HOST_REPO=/path/to/embassy-rp-pio-usb-host
mkdir -p .cargo
cp "$PIO_USB_HOST_REPO/.cargo/config.toml" .cargo/config.toml
cp "$PIO_USB_HOST_REPO/rust-toolchain.toml" rust-toolchain.toml
cp "$PIO_USB_HOST_REPO/memory.x" memory.x
```

Use an application-owned `build.rs` to make `memory.x` available to the
linker:

```rust
use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    println!("cargo:rerun-if-changed=memory.x");

    if env::var("TARGET").as_deref() != Ok("thumbv6m-none-eabi") {
        return;
    }

    let out = PathBuf::from(env::var_os("OUT_DIR").expect("OUT_DIR is set"));
    File::create(out.join("memory.x"))
        .expect("create memory.x")
        .write_all(include_bytes!("memory.x"))
        .expect("write memory.x");

    println!("cargo:rustc-link-arg-bins=-L{}", out.display());
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tlink-rp.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
```

The supplied `memory.x` describes the Feather board's 8 MiB flash and 264 KiB
RAM. Verify the flash size before reusing it for another RP2040 board.

## 4. Reserve the RP2040 resources

The concrete backend has a deliberately fixed, analyzer-verified allocation:

| Function | RP2040 resource |
|---|---|
| USB transmit | PIO0 SM0; 48 MHz full speed / 6 MHz low speed |
| USB receive | PIO1 SM0; 120 MHz |
| edge and EOP detection | PIO1 SM1; 96 MHz full speed / 12 MHz low speed |
| PIO1 instruction memory | all 32 instructions used: 15 + 17 |
| DMA | channel 0 and its IRQ |
| D+ | GPIO16 |
| D− | GPIO17 |

Treat both PIO blocks as reserved for the firmware lifetime. The application
must not give any of these resources or IRQs to another driver.

GPIO18 enables the Feather's current-limited VBUS load switch; it does not
drive VBUS directly. GPIO13 is used by the supplied examples as a status LED.
These two board-control pins are application resources and are not owned by
`Rp2040PioEngine`.

Changing only constructor arguments is not enough to use another data-pin
pair. The current implementation contains direct register access and PIO
configuration verified specifically for GPIO16/GPIO17.

## 5. Create the engine and Embassy bus

The essential imports and IRQ bindings are:

```rust
use embassy_rp::bind_interrupts;
use embassy_rp::clocks::ClockConfig;
use embassy_rp::dma::InterruptHandler as DmaInterruptHandler;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIO0, PIO1};
use embassy_rp::pio::InterruptHandler as PioInterruptHandler;
use embassy_rp_pio_usb_host::pio_host::PioHostState;
use embassy_rp_pio_usb_host::pio_host::rp2040::Rp2040PioEngine;
use embassy_time::{Duration, Timer};
use embassy_usb_host::BusState;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    PIO1_IRQ_0 => PioInterruptHandler<PIO1>;
    DMA_IRQ_0 => DmaInterruptHandler<DMA_CH0>;
});

const SYS_CLOCK_HZ: u32 = 120_000_000;
```

Initialize the board and bus in this order:

```rust
let clocks =
    ClockConfig::system_freq(SYS_CLOCK_HZ).expect("valid 120 MHz PLL setup");
let p = embassy_rp::init(embassy_rp::config::Config::new(clocks));

let mut vbus_enable = Output::new(p.PIN_18, Level::Low);

let engine = Rp2040PioEngine::new(
    p.PIO0,
    p.PIO1,
    p.DMA_CH0,
    p.PIN_16, // D+
    p.PIN_17, // D-
    Irqs,
    Irqs,
    Irqs,
);

let host_state = PioHostState::new(engine);
let bus_state = BusState::new();
let controller = host_state
    .controller()
    .expect("only one host controller may be acquired");
let (mut controller, bus_handle) =
    embassy_usb_host::bus(controller, &bus_state);

Timer::after_millis(100).await;
vbus_enable.set_high();
```

`Rp2040PioEngine::new` rejects any `clk_sys` other than 120 MHz, and
`PioHostState::controller()` may be called only once.

`host_state` and `bus_state` must outlive the controller, allocator, and all
pipes. Stack-local storage is sufficient when the owning async function never
returns. A separately spawned task needs `'static` storage, for example via
`StaticCell`.

## 6. Run the root-port monitor continuously

The root-port monitor is functional host code, not optional diagnostics. It:

- debounces attach and detach;
- performs bus reset before reporting a connection;
- suppresses false detach samples while the host drives reset SE0;
- services the 1 ms frame cadence while no pipe owns the packet engine.

The complete, current implementation is
[`root_port_monitor` in `src/main.rs`](../src/main.rs). Run it for the entire
host lifetime. A minimal single-core acceptance firmware can `join` it with an
otherwise quiet application, as `src/main.rs` does.

In a networked, interrupt-heavy, or otherwise busy application, place the
entire host manager on core 1. The backend contains short CPU-assisted timing
windows around token/data handoff and full-speed device-data-to-host-ACK. PIO
handles the wire encoding and decoding, but cannot make those software
handoffs immune to unrelated interrupt latency.

The core boundary should look like this:

```text
core 0: network, SCPI, web UI, product logic
        | bounded command and response queues
core 1: root monitor, enumeration, class ownership, USB reads/writes
        | exclusive ownership
        +-- PIO0 + PIO1 + DMA_CH0 + GPIO16/17
```

Do not share `CdcAcmHost`, pipes, or `Rp2040PioEngine` across cores. Send owned
messages through bounded queues and apply backpressure when a queue is full.

## 7. Enumerate and allocate `CdcAcmHost`

The connection loop follows this lifecycle:

```rust
use embassy_rp_pio_usb_host::cdc_acm::allocate_from_enumeration;
use embassy_rp_pio_usb_host::host::Speed;
use embassy_usb_host::BusRoute;

const CONFIG_DESCRIPTOR_CAPACITY: usize = 512;

loop {
    let speed = controller.wait_for_connection().await;
    if speed != Speed::Full {
        continue;
    }

    let mut configuration = [0_u8; CONFIG_DESCRIPTOR_CAPACITY];
    let (enumeration, configuration_len) = match bus_handle
        .enumerate(BusRoute::Direct(speed), &mut configuration)
        .await
    {
        Ok(result) => result,
        Err(_error) => {
            wait_for_detach(&mut controller).await;

            // embassy-usb-host 0.1.0 can retain an address lease on an
            // enumeration error. This is safe only for this one-device,
            // hubless root bus after physical detach.
            for address in 1_u8..=127 {
                bus_handle.free_address(address);
            }
            continue;
        }
    };

    let address = enumeration.device_address;

    {
        let mut cdc = match allocate_from_enumeration(
            &bus_handle,
            &configuration[..configuration_len],
            &enumeration,
        ) {
            Ok(cdc) => cdc,
            Err(_error) => {
                wait_for_detach(&mut controller).await;
                bus_handle.free_address(address);
                continue;
            }
        };

        // enumerate() has already sent SET_CONFIGURATION. Data pipes begin
        // with DATA0 for the newly configured interface.
        cdc.reset_data_toggles();

        // Configure advertised controls and run the product protocol here.
        // Keep the class instance alive until detach.
        wait_for_detach(&mut controller).await;
    } // All three pipes are dropped here.

    bus_handle.free_address(address);
}
```

The detach helper consumes the controller event that ends the physical
connection. Keep the wildcard arm because `DeviceEvent` is non-exhaustive:

```rust
use embassy_rp_pio_usb_host::host::{DeviceEvent, UsbHostController};
use embassy_usb_host::BusController;

async fn wait_for_detach<'d, C>(controller: &mut BusController<'d, C>)
where
    C: UsbHostController<'d>,
{
    loop {
        match controller.wait_for_device_event().await {
            DeviceEvent::Disconnected | DeviceEvent::Overcurrent => return,
            DeviceEvent::Connected(_) => {}
            _ => {}
        }
    }
}
```

Three details are essential:

1. Pass the complete `&configuration[..configuration_len]` slice to the
   allocator.
2. Never assume interface or endpoint numbers. Discovery follows the CDC Union
   Functional Descriptor from the communications interface to its data
   interface.
3. Drop `CdcAcmHost` and every pipe before returning the USB address.

`allocate_from_enumeration` selects the first valid CDC-ACM function. Use
`allocate_from_enumeration_for_control_interface` when a composite device has
multiple ACM functions and the application needs a specific one.

## 8. Send only advertised class requests

Line coding and modem-control requests are optional CDC-ACM capabilities:

```rust
use embassy_rp_pio_usb_host::usb::CdcLineCoding;

if cdc.function().supports_line_requests() {
    // These values are application policy, not host policy.
    cdc.set_line_coding(CdcLineCoding::eight_n_one(115_200))
        .await?;
    cdc.set_control_line_state(true, false).await?; // DTR, RTS
}

if cdc.function().supports_send_break() {
    cdc.send_break(250).await?;
}
```

A device that does not advertise line requests may still have fully usable
bulk-IN and bulk-OUT endpoints. Do not send unsupported requests merely
because another CDC product requires them.

Call `reset_data_toggles()` once after `SET_CONFIGURATION`, or after changing
the data interface's alternate setting. Never reset toggles between normal
commands, reads, or writes. IN and OUT toggle state is independent and belongs
to the corresponding pipe.

## 9. Treat CDC as a byte stream

`CdcAcmHost` implements:

- `embedded_io_async::Read`;
- `embedded_io_async::Write`.

A transport-independent protocol can therefore be written without USB packet
knowledge:

```rust
use embedded_io_async::{Read, Write};

async fn communicate<S>(stream: &mut S) -> Result<(), S::Error>
where
    S: Read + Write,
{
    stream.write_all(b"STATUS\r\n").await?;
    stream.flush().await?;

    let mut rx = [0_u8; 128];
    loop {
        let count = stream.read(&mut rx).await?;
        let bytes = &rx[..count];

        // Feed bytes into a persistent parser or framer. One read is not
        // necessarily one response, line, or USB packet.
        let _ = bytes;
    }
}
```

Read semantics:

- USB packet and application-message boundaries are not preserved.
- A read may return fewer bytes than the supplied buffer can hold.
- If the buffer is smaller than a USB packet, the class driver retains the
  remainder for the next read.
- A USB zero-length packet is skipped; it is not stream EOF.
- Bulk NAK can make a read wait indefinitely, so application protocols should
  impose their own deadline.

For example:

```rust
use embassy_time::{Duration, with_timeout};

let result = with_timeout(
    Duration::from_secs(2),
    cdc.read(&mut receive_buffer),
)
.await;
```

Write semantics:

- the class driver packetizes a buffer automatically;
- a successful return means the complete buffer was transported;
- no terminating zero-length packet is appended;
- `flush()` is currently a no-op because the USB transfer has already
  completed; it does not prove that the device parsed the command.

If a product protocol requires a terminating ZLP, it needs a lower-level pipe
operation or a future explicit class API.

## 10. Keep product protocols and network bridges above CDC

The generic USB/CDC layer owns:

- enumeration and descriptor discovery;
- interface selection and pipe allocation;
- standardized CDC class requests;
- USB packetization, ACK/NAK retries, and DATA toggles;
- the asynchronous byte stream.

The application owns:

- baud rate and DTR/RTS policy;
- command and response syntax;
- line, frame, or binary message boundaries;
- protocol deadlines and retries;
- validation and interpretation of product data;
- TCP, SCPI, web, logging, or telemetry interfaces.

For example, `pico-io-bridge` exposes a raw USB-serial TCP service on port
7000. That service is deliberately a transparent TCP byte stream, not Telnet
or RFC 2217. Its network task communicates with the core-1 host manager using
bounded queues. SCPI commands are a separate management interface and must not
concurrently consume the same serial stream while a raw TCP client owns it.

The same CDC host path has been exercised with BleuIO and with a Waveshare
USB-to-LoRa module using a WCH CH343-compatible CDC-ACM interface. Their AT
commands, terminators, and response parsers remain product-specific code.

Do not copy these BleuIO choices into an unrelated product unless its protocol
requires them:

- `AT\r` or `AT\r\n` commands;
- 115200 baud;
- asserted DTR/RTS;
- BleuIO response framing and timeouts;
- VID/PID assumptions.

## 11. Detach and reconnect safely

Physical detach invalidates every pipe from that connection. Pending I/O
normally returns `PipeError::Disconnected`, mapped by `CdcAcmError` to
`embedded_io_async::ErrorKind::NotConnected`.

Use this order:

1. stop or cancel the product-protocol session;
2. drop the protocol client;
3. drop `CdcAcmHost` and its control, bulk-IN, and bulk-OUT pipes;
4. return the old USB address;
5. wait for a new attach;
6. enumerate again and allocate entirely new pipes.

Never reuse a class instance or pipe from a previous physical connection.

If the application should stop on either an I/O error or a controller event,
combine the futures with `embassy_futures::select`. See the complete lifetime
pattern in [`src/main.rs`](../src/main.rs).

## 12. Current limitations

The RP2040 backend currently supports:

- one directly attached full-speed or low-speed root device;
- full-speed endpoint zero, bulk, and interrupt transfers up to 64 bytes;
- low-speed endpoint zero and interrupt transfers up to 8 bytes, with polling
  intervals of at least 10 ms;
- no hubs or split transactions;
- no high-speed or isochronous transfers.

The ergonomic CDC helper retains one 64-byte bulk-IN packet. A different
high-speed controller can use the 512-byte-capacity helper, but that does not
give the RP2040 backend high-speed signaling.

An optional CDC notification-IN endpoint is discovered when present but is
not yet consumed by the stream driver.

The fixed RP2040 backend remains experimental. Keep the host manager isolated
from unrelated real-time load and test the exact device and firmware build you
intend to deploy.

## 13. Build and flash

Build the release firmware:

```console
cargo build --release
```

With the repository's `elf2uf2-rs --deploy` runner configured:

1. disconnect the board;
2. hold **BOOTSEL**;
3. reconnect it and release **BOOTSEL** when `RPI-RP2` appears;
4. run:

```console
cargo run --release
```

`Unable to find mounted pico` means the board is not mounted in BOOTSEL mode.
`defmt-rtt` requires an SWD probe; without one, use a status LED, oscilloscope,
or USB protocol analyzer for board-level diagnostics.

## 14. Troubleshooting

### No attachment is detected

- verify GPIO16 is D+ and GPIO17 is D−;
- verify GPIO18 enabled the Feather VBUS switch;
- verify the root-port monitor is continuously running;
- verify no other code owns PIO0, PIO1, DMA0, or their IRQs;
- verify `clk_sys` is exactly 120 MHz;
- verify a CDC device is full speed.

### Enumeration succeeds but no CDC-ACM function is found

- pass the complete configuration descriptor to the allocator;
- verify the device advertises CDC communications and data interfaces;
- inspect its CDC Union Functional Descriptor;
- do not assume the first bulk-IN and bulk-OUT endpoints form one function;
- select a control interface explicitly for a multi-ACM composite device.

### A class request stalls

- check `supports_line_requests()` and `supports_send_break()`;
- send only capabilities advertised by the ACM descriptor;
- verify the product actually needs the chosen line coding and modem signals.

### The first command works but later data is corrupted or incomplete

- do not reset DATA toggles between commands;
- keep parser state across multiple reads;
- do not equate one read with one response;
- apply framing and response deadlines above the byte stream;
- on an integrated firmware, confirm that all host work runs on the dedicated
  core and that unrelated interrupts cannot delay the timing-critical path.

### Reconnection eventually fails

- consume the detach event;
- stop application I/O before destroying the session;
- drop every class pipe before `free_address`;
- allocate a new class instance after the next enumeration;
- retain the documented address-lease workaround while using
  `embassy-usb-host 0.1.0`.

## Reference implementations

- [`src/main.rs`](../src/main.rs): product-neutral CDC-ACM lifecycle;
- [`src/cdc_acm.rs`](../src/cdc_acm.rs): generic class driver and public API;
- [`src/pio_host.rs`](../src/pio_host.rs): Embassy controller, allocator, and
  typed pipes;
- [`src/pio_host/rp2040.rs`](../src/pio_host/rp2040.rs): concrete PIO backend;
- [`examples/bleuio/app.rs`](../examples/bleuio/app.rs): a product protocol
  above the generic CDC stream;
- [`examples/bleuio/protocol.rs`](../examples/bleuio/protocol.rs): a
  transport-independent streaming parser;
- [`tests/bleuio_protocol.rs`](../tests/bleuio_protocol.rs): parser tests that
  do not depend on USB or RP2040 hardware.
