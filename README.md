# Embassy RP2040 PIO USB Host

An experimental USB host controller built in Rust with Embassy and the RP2040
PIO. The project targets the **Adafruit Feather RP2040 with USB Type A Host**.

> **Alpha status:** CDC-ACM, FTDI UART, raw HID, USBTMC/USB488, and the bounded
> USB Audio capture path have been exercised on hardware; the timing-critical
> PIO paths have also been inspected with a Beagle USB 480 analyzer. The API and the fixed
> RP2040 resource allocation may still change. This is not yet a
> production-qualified or drop-in replacement for a general-purpose USB host
> stack.

## Known issues

- A Keysight MSO-X 3054A running firmware `02.66.2024012316` has intermittently
  failed the first complete device-descriptor request after `SET_ADDRESS`.
  Analyzer captures show a timeout while sending the control SETUP packet at
  address 1. Retrying enumeration recovers without user intervention and the
  tested USBTMC command path is stable once the device is ready. Increasing
  the post-`SET_ADDRESS` recovery interval from 2 ms to 10 ms did not reduce
  the failure rate, so timing margin alone is not considered the root cause.

## Current hardware contract

The reusable class drivers are controller-independent, but the concrete
`Rp2040PioEngine` is intentionally fixed to the analyzer-verified resource
layout below:

| Resource | Current use |
|---|---|
| `clk_sys` | exactly 120 MHz |
| PIO0 SM0 | USB TX; 22 instructions; 48 MHz full speed / 6 MHz low speed |
| PIO1 SM0 | RX decoder at 120 MHz |
| PIO1 SM1 | edge/EOP detector; 96 MHz full speed / 12 MHz low speed |
| PIO1 instruction memory | fully occupied: 15 + 17 = 32 instructions |
| DMA channel 0 | owned, IRQ-bound, and reserved by the constructor |
| GPIO16 | USB D+ |
| GPIO17 | USB D− |
| PIO0 IRQ 0, PIO1 IRQ 0, DMA IRQ 0 | backend interrupt bindings |

The supplied Feather firmware additionally uses GPIO18 to enable the board's
current-limited 5 V VBUS switch and GPIO13 for the red status LED. These two
board-control pins are owned by the application rather than by
`Rp2040PioEngine`.

Although only three state machines are active, treat both PIO blocks as
reserved for the firmware lifetime. The current `embassy-rp 0.10.0` ownership
workaround deliberately forgets the five unused state-machine handles. PIO1's
two active programs consume all instruction memory. GPIO16 and GPIO17 are
exact requirements rather than merely an example consecutive pair because
the backend also accesses their registers directly. The constructor consumes
both PIO peripherals, DMA channel 0, GPIO16, and GPIO17, and rejects any
system clock other than 120 MHz.

The RP2040 backend has short CPU-assisted handoffs between the PIO state
machines, including the full-speed device-data-to-host-ACK turnaround. Run the
host manager on a dedicated core, or otherwise guarantee that unrelated
interrupt and executor load cannot delay these paths. The
`pico-io-bridge` integration runs PIO/DMA ownership, enumeration, pipe
scheduling, and class transport on core 1 while networking and application
logic remain on core 0. Cross-core application traffic should use bounded
queues rather than calling the physical packet engine from both cores.

## Library and example split

The project is split into a reusable host library and board-level examples:

For a step-by-step integration guide for a new Embassy CDC-ACM project, see
[`docs/CDC_ACM_HOST_GUIDE.md`](docs/CDC_ACM_HOST_GUIDE.md).

- `src/cdc_acm.rs` contains a controller-independent `CdcAcmHost`. It uses
  the published `embassy-usb-driver` host-pipe traits directly, implements
  `embedded-io-async::Read` and `Write`, and retains strict CDC Union
  Functional Descriptor matching for composite devices.
- `src/ftdi.rs` contains a controller-independent `FtdiHost` for FTDI
  vendor-specific USB UARTs. It discovers the vendor interface and bulk
  endpoints, implements UART vendor requests, strips the two status bytes
  from every bulk-IN packet, and exposes `embedded-io-async::Read` and
  `Write`. FT232R (`0403:6001`) is the first hardware-acceptance target.
- `src/hid.rs` contains a controller-independent raw `HidHost`. It performs
  strict HID/report/interrupt-endpoint discovery and exposes raw reports plus
  the standard HID class requests without assuming a keyboard, mouse, report
  ID, or product-specific report format.
- `src/audio.rs` contains a deliberately bounded UAC1 capture helper. It
  discovers mono 16-bit PCM input at 48 kHz, selects its alternate setting,
  requests the sample rate, and exposes one isochronous-IN packet per frame.
- `src/host.rs` is the compatibility boundary for
  `UsbHostController`, `UsbHostAllocator`, and `UsbPipe`.
- `src/pio_host.rs` implements the official Embassy controller, allocator, and
  typed pipe traits over one serialized packet engine.
- `src/pio_host/rp2040.rs` owns the RP2040 PIO state machines, DMA channel,
  GPIO16/GPIO17, full-speed reset, SOF scheduling, and the SRAM-resident
  packet timing paths.
- `src/main.rs` is the product-neutral board acceptance firmware. It
  enumerates and retains a generic CDC-ACM session without speaking a device
  command language.
- `examples/bleuio.rs` is the hardware acceptance example. BleuIO AT commands,
  GAP-scan parsing, and LED diagnostics are example concerns and are not part
  of the USB or CDC library API.
- `examples/bleuio/protocol.rs` contains the example's allocation-free,
  packet-boundary-independent BleuIO protocol parser.
- `examples/p8055.rs` is the first low-speed HID acceptance example. Its
  product layer decodes the original Velleman K8055/P8055 eight-byte reports
  while the reusable HID layer remains device-independent.
- `examples/bleuio/legacy_m5i.rs` preserves the former monolithic,
  analyzer-verified implementation for comparison with the layered path.

The default binary is a product-neutral CDC-ACM probe, so the USB-bootloader
workflow remains:

```console
cargo run --release
```

It enumerates any descriptor-compliant full-speed CDC-ACM function, allocates
the class pipes, exercises only standard controls advertised by the ACM
descriptor, and keeps the session alive until detach. It never sends modem,
AT, BLE, or other product commands.

The BleuIO acceptance application is selected explicitly:

```console
cargo run --release --example bleuio
```

The low-speed Velleman K8055/P8055 HID acceptance application is:

```console
cargo run --release --example p8055
```

It accepts an original `10cf:5500`–`10cf:5503` board, reads its HID report
descriptor, sends the documented command-zero safe reset (all digital and
analog outputs off), and validates one exact eight-byte interrupt-IN report.
This complete low-speed path is hardware-verified with a Beagle USB 480 in
`beagle-p8055-reset.csv`.

The dependency versions are pinned to `embassy-rp 0.10.0`,
`embassy-usb-host 0.1.0`, `embassy-usb-driver 0.2.2`,
`embassy-futures 0.1.2`, and `embedded-io-async 0.7.0`.

`CdcAcmHost` is intended for descriptor-compliant CDC-ACM functions rather
than for one particular modem or BLE product. Product-specific command
languages, such as BleuIO's AT protocol, belong above its
`embedded-io-async` byte-stream interface. Devices that use a vendor-specific
USB protocol, a non-ACM CDC subclass, or a proprietary driver are outside that
class driver's scope. UAC1 capture is provided separately by `audio`.

The current RP2040 backend supports one directly connected **full-speed or
low-speed** root device. Full-speed supports endpoint-zero control,
bulk/interrupt endpoints up to 64 bytes, and one isochronous-IN endpoint up to
100 bytes per 1 ms frame. The initial audio helper intentionally accepts only
mono 16-bit PCM capture at 48 kHz; playback, feedback endpoints, asynchronous
rate adaptation, and general-purpose isochronous pipes remain out of scope.
Low-speed uses a separate 1.5 Mbit/s
PIO profile, low-speed J/K polarity, a standards-compliant 1 ms keep-alive,
endpoint-zero MPS 8, and interrupt endpoints up to 8 bytes with intervals of
at least 10 ms. Low-speed reset, keep-alives, EP0 enumeration, HID report
descriptor retrieval, and eight-byte interrupt OUT/IN are analyzer-verified
against an original K8055/P8055. The backend does not implement high speed or
hubs/split transactions. The CDC class can select
one of several ACM functions in a composite descriptor, follows the CDC Union
Functional Descriptor, and exposes a packet-boundary-independent async byte
stream. Its optional interrupt notification endpoint is discovered but not
yet consumed by the stream driver.

The ergonomic class helpers use a 64-byte receive-packet buffer, matching
full-speed bulk endpoints. The controller-independent class can also be
instantiated with an explicit capacity, for example
`allocate_from_enumeration_with_rx_capacity::<_, 512>` for a high-speed host
controller. The selected endpoint MPS is validated before any pipes are
allocated. This does not add high-speed signaling to the RP2040 PIO backend.

### Embassy host integration

The public boundary deliberately uses `embassy-usb-driver`'s host traits
instead of a project-local look-alike. The default firmware executes this
product-neutral control flow:

1. the PIO implementation exposes an `UsbHostController` and cloneable
   `UsbHostAllocator`;
2. `embassy_usb_host::bus` splits those into a root-port controller and a
   shareable bus handle;
3. `BusHandle::enumerate` performs standard device enumeration and returns
   `EnumerationInfo` plus the active configuration bytes;
4. `allocate_from_enumeration` selects a CDC-ACM function and allocates its
   endpoint-zero, bulk-IN and bulk-OUT pipes;
5. application code treats `CdcAcmHost` only as an
   `embedded-io-async::Read + Write` stream.

The class allocation itself is device-independent:

```rust
let (enumeration, config_len) = bus_handle
    .enumerate(BusRoute::Direct(Speed::Full), &mut config)
    .await?;
let mut cdc = allocate_from_enumeration(
    &bus_handle,
    &config[..config_len],
    &enumeration,
)?;
cdc.reset_data_toggles();

if cdc.function().supports_line_requests() {
    let current = cdc.get_line_coding().await?;
    cdc.set_line_coding(current).await?;
    cdc.set_control_line_state(false, false).await?;
}

// CdcAcmHost also implements embedded_io_async::Read and Write.
let received = cdc.read(&mut receive_buffer).await?;
cdc.write(&transmit_buffer[..transmit_len]).await?;
```

On physical detach, drop every class instance before calling
`bus_handle.free_address(enumeration.device_address)`.

Consequently another class driver can later allocate its own typed pipes from
the same bus handle. Adding HID, mass-storage, or another class does not put
device-specific behavior into the PIO wire engine. The current
`PioHostState`/`PioHostController`/`PioHostPipe` adapter establishes this
ownership and trait boundary, and the BleuIO example exercises it through
`embassy-usb-host` enumeration.

## Legacy M1–M5 development history

The sections below document the incremental wire-level bring-up that produced
the analyzer-verified `legacy_m5i.rs` reference. The default firmware now uses
the layered Embassy path described above.

### M1: host-port bring-up

- runs the RP2040 at 120 MHz, ready for USB's 12 MHz full-speed bit timing;
- loads a PIO line-sampling program;
- samples D+ and D- at 1 kHz;
- debounces and reports full-speed attach, low-speed attach, detach, and SE1;
- controls a logic-level enable for an external current-limited VBUS switch;
- uses the red D13 LED as an attached indicator;
- retains `defmt` instrumentation for optional SWD debugging, but does not
  require an SWD connection.

### M2: full-speed reset and packet TX

- dedicates PIO0 state machine 0 to a 48 MHz USB transmitter;
- keeps attach/detach monitoring in the 1 kHz host task;
- drives a 20 ms SE0 bus reset and observes 10 ms reset recovery;
- builds SOF tokens with PID and CRC-5;
- performs NRZI encoding and USB bit stuffing in PIO;
- generates two-bit SE0 plus one-bit J EOP entirely in PIO;
- starts a frame every 1 ms after reset.

While M2 is active, a heartbeat counter independent of USB's wrapping frame
number toggles the red D13 LED every 250 frames. A steady 2 Hz blink therefore
means that attach detection, bus reset, and the SOF scheduler have all run.

### M3: packet RX and handshake validation

- uses PIO1 state machine 0 for NRZI decoding and bit unstuffing;
- uses PIO1 state machine 1 for 96 MHz edge timing and EOP detection;
- validates SYNC, complementary PID bits, CRC-5, and CRC-16 in Rust;
- sends a valid address-0 `GET_DESCRIPTOR` SETUP/DATA0 transaction;
- receives and validates the device's ACK handshake;
- keeps the 22-instruction TX program on PIO0 and the 15 + 17 instruction RX
  programs on PIO1.
- installs the fully relocated Pico-PIO-USB-compatible PIO1 image explicitly,
  keeping the edge/EOP program and all of its jump targets at addresses 15–31.
- works around `embassy-rp 0.10.0` sharing one internal PIO ownership counter
  between PIO0 and PIO1. Exactly the unused state-machine handles are retained
  so their premature `Drop` cannot reset GPIO16/17 to `FUNCSEL=NULL`; the
  workaround can be removed when the dependency contains separate per-PIO
  state.

### M4a: first endpoint-zero control read

The current firmware extends the verified M3 SETUP transaction into a complete
address-zero `GET_DESCRIPTOR(Device, 8)` control read:

- receives and CRC-validates the device's eight-byte DATA1 response;
- preserves the control transfer across NAKs and retries IN once per USB frame
  for up to 128 frames;
- preloads and releases the host ACK from an SRAM-resident timing path;
- sends the first OUT/DATA1 zero-length status attempt immediately after the
  final data ACK, then waits a full millisecond from each NAK before emitting
  the next SOF and retrying, and validates the eventual ACK;
- parses the descriptor prefix and validates `bMaxPacketSize0`.

The first eight descriptor bytes are deliberately the only enumeration data
read so far. Address assignment and the remaining descriptors are the next M4
steps. M4a is verified on the BLE dongle with the Beagle USB 480 analyzer,
including DATA- and status-stage NAK retries.

### M4b: address assignment

The current firmware continues after M4a with:

- a standard `SET_ADDRESS(1)` request at address zero;
- the control-write IN/DATA1 zero-length status stage and its host ACK;
- two milliseconds of recovery with continuous, increasing SOF numbers;
- another complete descriptor-prefix control read at address 1;
- comparison with the descriptor prefix read at address zero.

M4b is verified on the BLE dongle in hardware and with a Beagle USB 480
capture. The capture confirms the address-zero request and status stage, the
recovery interval, and the repeated descriptor-prefix control read at address
1.

### M4c: complete device descriptor

The current firmware extends the address-1 control read to all 18 bytes of
the device descriptor. With
`bMaxPacketSize0 = 8`, the device response is collected over three IN
transactions:

1. DATA1 containing descriptor bytes 0–7;
2. DATA0 containing descriptor bytes 8–15;
3. DATA1 containing descriptor bytes 16–17.

The host ACKs each data packet, validates the alternating data toggle and
CRC-16, completes the control-read status stage, and then parses the complete
descriptor. A CRC-valid duplicate is ACKed and discarded without advancing
the expected toggle; NAKs and invalid packets retry the same IN transaction.
If the control-read status stage returns four NAKs, the host restarts the
complete request with a fresh SETUP, up to three transfer attempts, so EP0 and
its data toggle are resynchronized instead of repeating OUT status forever.
M4c is verified on the BLE dongle in hardware and with a Beagle USB 480
capture. The successful trace recovers from one status-stage NAK at both
address 0 and address 1, then reads the address-1 descriptor as
DATA1/DATA0/DATA1 payloads of 8, 8, and 2 bytes. The decoded descriptor
identifies USB 2.00, endpoint-zero MPS 8, VID:PID `2dcf:6002`, and one
configuration. The four-NAK whole-transfer fallback was not exercised by this
capture.

### M5a: CDC-ACM configuration discovery

After M4c, the current firmware:

- uses a reusable endpoint-zero control-IN implementation with the verified
  SETUP, data-toggle, short-packet, status-NAK, and whole-transfer recovery;
- requests the first nine bytes of configuration descriptor index zero;
- validates `wTotalLength` against a fixed 128-byte no-allocation buffer;
- requests exactly the complete configuration descriptor;
- safely iterates its variable-length descriptors without trusting
  `bLength`;
- finds an ACM communications interface, its Header, ACM and Union functional
  descriptors, the Union-linked CDC data interface, and its bulk IN/OUT
  endpoints;
- records an optional interrupt-IN notification endpoint.

The parser's host-side regression tests include the exact 67-byte
configuration captured from the target `2dcf:6002` dongle by the independent
reference host. M5a is verified on the dongle with a Beagle USB 480 capture:
all nine DATA1/DATA0 packets, the final three-byte short packet, every host
ACK, and the OUT/DATA1 status stage completed successfully.

### M5b: select the CDC-ACM configuration

After discovery, the current firmware:

- reuses one no-data endpoint-zero control-write implementation for both
  `SET_ADDRESS` and `SET_CONFIGURATION`;
- sends `SET_CONFIGURATION` with the discovered nonzero `bConfigurationValue`
  at device address 1;
- retries the SETUP stage once per frame and requires an IN/DATA1 zero-length
  status packet;
- sends the timing-critical status ACK from the existing SRAM receive path.

The M5b checkpoint stops in the USB Configured state. CDC-ACM class requests
and bulk data are separate follow-on steps.

M5b is verified on the dongle with a Beagle USB 480 capture. The trace contains
the exact `SET_CONFIGURATION(1)` setup packet, its device ACK, and the complete
IN/DATA1 zero-length status stage with the final host ACK.

### M5c: assert the CDC-ACM control lines

After selecting the configuration, the current firmware:

- verifies that the ACM functional descriptor advertises support for line
  requests;
- sends class/interface `SET_CONTROL_LINE_STATE` to the discovered
  communications interface;
- asserts both DTR and RTS;
- reuses the verified no-data control-write path and requires an IN/DATA1
  zero-length status packet.

M5c deliberately stops before `SET_LINE_CODING`, whose seven-byte OUT data
stage is the next increment.

M5c is verified on the dongle with a Beagle USB 480 capture. The class setup,
device ACK, IN/DATA1 zero-length status packet, and final host ACK all
completed without retries or analyzer errors.

### M5d: set the CDC-ACM line coding

After asserting the control lines, the current firmware:

- generalizes the endpoint-zero control-write path to support a single
  host-to-device data packet while preserving all verified no-data requests;
- sends class/interface `SET_LINE_CODING` to the discovered communications
  interface;
- selects 115200 bit/s, eight data bits, no parity, and one stop bit;
- sends the seven-byte payload in an OUT/DATA1 transaction and requires the
  device ACK before starting the IN/DATA1 zero-length status stage;
- retries a NAK, missing response, or invalid data-stage handshake once per
  frame with the same DATA1 toggle.

M5d stops before the first CDC bulk-OUT transfer.

M5d is verified on the dongle with a Beagle USB 480 capture. The trace contains
the complete setup, OUT/DATA1 payload, device ACKs, and IN/DATA1 zero-length
status stage without retries or analyzer errors.

### M5e: first CDC-ACM bulk OUT

After class initialization, the current firmware:

- uses the bulk-OUT endpoint discovered from the CDC data interface rather
  than hard-coding an endpoint number;
- initializes a separate bulk-OUT data toggle to DATA0, as required after
  `SET_CONFIGURATION`;
- sends `AT\r\n` as one short bulk packet;
- advances the toggle to DATA1 only after the device ACK;
- retries NAK, missing, or invalid handshakes once per frame with the same
  DATA0 packet, so a lost ACK is handled as a duplicate.

No zero-length packet is needed because the four-byte command is shorter than
the endpoint's 64-byte maximum packet size. M5e stops before polling bulk IN
for the dongle's response.

M5e is verified on the dongle with a Beagle USB 480 capture. The trace contains
the address-1/endpoint-2 OUT token, the `AT\r\n` DATA0 packet with valid CRC-16,
and the device ACK without retries or analyzer errors.

### M5f: first CDC-ACM bulk IN

After the dongle accepts `AT\r\n`, the current firmware:

- uses the bulk-IN endpoint discovered from the CDC data interface rather than
  hard-coding an endpoint number;
- initializes a separate bulk-IN toggle to DATA0, independently of bulk OUT;
- polls at most once per frame for up to one second while no non-empty valid
  packet is available;
- receives and streaming-CRC-validates any packet up to the endpoint's 64-byte
  maximum packet size on the SRAM-resident timing path;
- ACKs an expected packet and advances the toggle only after that ACK;
- ACKs and discards a valid wrong-toggle duplicate without advancing;
- accepts the first non-empty packet as the M5f checkpoint without assuming
  that its text is `OK`;
- ACKs a valid zero-length packet, advances its toggle, and keeps polling for
  application data.

M5f deliberately stops after the first non-empty packet. Reassembling a
multi-packet CDC stream and parsing complete command responses are later
class-driver steps.

M5f is verified on the dongle with a Beagle USB 480 capture. The first bulk-IN
poll receives a one-byte DATA0 payload containing `A`, with valid CRC-16, and
the host ACKs it. There are no preceding NAKs or analyzer errors. This proves
that a short USB packet is not an application-response boundary for this
dongle.

### M5g: reconstruct and validate the AT response

The current firmware continues after M5f by:

- retaining the same bulk-IN data toggle across every subsequent poll;
- using one shared 1,000-transaction budget and polling at most once per frame;
- accumulating payload bytes across arbitrary USB packet boundaries in a
  bounded 64-byte buffer;
- continuing after short packets, NAKs, valid zero-length packets, and
  acknowledged wrong-toggle duplicates;
- recognizing only an exact CRLF-delimited `OK` line as success;
- ignoring an echoed `AT` line and empty lines before `OK`;
- treating an exact `ERROR` line, response-buffer overflow, or an incomplete
  response at the poll limit as failure.

The line parser runs only after the SRAM-resident receive path has completed
the packet ACK, so response reconstruction cannot lengthen the USB handshake
turnaround. The basic `AT` command and its default-mode `OK` response follow
the [BleuIO command reference](https://www.bleuio.com/getting_started/docs/commands.html#at).

M5g is verified on the dongle with a Beagle USB 480 capture. After the
independently verified one-byte DATA0 `A` packet, the trace contains a DATA1
`T` packet and a DATA0 `\r\nOK\r\n` packet, each followed by the host ACK.
Their payloads reconstruct exactly `AT\r\nOK\r\n`. The Beagle rendered the
first packet in this particular capture as an invalid `83 00 1F` packet, but
the host emitted ACK and the device's next packet used DATA1. Together with
the earlier exact M5f capture of `80 C3 41 80 8F`, that isolated row is treated
as an analyzer decode anomaly rather than a host protocol failure.

### M5h: persistent CDC session and BLE central role

The current firmware now continues beyond the first command by:

- keeping one `CdcAcmDataState` with independent bulk-OUT and bulk-IN toggles
  for the lifetime of the configured CDC session;
- wrapping one bulk OUT, one continued SOF, and packet-independent bulk IN
  response reconstruction in a reusable command/`OK` transaction;
- reusing that same state for both `AT\r\n` and `AT+CENTRAL\r\n`, without a
  bus reset, configuration request, or DATA-toggle reset between commands;
- advancing either direction only after its corresponding ACK;
- requiring another exact CRLF-delimited `OK` line before reporting success.

The first `AT` ACK leaves bulk OUT at DATA1, so the second command packet must
be exactly:

```text
80 4B 41 54 2B 43 45 4E 54 52 41 4C 0D 0A A9 E3
```

Its payload is `AT+CENTRAL\r\n`; `A9 E3` is its CRC-16. The
[BleuIO command reference](https://www.bleuio.com/getting_started/docs/commands.html#atcentral)
defines this command as switching an idle dongle to BLE central role and
returning `OK`.

M5h is verified on the dongle with a Beagle USB 480 capture. The first
`AT\r\n` exchange is fully clean, including DATA0 `A`, DATA1 `T`, DATA0
`\r\nOK\r\n`, and every host ACK. The next bulk-OUT packet uses DATA1 and
contains the exact `AT+CENTRAL\r\n` bytes and CRC shown above, followed by the
device ACK. Its bulk-IN response retains the prior IN toggle and reconstructs
`AT+CENTRAL\r\nOK\r\n`.

The Beagle misdecoded the one-byte DATA0 `C` response packet as the invalid
three-byte sequence `87 02 9C`. The host nevertheless ACKed it, the device
advanced to DATA1 for `E`, and all surrounding response bytes and CRCs are
valid. As with the isolated M5g artifact, this cannot be the byte sequence
accepted by the firmware's PID/CRC gate and is recorded as an analyzer or
signal-sampling anomaly rather than a USB state error.

### M5i: first timed BLE GAP scan

The current firmware continues on that same configured CDC session by:

- sending `AT+GAPSCAN=1\r\n` without resetting either bulk endpoint's DATA
  toggle;
- allowing up to 3,000 once-per-frame bulk-IN polls for the one-second scan
  and its queued output;
- streaming arbitrary USB packet splits through one fixed 128-byte line
  buffer, without allocation or an unbounded result list;
- requiring the exact `SCANNING...` start marker, at least one syntactically
  valid default-mode device line, and the exact `SCAN COMPLETE` marker;
- parsing the first result into its numeric index, address type, six-byte BLE
  address, and signed RSSI while safely draining any additional result lines;
- feeding the parser only newly ACKed packets. A NAK, zero-length packet,
  invalid packet, or acknowledged wrong-toggle duplicate never becomes
  application text.

After the acknowledged DATA1 `AT+CENTRAL\r\n` packet, bulk OUT is DATA0.
The complete raw scan-command packet is therefore:

```text
80 C3 41 54 2B 47 41 50 53 43 41 4E 3D 31 0D 0A D7 AE
```

Its payload is `AT+GAPSCAN=1\r\n`; `D7 AE` is its CRC-16. The
[BleuIO command reference](https://www.bleuio.com/getting_started/docs/commands.html#atgapscan)
defines the timed scan and its default-mode `SCANNING...`, device-result, and
`SCAN COMPLETE` output.

M5i is verified on the dongle with a Beagle USB 480 capture. The trace contains
the exact DATA0 scan-command packet and device ACK, the echoed command, an
ACKed DATA0 packet containing `\r\nSCANNING...\r\n`, many result lines, and an
ACKed DATA1 packet containing `\r\nSCAN COMPLETE\r\n`. The first result accepted
by the firmware is:

```text
[01] Device: [1]77:57:C1:41:37:1F  RSSI: -64
```

It arrives as one CRC-valid DATA1 packet and receives the host ACK. The capture
continues through at least result index 20 before completion, demonstrating
that the fixed-memory parser keeps only the first structured result while
draining a much longer scan stream.

The M5i success indication is a **slow six-pulse burst every 2.5 seconds**:
100 ms on at 0, 250, 500, 750, 1,000, and 1,250 ms, followed by 1,150 ms off.
This intentionally replaces the fast M5h five-pulse pattern. A failed probe
instead repeats a five-second diagnostic cycle:
one 500 ms marker blink, a 500 ms pause, and then the short blinks to count:

| Blinks | Meaning |
| ---: | --- |
| 1 | TX DMA stalled |
| 2 | TX EOP flag was not raised |
| 3 | the configuration is malformed or lacks the required CDC-ACM topology/capability |
| 4 | RX edge detector remained at IRQ2 during a directly driven K pulse |
| 5 | RX edge detector armed but did not see the directly driven K pulse |
| 6 | SM0 side-set/SET PINDIRS did not drive the physical pins to K |
| 7 | SM0 drove the pins to K, but the already-verified receiver missed it |
| 8 | both synthetic K pulses worked, but the DMA/NRZI TX packet did not start locally |
| 9 | the TX packet started, but local RX could not validate the looped-back SOF |
| 10 | the transmitted SOF validated locally, but no device response or EOP was detected |
| 11 | a device response started but the packet was invalid or incomplete |
| 12 | a valid NAK exhausted a SETUP, control-data, or bulk retry budget |
| 13 | a descriptor, control request/status stage, bulk transfer, CDC response, or BLE scan did not complete |

## Default pinout

| Signal | Pico GPIO | Notes |
| --- | ---: | --- |
| D+ | GPIO16 | Wired to the board's USB-A host connector |
| D- | GPIO17 | Wired to the board's USB-A host connector |
| VBUS enable | GPIO18 | Enables the on-board 5 V boost converter |
| Status LED | GPIO13 | On-board red D13 LED |

The current backend requires exactly GPIO16 for D+ and GPIO17 for D−. Another
consecutive pin pair is not sufficient because the verified implementation
also accesses the GPIO16/GPIO17 registers directly.

The Feather provides the USB-A connector, data-line components, 5 V boost
converter, and resettable fuse on-board. GPIO18 is only the converter's enable
signal; it does not source VBUS directly.

## Build and run

Install stable Rust and `elf2uf2-rs`:

```console
cargo install elf2uf2-rs
```

Put the board in its USB bootloader:

1. Disconnect USB.
2. Hold **BOOTSEL**.
3. Connect USB and release **BOOTSEL** when the `RPI-RP2` drive appears.
4. From the project directory, run:

```console
cargo run --release
```

The configured runner invokes `elf2uf2-rs --deploy`, converts the ELF image to
UF2, copies it to the board, and starts the firmware. No SWD probe is needed.

To create a UF2 file without flashing:

```console
cargo build --release
elf2uf2-rs target/thumbv6m-none-eabi/release/pio-usb-host pio-usb-host.uf2
```

An independent Pico-PIO-USB control firmware is available under
`reference/pico-pio-usb-control`. Its UF2 can be flashed in BOOTSEL mode with:

```console
elf2uf2-rs --deploy \
  reference/pico-pio-usb-control/build/pico_pio_usb_control.elf
```

This control uses the original C implementation at 120 MHz with D+ on GPIO16,
D- on GPIO17, and the USB host power enable on GPIO18. Reflash the Rust
firmware with `cargo run --release` after a control capture.

The RP2040 bootloader disconnects after starting the firmware. The application
does not expose a serial console on the Pico's native USB port, so `defmt`
output is only available if an SWD probe is added later.

Run the target-independent state-machine tests on macOS:

```console
cargo test --all-features --target aarch64-apple-darwin
```

## Generic CDC-ACM acceptance test

1. Power the Pico through the debugger or device USB connector.
2. Run `cargo run --release` and confirm that the `RPI-RP2` drive disappears.
3. Confirm that the green 5 V LED beside the USB-A connector turns on after
   roughly 100 ms.
4. Connect any descriptor-compliant full-speed CDC-ACM device. The red D13 LED
   is solid while Embassy enumerates it and initializes the class.
   Afterwards it repeats one of these two-second patterns:
   - **two pulses:** generic CDC-ACM allocation succeeded and every advertised
     standard line control completed;
   - **three pulses:** standard USB enumeration failed;
   - **four pulses:** no usable CDC-ACM function could be discovered or
     allocated;
   - **five pulses:** the device advertised CDC line requests, but the
     capability-safe standard-control probe failed or timed out.
5. Disconnect it and confirm that the LED turns off.

For electrical validation with a logic analyzer, D+/D- should show:

1. 20 ms SE0 after full-speed attach;
2. 10 ms reset recovery;
3. one SOF token approximately every 1 ms, with a bit time of 83.3 ns.

### BleuIO example acceptance test

Flash the product-specific example explicitly:

```console
cargo run --release --example bleuio
```

Connect a full-speed BleuIO dongle. Its LED patterns are:

- **two pulses:** generic CDC-ACM allocation succeeded, `AT` and
  `AT+CENTRAL` returned `OK`, and `AT+GAPSCAN=1` returned a parsed BLE result;
- **three pulses:** Embassy could not allocate the initial endpoint-zero pipe;
- **four pulses:** a device or configuration descriptor was invalid or too
  large for the fixed descriptor buffer;
- **five pulses:** CDC-ACM discovery/allocation, the BleuIO exchange, or its
  ten-second deadline failed;
- **six pulses:** the device did not answer an enumeration request after
  Embassy's retries;
- **seven pulses:** an enumeration transfer timed out;
- **eight pulses:** an enumeration packet failed framing, CRC, or response
  validation;
- **nine pulses:** another endpoint-zero transfer error occurred.

During RP2040 wire bring-up, an eight-pulse `BadResponse` is expanded into a
long 500 ms marker, a 500 ms pause, a short-pulse stage code, another pause,
and six short-pulse groups: handshake detail, received-length code,
received-prefix code, edge-detector program counter, GPIO input snapshot, and
GPIO input-override readback. The complete sequence repeats every 16 seconds.
Stage codes are:

- **one:** local control-IN contract validation;
- **two:** ACK validation after the control-IN SETUP stage;
- **three:** control-IN DATA packet validation;
- **four:** ACK validation after the control-IN status stage;
- **five:** local control-OUT contract validation;
- **six:** ACK validation after the control-OUT SETUP stage;
- **seven:** ACK validation after a control-OUT DATA stage;
- **eight:** control-OUT status packet validation.

Handshake-detail codes are:

- **one:** RX decoder IRQ/error;
- **two:** edge start without a received byte or EOP;
- **three:** incomplete packet or packet deadline;
- **four:** EOP with a length other than two bytes;
- **five:** invalid USB SYNC byte;
- **six:** invalid PID complement;
- **seven:** valid but unexpected PID;
- **eight:** an otherwise unclassified receiver state.

The received-length group is one-based so that zero bytes remain visible:
one pulse means zero bytes, two means one byte, and so on; eight means seven
or more bytes. Received-prefix codes are:

- **one:** no received byte;
- **two:** the first byte was not USB SYNC;
- **three:** only the SYNC byte was received;
- **four:** SYNC followed by ACK;
- **five:** SYNC followed by NAK;
- **six:** SYNC followed by STALL;
- **seven:** SYNC followed by another PID with a valid complement;
- **eight:** SYNC followed by a PID with an invalid complement.

Edge-detector program-counter codes are:

- **one:** parked at instruction 15 before release;
- **two:** waiting for a bus edge at instruction 16;
- **three:** start-edge handling at instruction 17;
- **four:** bit-edge tracking at instructions 18–24;
- **five:** EOP candidate handling at instructions 25–26;
- **six:** EOP validation at instructions 27–29;
- **seven:** end/error handling at instructions 30–31;
- **eight:** another or unavailable program-counter value.

GPIO input-snapshot codes compare the raw D+/D- pad state with the peripheral
input seen after `INOVER`:

- **one:** expected full-speed idle J with input inversion (`0x12`);
- **two:** full-speed idle J without the expected inversion (`0x11`);
- **three:** correctly inverted SE0 (`0x03`);
- **four:** correctly inverted K (`0x21`);
- **five:** correctly inverted SE1 (`0x30`);
- **six:** another wholly non-inverted line-state snapshot;
- **seven:** mixed or partial input inversion;
- **eight:** another or unavailable snapshot.

GPIO input-override readback codes are:

- **one:** both D+ and D- are configured as `INVERT`;
- **two:** both are configured as `NORMAL`;
- **three:** only D+ is configured as `INVERT`;
- **four:** only D- is configured as `INVERT`;
- **five:** at least one input is forced low or high;
- **eight:** another or unavailable configuration.

### Legacy analyzer capture reference

The detailed M4–M5 packet sequences below describe the preserved monolithic
M5i reference and remain useful when comparing the new Embassy-based capture.
The layered implementation performs the same USB requests through
`embassy-usb-host`, but retry timing and the exact number of intervening SOFs
may differ.

For an electrical analyzer-path check, flash the preserved legacy transmitter
in its deterministic capture mode:

```console
cargo run --release --example bleuio --features analyzer-capture
```

After a full-speed attach it emits a fixed `SOF0` every millisecond and shows
nine LED pulses. Without `analyzer-capture`, the same example name continues
to build the layered generic CDC-ACM/BleuIO application described above.

The M4b control probe should additionally show:

1. SETUP + DATA0 containing `80 06 00 01 00 00 08 00`, then device ACK;
2. IN + device DATA1 containing eight descriptor bytes, then host ACK;
3. OUT + zero-length DATA1, then device ACK;
4. SETUP + DATA0 containing `00 05 01 00 00 00 00 00`, then device ACK;
5. IN + device DATA1 zero-length packet, then host ACK;
6. two recovery frames followed by the same three descriptor-read stages at
   device address 1.

Firmware stops retrying only after the descriptor prefix read at address 1
matches the prefix read at address zero.

The M4c capture should then show an address-1
`GET_DESCRIPTOR(Device, 18)` data stage split into DATA1/DATA0/DATA1 payloads
of 8, 8, and 2 bytes, with a host ACK after each packet, followed by the
zero-length DATA1 status stage. If a device persistently NAKs that status
stage, a fresh SETUP should appear after four NAK responses.

The M5a capture should continue with two address-1 requests for configuration
descriptor index zero. The first requests 9 bytes and normally returns
DATA1/8 plus DATA0/1. The second requests exactly the reported
`wTotalLength`, alternates DATA1/DATA0 for every full endpoint-zero packet,
and ends with the control-read OUT/DATA1 zero-length status stage.

M5b should then show SETUP + DATA0 containing
`00 09 01 00 00 00 00 00` at address 1, followed by a device ACK. Its status
stage is IN + device DATA1 zero-length packet + host ACK.

M5c should follow with SETUP + DATA0 containing
`21 22 03 00 00 00 00 00` at address 1. This targets control interface zero
and asserts DTR plus RTS. It must likewise receive a device ACK followed by an
IN/DATA1 zero-length status packet and a final host ACK.

M5d should then show SETUP + DATA0 containing
`21 20 00 00 00 00 07 00`, followed by the device ACK. Its data stage is:

1. OUT to address 1, endpoint zero;
2. DATA1 containing `00 C2 01 00 00 00 08` for 115200 8N1;
3. device ACK.

The transfer ends with IN + device DATA1 zero-length packet + host ACK.

M5e should then show the first CDC bulk transaction on the descriptor-selected
OUT endpoint. For the target endpoint 2, address 1, it is:

1. OUT token `80 E1 01 C1`;
2. DATA0 packet `80 C3 41 54 0D 0A 2E A0`, whose payload is `AT\r\n`;
3. device ACK `80 D2`.

There is no control-style status stage after a bulk transfer.

M5f should then poll the descriptor-selected bulk-IN endpoint. For the target
endpoint 1 at address 1:

1. each IN token is `80 69 81 58`;
2. the dongle may return one or more NAK handshakes `80 5A`;
3. the first new data packet normally uses DATA0 and carries a CRC-16-valid,
   non-empty payload of at most 64 bytes; if the dongle first sends a valid
   DATA0 zero-length packet, the following non-empty packet instead uses
   DATA1;
4. the host answers that packet with ACK `80 D2`.

The exact response bytes are intentionally established from the Beagle capture
instead of being hard-coded before the first hardware run.

The M5f capture establishes the first response packet exactly:

1. IN token `80 69 81 58`;
2. DATA0 packet `80 C3 41 80 8F`, containing the single byte `A`;
3. host ACK `80 D2`.

M5g should continue polling endpoint 1 after that short packet. Every new
CRC-valid data packet alternates DATA1/DATA0 and receives a host ACK. NAKs may
appear between characters without changing the expected toggle. Concatenating
the payload bytes, independently of packet boundaries, must eventually produce
an exact CRLF-delimited `OK` line.

M5h should then continue on the already configured endpoints:

1. the next OUT endpoint-2 data packet uses DATA1 and contains the exact
   `AT+CENTRAL\r\n` packet shown above;
2. the dongle ACKs that packet;
3. endpoint-1 polling continues with the retained bulk-IN toggle, which is
   DATA1 after the three acknowledged M5g response packets in the captured
   sequence;
4. every new response packet alternates only after ACK, and their concatenated
   payload contains the echoed command followed by an exact `OK` line;
5. no new bus reset, `SET_CONFIGURATION`, or endpoint toggle reset occurs.

M5i should continue without resetting either toggle:

1. the next endpoint-2 packet uses DATA0 and is exactly the
   `AT+GAPSCAN=1\r\n` packet shown above;
2. the dongle ACKs it;
3. endpoint-1 polling and host ACKs retain the independent IN toggle while
   reconstructing the echoed command, `SCANNING...`, zero or more result
   lines, and `SCAN COMPLETE`;
4. at least one result line has the form
   `[index] Device: [type]XX:XX:XX:XX:XX:XX RSSI: value`;
5. no standalone `OK` is required to complete this timed scan.

Only after `SCAN COMPLETE` and a parsed device result does the firmware enter
the slow six-pulse success state.

## Roadmap

- **M1:** complete and verified on hardware.
- **M2:** complete and verified on hardware.
- **M3:** complete and verified on hardware with a Beagle USB 480 analyzer.
- **M4:** complete; M4a, M4b, and M4c are verified on the BLE dongle with the
  Beagle USB 480 analyzer.
- **M5:** complete in the legacy wire-level implementation. CDC discovery,
  configuration, control requests, bulk IN/OUT, persistent toggles, BleuIO
  central mode, and timed GAP scanning are verified on hardware and with the
  analyzer.
- **M6:** complete and verified on hardware with a Beagle USB 480 analyzer.
  The RP2040 packet engine implements the official Embassy host traits; the
  generic `CdcAcmHost` and layered BleuIO example pass host tests, Clippy, and
  release linking. The `beagle-layered-irqguard.csv` acceptance capture shows
  standard enumeration, CDC control requests, bulk IN/OUT, acknowledged long
  scan-result packets, and the terminal `SCAN COMPLETE` response; the example
  reports its two-pulse success pattern.
- **M7:** complete and verified on hardware with a Beagle USB 480. The generic
  raw `HidHost`, strict HID descriptor discovery, Velleman P8055 product
  example, controller speed plumbing, and RP2040 low-speed PIO profile are
  implemented and covered by host-side tests. The
  `beagle-p8055-reset.csv` capture shows a 20 ms low-speed reset, 1 kHz
  keep-alives, complete address/configuration/HID enumeration, the 29-byte
  report descriptor, one acknowledged eight-byte interrupt OUT reset report,
  and one acknowledged eight-byte interrupt IN report.
- **Later:** add further HID product profiles and class drivers such as mass
  storage over the same bus handle, then add hub/split support independently
  of each class API.

## Credits

The PIO TX/RX timing models are adapted from
[Pico-PIO-USB](https://github.com/sekigon-gonnoc/Pico-PIO-USB). See
`THIRD_PARTY_NOTICES.md` for its MIT license.

## License

Except for third-party material identified in
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md), this project is licensed
under either of

- Apache License, Version 2.0
  ([LICENSE-APACHE](LICENSE-APACHE)); or
- MIT License ([LICENSE-MIT](LICENSE-MIT))

at your option.

Unless explicitly stated otherwise, contributions intentionally submitted for
inclusion in this project are licensed under the same terms.
