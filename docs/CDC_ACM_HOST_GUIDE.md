# Använd RP2040 PIO USB Host med CDC-ACM i ett nytt Embassy-projekt

Den här guiden visar hur ett nytt Rust/Embassy-projekt använder
`embassy-rp-pio-usb-host` för att kommunicera med en descriptor-kompatibel
CDC-ACM-enhet.

Guiden gäller den nu verifierade RP2040-backenden på **Adafruit Feather RP2040
med USB Type A Host**:

- D+ på GPIO16;
- D− på GPIO17;
- GPIO18 som enable-signal till kortets strömbegränsade 5 V-switch;
- PIO0, PIO1 och DMA-kanal 0 reserverade för USB Host;
- exakt 120 MHz systemklocka;
- en direktansluten full- eller low-speed-enhet.

`CdcAcmHost` är generell och descriptorstyrd. Baudrate, DTR/RTS,
kommandosyntax, radprotokoll och svarstolkning tillhör däremot applikationen.
BleuIO-exemplet är därför ett exempel på ett produktprotokoll ovanpå
CDC-strömmen, inte en del av class-drivern. CDC-ACM använder bulk-endpoints
och kör därför på full speed med den nuvarande RP2040-backenden; low-speed-
stödet används av exempelvis den generella HID-drivern.

## Översikt

Ett nytt projekt gör följande:

1. lägger till biblioteket och matchande Embassy-versioner;
2. konfigurerar RP2040-target, linker och UF2-runner;
3. initierar RP2040 med 120 MHz;
4. skapar `Rp2040PioEngine`, `PioHostState` och Embassy-bussen;
5. kör root-port-monitorn parallellt med applikationen;
6. väntar på attach och enumererar enheten;
7. låter `allocate_from_enumeration` hitta CDC-ACM-funktionen från hela
   configuration descriptor;
8. gör eventuella class requests och använder `CdcAcmHost` som en
   `embedded-io-async` byte stream;
9. droppar alla pipes innan USB-adressen frigörs efter detach.

Den snabbaste fungerande startpunkten är den produktneutrala
[`src/main.rs`](../src/main.rs). Kopiera den till det nya projektet och ersätt
den neutrala kontrollproben med det egna applikationsprotokollet.

## 1. Skapa projektet

```console
cargo new --bin my-cdc-host
cd my-cdc-host
rustup target add thumbv6m-none-eabi
cargo install elf2uf2-rs
```

Biblioteket är ännu inte publicerat på crates.io. Använd ett Git dependency
låst till en testad commit:

```toml
embassy-rp-pio-usb-host = {
    git = "https://github.com/ulso/embassy-rp-pio-usb-host",
    rev = "<testad commit-SHA>",
    default-features = false,
    features = ["embassy-usb-host"],
}
```

Under samtidig lokal utveckling kan Git-raderna ersättas med
`path = "../embassy-rp-pio-usb-host"`.

Ett minimalt `Cargo.toml` för samma kort är:

```toml
[package]
name = "my-cdc-host"
version = "0.1.0"
edition = "2024"

[dependencies]
embassy-rp-pio-usb-host = {
    path = "../embassy-rp-pio-usb-host",
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
lto = "fat"
opt-level = "s"
codegen-units = 1
```

Behåll versionerna synkroniserade med bibliotekets
[`Cargo.toml`](../Cargo.toml). Host-traits som råkar komma från två
inkompatibla Embassy-versioner är olika Rust-typer även om de heter samma sak.
Releaseprofilen är också relevant eftersom PIO-backendens verifierade
timingvägar är byggda med denna profil.

## 2. Äg target- och linkerkonfigurationen i applikationen

Ett dependency bestämmer inte det konsumerande programmets minneslayout.
Projektet som bygger den slutliga firmwaren måste själv äga `memory.x`,
linkerargumenten och UF2-runnern. Om projektet redan har en fungerande
RP2040-konfiguration ska den behållas och anpassas för det aktuella kortets
flashstorlek.

För ett nytt projekt kan följande filer användas som utgångspunkt:

```console
export PIO_USB_HOST_REPO=/path/to/embassy-rp-pio-usb-host

mkdir -p .cargo
cp "$PIO_USB_HOST_REPO/.cargo/config.toml" .cargo/config.toml
cp "$PIO_USB_HOST_REPO/rust-toolchain.toml" rust-toolchain.toml
cp "$PIO_USB_HOST_REPO/memory.x" memory.x
```

Skapa därefter applikationens egen `build.rs`:

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

Tillsammans ger filerna projektet:

- default-target `thumbv6m-none-eabi`;
- `elf2uf2-rs --deploy` som Cargo-runner;
- `stable` Rust med `rust-src`, `rustfmt` och Cortex-M0+-target;
- Cortex-M-linkerskript och `defmt`-sektioner;
- minneslayout för Feather-kortets 8 MiB flash och 264 KiB RAM.

`memory.x` är kortspecifik. Kontrollera framför allt flashstorleken innan
samma fil används för ett annat RP2040-kort. Host-bibliotekets egen
`build.rs` kopplar bara det här repositoryts 8 MiB-layout till dess egna
binärer och exempel; den exporterar inte layouten till ett konsumerande
projekt.

## 3. Reservera hårdvaruresurserna

Den nuvarande konkreta backenden använder fasta resurser:

| Funktion | RP2040-resurs |
|---|---|
| USB TX | PIO0 SM0, 48 MHz full speed / 6 MHz low speed |
| USB RX | PIO1 SM0, 120 MHz |
| kant/EOP-detektering | PIO1 SM1, 96 MHz full speed / 12 MHz low speed |
| PIO1 instruktionsminne | helt upptaget, 15 + 17 instruktioner |
| reserverad DMA-resurs | DMA channel 0, inklusive IRQ |
| D+ | GPIO16 |
| D− | GPIO17 |

Applikationen får inte samtidigt ge dessa resurser till någon annan driver.
På Feather-kortet använder exempelapplikationen dessutom GPIO18 som
enable-signal till den strömbegränsade 5 V-switchen och GPIO13 för status-LED.
De två pinnarna tillhör kortapplikationen och ägs inte av
`Rp2040PioEngine`.

En annan pinout kräver för närvarande ändringar i den konkreta backenden.
Det räcker inte att bara byta argument vid konstruktionen, eftersom
registeråtkomst, input-invertering och PIO-konfiguration är hårdkodade och
verifierade för exakt GPIO16/GPIO17.

## 4. Bind IRQ och skapa host-motorn

Firmwarefilen börjar som ett vanligt Embassy `no_std`-program:

```rust
#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::bind_interrupts;
use embassy_rp::clocks::ClockConfig;
use embassy_rp::dma::InterruptHandler as DmaInterruptHandler;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIO0, PIO1};
use embassy_rp::pio::InterruptHandler as PioInterruptHandler;
use embassy_time::{Duration, Ticker, Timer};

use embassy_rp_pio_usb_host::cdc_acm::allocate_from_enumeration;
use embassy_rp_pio_usb_host::host::{
    DeviceEvent, Speed, UsbHostController,
};
use embassy_rp_pio_usb_host::pio_host::PioHostState;
use embassy_rp_pio_usb_host::pio_host::rp2040::Rp2040PioEngine;
use embassy_rp_pio_usb_host::{
    AttachDetector, BusEvent, DeviceSpeed,
};
use embassy_usb_host::{BusController, BusRoute, BusState};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    PIO1_IRQ_0 => PioInterruptHandler<PIO1>;
    DMA_IRQ_0 => DmaInterruptHandler<DMA_CH0>;
});

const SYS_CLOCK_HZ: u32 = 120_000_000;
const CONFIG_DESCRIPTOR_CAPACITY: usize = 512;
```

Koden i de följande avsnitten placeras i Embassy-entrypointen:

```rust
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Initiering, bus setup och join(...) från avsnitten nedan.
}
```

Initiera därefter klocka, pins och bus storage i denna ordning:

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

`Rp2040PioEngine::new` kontrollerar att `clk_sys` verkligen är 120 MHz.
`PioHostState::controller()` får bara anropas en gång.

I den verifierade strukturen ligger `host_state` och `bus_state` i samma
stack frame som två futures som aldrig avslutas. Om monitorn i stället spawnas
som en fristående Embassy-task måste båda ligga i `'static` storage, till
exempel `StaticCell`.

## 5. Kör root-port-monitorn parallellt

Root-port-monitorn är en del av hostens funktion, inte valfri diagnostik. Den:

- debouncar attach och detach;
- utför USB reset före anslutningseventet;
- ignorerar hostens egen SE0 medan reset pågår;
- genererar SOF ungefär varje millisekund när ingen pipe använder motorn.

Använd följande mönster:

```rust
async fn root_port_monitor<'d>(
    host: &PioHostState<Rp2040PioEngine<'d>>,
) {
    let mut detector = AttachDetector::new(100);
    let mut ticker = Ticker::every(Duration::from_millis(1));
    let mut connected = false;

    loop {
        ticker.next().await;

        let Some(line_state) = host.root_line_state_if_not_resetting() else {
            continue;
        };

        if let Some(event) = detector.update(line_state) {
            match event {
                BusEvent::Attached(DeviceSpeed::Full) => {
                    connected = host
                        .reset_and_report_connected(Speed::Full)
                        .await
                        .is_ok();

                    if connected {
                        // Ta inte igen ticks som förbrukades under reset.
                        ticker = Ticker::every(Duration::from_millis(1));
                    }
                }
                BusEvent::Attached(DeviceSpeed::Low)
                | BusEvent::Detached
                | BusEvent::Invalid => {
                    if connected
                        && host.report_disconnected_if_not_resetting()
                    {
                        connected = false;
                    }
                }
            }
        }

        if connected {
            // En aktiv transfer äger motorn och genererar själv SOF.
            let _ = host.service_frame().await;
        }
    }
}
```

Kör monitorn tillsammans med applikationsloopen:

```rust
let application = async move {
    // Behåll VBUS-enable och kör anslutningsloopen här.
    let _vbus_enable = vbus_enable;
    // ...
};

join(root_port_monitor(&host_state), application).await;
```

## 6. Enumerera och skapa `CdcAcmHost`

Den centrala anslutningsloopen ser ut så här:

```rust
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
            // Vänta på fysisk detach innan nästa försök.
            wait_for_detach(&mut controller).await;

            // Workaround för address leaks på vissa felvägar i
            // embassy-usb-host 0.1.0. Säker här eftersom backenden bara
            // stöder en direkt root-enhet.
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

        // SET_CONFIGURATION är redan utförd av enumerate().
        // Starta bulk-IN och bulk-OUT med DATA0.
        cdc.reset_data_toggles();

        // Gör class requests och kör produktprotokollet här.
        // Exempel: let _session_result = communicate(&mut cdc).await;

        // Behåll cdc vid liv tills detach har konsumerats.
        wait_for_detach(&mut controller).await;
    } // CdcAcmHost och samtliga pipes droppas här.

    bus_handle.free_address(address);
}
```

Detach-hjälparen behöver ett wildcard eftersom `DeviceEvent` är
`non_exhaustive`:

```rust
async fn wait_for_detach<'d, C>(
    controller: &mut BusController<'d, C>,
) where
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

Tre detaljer är viktiga:

1. Skicka alltid hela `&configuration[..configuration_len]` till
   allokeringsfunktionen.
2. Anta inte interface- eller endpointnummer. Discovery följer CDC Union
   Functional Descriptor från communications-interface till rätt
   data-interface.
3. Droppa `CdcAcmHost` och alla pipes innan
   `bus_handle.free_address(address)`.

`allocate_from_enumeration` väljer den första giltiga CDC-ACM-funktionen.
Om en composite-enhet har flera ACM-funktioner används i stället
`allocate_from_enumeration_for_control_interface`.

## 7. Konfigurera endast annonserade CDC-funktioner

Alla CDC-ACM-enheter kräver inte line coding eller modemsignaler. Kontrollera
descriptorns capability innan class requests:

```rust
use embassy_rp_pio_usb_host::usb::CdcLineCoding;

if cdc.function().supports_line_requests() {
    // Dessa värden är applikationspolicy, inte USB Host-policy.
    cdc.set_line_coding(CdcLineCoding::eight_n_one(115_200))
        .await?;
    cdc.set_control_line_state(true, false).await?; // DTR, RTS
}

if cdc.function().supports_send_break() {
    cdc.send_break(250).await?;
}
```

Välj baudrate, DTR/RTS och ordning enligt enhetens protokoll. En enhet som inte
annonserar line requests kan fortfarande vara fullt användbar via bulk-IN och
bulk-OUT.

Anropa `reset_data_toggles()` en gång efter `SET_CONFIGURATION` eller efter
ett byte av data-interfacets alternate setting. Anropa den aldrig mellan
vanliga kommandon, reads eller writes. IN- och OUT-toggles är oberoende,
pipeägda och avanceras endast efter ACK.

## 8. Lägg produktprotokollet ovanpå byte-strömmen

`CdcAcmHost` implementerar både:

- `embedded_io_async::Read`;
- `embedded_io_async::Write`.

En transportoberoende protokollfunktion kan därför se ut så här:

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

        // Mata bytes till en persistent parser/framer.
        // Ett read motsvarar inte nödvändigtvis ett helt svar.
        let _ = bytes;
    }
}
```

Det här protokollagret bör inte känna till USB PID, DATA0/DATA1, NAK-retries,
SOF eller endpointnummer.

### Semantik för `read`

- Det är en byte stream utan USB-paket- eller meddelandegränser.
- Ett anrop kan returnera färre bytes än buffertens storlek.
- Om anroparens buffert är mindre än ett USB-paket behåller class-drivern
  resten till nästa anrop.
- USB zero-length packets hoppas över och betyder inte EOF.
- Bulk-NAK kan göra att ett anrop väntar länge; använd en
  applikationstimeout när protokollet behöver en deadline.

Exempel:

```rust
use embassy_time::{Duration, with_timeout};

let result = with_timeout(
    Duration::from_secs(2),
    cdc.read(&mut receive_buffer),
)
.await;
```

### Semantik för `write`

- Bufferten USB-paketiseras automatiskt.
- En lyckad retur betyder att hela bufferten transporterades.
- Ingen extra avslutande ZLP läggs till.
- `flush()` är för närvarande en no-op eftersom transporten redan är klar;
  det bevisar inte att enheten har tolkat kommandot.

Ett protokoll som kräver ZLP-terminering behöver använda ett lägre
pipe-gränssnitt eller en framtida explicit API-funktion.

## 9. Hantera detach och återanslutning

Vid fysisk detach gör root-port-monitorn alla pipes från den gamla
anslutningen ogiltiga. Pågående I/O returnerar normalt
`PipeError::Disconnected`, vilket `CdcAcmError` mappar till
`embedded_io_async::ErrorKind::NotConnected`.

Livscykeln ska vara:

1. avbryt eller låt protokollsessionen lämna sitt I/O-anrop;
2. droppa protokollklienten;
3. droppa `CdcAcmHost` och därmed control-, bulk-IN- och bulk-OUT-pipes;
4. frigör den gamla USB-adressen;
5. vänta på ny attach;
6. enumerera och allokera helt nya pipes.

Återanvänd aldrig en class-instans eller pipe från en tidigare fysisk
anslutning.

Om applikationssessionen ska köras tills antingen I/O felar eller en
controller-event anländer kan den kombineras med
`embassy_futures::select`. Se det verifierade livstidsmönstret i
[`src/main.rs`](../src/main.rs).

## 10. Vad som är generellt och vad som är produktspecifikt

Följande hör hemma i det generella USB/CDC-lagret:

- USB enumeration;
- descriptor discovery;
- val och allokering av pipes;
- standardiserade CDC class requests;
- USB-paketisering, ACK/NAK, retries och DATA-toggles;
- en asynkron byte stream.

Följande hör hemma i applikationen:

- baudrate och DTR/RTS-policy;
- kommandon och svar;
- rad-, ram- eller binär framing;
- timeouts och retries på protokollnivå;
- validering och tolkning av produktdata.

Kopiera därför inte följande från BleuIO-exemplet om den nya enheten inte
uttryckligen kräver det:

- `AT\r\n`, `AT+CENTRAL` eller GAP-scan;
- 115200 baud;
- DTR/RTS satta till `true`;
- BleuIO-parsern eller dess 10-sekunderstimeout;
- LED- och analyzerdiagnostik;
- VID/PID-antaganden;
- kod från `legacy_m5i.rs`.

Det återanvändbara mönstret i BleuIO-exemplet är i stället att lägga ett
transportoberoende protokoll ovanpå `embedded-io-async::Read + Write`.

## 11. Nuvarande begränsningar

RP2040-backenden stöder för närvarande:

- en direktansluten full- eller low-speed-enhet;
- full-speed endpoint zero, bulk och interrupt med högst 64 byte;
- low-speed endpoint zero och interrupt med högst 8 byte och minst 10 ms
  pollingintervall;
- ingen hubb och inga split transactions;
- inte high speed eller isochronous transfers.

`CdcAcmHost` kan konstrueras med 512 bytes intern RX-paketbuffert för en annan
high-speed-controller. Det ger inte RP2040-backenden high-speed-signalering.

En CDC notification-IN-endpoint upptäcks om den finns, men konsumeras ännu
inte av stream-drivern.

## 12. Bygg och flasha via USB-bootloader

Bygg utan att flasha:

```console
cargo build --release
```

Flasha utan SWD:

1. koppla från kortet;
2. håll **BOOTSEL** intryckt;
3. anslut kortet och släpp **BOOTSEL** när volymen `RPI-RP2` visas;
4. kör:

```console
cargo run --release
```

Runnern skapar UF2-filen, kopierar den till `RPI-RP2` och startar firmware.
Felet `Unable to find mounted pico` betyder att kortet inte är monterat i
BOOTSEL-läge.

`defmt-rtt` kräver SWD för att läsas. Utan SWD används i stället exempelvis
status-LED, oscilloskop eller USB-protokollanalysator för boarddiagnostik.

## 13. Felsökningschecklista

### Ingen attach registreras

- kontrollera att enheten är full speed;
- kontrollera D+ GPIO16 och D− GPIO17;
- kontrollera att GPIO18 har aktiverat kortets VBUS-switch;
- kontrollera att root-port-monitorn faktiskt körs;
- kontrollera att ingen annan kod använder PIO0, PIO1 eller DMA0;
- kontrollera exakt 120 MHz `clk_sys`.

### Enumeration lyckas men ingen CDC-ACM hittas

- skicka hela configuration descriptor till `allocate_from_enumeration`;
- kontrollera att enheten verkligen annonserar CDC communications/data;
- kontrollera CDC Union Functional Descriptor;
- anta inte att första bulk-IN och bulk-OUT tillhör samma funktion;
- använd interface-specifik allokering om composite-enheten har flera ACM.

### Class request ger STALL

- kontrollera `supports_line_requests()` och `supports_send_break()`;
- skicka bara requests som descriptorn annonserar;
- kontrollera att baudrate och modemsignaler krävs av den aktuella produkten.

### Första kommandot fungerar men följande data blir fel

- nollställ inte DATA-toggles mellan kommandon;
- behandla CDC som en ström, inte som ett USB-paket;
- behåll parserstate över flera `read()`-anrop;
- lägg framing och protokolltimeout ovanpå streamen.

### Återanslutning slutar fungera

- konsumera detach-eventet;
- droppa alla pipes före `free_address`;
- skapa en helt ny class-instans efter nästa enumeration;
- använd den defensiva address-workarounden för enumeration errors så länge
  projektet ligger kvar på `embassy-usb-host 0.1.0`.

## Referensimplementationer

- [`src/main.rs`](../src/main.rs): produktneutral CDC-ACM-livscykel;
- [`src/cdc_acm.rs`](../src/cdc_acm.rs): generell class driver och API;
- [`src/pio_host.rs`](../src/pio_host.rs): Embassy controller/allocator/pipes;
- [`src/pio_host/rp2040.rs`](../src/pio_host/rp2040.rs): konkret PIO-backend;
- [`examples/bleuio/app.rs`](../examples/bleuio/app.rs): produktspecifikt
  protokoll ovanpå den generella CDC-strömmen;
- [`examples/bleuio/protocol.rs`](../examples/bleuio/protocol.rs):
  transportoberoende streamparser;
- [`tests/bleuio_protocol.rs`](../tests/bleuio_protocol.rs): test av parsern
  oberoende av USB och RP2040.
