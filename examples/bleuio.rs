#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

#[cfg(target_os = "none")]
use defmt_rtt as _;
#[cfg(target_os = "none")]
use embassy_executor::Spawner;
#[cfg(target_os = "none")]
use panic_probe as _;

#[cfg(all(target_os = "none", not(feature = "analyzer-capture")))]
#[path = "bleuio/app.rs"]
mod app;

#[cfg(all(target_os = "none", feature = "analyzer-capture"))]
#[path = "bleuio/legacy_m5i.rs"]
mod app;

#[cfg(target_os = "none")]
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    app::run(spawner).await;
}

#[cfg(not(target_os = "none"))]
fn main() {}
