use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").expect("OUT_DIR is set by Cargo"));
    File::create(out.join("memory.x"))
        .expect("create memory.x")
        .write_all(include_bytes!("memory.x"))
        .expect("write memory.x");

    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");
    if env::var("TARGET").as_deref() == Ok("thumbv6m-none-eabi") {
        println!("cargo:rustc-link-arg-bins=--nmagic");
        println!("cargo:rustc-link-arg-bins=-Tlink.x");
        println!("cargo:rustc-link-arg-bins=-Tlink-rp.x");
        println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
        println!("cargo:rustc-link-arg-examples=--nmagic");
        println!("cargo:rustc-link-arg-examples=-Tlink.x");
        println!("cargo:rustc-link-arg-examples=-Tlink-rp.x");
        println!("cargo:rustc-link-arg-examples=-Tdefmt.x");
    }
}
