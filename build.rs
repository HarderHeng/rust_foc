//! Build script for STM32G431 FOC project

fn main() {
    // Put linker script in build directory
    println!("cargo:rustc-linker-search-path={}", std::env::var("OUT_DIR").unwrap());

    // Rebuild if linker script changes
    println!("cargo:rerun-if-changed=memory.x");
}