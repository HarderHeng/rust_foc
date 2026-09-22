//! Build script for STM32G431 FOC project

fn main() {
    // Rebuild if linker script changes
    println!("cargo:rerun-if-changed=memory.x");
}
