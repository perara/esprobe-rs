/// Environment that changes generated code, and so must invalidate the build.
///
/// `option_env!` is read at compile time but cargo does not know that, so
/// without these a pin-map or credential change silently reuses the previous
/// binary. For the pin map that is not a stale build, it is a firmware that
/// drives the wrong pads into another board's outputs.
///
/// This list went stale once already: it kept naming the pins of a product
/// that had moved to its own repository, so the five overrides this firmware
/// actually reads were untracked and changing one rebuilt nothing.
const TRACKED: [&str; 9] = [
    "PIN_SWDIO",
    "PIN_SWCLK",
    "PIN_RESET",
    "PIN_UART_TX",
    "PIN_UART_RX",
    "WIFI_SSID",
    "WIFI_PASSWORD",
    "WIFI_SSID_FALLBACK",
    "WIFI_PASSWORD_FALLBACK",
];

/// The parts `chip.rs` has a register map for.
///
/// `esp-idf-sys` emits one of these as a `cfg` from the build's own sdkconfig.
/// Declaring them here is what stops `unexpected_cfgs` firing on every arm —
/// and, more usefully, means a typo in a chip name is a warning rather than an
/// arm that silently never compiles.
const CHIPS: [&str; 6] = [
    "esp32", "esp32s2", "esp32s3", "esp32c3", "esp32c6", "esp32h2",
];

fn main() {
    for variable in TRACKED {
        println!("cargo:rerun-if-env-changed={variable}");
    }
    for chip in CHIPS {
        println!("cargo::rustc-check-cfg=cfg({chip})");
    }
    if std::env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("espidf") {
        embuild::espidf::sysenv::output();
    }
}
