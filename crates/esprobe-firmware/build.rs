/// Environment that changes generated code, and so must invalidate the build.
///
/// `option_env!` is read at compile time but cargo does not know that, so
/// without these a pin-map or credential change silently reuses the previous
/// binary. For the pin map that is not a stale build, it is a firmware that
/// drives the wrong pads into another board's outputs.
const TRACKED: [&str; 11] = [
    "PIN_SWDIO",
    "PIN_SWCLK",
    "PIN_RESET_ALL",
    "PIN_ASW_S0",
    "PIN_ASW_S1",
    "PIN_DISP_TX",
    "PIN_DISP_RX",
    "WIFI_SSID",
    "WIFI_PASSWORD",
    "WIFI_SSID_FALLBACK",
    "WIFI_PASSWORD_FALLBACK",
];

fn main() {
    for variable in TRACKED {
        println!("cargo:rerun-if-env-changed={variable}");
    }
    if std::env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("espidf") {
        embuild::espidf::sysenv::output();
    }
}
