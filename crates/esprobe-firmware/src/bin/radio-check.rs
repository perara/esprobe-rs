//! Brings up nothing but the radio, to tell a firmware fault from a hardware one.
//!
//! The bridge firmware claims seven GPIOs, enables GPSPI2, installs the USB
//! Serial/JTAG driver and runs two servers before Wi-Fi ever associates. When
//! the radio misbehaves, any of that is a suspect. This image is the control:
//! an access point and a scan, and nothing else at all. If its access point is
//! visible from another machine, the radio works and the bridge firmware is at
//! fault; if it is not, no amount of firmware will fix it.
//!
//! Flash it the same way as the bridge, look for the SSID below, then flash the
//! bridge back.

#[cfg(not(target_os = "espidf"))]
fn main() {
    panic!("build this firmware for riscv32imc-esp-espidf");
}

#[cfg(target_os = "espidf")]
fn main() -> anyhow::Result<()> {
    use embedded_svc::wifi::{
        AccessPointConfiguration, AuthMethod, ClientConfiguration, Configuration,
    };
    use esp_idf_svc::eventloop::EspSystemEventLoop;
    use esp_idf_svc::hal::peripherals::Peripherals;
    use esp_idf_svc::nvs::EspDefaultNvsPartition;
    use esp_idf_svc::wifi::{BlockingWifi, EspWifi};
    use log::info;

    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take()?;
    let sys_loop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;
    let mut wifi = BlockingWifi::wrap(
        EspWifi::new(peripherals.modem, sys_loop.clone(), Some(nvs))?,
        sys_loop,
    )?;

    // Open, so nothing about the passphrase can be blamed either. Mixed with
    // an unconfigured station purely so scanning is allowed: the point is to
    // show receive and transmit in the same run, on the same radio, seconds
    // apart.
    wifi.set_configuration(&Configuration::Mixed(
        ClientConfiguration::default(),
        AccessPointConfiguration {
            ssid: "esprobe-radio-check".try_into().unwrap(),
            auth_method: AuthMethod::None,
            channel: 1,
            ..Default::default()
        },
    ))?;
    wifi.start()?;
    info!("Access point 'esprobe-radio-check' started on channel 1");

    // Steps the transmit power down over time. If nothing is heard at 20 dBm
    // but beacons appear at 5, the power amplifier is browning out its supply
    // rather than radiating into a broken antenna — the two hardware faults
    // that both look like "receive works, transmit does not", and they need
    // completely different fixes. Quarter-dBm units, as the API takes them.
    const TX_POWER_STEPS: [i8; 5] = [80, 52, 32, 20, 8];

    // What the radio can hear, next to what it claims to transmit. A scan that
    // works while the access point stays invisible is receive without
    // transmit, which is the whole question.
    for step in TX_POWER_STEPS.into_iter().cycle() {
        unsafe { esp_idf_svc::sys::esp_wifi_set_max_tx_power(step) };
        std::thread::sleep(std::time::Duration::from_secs(10));
        let max_tx_power = unsafe {
            let mut power = 0i8;
            esp_idf_svc::sys::esp_wifi_get_max_tx_power(&mut power);
            power
        };
        match wifi.scan() {
            Ok(found) => info!(
                "tx power {}.{} dBm, {} access points heard, strongest {:?}",
                max_tx_power / 4,
                (max_tx_power % 4) * 25,
                found.len(),
                found
                    .iter()
                    .max_by_key(|point| point.signal_strength)
                    .map(|point| (point.ssid.as_str(), point.signal_strength)),
            ),
            Err(error) => info!("tx power {max_tx_power}, scan failed: {error}"),
        }
    }
    unreachable!("the power sweep cycles forever")
}
