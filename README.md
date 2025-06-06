A 1-Wire DS18B20 temperature sensor driver for ESP32 devices.

Requires `esp-hal`. Use with `esp-onewire`.

```rust
use embassy_time::{Duration, Timer};
use esp_ds18b20::{DS18B20Error, Ds18b20, Resolution, SensorData};
use esp_onewire::OneWireBus;

const TEMP_SENSOR_ADDRESS: u64 = 0x0011223344556677;

// Set up the bus and sensor.
let onewire_bus = OneWireBus::new(onewire_pin);
let mut sensor = Ds18b20::new(TEMP_SENSOR_ADDRESS, onewire_bus)?;

// Begin a measurement and wait for it to complete.
sensor.start_temp_measurement()?;
// 12bit resolution is the default, expects a 750ms wait time.
let wait_time_ms = Resolution::Bits12.max_measurement_time_ms();
let wait_time = Duration::from_millis(wait_time_ms as u64);
Timer::after(wait_time).await;

// Retrieve the measurement.
let data = sensor.read_sensor_data()?;
```
