#![no_std]

use esp_onewire::{OneWireBus, OneWireBusError};
use thiserror::Error;

const FAMILY_CODE: u8 = 0x28;

/// A DS18B20 temperature sensor on a 1-Wire bus.
pub struct Ds18b20 {
    address: u64,
    bus: OneWireBus,
}

/// Sensor resolution.
#[derive(Copy, Clone, Debug)]
pub enum Resolution {
    Bits9,
    Bits10,
    Bits11,
    Bits12,
}

/// Sensor data reading.
#[derive(Clone, Copy, Debug)]
pub struct SensorData {
    /// Temperature readout, in Celsius.
    pub temperature: f32,

    /// Sensor resolution (defaults to 12 bits).
    pub resolution: Resolution,

    /// Low temperature alarm setting.
    pub low_temp_alarm: i8,

    /// High temperature alarm setting.
    pub high_temp_alarm: i8,
}

/// Temperature sensor errors.
#[derive(Clone, Copy, Debug, Error)]
pub enum Ds18b20Error {
    /// The 1-Wire bus reported an error.
    #[error("1wire bus error")]
    OneWireError(#[from] OneWireBusError),

    /// Received an invalid resolution byte.
    #[error("received an invalid resolution byte")]
    InvalidResolution,

    /// Family code of sensor address does not match the expected device.
    #[error("family code of sensor address does not match the expected device")]
    FamilyCodeMismatch,
}

const CMD_CONVERT_TEMP: u8 = 0x44;
// const CMD_COPY_SCRATCHPAD: u8 = 0x48;
const CMD_READ_SCRATCHPAD: u8 = 0xBE;
// const CMD_RECALL_EEPROM: u8 = 0xB8;
// const CMD_WRITE_SCRATCHPAD: u8 = 0x4E;

impl Ds18b20 {
    pub fn new(address: u64, bus: OneWireBus) -> Result<Self, Ds18b20Error> {
        if address.to_le_bytes()[0] == FAMILY_CODE {
            Ok(Self { address, bus })
        } else {
            Err(Ds18b20Error::FamilyCodeMismatch)
        }
    }

    pub fn start_temp_measurement(&mut self) -> Result<(), Ds18b20Error> {
        self.bus.send_command(CMD_CONVERT_TEMP, self.address)?;
        Ok(())
    }

    pub fn read_sensor_data(&mut self) -> Result<SensorData, Ds18b20Error> {
        let scratchpad = self.read_scratchpad()?;

        let resolution = Resolution::try_from(scratchpad[4])?;

        let raw_temp = i16::from_le_bytes([scratchpad[0], scratchpad[1]]);
        let temperature = match resolution {
            Resolution::Bits12 => (raw_temp as f32) / 16.0,
            Resolution::Bits11 => (raw_temp as f32) / 8.0,
            Resolution::Bits10 => (raw_temp as f32) / 4.0,
            Resolution::Bits9 => (raw_temp as f32) / 2.0,
        };

        Ok(SensorData {
            temperature,
            resolution,
            high_temp_alarm: i8::from_le_bytes([scratchpad[2]]),
            low_temp_alarm: i8::from_le_bytes([scratchpad[3]]),
        })
    }

    fn read_scratchpad(&mut self) -> Result<[u8; 9], Ds18b20Error> {
        let mut scratchpad = [0; 9];

        self.bus.reset()?;
        self.bus.match_address(self.address);
        self.bus.write_byte(CMD_READ_SCRATCHPAD);
        self.bus.read_bytes(&mut scratchpad);
        OneWireBus::check_crc8(&scratchpad)?;

        Ok(scratchpad)
    }
}

impl Resolution {
    /// Returns the time the sensor needs to perform a measurement, in milliseconds.
    pub const fn measurement_time_ms(&self) -> u32 {
        match self {
            Resolution::Bits9 => 94,
            Resolution::Bits10 => 188,
            Resolution::Bits11 => 375,
            Resolution::Bits12 => 750,
        }
    }
}

impl TryFrom<u8> for Resolution {
    type Error = Ds18b20Error;

    /// Attempts to decode a resolution from a bitfield.
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0b0001_1111 => Ok(Resolution::Bits9),
            0b0011_1111 => Ok(Resolution::Bits10),
            0b0101_1111 => Ok(Resolution::Bits11),
            0b0111_1111 => Ok(Resolution::Bits12),
            _ => Err(Ds18b20Error::InvalidResolution),
        }
    }
}
