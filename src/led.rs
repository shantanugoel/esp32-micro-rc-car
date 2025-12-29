//! WS2812B RGB LED driver using RMT peripheral

use esp_hal::Async;
use esp_hal::gpio::Level;
use esp_hal::peripherals::{GPIO21, RMT};
use esp_hal::rmt::{Channel, PulseCode, Rmt, Tx, TxChannelConfig, TxChannelCreator};
use esp_hal::time::Rate;

/// RGB color with brightness support
#[derive(Clone, Copy, Default)]
pub struct Color {
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

impl Color {
    pub const fn new(r: u8, g: u8, b: u8) -> Self {
        Self { r, g, b }
    }

    pub const fn off() -> Self {
        Self { r: 0, g: 0, b: 0 }
    }

    pub const fn red(brightness: u8) -> Self {
        Self {
            r: brightness,
            g: 0,
            b: 0,
        }
    }

    pub const fn green(brightness: u8) -> Self {
        Self {
            r: 0,
            g: brightness,
            b: 0,
        }
    }

    pub const fn blue(brightness: u8) -> Self {
        Self {
            r: 0,
            g: 0,
            b: brightness,
        }
    }

    pub const fn white(brightness: u8) -> Self {
        Self {
            r: brightness,
            g: brightness,
            b: brightness,
        }
    }

    /// Scale color by brightness factor (0-255)
    pub fn with_brightness(self, brightness: u8) -> Self {
        Self {
            r: ((self.r as u16 * brightness as u16) / 255) as u8,
            g: ((self.g as u16 * brightness as u16) / 255) as u8,
            b: ((self.b as u16 * brightness as u16) / 255) as u8,
        }
    }
}

/// WS2812B LED controller
pub struct Ws2812<'d> {
    channel: Channel<'d, Async, Tx>,
}

impl<'d> Ws2812<'d> {
    /// Create a new WS2812B LED controller
    ///
    /// Uses GPIO21 which is the onboard LED on Waveshare ESP32-S3 Zero
    pub fn new(rmt: RMT<'d>, pin: GPIO21<'d>) -> Self {
        let freq = Rate::from_mhz(80);
        let rmt = Rmt::new(rmt, freq).unwrap().into_async();

        let tx_config = TxChannelConfig::default()
            .with_clk_divider(1)
            .with_idle_output_level(Level::Low)
            .with_idle_output(true);

        let channel = rmt.channel0.configure_tx(pin, tx_config).unwrap();

        Self { channel }
    }

    /// Set the LED color
    pub async fn set_color(&mut self, color: Color) {
        let data = Self::encode_color(color);
        self.channel.transmit(&data).await.unwrap();
    }

    /// Turn off the LED
    pub async fn off(&mut self) {
        self.set_color(Color::off()).await;
    }

    /// Encode RGB color to WS2812B pulse codes
    ///
    /// WS2812B timing at 80MHz RMT clock:
    /// - T0H = 0.4µs = 32 ticks, T0L = 0.85µs = 68 ticks
    /// - T1H = 0.8µs = 64 ticks, T1L = 0.45µs = 36 ticks
    /// - Format: GRB (Green-Red-Blue), MSB first
    fn encode_color(color: Color) -> [PulseCode; 25] {
        let mut pulses = [PulseCode::default(); 25];

        // WS2812B expects GRB order
        let grb: u32 = ((color.g as u32) << 16) | ((color.r as u32) << 8) | (color.b as u32);

        for (i, pulse) in pulses.iter_mut().take(24).enumerate() {
            let bit = (grb >> (23 - i)) & 1 != 0;
            *pulse = if bit {
                // '1' bit
                PulseCode::new(Level::High, 64, Level::Low, 36)
            } else {
                // '0' bit
                PulseCode::new(Level::High, 32, Level::Low, 68)
            };
        }
        pulses[24] = PulseCode::end_marker();
        pulses
    }
}
