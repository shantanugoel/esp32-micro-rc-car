//! TB6612FNG Motor Driver using LEDC PWM
//!
//! TB6612FNG control:
//! - IN1=HIGH, IN2=LOW  → Forward
//! - IN1=LOW,  IN2=HIGH → Reverse
//! - IN1=HIGH, IN2=HIGH → Brake
//! - IN1=LOW,  IN2=LOW  → Coast
//! - PWM duty cycle controls speed (0-100%)
//! - STBY must be HIGH to enable driver

use esp_hal::gpio::{DriveMode, Level, Output, OutputConfig};
use esp_hal::ledc::{
    Ledc, LowSpeed,
    channel::{self, ChannelIFace},
    timer::{self, TimerIFace},
};
use esp_hal::time::Rate;

/// Motor direction
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Direction {
    Forward,
    Reverse,
    Brake,
    Coast,
}

/// Single motor channel on TB6612FNG
pub struct Motor<'d> {
    in1: Output<'d>,
    in2: Output<'d>,
    pwm_channel: channel::Channel<'d, LowSpeed>,
    current_speed: u8,
    current_direction: Direction,
}

impl<'d> Motor<'d> {
    /// Create a new motor instance
    ///
    /// - `in1`, `in2`: Direction control pins
    /// - `pwm_channel`: Configured LEDC channel for speed control
    pub fn new(
        in1: Output<'d>,
        in2: Output<'d>,
        pwm_channel: channel::Channel<'d, LowSpeed>,
    ) -> Self {
        let mut motor = Self {
            in1,
            in2,
            pwm_channel,
            current_speed: 0,
            current_direction: Direction::Coast,
        };
        motor.stop();
        motor
    }

    /// Set motor direction
    pub fn set_direction(&mut self, direction: Direction) {
        self.current_direction = direction;
        match direction {
            Direction::Forward => {
                self.in1.set_high();
                self.in2.set_low();
            }
            Direction::Reverse => {
                self.in1.set_low();
                self.in2.set_high();
            }
            Direction::Brake => {
                self.in1.set_high();
                self.in2.set_high();
            }
            Direction::Coast => {
                self.in1.set_low();
                self.in2.set_low();
            }
        }
    }

    /// Set motor speed (0-100%)
    pub fn set_speed(&mut self, speed: u8) {
        let speed = speed.min(100);
        self.current_speed = speed;
        let _ = self.pwm_channel.set_duty(speed);
    }

    /// Set motor speed and direction in one call
    /// Positive speed = forward, negative = reverse, 0 = stop
    pub fn set(&mut self, speed: i8) {
        if speed == 0 {
            self.stop();
        } else if speed > 0 {
            self.set_direction(Direction::Forward);
            self.set_speed(speed as u8);
        } else {
            self.set_direction(Direction::Reverse);
            self.set_speed(speed.unsigned_abs());
        }
    }

    /// Stop the motor (coast)
    pub fn stop(&mut self) {
        self.set_speed(0);
        self.set_direction(Direction::Coast);
    }

    /// Brake the motor (active stop)
    pub fn brake(&mut self) {
        self.set_speed(0);
        self.set_direction(Direction::Brake);
    }

    /// Get current speed percentage
    pub fn speed(&self) -> u8 {
        self.current_speed
    }

    /// Get current direction
    pub fn direction(&self) -> Direction {
        self.current_direction
    }
}

/// Standby pin controller for TB6612FNG
/// STBY must be HIGH for motors to operate
pub struct StandbyPin<'d> {
    pin: Output<'d>,
}

impl<'d> StandbyPin<'d> {
    pub fn new(pin: impl esp_hal::gpio::OutputPin + 'd) -> Self {
        let pin = Output::new(pin, Level::Low, OutputConfig::default());
        Self { pin }
    }

    /// Enable the motor driver
    pub fn enable(&mut self) {
        self.pin.set_high();
    }

    /// Disable the motor driver (all motors stop)
    pub fn disable(&mut self) {
        self.pin.set_low();
    }
}

/// Helper to create output pins for motor control
pub fn output_pin<'d>(pin: impl esp_hal::gpio::OutputPin + 'd) -> Output<'d> {
    Output::new(pin, Level::Low, OutputConfig::default())
}

/// PWM frequency for motors (20kHz is typical for DC motors - above audible range)
pub const MOTOR_PWM_FREQ: Rate = Rate::from_khz(20);

/// Configure LEDC timer for motor PWM
pub fn configure_timer<'d>(
    ledc: &'d Ledc<'d>,
    timer_num: timer::Number,
) -> timer::Timer<'d, LowSpeed> {
    let mut timer = ledc.timer::<LowSpeed>(timer_num);
    timer
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty8Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: MOTOR_PWM_FREQ,
        })
        .unwrap();
    timer
}

/// Configure LEDC channel for motor PWM
pub fn configure_channel<'d>(
    ledc: &'d Ledc<'d>,
    channel_num: channel::Number,
    pin: impl esp_hal::gpio::OutputPin + 'd,
    timer: &'d timer::Timer<'d, LowSpeed>,
) -> channel::Channel<'d, LowSpeed> {
    let mut channel = ledc.channel(channel_num, pin);
    channel
        .configure(channel::config::Config {
            timer,
            duty_pct: 0,
            drive_mode: DriveMode::PushPull,
        })
        .unwrap();
    channel
}
