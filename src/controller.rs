//! Movement controller for tank-style differential drive
//!
//! Converts joystick-style inputs (throttle + steering) to individual
//! motor speeds for tank drive control.

use crate::motor::Motor;

/// Tank drive controller managing left and right motor channels
pub struct DriveTrain<'d> {
    left_motor: Motor<'d>,
    right_motor: Motor<'d>,
}

impl<'d> DriveTrain<'d> {
    /// Create a new drive train with left and right motors
    pub fn new(left_motor: Motor<'d>, right_motor: Motor<'d>) -> Self {
        Self {
            left_motor,
            right_motor,
        }
    }

    /// Direct tank control - set left and right motor speeds independently
    ///
    /// Values range from -100 (full reverse) to +100 (full forward)
    pub fn tank_drive(&mut self, left_speed: i8, right_speed: i8) {
        self.left_motor.set(left_speed);
        self.right_motor.set(right_speed);
    }

    /// Arcade-style control - single joystick controlling throttle and steering
    ///
    /// - `throttle`: -100 (full reverse) to +100 (full forward)
    /// - `steering`: -100 (full left) to +100 (full right)
    ///
    /// This maps to tank drive using differential steering:
    /// - Moving forward: both motors forward
    /// - Turning: reduce speed on one side
    /// - Spin in place: opposite motor directions
    pub fn arcade_drive(&mut self, throttle: i8, steering: i8) {
        let (left, right) = arcade_to_tank(throttle, steering);
        self.tank_drive(left, right);
    }

    /// Stop both motors (coast)
    pub fn stop(&mut self) {
        self.left_motor.stop();
        self.right_motor.stop();
    }

    /// Brake both motors (active stop)
    pub fn brake(&mut self) {
        self.left_motor.brake();
        self.right_motor.brake();
    }

    /// Spin in place - positive = clockwise (right), negative = counter-clockwise (left)
    pub fn spin(&mut self, speed: i8) {
        // Clockwise: left forward, right reverse
        self.left_motor.set(speed);
        self.right_motor.set(-speed);
    }

    /// Get reference to left motor for direct access
    pub fn left(&mut self) -> &mut Motor<'d> {
        &mut self.left_motor
    }

    /// Get reference to right motor for direct access
    pub fn right(&mut self) -> &mut Motor<'d> {
        &mut self.right_motor
    }
}

/// Configurable spin/turn parameters
#[derive(Clone, Copy)]
pub struct TurnConfig {
    /// Speed for spinning (0-100)
    pub spin_speed: u8,
    /// Duration in milliseconds for a 180° turn (calibrate this!)
    pub turn_180_ms: u32,
}

impl Default for TurnConfig {
    fn default() -> Self {
        Self {
            spin_speed: 60,
            turn_180_ms: 500, // Start with 500ms, calibrate on real hardware
        }
    }
}

/// Convert arcade-style (throttle, steering) to tank-style (left, right) motor values
///
/// Uses a simple mixing algorithm:
/// - left  = throttle + steering
/// - right = throttle - steering
///
/// Values are clamped to -100..+100 range
pub fn arcade_to_tank(throttle: i8, steering: i8) -> (i8, i8) {
    // Use i16 for intermediate calculations to avoid overflow
    let throttle = throttle as i16;
    let steering = steering as i16;

    let left = throttle + steering;
    let right = throttle - steering;

    // Clamp to i8 range (-128..127), but our logical range is -100..100
    let left = left.clamp(-100, 100) as i8;
    let right = right.clamp(-100, 100) as i8;

    (left, right)
}

/// Apply a deadzone to joystick input
///
/// Values within the deadzone (e.g., -5..+5) are treated as 0.
/// Values outside are scaled to use the full range.
pub fn apply_deadzone(value: i8, deadzone: u8) -> i8 {
    let deadzone = deadzone as i8;
    if value.abs() < deadzone {
        0
    } else {
        // Scale remaining range to full output
        let sign = value.signum();
        let magnitude = value.abs() - deadzone;
        let max_input = 100 - deadzone;
        if max_input == 0 {
            return 0;
        }
        let scaled = (magnitude as i16 * 100) / max_input as i16;
        (sign as i16 * scaled).clamp(-100, 100) as i8
    }
}

/// Exponential curve for smoother control at low speeds
///
/// Applies an exponential curve to give finer control at low speeds
/// while maintaining full power at high speeds.
/// - `expo`: 0.0 = linear, 1.0 = fully exponential
pub fn apply_expo(value: i8, expo: f32) -> i8 {
    let expo = expo.clamp(0.0, 1.0);
    let normalized = value as f32 / 100.0;
    let sign = if normalized >= 0.0 { 1.0 } else { -1.0 };
    let abs_val = normalized.abs();

    // Mix between linear and cubic response
    let linear = abs_val;
    let cubic = abs_val * abs_val * abs_val;
    let mixed = (1.0 - expo) * linear + expo * cubic;

    (sign * mixed * 100.0) as i8
}
