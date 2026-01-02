//! BLE HID Host for gamepad controllers (e.g., 8BitDo Pro 2)
//!
//! This module implements a BLE Central that connects to HID gamepads
//! and parses their input reports for motor control.

use trouble_host::prelude::*;

/// Standard BLE HID Service UUID
pub const HID_SERVICE_UUID: Uuid = Uuid::new_short(0x1812);

/// HID Report characteristic UUID
pub const HID_REPORT_UUID: Uuid = Uuid::new_short(0x2A4D);

/// HID Report Map characteristic UUID  
pub const HID_REPORT_MAP_UUID: Uuid = Uuid::new_short(0x2A4B);

/// Gamepad input state from HID reports
#[derive(Clone, Copy, Debug, Default)]
pub struct GamepadState {
    /// Left stick X axis (-128 to 127, 0 = center)
    pub left_x: i8,
    /// Left stick Y axis (-128 to 127, 0 = center, negative = up)
    pub left_y: i8,
    /// Right stick X axis (-128 to 127)
    pub right_x: i8,
    /// Right stick Y axis (-128 to 127)
    pub right_y: i8,
    /// Left trigger (0-255)
    pub left_trigger: u8,
    /// Right trigger (0-255)
    pub right_trigger: u8,
    /// D-pad state
    pub dpad: DPad,
    /// Button state (bitfield)
    pub buttons: GamepadButtons,
}

/// D-pad direction
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum DPad {
    #[default]
    None,
    Up,
    UpRight,
    Right,
    DownRight,
    Down,
    DownLeft,
    Left,
    UpLeft,
}

impl DPad {
    /// Parse D-pad from HID hat switch value (0-7 for directions, 8 or 15 for none)
    pub fn from_hat(value: u8) -> Self {
        match value {
            0 => DPad::Up,
            1 => DPad::UpRight,
            2 => DPad::Right,
            3 => DPad::DownRight,
            4 => DPad::Down,
            5 => DPad::DownLeft,
            6 => DPad::Left,
            7 => DPad::UpLeft,
            _ => DPad::None,
        }
    }
}

/// Gamepad button flags
#[derive(Clone, Copy, Debug, Default)]
pub struct GamepadButtons(pub u16);

impl GamepadButtons {
    pub const A: u16 = 1 << 0;
    pub const B: u16 = 1 << 1;
    pub const X: u16 = 1 << 2;
    pub const Y: u16 = 1 << 3;
    pub const L1: u16 = 1 << 4;
    pub const R1: u16 = 1 << 5;
    pub const L2: u16 = 1 << 6;
    pub const R2: u16 = 1 << 7;
    pub const SELECT: u16 = 1 << 8;
    pub const START: u16 = 1 << 9;
    pub const L3: u16 = 1 << 10; // Left stick click
    pub const R3: u16 = 1 << 11; // Right stick click
    pub const HOME: u16 = 1 << 12;

    pub fn is_pressed(&self, button: u16) -> bool {
        self.0 & button != 0
    }

    pub fn set(&mut self, button: u16, pressed: bool) {
        if pressed {
            self.0 |= button;
        } else {
            self.0 &= !button;
        }
    }
}

/// Motor command derived from gamepad state
#[derive(Clone, Copy, Debug, Default)]
pub struct MotorCommand {
    /// Left motor speed (-100 to 100)
    pub left: i8,
    /// Right motor speed (-100 to 100)
    pub right: i8,
    /// Special command flags
    pub flags: CommandFlags,
}

/// Special command flags
#[derive(Clone, Copy, Debug, Default)]
pub struct CommandFlags(pub u8);

impl CommandFlags {
    pub const ABOUT_TURN_LEFT: u8 = 1 << 0;
    pub const ABOUT_TURN_RIGHT: u8 = 1 << 1;
    pub const EMERGENCY_STOP: u8 = 1 << 2;

    pub fn has(&self, flag: u8) -> bool {
        self.0 & flag != 0
    }

    pub fn set(&mut self, flag: u8) {
        self.0 |= flag;
    }
}

/// Convert gamepad state to motor command using arcade-style mapping
pub fn gamepad_to_motor_cmd(state: &GamepadState) -> MotorCommand {
    let mut cmd = MotorCommand::default();

    // Check for special button commands first
    if state.buttons.is_pressed(GamepadButtons::L1) {
        cmd.flags.set(CommandFlags::ABOUT_TURN_LEFT);
        return cmd;
    }
    if state.buttons.is_pressed(GamepadButtons::R1) {
        cmd.flags.set(CommandFlags::ABOUT_TURN_RIGHT);
        return cmd;
    }
    if state.buttons.is_pressed(GamepadButtons::B) {
        cmd.flags.set(CommandFlags::EMERGENCY_STOP);
        return cmd;
    }

    // Use left stick for arcade drive (throttle on Y, steering on X)
    // Note: Y axis is typically inverted (negative = forward/up)
    let throttle = -(state.left_y as i16); // Invert so pushing up = forward
    let steering = state.left_x as i16;

    // Scale from -128..127 to -100..100
    let throttle = ((throttle * 100) / 128).clamp(-100, 100) as i8;
    let steering = ((steering * 100) / 128).clamp(-100, 100) as i8;

    // Apply deadzone
    let throttle = if throttle.abs() < 10 { 0 } else { throttle };
    let steering = if steering.abs() < 10 { 0 } else { steering };

    // Arcade to tank conversion
    let left = (throttle as i16 + steering as i16).clamp(-100, 100) as i8;
    let right = (throttle as i16 - steering as i16).clamp(-100, 100) as i8;

    cmd.left = left;
    cmd.right = right;
    cmd
}

/// Parse 8BitDo Pro 2 HID report (XInput/DirectInput mode)
///
/// The 8BitDo Pro 2 report format varies by mode:
/// - XInput mode: Similar to Xbox controller
/// - DirectInput mode: Standard HID gamepad
/// - Switch mode: Nintendo Switch Pro Controller format
///
/// This parser handles the common DirectInput format.
pub fn parse_8bitdo_report(data: &[u8]) -> Option<GamepadState> {
    // 8BitDo Pro 2 in DirectInput mode typically sends 8-10 byte reports
    if data.len() < 8 {
        return None;
    }

    // Common DirectInput layout (may need adjustment based on actual controller):
    // Byte 0: Left X (0-255, 128 = center)
    // Byte 1: Left Y (0-255, 128 = center)
    // Byte 2: Right X (0-255, 128 = center)
    // Byte 3: Right Y (0-255, 128 = center)
    // Byte 4: D-pad + some buttons
    // Byte 5-6: Button states
    // Byte 7+: Triggers, etc.

    // Convert 0-255 unsigned to -128 to 127 signed (center at 0)
    let mut state = GamepadState {
        left_x: (data[0] as i16 - 128) as i8,
        left_y: (data[1] as i16 - 128) as i8,
        right_x: (data[2] as i16 - 128) as i8,
        right_y: (data[3] as i16 - 128) as i8,
        // D-pad is often in lower nibble of byte 4
        dpad: DPad::from_hat(data[4] & 0x0F),
        ..Default::default()
    };

    // Buttons - layout varies, this is a common mapping
    let buttons_low = data[5];
    let buttons_high = if data.len() > 6 { data[6] } else { 0 };

    state
        .buttons
        .set(GamepadButtons::A, buttons_low & 0x01 != 0);
    state
        .buttons
        .set(GamepadButtons::B, buttons_low & 0x02 != 0);
    state
        .buttons
        .set(GamepadButtons::X, buttons_low & 0x04 != 0);
    state
        .buttons
        .set(GamepadButtons::Y, buttons_low & 0x08 != 0);
    state
        .buttons
        .set(GamepadButtons::L1, buttons_low & 0x10 != 0);
    state
        .buttons
        .set(GamepadButtons::R1, buttons_low & 0x20 != 0);
    state
        .buttons
        .set(GamepadButtons::L2, buttons_low & 0x40 != 0);
    state
        .buttons
        .set(GamepadButtons::R2, buttons_low & 0x80 != 0);
    state
        .buttons
        .set(GamepadButtons::SELECT, buttons_high & 0x01 != 0);
    state
        .buttons
        .set(GamepadButtons::START, buttons_high & 0x02 != 0);
    state
        .buttons
        .set(GamepadButtons::L3, buttons_high & 0x04 != 0);
    state
        .buttons
        .set(GamepadButtons::R3, buttons_high & 0x08 != 0);
    state
        .buttons
        .set(GamepadButtons::HOME, buttons_high & 0x10 != 0);

    // Triggers if available
    if data.len() > 7 {
        state.left_trigger = data[7];
    }
    if data.len() > 8 {
        state.right_trigger = data[8];
    }

    Some(state)
}

/// Generic HID gamepad report parser
/// Tries to parse various common gamepad formats
pub fn parse_generic_gamepad(data: &[u8]) -> Option<GamepadState> {
    // Try 8BitDo format first
    if let Some(state) = parse_8bitdo_report(data) {
        return Some(state);
    }

    // Fallback: minimal parsing for any HID gamepad
    if data.len() >= 4 {
        let state = GamepadState {
            left_x: (data[0] as i16 - 128) as i8,
            left_y: (data[1] as i16 - 128) as i8,
            ..Default::default()
        };
        return Some(state);
    }

    None
}
