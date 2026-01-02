#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![allow(clippy::large_stack_frames)]

use bt_hci::controller::ExternalController;
use defmt::info;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::ledc::{LSGlobalClkSource, Ledc, channel, timer};
use esp_hal::timer::timg::TimerGroup;
use esp_radio::ble::controller::BleConnector;
use esp32_micro_rc_car::ble::{CommandFlags, MotorCommand};
use esp32_micro_rc_car::controller::{DriveTrain, TurnConfig};
use esp32_micro_rc_car::gamepad::{self, GamepadLedStatus};
use esp32_micro_rc_car::led::{Color, Ws2812};
use esp32_micro_rc_car::motor::{self, Motor, StandbyPin};
use esp32_micro_rc_car::web_ble::{self, WebBleLedStatus};
use static_cell::StaticCell;
use trouble_host::prelude::*;
use {esp_backtrace as _, esp_println as _};

extern crate alloc;

const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 3;

esp_bootloader_esp_idf::esp_app_desc!();

// ========================================
// CONTROL MODE SELECTION
// ========================================
/// Control mode for the RC car
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ControlMode {
    /// Web Bluetooth mode - ESP32 acts as peripheral, controlled from web browser
    WebBle,
    /// Gamepad mode - ESP32 connects to BLE gamepad as central
    #[allow(dead_code)]
    Gamepad,
}

/// Channel for motor commands
static MOTOR_CMD_CHANNEL: Channel<CriticalSectionRawMutex, MotorCommand, 4> = Channel::new();

/// Channel for LED status updates
static LED_STATUS_CHANNEL: Channel<CriticalSectionRawMutex, LedStatus, 4> = Channel::new();

/// LED status states
#[derive(Clone, Copy, Debug)]
pub enum LedStatus {
    /// Boot/startup - white
    Booting,
    /// Motors ready, waiting for BLE - cyan pulse
    MotorsReady,
    /// Advertising (Web BLE mode) - blue pulse
    Advertising,
    /// Scanning for gamepad - blue blink
    Scanning,
    /// Connecting to gamepad - yellow blink
    Connecting,
    /// Connected and ready - solid green
    Connected,
    /// Disconnected - red blink
    Disconnected,
    /// Raw input received (debug) - magenta flash
    InputReceived,
    /// Receiving input - color based on motor speed
    Driving {
        left: i8,
        right: i8,
    },
    /// Emergency stop - solid red
    EmergencyStop,
    /// Special turn - purple/yellow
    TurnLeft,
    TurnRight,
}

// Implement traits for LED status to work with web_ble and gamepad modules
impl WebBleLedStatus for LedStatus {
    fn advertising() -> Self {
        Self::Advertising
    }
    fn connected() -> Self {
        Self::Connected
    }
    fn disconnected() -> Self {
        Self::Disconnected
    }
    fn driving(left: i8, right: i8) -> Self {
        Self::Driving { left, right }
    }
}

impl GamepadLedStatus for LedStatus {
    fn scanning() -> Self {
        Self::Scanning
    }
    fn connecting() -> Self {
        Self::Connecting
    }
    fn connected() -> Self {
        Self::Connected
    }
    fn disconnected() -> Self {
        Self::Disconnected
    }
    fn input_received() -> Self {
        Self::InputReceived
    }
}

static LEDC_CELL: StaticCell<Ledc<'static>> = StaticCell::new();
static TIMER0_CELL: StaticCell<timer::Timer<'static, esp_hal::ledc::LowSpeed>> = StaticCell::new();
static TIMER1_CELL: StaticCell<timer::Timer<'static, esp_hal::ledc::LowSpeed>> = StaticCell::new();

/// LED status task - handles blinking patterns and status display
#[embassy_executor::task]
async fn led_status_task(mut led: Ws2812<'static>) {
    info!("LED status task started");
    let mut current_status = LedStatus::Booting;
    let mut blink_on = true;

    // Initial boot color
    led.set_color(Color::new(20, 20, 20)).await; // White

    loop {
        // Check for new status (non-blocking)
        if let Ok(new_status) = LED_STATUS_CHANNEL.try_receive() {
            current_status = new_status;
            blink_on = true; // Reset blink state on status change
        }

        match current_status {
            LedStatus::Booting => {
                led.set_color(Color::new(20, 20, 20)).await; // White
            }
            LedStatus::MotorsReady => {
                // Cyan pulse
                let intensity = if blink_on { 20 } else { 5 };
                led.set_color(Color::new(0, intensity, intensity)).await;
                blink_on = !blink_on;
            }
            LedStatus::Advertising => {
                // Blue pulse (slower, inviting)
                let intensity = if blink_on { 25 } else { 8 };
                led.set_color(Color::new(0, 0, intensity)).await;
                blink_on = !blink_on;
            }
            LedStatus::Scanning => {
                // Blue blink (fast)
                if blink_on {
                    led.set_color(Color::new(0, 0, 30)).await;
                } else {
                    led.set_color(Color::new(0, 0, 5)).await;
                }
                blink_on = !blink_on;
            }
            LedStatus::Connecting => {
                // Yellow blink
                if blink_on {
                    led.set_color(Color::new(30, 20, 0)).await;
                } else {
                    led.set_color(Color::new(5, 3, 0)).await;
                }
                blink_on = !blink_on;
            }
            LedStatus::Connected => {
                // Solid green
                led.set_color(Color::green(20)).await;
            }
            LedStatus::Disconnected => {
                // Red blink
                if blink_on {
                    led.set_color(Color::red(30)).await;
                } else {
                    led.set_color(Color::new(0, 0, 0)).await;
                }
                blink_on = !blink_on;
            }
            LedStatus::InputReceived => {
                // Magenta flash for raw input debug
                led.set_color(Color::new(40, 0, 40)).await;
            }
            LedStatus::Driving { left, right } => {
                // Green intensity based on speed
                let intensity =
                    ((left.unsigned_abs() as u16 + right.unsigned_abs() as u16) / 4) as u8;
                led.set_color(Color::green(intensity.max(10))).await;
            }
            LedStatus::EmergencyStop => {
                // Solid bright red
                led.set_color(Color::red(50)).await;
            }
            LedStatus::TurnLeft => {
                led.set_color(Color::new(25, 0, 25)).await; // Purple
            }
            LedStatus::TurnRight => {
                led.set_color(Color::new(25, 25, 0)).await; // Yellow
            }
        }

        // Blink rate depends on status
        let delay = match current_status {
            LedStatus::Scanning => Duration::from_millis(150),
            LedStatus::Connecting => Duration::from_millis(200),
            LedStatus::Disconnected => Duration::from_millis(500),
            LedStatus::MotorsReady => Duration::from_millis(800),
            LedStatus::Advertising => Duration::from_millis(600),
            _ => Duration::from_millis(100),
        };
        Timer::after(delay).await;
    }
}

/// Motor control task
#[embassy_executor::task]
async fn motor_control_task(mut drive: DriveTrain<'static>, turn_config: TurnConfig) {
    info!("Motor control task started");

    loop {
        let cmd = MOTOR_CMD_CHANNEL.receive().await;

        if cmd.flags.has(CommandFlags::EMERGENCY_STOP) {
            info!("Emergency stop!");
            let _ = LED_STATUS_CHANNEL.try_send(LedStatus::EmergencyStop);
            drive.brake();
            Timer::after(Duration::from_millis(100)).await;
            continue;
        }

        if cmd.flags.has(CommandFlags::ABOUT_TURN_LEFT) {
            info!("About turn left");
            let _ = LED_STATUS_CHANNEL.try_send(LedStatus::TurnLeft);
            // Use clamped spin speed to prevent brownouts
            let spin = turn_config.clamp_speed(-(turn_config.spin_speed as i8));
            drive.spin(spin);
            Timer::after(Duration::from_millis(turn_config.turn_180_ms as u64)).await;
            drive.brake();
            continue;
        }

        if cmd.flags.has(CommandFlags::ABOUT_TURN_RIGHT) {
            info!("About turn right");
            let _ = LED_STATUS_CHANNEL.try_send(LedStatus::TurnRight);
            // Use clamped spin speed to prevent brownouts
            let spin = turn_config.clamp_speed(turn_config.spin_speed as i8);
            drive.spin(spin);
            Timer::after(Duration::from_millis(turn_config.turn_180_ms as u64)).await;
            drive.brake();
            continue;
        }

        // Clamp motor speeds to configured maximum to prevent current spikes
        let left = turn_config.clamp_speed(cmd.left);
        let right = turn_config.clamp_speed(cmd.right);

        if left == 0 && right == 0 {
            drive.stop();
            let _ = LED_STATUS_CHANNEL.try_send(LedStatus::Connected);
        } else {
            drive.tank_drive(left, right);
            let _ = LED_STATUS_CHANNEL.try_send(LedStatus::Driving { left, right });
        }
    }
}

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);
    esp_alloc::heap_allocator!(size: 64 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    info!("ESP32 Micro RC Car - BLE Control");
    info!("=================================");

    let led = Ws2812::new(peripherals.RMT, peripherals.GPIO21);

    // Start LED status task immediately
    spawner.spawn(led_status_task(led)).unwrap();
    let _ = LED_STATUS_CHANNEL.try_send(LedStatus::Booting);

    // ========== MOTOR SETUP ==========
    info!("Setting up motors...");

    let ledc = LEDC_CELL.init(Ledc::new(peripherals.LEDC));
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let timer0 = TIMER0_CELL.init(motor::configure_timer(ledc, timer::Number::Timer0));
    let timer1 = TIMER1_CELL.init(motor::configure_timer(ledc, timer::Number::Timer1));

    let pwm_left =
        motor::configure_channel(ledc, channel::Number::Channel0, peripherals.GPIO1, timer0);
    let pwm_right =
        motor::configure_channel(ledc, channel::Number::Channel1, peripherals.GPIO6, timer1);

    let left_in1 = motor::output_pin(peripherals.GPIO3);
    let left_in2 = motor::output_pin(peripherals.GPIO2);
    let motor_left = Motor::new(left_in1, left_in2, pwm_left);

    let right_in1 = motor::output_pin(peripherals.GPIO4);
    let right_in2 = motor::output_pin(peripherals.GPIO5);
    let motor_right = Motor::new(right_in1, right_in2, pwm_right);

    let mut stby = StandbyPin::new(peripherals.GPIO7);
    stby.enable();

    info!("Motors ready!");
    let _ = LED_STATUS_CHANNEL.try_send(LedStatus::MotorsReady);

    let drive = DriveTrain::new(motor_left, motor_right);
    let turn_config = TurnConfig::default();
    spawner
        .spawn(motor_control_task(drive, turn_config))
        .unwrap();

    // ========================================
    // CONTROL MODE SELECTION
    // ========================================
    // WebBle = Control from web browser (ESP32 advertises, browser connects)
    // Gamepad = Connect to BLE gamepad (ESP32 scans and connects)
    let control_mode = ControlMode::WebBle; // <-- CHANGE THIS TO SWITCH MODES

    // ========== BLE SETUP ==========
    info!("Initializing BLE...");

    let radio_init = esp_radio::init().expect("Failed to initialize radio");

    let (_wifi_controller, _interfaces) =
        esp_radio::wifi::new(&radio_init, peripherals.WIFI, Default::default())
            .expect("Failed to initialize Wi-Fi controller");

    let transport = BleConnector::new(&radio_init, peripherals.BT, Default::default()).unwrap();
    let ble_controller = ExternalController::<_, 20>::new(transport);

    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();

    let address = Address::random([0xCA, 0xFE, 0xBA, 0xBE, 0x00, 0x01]);

    match control_mode {
        ControlMode::WebBle => {
            web_ble::run(
                ble_controller,
                &mut resources,
                address,
                &MOTOR_CMD_CHANNEL,
                &LED_STATUS_CHANNEL,
            )
            .await;
        }
        ControlMode::Gamepad => {
            // Set to None to scan for devices, or Some([...]) to connect directly
            // Address bytes are in REVERSE order (little-endian)
            let target_addr: Option<[u8; 6]> = None;

            if let Some(addr) = target_addr {
                gamepad::run_connection(
                    ble_controller,
                    &mut resources,
                    address,
                    addr,
                    &MOTOR_CMD_CHANNEL,
                    &LED_STATUS_CHANNEL,
                )
                .await;
            } else {
                gamepad::run_scan(ble_controller, &mut resources, address, &LED_STATUS_CHANNEL)
                    .await;
            }
        }
    }

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
