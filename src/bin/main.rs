#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use bt_hci::cmd::le::LeSetScanParams;
use bt_hci::controller::{ControllerCmdSync, ExternalController};
use bt_hci::param::{AddrKind, BdAddr};
use core::cell::RefCell;
use defmt::{debug, error, info, warn};
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::ledc::{LSGlobalClkSource, Ledc, channel, timer};
use esp_hal::timer::timg::TimerGroup;
use esp_radio::ble::controller::BleConnector;
use esp32_micro_rc_car::ble::{
    CommandFlags, GamepadState, MotorCommand, gamepad_to_motor_cmd, parse_generic_gamepad,
};
use esp32_micro_rc_car::controller::{DriveTrain, TurnConfig};
use esp32_micro_rc_car::led::{Color, Ws2812};
use esp32_micro_rc_car::motor::{self, Motor, StandbyPin};
use static_cell::StaticCell;
use trouble_host::prelude::*;
use trouble_host::scan::Scanner;
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
enum ControlMode {
    /// Web Bluetooth mode - ESP32 acts as peripheral, controlled from web browser
    WebBle,
    /// Gamepad mode - ESP32 connects to BLE gamepad as central
    Gamepad,
}

/// Channel for motor commands
static MOTOR_CMD_CHANNEL: Channel<CriticalSectionRawMutex, MotorCommand, 4> = Channel::new();

/// Channel for LED status updates
static LED_STATUS_CHANNEL: Channel<CriticalSectionRawMutex, LedStatus, 4> = Channel::new();

/// LED status states
#[derive(Clone, Copy, Debug)]
enum LedStatus {
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

static LEDC_CELL: StaticCell<Ledc<'static>> = StaticCell::new();
static TIMER0_CELL: StaticCell<timer::Timer<'static, esp_hal::ledc::LowSpeed>> = StaticCell::new();
static TIMER1_CELL: StaticCell<timer::Timer<'static, esp_hal::ledc::LowSpeed>> = StaticCell::new();

/// LED status task - handles blinking patterns and status display
#[allow(
    clippy::large_stack_frames,
    reason = "async task with awaits needs stack space"
)]
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
#[allow(
    clippy::large_stack_frames,
    reason = "async task with awaits needs stack space"
)]
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
            drive.spin(-(turn_config.spin_speed as i8));
            Timer::after(Duration::from_millis(turn_config.turn_180_ms as u64)).await;
            drive.brake();
            continue;
        }

        if cmd.flags.has(CommandFlags::ABOUT_TURN_RIGHT) {
            info!("About turn right");
            let _ = LED_STATUS_CHANNEL.try_send(LedStatus::TurnRight);
            drive.spin(turn_config.spin_speed as i8);
            Timer::after(Duration::from_millis(turn_config.turn_180_ms as u64)).await;
            drive.brake();
            continue;
        }

        if cmd.left == 0 && cmd.right == 0 {
            drive.stop();
            let _ = LED_STATUS_CHANNEL.try_send(LedStatus::Connected);
        } else {
            drive.tank_drive(cmd.left, cmd.right);
            let _ = LED_STATUS_CHANNEL.try_send(LedStatus::Driving {
                left: cmd.left,
                right: cmd.right,
            });
        }
    }
}

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);
    esp_alloc::heap_allocator!(size: 64 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    info!("ESP32 Micro RC Car - BLE Gamepad Mode");
    info!("=====================================");

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

    // ========================================
    // MOTOR TEST MODE
    // ========================================
    // Set to true to test motors without BLE (runs both at 50% forever)
    let test_motors = false;

    let mut drive = DriveTrain::new(motor_left, motor_right);

    if test_motors {
        info!("========================================");
        info!("MOTOR TEST MODE - Running at 50% speed");
        info!("========================================");

        loop {
            // Forward at 50%
            info!("Forward 50%");
            let _ = LED_STATUS_CHANNEL.try_send(LedStatus::Driving {
                left: 64,
                right: 64,
            });
            drive.tank_drive(64, 64);
            Timer::after(Duration::from_secs(5)).await;

            // Stop
            info!("Stop");
            let _ = LED_STATUS_CHANNEL.try_send(LedStatus::Connected);
            drive.stop();
            Timer::after(Duration::from_secs(5)).await;

            // Reverse at 50%
            info!("Reverse 50%");
            let _ = LED_STATUS_CHANNEL.try_send(LedStatus::Driving {
                left: -64,
                right: -64,
            });
            drive.tank_drive(-64, -64);
            Timer::after(Duration::from_secs(2)).await;

            // Stop
            info!("Stop");
            let _ = LED_STATUS_CHANNEL.try_send(LedStatus::Connected);
            drive.stop();
            Timer::after(Duration::from_secs(1)).await;

            // Spin left
            info!("Spin left");
            let _ = LED_STATUS_CHANNEL.try_send(LedStatus::TurnLeft);
            drive.spin(-64);
            Timer::after(Duration::from_secs(1)).await;

            // Stop
            info!("Stop");
            let _ = LED_STATUS_CHANNEL.try_send(LedStatus::Connected);
            drive.stop();
            Timer::after(Duration::from_secs(1)).await;

            // Spin right
            info!("Spin right");
            let _ = LED_STATUS_CHANNEL.try_send(LedStatus::TurnRight);
            drive.spin(64);
            Timer::after(Duration::from_secs(1)).await;

            // Stop
            info!("Stop");
            let _ = LED_STATUS_CHANNEL.try_send(LedStatus::Connected);
            drive.stop();
            Timer::after(Duration::from_secs(1)).await;
        }
    }

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
            run_web_ble_mode(ble_controller, &mut resources, address).await;
        }
        ControlMode::Gamepad => {
            run_gamepad_mode(ble_controller, &mut resources, address).await;
        }
    }

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}

// ========================================
// WEB BLE MODE (Peripheral)
// ========================================
// Custom service UUID for RC car control
// Service UUID: 19b10000-e8f2-537e-4f6c-d104768a1214
// Motor Char:   19b10001-e8f2-537e-4f6c-d104768a1214

/// Motor control service for RC car
#[gatt_service(uuid = "19b10000-e8f2-537e-4f6c-d104768a1214")]
struct RcCarService {
    /// Motor control: 2 bytes [left_speed: i8, right_speed: i8]
    #[characteristic(
        uuid = "19b10001-e8f2-537e-4f6c-d104768a1214",
        write,
        write_without_response
    )]
    motor_control: [u8; 2],
}

/// GATT server containing our RC car service
#[gatt_server]
struct RcCarServer {
    rc_service: RcCarService,
}

#[allow(clippy::large_stack_frames)]
async fn run_web_ble_mode<C: Controller>(
    controller: C,
    resources: &mut HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX>,
    address: Address,
) {
    info!("========================================");
    info!("WEB BLE MODE - Browser Control");
    info!("========================================");
    info!("1. Open web/index.html in Chrome/Edge");
    info!("2. Click 'Connect to Car'");
    info!("3. Select 'ESP32-RC-Car'");
    info!("4. Use joystick to drive!");
    info!("========================================");
    info!("");

    let stack = trouble_host::new(controller, resources).set_random_address(address);

    let Host {
        mut peripheral,
        mut runner,
        ..
    } = stack.build();

    // Create GATT server with GAP service
    let server: RcCarServer<'_> =
        RcCarServer::new_with_config(GapConfig::Peripheral(PeripheralConfig {
            name: "ESP32-RC-Car",
            appearance: &appearance::UNKNOWN,
        }))
        .expect("Failed to create GATT server");

    info!("GATT server created");
    info!("Service UUID: 19b10000-e8f2-537e-4f6c-d104768a1214");
    info!("Motor Char:   19b10001-e8f2-537e-4f6c-d104768a1214");

    let _ = join(runner.run(), async {
        loop {
            info!("Starting advertising as 'ESP32-RC-Car'...");
            let _ = LED_STATUS_CHANNEL.try_send(LedStatus::Advertising);

            // Build advertising data
            let mut adv_data = [0u8; 31];
            let adv_len = AdStructure::encode_slice(
                &[
                    AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
                    AdStructure::CompleteLocalName(b"ESP32-RC-Car"),
                ],
                &mut adv_data[..],
            )
            .unwrap_or(0);

            // Build scan response (empty since 128-bit service UUIDs are long)
            let scan_data = [0u8; 31];
            let scan_len = 0;

            let adv = Advertisement::ConnectableScannableUndirected {
                adv_data: &adv_data[..adv_len],
                scan_data: &scan_data[..scan_len],
            };

            match peripheral.advertise(&Default::default(), adv).await {
                Ok(advertiser) => {
                    info!("Advertising started, waiting for connection...");

                    match advertiser.accept().await {
                        Ok(accept_result) => {
                            match accept_result.with_attribute_server(&server) {
                                Ok(conn) => {
                                    info!("========================================");
                                    info!("Web browser connected!");
                                    info!("========================================");
                                    let _ = LED_STATUS_CHANNEL.try_send(LedStatus::Connected);

                                    // Handle GATT events
                                    handle_gatt_events(&server, &conn).await;

                                    info!("Browser disconnected");
                                    let _ = LED_STATUS_CHANNEL.try_send(LedStatus::Disconnected);

                                    // Stop motors on disconnect
                                    let _ = MOTOR_CMD_CHANNEL.try_send(MotorCommand::default());
                                }
                                Err(_e) => {
                                    warn!("Failed to set up GATT server");
                                }
                            }
                        }
                        Err(_e) => {
                            warn!("Accept failed");
                        }
                    }
                }
                Err(_e) => {
                    error!("Advertising failed");
                }
            }

            Timer::after(Duration::from_millis(500)).await;
        }
    })
    .await;
}

/// Handle GATT events and motor commands from Web BLE
#[allow(clippy::large_stack_frames)]
async fn handle_gatt_events<'a>(
    server: &RcCarServer<'a>,
    conn: &GattConnection<'a, '_, DefaultPacketPool>,
) {
    info!("Waiting for motor commands...");

    loop {
        match conn.next().await {
            GattConnectionEvent::Disconnected { reason } => {
                info!("Disconnected: {:?}", reason);
                break;
            }
            GattConnectionEvent::Gatt {
                event: GattEvent::Write(write_event),
            } => {
                // Check if this is a write to our motor characteristic
                if write_event.handle() == server.rc_service.motor_control.handle {
                    // Get the data from the server
                    if let Ok(data) = server.get(&server.rc_service.motor_control) {
                        let left = data[0] as i8;
                        let right = data[1] as i8;

                        debug!("Web BLE cmd: L={} R={}", left, right);

                        let cmd = MotorCommand {
                            left,
                            right,
                            flags: CommandFlags(0),
                        };

                        let _ = MOTOR_CMD_CHANNEL.try_send(cmd);
                        let _ = LED_STATUS_CHANNEL.try_send(if left == 0 && right == 0 {
                            LedStatus::Connected
                        } else {
                            LedStatus::Driving { left, right }
                        });
                    }
                }

                // Accept the write
                if let Ok(reply) = write_event.accept() {
                    reply.send().await;
                }
            }
            _ => {}
        }
    }
}

// ========================================
// GAMEPAD MODE (Central)
// ========================================
#[allow(clippy::large_stack_frames)]
async fn run_gamepad_mode<C: Controller + ControllerCmdSync<LeSetScanParams>>(
    controller: C,
    resources: &mut HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX>,
    address: Address,
) {
    let stack = trouble_host::new(controller, resources).set_random_address(address);

    let Host {
        mut central,
        mut runner,
        ..
    } = stack.build();

    info!("BLE initialized as Central");
    info!("");
    info!("========================================");
    info!("GAMEPAD CONNECTION MODE");
    info!("========================================");
    info!("Controls:");
    info!("  Left Stick: Drive (Y=throttle, X=turn)");
    info!("  L1: Spin left 180 deg");
    info!("  R1: Spin right 180 deg");
    info!("  B:  Emergency stop");
    info!("========================================");
    info!("");

    // ========================================
    // TARGET ADDRESS CONFIGURATION
    // ========================================
    // Set to None to scan for devices, or Some([...]) to connect directly
    // Address bytes are in REVERSE order (little-endian)
    //
    // Examples:
    //   None => Scan mode (discovers nearby BLE devices)
    //   Some([0x2A, 0x86, 0x13, 0xD8, 0x17, 0xE4]) => E4:17:D8:13:86:2A (8BitDo)
    //
    // let target_addr: Option<[u8; 6]> = Some([0x90, 0xCA, 0x7D, 0x22, 0x16, 0x44]); //None; // <-- SET YOUR ADDRESS HERE or None to scan
    // let target_addr: Option<[u8; 6]> = Some([0x25, 0x63, 0x0D, 0x68, 0x7B, 0xFF]);
    let target_addr: Option<[u8; 6]> = None;

    if let Some(addr_bytes) = target_addr {
        // ========== CONNECTION MODE ==========
        let target = Address {
            kind: AddrKind::PUBLIC,
            addr: BdAddr::new(addr_bytes),
        };

        // Print address in readable format
        info!(
            "Target: {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
            addr_bytes[5],
            addr_bytes[4],
            addr_bytes[3],
            addr_bytes[2],
            addr_bytes[1],
            addr_bytes[0]
        );

        let connect_config = ConnectConfig {
            connect_params: Default::default(),
            scan_config: ScanConfig {
                filter_accept_list: &[(target.kind, &target.addr)],
                ..Default::default()
            },
        };

        let _ = join(runner.run(), async {
            loop {
                info!("Scanning for gamepad...");
                let _ = LED_STATUS_CHANNEL.try_send(LedStatus::Scanning);

                match central.connect(&connect_config).await {
                    Ok(conn) => {
                        info!("Connected to gamepad!");
                        let _ = LED_STATUS_CHANNEL.try_send(LedStatus::Connecting);

                        match GattClient::<_, DefaultPacketPool, 10>::new(&stack, &conn).await {
                            Ok(client) => {
                                let _ = LED_STATUS_CHANNEL.try_send(LedStatus::Connected);
                                let _ = join(client.task(), async {
                                    handle_gamepad(&client).await;
                                })
                                .await;
                            }
                            Err(_) => {
                                error!("Failed to create GATT client");
                            }
                        }

                        info!("Gamepad disconnected");
                        let _ = LED_STATUS_CHANNEL.try_send(LedStatus::Disconnected);
                    }
                    Err(_) => {
                        info!("Connection failed, retrying...");
                        let _ = LED_STATUS_CHANNEL.try_send(LedStatus::Disconnected);
                    }
                }

                Timer::after(Duration::from_secs(2)).await;
            }
        })
        .await;
    } else {
        // ========== SCAN MODE ==========
        info!("========================================");
        info!("SCAN MODE - No target address set");
        info!("========================================");
        info!("Put your gamepad in pairing mode now!");
        info!("========================================");
        info!("");

        struct DevicePrinter {
            seen: RefCell<[Option<BdAddr>; 32]>,
            count: RefCell<usize>,
        }

        impl EventHandler for DevicePrinter {
            fn on_adv_reports(&self, mut it: LeAdvReportsIter<'_>) {
                let mut seen = self.seen.borrow_mut();
                let mut count = self.count.borrow_mut();
                while let Some(Ok(report)) = it.next() {
                    let already_seen = seen.iter().any(|s| {
                        s.map(|addr| addr.raw() == report.addr.raw())
                            .unwrap_or(false)
                    });
                    if !already_seen && *count < 32 {
                        let addr = report.addr.raw();
                        info!(
                            "Found: {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
                            addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]
                        );
                        // Also print the bytes for easy copy-paste
                        info!(
                            "  -> Use: Some([0x{:02X}, 0x{:02X}, 0x{:02X}, 0x{:02X}, 0x{:02X}, 0x{:02X}])",
                            addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]
                        );
                        seen[*count] = Some(report.addr);
                        *count += 1;
                    }
                }
            }
        }

        let printer = DevicePrinter {
            seen: RefCell::new([None; 32]),
            count: RefCell::new(0),
        };

        let mut scanner = Scanner::new(central);
        let _ = LED_STATUS_CHANNEL.try_send(LedStatus::Scanning);

        let _ = join(runner.run_with_handler(&printer), async {
            let config = ScanConfig {
                active: true,
                interval: Duration::from_millis(100),
                window: Duration::from_millis(100),
                ..Default::default()
            };

            info!("Starting 60 second scan...");
            let _session = scanner.scan(&config).await;

            Timer::after(Duration::from_secs(60)).await;
            info!("");
            info!("========================================");
            info!("Scan complete!");
            info!("Copy the address bytes above and set:");
            info!("  let target_addr: Option<[u8; 6]> = Some([...]);");
            info!("========================================");
        })
        .await;
    }

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}

/// Handle gamepad HID reports
#[allow(
    clippy::large_stack_frames,
    reason = "async fn with BLE operations needs stack space"
)]
async fn handle_gamepad<C: Controller, P: PacketPool>(client: &GattClient<'_, C, P, 10>) {
    info!("Discovering services...");

    // Check what services are available
    info!("Checking common BLE services...");

    info!("Trying Generic Access (0x1800)...");
    match client.services_by_uuid(&Uuid::new_short(0x1800)).await {
        Ok(s) => info!("  Generic Access: {} found", s.len()),
        Err(_) => info!("  Generic Access: query failed"),
    }

    info!("Trying Device Info (0x180A)...");
    match client.services_by_uuid(&Uuid::new_short(0x180A)).await {
        Ok(s) => info!("  Device Info: {} found", s.len()),
        Err(_) => info!("  Device Info: query failed"),
    }

    info!("Trying Battery (0x180F)...");
    match client.services_by_uuid(&Uuid::new_short(0x180F)).await {
        Ok(s) => info!("  Battery: {} found", s.len()),
        Err(_) => info!("  Battery: query failed"),
    }

    info!("Trying HID (0x1812)...");
    let hid_services = match client.services_by_uuid(&Uuid::new_short(0x1812)).await {
        Ok(s) => {
            info!("  HID: {} found", s.len());
            s
        }
        Err(_) => {
            error!("  HID query failed");
            return;
        }
    };

    let hid_service = match hid_services.first() {
        Some(s) => {
            info!("Using HID service");
            s.clone()
        }
        None => {
            error!("No HID service found!");
            error!("Xbox controller may require bonding first");
            return;
        }
    };

    info!("Found HID service! Looking for Report characteristic (0x2A4D)...");

    // HID Report UUID is 0x2A4D
    let report_uuid = Uuid::new_short(0x2A4D);

    let report_char: Characteristic<Uuid> = match client
        .characteristic_by_uuid(&hid_service, &report_uuid)
        .await
    {
        Ok(c) => {
            info!("  Found Report characteristic");
            c
        }
        Err(_) => {
            error!("  Failed to find Report characteristic");
            error!("Xbox controller likely requires BLE bonding/pairing");
            error!("which isn't implemented. Try a different controller.");
            return;
        }
    };

    info!("Subscribing to Report characteristic (notifications)...");

    match client.subscribe(&report_char, true).await {
        Ok(mut listener) => {
            info!("========================================");
            info!("Subscribe SUCCEEDED!");
            info!("Waiting for HID notifications...");
            info!("Move the controller sticks NOW!");
            info!("========================================");

            // Debug flag - set to true to log all raw input data
            let log_controller_input = true;

            let mut last_state = GamepadState::default();
            let mut packet_count: u32 = 0;
            let mut got_first_packet = false;

            loop {
                // Use select with timeout to detect if no data is coming
                let timeout_duration = if got_first_packet {
                    Duration::from_secs(30) // Long timeout after we get data
                } else {
                    Duration::from_secs(10) // Short timeout waiting for first packet
                };

                let result = select(listener.next(), Timer::after(timeout_duration)).await;

                match result {
                    Either::First(notification) => {
                        let data = notification.as_ref();
                        packet_count += 1;

                        if !got_first_packet {
                            got_first_packet = true;
                            info!("========================================");
                            info!("GOT FIRST HID PACKET! Controller working!");
                            info!("========================================");
                        }

                        // Flash LED on any input if debugging
                        if log_controller_input {
                            let _ = LED_STATUS_CHANNEL.try_send(LedStatus::InputReceived);

                            // Log raw data every 50 packets to avoid spam
                            if packet_count % 50 == 1 {
                                info!("Raw HID packet #{} (len={})", packet_count, data.len());
                                // Print first 16 bytes
                                if data.len() >= 16 {
                                    info!(
                                        "  [{:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X}]",
                                        data[0],
                                        data[1],
                                        data[2],
                                        data[3],
                                        data[4],
                                        data[5],
                                        data[6],
                                        data[7],
                                        data[8],
                                        data[9],
                                        data[10],
                                        data[11],
                                        data[12],
                                        data[13],
                                        data[14],
                                        data[15]
                                    );
                                } else {
                                    info!("  Data: {:?}", &data[..data.len().min(16)]);
                                }
                            }
                        }

                        if let Some(state) = parse_generic_gamepad(data) {
                            if log_controller_input && packet_count % 50 == 1 {
                                info!(
                                    "  Parsed: LX={} LY={} RX={} RY={} Btn={:04X}",
                                    state.left_x,
                                    state.left_y,
                                    state.right_x,
                                    state.right_y,
                                    state.buttons.0
                                );
                            }

                            if state_changed(&last_state, &state) {
                                debug!(
                                    "Stick: ({}, {}) Btn: {:04x}",
                                    state.left_x, state.left_y, state.buttons.0
                                );

                                let cmd = gamepad_to_motor_cmd(&state);

                                if log_controller_input {
                                    info!(
                                        "Motor cmd: L={} R={} flags={:02X}",
                                        cmd.left, cmd.right, cmd.flags.0
                                    );
                                }

                                let _ = MOTOR_CMD_CHANNEL.try_send(cmd);

                                last_state = state;
                            }
                        } else if log_controller_input && packet_count % 50 == 1 {
                            info!("  Parse failed - unknown format");
                        }
                    }
                    Either::Second(_) => {
                        // Timeout - no data received
                        if !got_first_packet {
                            error!("========================================");
                            error!("TIMEOUT: No HID data received!");
                            error!("========================================");
                            error!("Xbox controller requires BLE BONDING");
                            error!("which this firmware doesn't support.");
                            error!("");
                            error!("Try one of these controllers instead:");
                            error!("  - 8BitDo Pro 2 (Switch mode: START+Y)");
                            error!("  - 8BitDo Lite 2 ");
                            error!("  - Generic BLE gamepad");
                            error!("  - Nintendo Joy-Con");
                            error!("========================================");
                            let _ = LED_STATUS_CHANNEL.try_send(LedStatus::Disconnected);
                            return;
                        } else {
                            info!("Input timeout - controller may have disconnected");
                        }
                    }
                }
            }
        }
        Err(_) => {
            error!("Failed to subscribe to HID reports");
        }
    }

    let _ = MOTOR_CMD_CHANNEL.try_send(MotorCommand::default());
}

fn state_changed(old: &GamepadState, new: &GamepadState) -> bool {
    const AXIS_THRESHOLD: i8 = 5;

    (old.left_x - new.left_x).abs() > AXIS_THRESHOLD
        || (old.left_y - new.left_y).abs() > AXIS_THRESHOLD
        || old.buttons.0 != new.buttons.0
        || old.dpad != new.dpad
}
