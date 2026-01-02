#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use bt_hci::controller::ExternalController;
use bt_hci::param::{AddrKind, BdAddr};
use core::cell::RefCell;
use defmt::{debug, error, info};
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::ledc::{LSGlobalClkSource, Ledc, channel, timer};
use esp_hal::timer::timg::TimerGroup;
use esp_radio::ble::controller::BleConnector;
use esp32_micro_rc_car::ble::{
    CommandFlags, GamepadState, HID_REPORT_UUID, MotorCommand, gamepad_to_motor_cmd,
    parse_generic_gamepad,
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

/// Channel for motor commands
static MOTOR_CMD_CHANNEL: Channel<CriticalSectionRawMutex, MotorCommand, 4> = Channel::new();

static LEDC_CELL: StaticCell<Ledc<'static>> = StaticCell::new();
static TIMER0_CELL: StaticCell<timer::Timer<'static, esp_hal::ledc::LowSpeed>> = StaticCell::new();
static TIMER1_CELL: StaticCell<timer::Timer<'static, esp_hal::ledc::LowSpeed>> = StaticCell::new();

/// Motor control task
#[allow(
    clippy::large_stack_frames,
    reason = "async task with awaits needs stack space"
)]
#[embassy_executor::task]
async fn motor_control_task(
    mut drive: DriveTrain<'static>,
    mut led: Ws2812<'static>,
    turn_config: TurnConfig,
) {
    info!("Motor control task started");
    led.set_color(Color::blue(10)).await;

    loop {
        let cmd = MOTOR_CMD_CHANNEL.receive().await;

        if cmd.flags.has(CommandFlags::EMERGENCY_STOP) {
            info!("Emergency stop!");
            led.set_color(Color::red(50)).await;
            drive.brake();
            Timer::after(Duration::from_millis(100)).await;
            continue;
        }

        if cmd.flags.has(CommandFlags::ABOUT_TURN_LEFT) {
            info!("About turn left");
            led.set_color(Color::new(25, 0, 25)).await;
            drive.spin(-(turn_config.spin_speed as i8));
            Timer::after(Duration::from_millis(turn_config.turn_180_ms as u64)).await;
            drive.brake();
            continue;
        }

        if cmd.flags.has(CommandFlags::ABOUT_TURN_RIGHT) {
            info!("About turn right");
            led.set_color(Color::new(25, 25, 0)).await;
            drive.spin(turn_config.spin_speed as i8);
            Timer::after(Duration::from_millis(turn_config.turn_180_ms as u64)).await;
            drive.brake();
            continue;
        }

        if cmd.left == 0 && cmd.right == 0 {
            drive.stop();
            led.set_color(Color::blue(10)).await;
        } else {
            drive.tank_drive(cmd.left, cmd.right);
            let intensity = ((cmd.left.abs() + cmd.right.abs()) / 4) as u8;
            led.set_color(Color::green(intensity.max(10))).await;
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

    let drive = DriveTrain::new(motor_left, motor_right);

    let mut stby = StandbyPin::new(peripherals.GPIO7);
    stby.enable();

    info!("Motors ready!");

    let turn_config = TurnConfig::default();
    spawner
        .spawn(motor_control_task(drive, led, turn_config))
        .unwrap();

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
    let stack = trouble_host::new(ble_controller, &mut resources).set_random_address(address);

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
    let target_addr: Option<[u8; 6]> = Some([0x90, 0xCA, 0x7D, 0x22, 0x16, 0x44]); //None; // <-- SET YOUR ADDRESS HERE or None to scan

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

                match central.connect(&connect_config).await {
                    Ok(conn) => {
                        info!("Connected to gamepad!");

                        match GattClient::<_, DefaultPacketPool, 10>::new(&stack, &conn).await {
                            Ok(client) => {
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
                    }
                    Err(_) => {
                        info!("Connection failed, retrying...");
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
            error!("No HID service found - wrong controller mode?");
            error!("8BitDo Pro 2 modes:");
            error!("  START+Y = Switch mode (BLE HID) <- USE THIS");
            error!("  START+B = Android (may be Classic BT)");
            error!("  START+X = Xbox mode");
            error!("Try: Turn off, then START+Y for 3 seconds");
            return;
        }
    };

    info!("Found HID service! Looking for Report characteristic...");

    let report_char: Characteristic<Uuid> = match client
        .characteristic_by_uuid(&hid_service, &HID_REPORT_UUID)
        .await
    {
        Ok(c) => c,
        Err(_) => {
            error!("Failed to find Report characteristic");
            return;
        }
    };

    info!("Found Report characteristic, subscribing...");

    match client.subscribe(&report_char, true).await {
        Ok(mut listener) => {
            info!("Subscribed! Move the left stick to drive.");

            let mut last_state = GamepadState::default();

            loop {
                let notification = listener.next().await;
                let data = notification.as_ref();

                if let Some(state) = parse_generic_gamepad(data)
                    && state_changed(&last_state, &state)
                {
                    debug!(
                        "Stick: ({}, {}) Btn: {:04x}",
                        state.left_x, state.left_y, state.buttons.0
                    );

                    let cmd = gamepad_to_motor_cmd(&state);
                    let _ = MOTOR_CMD_CHANNEL.try_send(cmd);

                    last_state = state;
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
