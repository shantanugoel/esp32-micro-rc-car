//! Gamepad BLE central mode for RC car control.
//!
//! This module implements a BLE central that connects to gamepads
//! and reads HID reports for control input.

use bt_hci::cmd::le::LeSetScanParams;
use bt_hci::controller::ControllerCmdSync;
use bt_hci::param::{AddrKind, BdAddr};
use core::cell::RefCell;
use defmt::{debug, error, info};
use embassy_futures::join::join;
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Timer};
use trouble_host::prelude::*;
use trouble_host::scan::Scanner;

use crate::ble::{GamepadState, MotorCommand, gamepad_to_motor_cmd, parse_generic_gamepad};

/// Channel type for motor commands
pub type MotorCmdChannel =
    embassy_sync::channel::Channel<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, MotorCommand, 4>;

/// Channel type for LED status updates
pub type LedStatusChannel<S> =
    embassy_sync::channel::Channel<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, S, 4>;

/// LED status variants needed by gamepad mode
pub trait GamepadLedStatus: Clone + Copy {
    fn scanning() -> Self;
    fn connecting() -> Self;
    fn connected() -> Self;
    fn disconnected() -> Self;
    fn input_received() -> Self;
}

/// Run gamepad central mode with connection to a specific address
#[allow(clippy::large_stack_frames)]
pub async fn run_connection<C, S>(
    controller: C,
    resources: &mut HostResources<DefaultPacketPool, 1, 3>,
    address: Address,
    target_addr: [u8; 6],
    motor_channel: &MotorCmdChannel,
    led_channel: &LedStatusChannel<S>,
) where
    C: Controller,
    S: GamepadLedStatus,
{
    let stack = trouble_host::new(controller, resources).set_random_address(address);

    let Host {
        mut central,
        mut runner,
        ..
    } = stack.build();

    print_gamepad_banner();

    let target = Address {
        kind: AddrKind::PUBLIC,
        addr: BdAddr::new(target_addr),
    };

    // Print address in readable format
    info!(
        "Target: {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
        target_addr[5],
        target_addr[4],
        target_addr[3],
        target_addr[2],
        target_addr[1],
        target_addr[0]
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
            let _ = led_channel.try_send(S::scanning());

            match central.connect(&connect_config).await {
                Ok(conn) => {
                    info!("Connected to gamepad!");
                    let _ = led_channel.try_send(S::connecting());

                    match GattClient::<_, DefaultPacketPool, 10>::new(&stack, &conn).await {
                        Ok(client) => {
                            let _ = led_channel.try_send(S::connected());
                            let _ = join(client.task(), async {
                                handle_gamepad(&client, motor_channel, led_channel).await;
                            })
                            .await;
                        }
                        Err(_) => {
                            error!("Failed to create GATT client");
                        }
                    }

                    info!("Gamepad disconnected");
                    let _ = led_channel.try_send(S::disconnected());
                }
                Err(_) => {
                    info!("Connection failed, retrying...");
                    let _ = led_channel.try_send(S::disconnected());
                }
            }

            Timer::after(Duration::from_secs(2)).await;
        }
    })
    .await;
}

/// Run gamepad scan mode to discover nearby BLE devices
#[allow(clippy::large_stack_frames)]
pub async fn run_scan<C, S>(
    controller: C,
    resources: &mut HostResources<DefaultPacketPool, 1, 3>,
    address: Address,
    led_channel: &LedStatusChannel<S>,
) where
    C: Controller + ControllerCmdSync<LeSetScanParams>,
    S: GamepadLedStatus,
{
    let stack = trouble_host::new(controller, resources).set_random_address(address);

    let Host {
        central,
        mut runner,
        ..
    } = stack.build();

    print_gamepad_banner();

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
                let already_seen = seen
                    .iter()
                    .any(|s| s.map(|addr| addr.raw() == report.addr.raw()).unwrap_or(false));
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
    let _ = led_channel.try_send(S::scanning());

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

fn print_gamepad_banner() {
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
}

/// Handle gamepad HID reports
#[allow(clippy::large_stack_frames)]
async fn handle_gamepad<C, P, S>(
    client: &GattClient<'_, C, P, 10>,
    motor_channel: &MotorCmdChannel,
    led_channel: &LedStatusChannel<S>,
) where
    C: Controller,
    P: PacketPool,
    S: GamepadLedStatus,
{
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

    let report_char: Characteristic<Uuid> =
        match client.characteristic_by_uuid(&hid_service, &report_uuid).await {
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
                            let _ = led_channel.try_send(S::input_received());

                            // Log raw data every 50 packets to avoid spam
                            if packet_count % 50 == 1 {
                                info!("Raw HID packet #{} (len={})", packet_count, data.len());
                                // Print first 16 bytes
                                if data.len() >= 16 {
                                    info!(
                                        "  [{:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X}]",
                                        data[0], data[1], data[2], data[3],
                                        data[4], data[5], data[6], data[7],
                                        data[8], data[9], data[10], data[11],
                                        data[12], data[13], data[14], data[15]
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
                                    state.left_x, state.left_y, state.right_x, state.right_y, state.buttons.0
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

                                let _ = motor_channel.try_send(cmd);

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
                            let _ = led_channel.try_send(S::disconnected());
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

    let _ = motor_channel.try_send(MotorCommand::default());
}

/// Check if gamepad state has changed significantly
fn state_changed(old: &GamepadState, new: &GamepadState) -> bool {
    const AXIS_THRESHOLD: i8 = 5;

    (old.left_x - new.left_x).abs() > AXIS_THRESHOLD
        || (old.left_y - new.left_y).abs() > AXIS_THRESHOLD
        || old.buttons.0 != new.buttons.0
        || old.dpad != new.dpad
}
