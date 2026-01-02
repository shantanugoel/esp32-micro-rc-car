//! Web Bluetooth peripheral mode for RC car control.
//!
//! This module implements a BLE GATT server that allows control from
//! a web browser using the Web Bluetooth API.

use defmt::{debug, error, info, warn};
use embassy_futures::join::join;
use embassy_time::{Duration, Timer};
use trouble_host::prelude::*;

use crate::ble::{CommandFlags, MotorCommand};

/// Channel type for motor commands
pub type MotorCmdChannel = embassy_sync::channel::Channel<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    MotorCommand,
    4,
>;

/// Channel type for LED status updates  
pub type LedStatusChannel<S> = embassy_sync::channel::Channel<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    S,
    4,
>;

// ========================================
// GATT Service Definition
// ========================================
// Custom service UUID for RC car control
// Service UUID: 19b10000-e8f2-537e-4f6c-d104768a1214
// Motor Char:   19b10001-e8f2-537e-4f6c-d104768a1214

/// Motor control service for RC car
#[gatt_service(uuid = "19b10000-e8f2-537e-4f6c-d104768a1214")]
pub struct RcCarService {
    /// Motor control: 2 bytes [left_speed: i8, right_speed: i8]
    #[characteristic(
        uuid = "19b10001-e8f2-537e-4f6c-d104768a1214",
        write,
        write_without_response
    )]
    pub motor_control: [u8; 2],
}

/// GATT server containing our RC car service
#[gatt_server]
pub struct RcCarServer {
    pub rc_service: RcCarService,
}

/// LED status variants needed by WebBLE mode
pub trait WebBleLedStatus: Clone + Copy {
    fn advertising() -> Self;
    fn connected() -> Self;
    fn disconnected() -> Self;
    fn driving(left: i8, right: i8) -> Self;
}

/// Run WebBLE peripheral mode
///
/// The ESP32 advertises as "ESP32-RC-Car" and accepts connections from
/// web browsers using the Web Bluetooth API.
#[allow(clippy::large_stack_frames)]
pub async fn run<C, S>(
    controller: C,
    resources: &mut HostResources<DefaultPacketPool, 1, 3>,
    address: Address,
    motor_channel: &MotorCmdChannel,
    led_channel: &LedStatusChannel<S>,
) where
    C: Controller,
    S: WebBleLedStatus,
{
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
            let _ = led_channel.try_send(S::advertising());

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
                        Ok(accept_result) => match accept_result.with_attribute_server(&server) {
                            Ok(conn) => {
                                info!("========================================");
                                info!("Web browser connected!");
                                info!("========================================");
                                let _ = led_channel.try_send(S::connected());

                                // Handle GATT events
                                handle_gatt_events(&server, &conn, motor_channel, led_channel)
                                    .await;

                                info!("Browser disconnected");
                                let _ = led_channel.try_send(S::disconnected());

                                // Stop motors on disconnect
                                let _ = motor_channel.try_send(MotorCommand::default());
                            }
                            Err(_e) => {
                                warn!("Failed to set up GATT server");
                            }
                        },
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
async fn handle_gatt_events<'a, S>(
    server: &RcCarServer<'a>,
    conn: &GattConnection<'a, '_, DefaultPacketPool>,
    motor_channel: &MotorCmdChannel,
    led_channel: &LedStatusChannel<S>,
) where
    S: WebBleLedStatus,
{
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

                        let _ = motor_channel.try_send(cmd);
                        let _ = led_channel.try_send(if left == 0 && right == 0 {
                            S::connected()
                        } else {
                            S::driving(left, right)
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
