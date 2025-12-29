#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use bt_hci::controller::ExternalController;
use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::ledc::{LSGlobalClkSource, Ledc, channel, timer};
use esp_hal::timer::timg::TimerGroup;
use esp_radio::ble::controller::BleConnector;
use esp32_micro_rc_car::led::{Color, Ws2812};
use esp32_micro_rc_car::motor::{self, Direction, Motor, StandbyPin};
use trouble_host::prelude::*;
use {esp_backtrace as _, esp_println as _};

extern crate alloc;

const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 1;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 1.1.0

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);
    // COEX needs more RAM - so we've added some more
    esp_alloc::heap_allocator!(size: 64 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    info!("Embassy initialized!");

    let radio_init = esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller");
    let (mut _wifi_controller, _interfaces) =
        esp_radio::wifi::new(&radio_init, peripherals.WIFI, Default::default())
            .expect("Failed to initialize Wi-Fi controller");
    // find more examples https://github.com/embassy-rs/trouble/tree/main/examples/esp32
    let transport = BleConnector::new(&radio_init, peripherals.BT, Default::default()).unwrap();
    let ble_controller = ExternalController::<_, 1>::new(transport);
    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    let _stack = trouble_host::new(ble_controller, &mut resources);

    // TODO: Spawn some tasks
    let _ = spawner;

    // Initialize WS2812B LED on GPIO21
    let mut led = Ws2812::new(peripherals.RMT, peripherals.GPIO21);

    // ========== MOTOR PWM TEST ==========
    // Using Driver 1, Channel A (Left-Front motor):
    // - PWMA → GPIO1 (PWM speed control)
    // - AIN1 → GPIO2 (direction)
    // - AIN2 → GPIO3 (direction)
    // - STBY → GPIO13 (enable driver)

    info!("Setting up motor PWM test...");

    // Configure LEDC for PWM
    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    // Configure timer for 20kHz PWM (above audible range)
    let timer0 = motor::configure_timer(&ledc, timer::Number::Timer0);

    // Configure PWM channel on GPIO1
    let pwm_channel =
        motor::configure_channel(&ledc, channel::Number::Channel0, peripherals.GPIO1, &timer0);

    // Create motor with direction pins
    let in1 = motor::output_pin(peripherals.GPIO2);
    let in2 = motor::output_pin(peripherals.GPIO3);
    let mut motor_lf = Motor::new(in1, in2, pwm_channel);

    // Enable motor driver via STBY pin
    let mut stby = StandbyPin::new(peripherals.GPIO13);
    stby.enable();

    info!("Motor test starting - connect motor to Driver 1 Channel A");
    info!("Pins: PWM=GPIO1, IN1=GPIO2, IN2=GPIO3, STBY=GPIO13");

    // LED indicates test state
    led.set_color(Color::green(10)).await;

    loop {
        // Test: Ramp up forward
        info!("Forward: ramping up 0->100%%");
        led.set_color(Color::green(25)).await;
        motor_lf.set_direction(Direction::Forward);
        for speed in (0..=100).step_by(10) {
            motor_lf.set_speed(speed);
            info!("  Speed: {}%%", speed);
            Timer::after(Duration::from_millis(200)).await;
        }

        Timer::after(Duration::from_millis(500)).await;

        // Test: Ramp down forward
        info!("Forward: ramping down 100->0%%");
        for speed in (0..=100).rev().step_by(10) {
            motor_lf.set_speed(speed);
            info!("  Speed: {}%%", speed);
            Timer::after(Duration::from_millis(200)).await;
        }

        // Brake
        info!("Brake");
        led.set_color(Color::red(25)).await;
        motor_lf.brake();
        Timer::after(Duration::from_millis(1000)).await;

        // Test: Ramp up reverse
        info!("Reverse: ramping up 0->100%%");
        led.set_color(Color::blue(25)).await;
        motor_lf.set_direction(Direction::Reverse);
        for speed in (0..=100).step_by(10) {
            motor_lf.set_speed(speed);
            info!("  Speed: {}%%", speed);
            Timer::after(Duration::from_millis(200)).await;
        }

        Timer::after(Duration::from_millis(500)).await;

        // Test: Ramp down reverse
        info!("Reverse: ramping down 100->0%%");
        for speed in (0..=100).rev().step_by(10) {
            motor_lf.set_speed(speed);
            info!("  Speed: {}%%", speed);
            Timer::after(Duration::from_millis(200)).await;
        }

        // Coast stop
        info!("Coast stop");
        led.set_color(Color::off()).await;
        motor_lf.stop();
        Timer::after(Duration::from_millis(2000)).await;

        info!("--- Test cycle complete, repeating ---");
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v~1.0/examples
}
