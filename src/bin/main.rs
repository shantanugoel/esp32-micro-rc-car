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
use esp32_micro_rc_car::controller::DriveTrain;
use esp32_micro_rc_car::led::{Color, Ws2812};
use esp32_micro_rc_car::motor::{self, Motor, StandbyPin};
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

    // ========== DUAL MOTOR DRIVE TRAIN TEST ==========
    // TB6612FNG Pin Configuration:
    // Channel A (Left motors):   PWMA→GPIO1, AIN2→GPIO2, AIN1→GPIO3
    // Channel B (Right motors):  PWMB→GPIO6, BIN1→GPIO4, BIN2→GPIO5
    // Shared: STBY→GPIO14

    info!("Setting up drive train...");

    // Configure LEDC for PWM
    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    // Configure timers for 20kHz PWM (above audible range)
    let timer0 = motor::configure_timer(&ledc, timer::Number::Timer0);
    let timer1 = motor::configure_timer(&ledc, timer::Number::Timer1);

    // Configure PWM channels
    // Left motor: Channel 0 on GPIO1
    let pwm_left =
        motor::configure_channel(&ledc, channel::Number::Channel0, peripherals.GPIO1, &timer0);
    // Right motor: Channel 1 on GPIO6
    let pwm_right =
        motor::configure_channel(&ledc, channel::Number::Channel1, peripherals.GPIO6, &timer1);

    // Create motors
    // Left motor: AIN1=GPIO3, AIN2=GPIO2
    let left_in1 = motor::output_pin(peripherals.GPIO3);
    let left_in2 = motor::output_pin(peripherals.GPIO2);
    let motor_left = Motor::new(left_in1, left_in2, pwm_left);

    // Right motor: BIN1=GPIO4, BIN2=GPIO5
    let right_in1 = motor::output_pin(peripherals.GPIO4);
    let right_in2 = motor::output_pin(peripherals.GPIO5);
    let motor_right = Motor::new(right_in1, right_in2, pwm_right);

    // Create drive train
    let mut drive = DriveTrain::new(motor_left, motor_right);

    // Enable motor driver via STBY pin
    let mut stby = StandbyPin::new(peripherals.GPIO14);
    stby.enable();

    info!("Drive train ready!");
    info!("Pins: Left(PWM=1,IN2=2,IN1=3) Right(PWM=6,IN1=4,IN2=5) STBY=14");

    // LED indicates test state
    led.set_color(Color::green(10)).await;

    loop {
        // Test 1: Both forward
        info!("Test 1: Both motors forward");
        led.set_color(Color::green(25)).await;
        drive.arcade_drive(75, 0); // Forward at 75%
        Timer::after(Duration::from_millis(2000)).await;

        // Test 2: Turn right (slow right motor)
        info!("Test 2: Turn right");
        led.set_color(Color::new(25, 12, 0)).await; // Orange
        drive.arcade_drive(60, 40); // Forward + turn right
        Timer::after(Duration::from_millis(2000)).await;

        // Test 3: Turn left (slow left motor)
        info!("Test 3: Turn left");
        led.set_color(Color::new(0, 12, 25)).await; // Cyan
        drive.arcade_drive(60, -40); // Forward + turn left
        Timer::after(Duration::from_millis(2000)).await;

        // Test 4: Spin clockwise
        info!("Test 4: Spin clockwise");
        led.set_color(Color::blue(25)).await;
        drive.spin(50);
        Timer::after(Duration::from_millis(1500)).await;

        // Test 5: Spin counter-clockwise
        info!("Test 5: Spin counter-clockwise");
        led.set_color(Color::new(25, 0, 25)).await; // Purple
        drive.spin(-50);
        Timer::after(Duration::from_millis(1500)).await;

        // Test 6: Reverse
        info!("Test 6: Both motors reverse");
        led.set_color(Color::red(25)).await;
        drive.arcade_drive(-75, 0); // Reverse at 75%
        Timer::after(Duration::from_millis(2000)).await;

        // Test 7: Tank drive - direct control
        info!("Test 7: Tank drive - left 50%%, right 100%%");
        led.set_color(Color::new(25, 25, 0)).await; // Yellow
        drive.tank_drive(50, 100);
        Timer::after(Duration::from_millis(2000)).await;

        // Stop and pause
        info!("Stopping...");
        led.set_color(Color::off()).await;
        drive.brake();
        Timer::after(Duration::from_millis(3000)).await;

        info!("--- Test cycle complete, repeating ---");
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v~1.0/examples
}
