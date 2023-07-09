//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    self,
    clocks::init_clocks_and_plls,
    pac,
    sio::Sio,
    watchdog::Watchdog,
    pio::{PIOExt, PIOBuilder},
    gpio::{Pin, FunctionPio0},
};

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let _clocks = init_clocks_and_plls(
        bsp::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Set dividers to determine output clock frequency.
    //
    //    f_out = (sys_clk / 4) / (int + frac/256)
    //
    // Example: 125 MHz / 4 / (31 + 64/256) = 1 MHz
    //

    let (int, frac) = (31, 64); // 1 MHz
    //let (int, frac) = (15, 160); // 2 MHz
    //let (int, frac) = (4, 119); // ~7 MHz
    //let (int, frac) = (2, 59); // ~14 MHz
    //let (int, frac) = (1, 0); // 31.25 MHz (maximum)
    //let (int, frac) = (0, 0); // 476.84 Hz (minimum)

    // Select two pins for the output.
    // NOTE: these pins MUST have consecutive pin numbers, and the lower one MUST be pin1.
    let pin1: Pin<_, FunctionPio0, _> = pins.gpio16.into_function();
    let pin2: Pin<_, FunctionPio0, _> = pins.gpio17.into_function();
    let pin_base = pin1.id().num;

    let program = pio_proc::pio_asm!(
        ".wrap_target",
        "set pins, 0b00 [0]",
        "set pins, 0b01 [0]",
        "set pins, 0b11 [0]",
        "set pins, 0b10 [0]",
        ".wrap"
    );

    // Initialize and start PIO
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    // Install the program
    let installed = pio.install(&program.program).unwrap();

    // Set up state machine
    let (mut sm, _, _) = PIOBuilder::from_program(installed)
        .set_pins(pin_base, 2)
        .clock_divisor_fixed_point(int, frac)
        .build(sm0);
    // The GPIO pin needs to be configured as an output.
    sm.set_pindirs([
        (pin1.id().num, hal::pio::PinDir::Output),
        (pin2.id().num, hal::pio::PinDir::Output)
    ]);
    sm.start();

    // PIO runs in background, independently from CPU
    loop {
        cortex_m::asm::wfi();
    }
}

// End of file
