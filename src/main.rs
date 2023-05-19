//! Using shift register 74hc595 and multiplexer cd4051b with ESP32
//!

#![no_std]
#![no_main]

use drive_74hc595::ShiftRegister as ShiftRegister74hc595;
use esp_backtrace as _;
use esp_println::println;
use hal::ehal::digital::v2::OutputPin;
use hal::{
    adc::{AdcConfig, AdcPin, Attenuation, RegisterAccess, ADC, ADC2},
    clock::ClockControl,
    ehal::adc::Channel,
    gpio::IO,
    peripherals::Peripherals,
    prelude::*,
    timer::TimerGroup,
    Delay, Rtc,
};

//
// Refer https://wokwi.com/projects/343522915673702994
// for cd4051b.chip.c and cd4051b.chip.json
//

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.DPORT.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();
    println!("Hello ESP!");

    // Setup the GPIOs.
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    // water pump (23)
    let mut pin_23_pump = io.pins.gpio23.into_push_pull_output();
    // 74hc595 (1, 3, 18, 19, 21)
    let pin_17_oe = io.pins.gpio17.into_push_pull_output();
    let pin_21_ser = io.pins.gpio21.into_push_pull_output();
    let pin_16_srclr = io.pins.gpio16.into_push_pull_output();
    let pin_19_srclk = io.pins.gpio19.into_push_pull_output();
    let pin_18_rclk = io.pins.gpio18.into_push_pull_output();
    // cd4051b (33, 25, 26, 14)
    let pin_33_a = io.pins.gpio33.into_push_pull_output();
    let pin_25_b = io.pins.gpio25.into_push_pull_output();
    let pin_26_c = io.pins.gpio26.into_push_pull_output();

    // Create ADC instances
    let analog = peripherals.SENS.split();

    let mut adc2_config = AdcConfig::new();
    let pin_14_adc =
        adc2_config.enable_pin(io.pins.gpio14.into_analog(), Attenuation::Attenuation11dB);
    let mut adc2 = ADC::<ADC2>::adc(analog.adc2, adc2_config).unwrap();

    let mut multiplexer = Multiplexer::new(&mut adc2, pin_14_adc, pin_33_a, pin_25_b, pin_26_c);

    let mut shift_register = ShiftRegister::new(
        pin_17_oe,
        pin_21_ser,
        pin_16_srclr,
        pin_19_srclk,
        pin_18_rclk,
    );

    // Initialize the Delay peripheral, and use it to toggle the LED state in a
    // loop.
    let mut delay = Delay::new(&clocks);

    let _ = pin_23_pump.set_low();
    shift_register.begin();
    multiplexer.reset();

    let mut loop_counter = 0;

    loop {
        let mut pump = false;
        for addr in 0x0..=0x7 {
            let analog_read = multiplexer.read(addr);
            let value =
                (analog_read - u16::MIN) as f32 / (4096 - u16::MIN) as f32 * (3.3 - 0.0) as f32;
            // println!("[read] {addr:03b} -> {}", value);
            if value < 1.5 && loop_counter > 3 {
                shift_register.set_high(addr);
                pump = true;
            } else {
                shift_register.set_low(addr);
            }
        }
        delay.delay_ms(10u32);
        if pump && loop_counter > 3 {
            let _ = pin_23_pump.set_high();
        } else {
            let _ = pin_23_pump.set_low();
        }
        delay.delay_ms(10u32);
        loop_counter += 1;
    }
}

struct Multiplexer<'d, ADCP, ADCI, A, B, C>
where
    ADCI: RegisterAccess,
    ADCP: Channel<ADCI, ID = u8>,
    A: OutputPin,
    B: OutputPin,
    C: OutputPin,
{
    adc: &'d mut ADC<'d, ADCI>,
    adc_pin: AdcPin<ADCP, ADCI>,
    a_pin: A,
    b_pin: B,
    c_pin: C,
}
// AdcPin<PIN, ADCI>
impl<'d, ADCP, ADCI, A, B, C> Multiplexer<'d, ADCP, ADCI, A, B, C>
where
    ADCI: RegisterAccess,
    ADCP: Channel<ADCI, ID = u8>,
    A: OutputPin,
    B: OutputPin,
    C: OutputPin,
{
    pub fn new(
        adc: &'d mut ADC<'d, ADCI>,
        adc_pin: AdcPin<ADCP, ADCI>,
        a_pin: A,
        b_pin: B,
        c_pin: C,
    ) -> Self {
        Self {
            adc,
            adc_pin,
            a_pin,
            b_pin,
            c_pin,
        }
    }

    pub fn reset(&mut self) {
        let _ = self.a_pin.set_low();
        let _ = self.b_pin.set_low();
        let _ = self.c_pin.set_low();
    }

    pub fn read(&mut self, addr: u8) -> u16 {
        if addr & 0x1 == 0x1 {
            let _ = self.a_pin.set_high();
        }
        if addr & 0x2 == 0x2 {
            let _ = self.b_pin.set_high();
        }
        if addr & 0x4 == 0x4 {
            let _ = self.c_pin.set_high();
        }
        let pin25_value: u16 = nb::block!(self.adc.read(&mut self.adc_pin)).unwrap();
        self.reset();
        pin25_value
    }
}

struct ShiftRegister<OE, SER, SRCLR, SRCLK, RCLK>
where
    OE: OutputPin,
    SER: OutputPin,
    SRCLR: OutputPin,
    SRCLK: OutputPin,
    RCLK: OutputPin,
{
    drive: ShiftRegister74hc595<OE, SER, SRCLR, SRCLK, RCLK>,
    curr_val: u8,
}

impl<OE, SER, SRCLR, SRCLK, RCLK> ShiftRegister<OE, SER, SRCLR, SRCLK, RCLK>
where
    OE: OutputPin,
    SER: OutputPin,
    SRCLR: OutputPin,
    SRCLK: OutputPin,
    RCLK: OutputPin,
{
    pub fn new(oe: OE, ser: SER, srclr: SRCLR, srclk: SRCLK, rclk: RCLK) -> Self {
        let drive = ShiftRegister74hc595::new(oe, ser, srclr, srclk, rclk);
        Self { drive, curr_val: 0 }
    }

    pub fn begin(&mut self) {
        self.drive.begin();
    }

    pub fn set_high(&mut self, addr: u8) {
        assert!(addr < 8);
        let addr = u8::pow(2, addr as u32);
        self.curr_val |= addr;
        self.drive.load(self.curr_val);
    }

    pub fn set_low(&mut self, addr: u8) {
        assert!(addr < 8);
        let addr = u8::pow(2, addr as u32);
        self.curr_val &= !addr;
        self.drive.load(self.curr_val);
    }
}
