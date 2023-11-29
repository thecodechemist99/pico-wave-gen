#![no_std]

//! Arbitrary waveform generator for the Raspberry Pi Pico.
//!
//! Allows the construction, manipulation, combination, and output of arbitrary
//! waveforms on the Raspberry Pi Pico in combination with an R2R DAC.
//!
//! # Features
//!
//! - 8 or 10-bit DAC resolution
//! - Minimal rise time of 14ns (Pico board)
//! - Achieves 125MSps when running with a 125MHz clock. Higher sample rates can
//!   be achieved by overclocking the Pico.
//! - Waveforms are generated in CPU and saved to memory. Streaming is performed
//!   solely by peripherals, therefore the CPU is available to perform other tasks
//!   after initialisation.
//! - Very fast setup time, even at maximum buffer size. In this regard, this
//!   implementation surpasses the original python version by a factor of about 10–30.
//!
//! # Limitations
//!
//! - The buffer size limits how complex shapes can be made, and how accurately the
//!   frequency can be set. For simple waves, like sine or wide pulses, 1024 samples
//!   may be sufficient. A buffer size of 65_536 (64kB) is the practical maximum
//!   because of limited memory. A buffer of this size can still be filled in 1–2
//!   seconds.
//!
//! - Higher DAC resolutions up to 12-bit or 2-channel configurations are theoretically
//!   possible but hard to achieve due to DMA speed limitations. More information about
//!   this is provided in the original [instructables article].
//!
//! # Usage
//!
//! A waveform has a shape, amplitude and offset. A non-zero phase shift can also be
//! set, but its effect is only noticeable when combining waves.
//!
//! There are 6 basic pulse shapes (some with additional parameters):
//! - Sine
//! - Pulse (risetime, uptime and falltime)
//! - Gaussian (width)
//! - Sinc (width)
//! - Exponential (width)
//! - Noise (probability density function, from 1: flat to ≥8: nearly gaussian).
//!
//! It is possible to create additional waveform definitions. Also, waveforms can
//! be combined by summing, multiplying or phase modulation. These operations can
//! be chained to create fairly complex shapes.
//!
//! # Credits
//!
//! This is a Rust implementation of the Arbitrary waveform generator for Rasberry Pi
//! Pico, originally written in Python by Rolf Oldeman (13-2-2021), including some
//! [modifications] made by wolfi (3-11-2021).
//! CC BY-NC-SA 4.0 license.
//!
//! [instructables article]: https://www.instructables.com/Arbitrary-Wave-Generator-With-the-Raspberry-Pi-Pic/
//! [modifications]: https://www.instructables.com/Poor-Mans-Waveform-Generator-Based-on-RP2040-Raspb/
//!

#[cfg(feature = "serial")]
pub mod serial;

#[cfg(feature = "pico")]
mod functions;

// Low-level traits
use core::str::FromStr;
#[cfg(feature = "pico")]
use core::{
    array,
    cmp::{max, min},
    ptr,
    slice::IterMut,
};

// Pico traits
#[cfg(feature = "pico")]
use rp2040_hal::{
    dma::SingleChannel,
    gpio::{DynPinId, FunctionPio0, Pin, PullNone},
    pio::{
        PIOBuilder, PIOExt, PinDir, PinState, Running, ShiftDirection::Right, StateMachine,
        StateMachineIndex, Stopped, Tx, UninitStateMachine, PIO,
    },
    rom_data::float_funcs::{
        fadd, fcmp, fdiv, float_to_uint, fmul, fsub, int_to_float, uint_to_float,
    },
};

// Math and number related traits
#[cfg(feature = "pico")]
use fugit::HertzU32;

// Crate internal traits
#[cfg(feature = "pico")]
use functions::*;

/// Supported DAC resolutions
#[repr(u32)]
pub enum DACResolution {
    BIT8 = 8,
    BIT10 = 10,
}

#[cfg(feature = "pico")]
impl DACResolution {
    /// Returns the size in bits
    fn get_size(&self) -> u32 {
        float_to_uint(nth_power(
            2.0,
            match self {
                Self::BIT8 => 8_u32,
                Self::BIT10 => 10_u32,
            },
        ))
    }

    /// Returns the maximum value of the DAC
    fn get_max_value(&self) -> u32 {
        self.get_size() - 1
    }
}

/// Constructor for enums with equally typed array variants
#[cfg(feature = "pico")]
macro_rules! define_enum_of_arrays {
    (#[doc = $doc:expr] $scope:vis $enum:ident: $type:ty { $($variant:ident($len:expr)),* }) => {
        #[doc = $doc]
        $scope enum $enum {$(
            $variant([$type; $len]),
        )*}

        impl $enum {
            #[allow(unused)]
            fn get(&mut self, index: usize) -> &$type {
                match self {
                    $(Self::$variant(var) => &var[index],)*
                }
            }

            #[allow(unused)]
            fn insert(&mut self, index: usize, val: $type) {
                match self {
                    $(Self::$variant(var) => var[index] = val,)*
                }
            }

            #[allow(unused)]
            fn iter_mut(&mut self) -> IterMut<'_, $type> {
                match self {
                    $(Self::$variant(var) => var.iter_mut(),)*
                }
            }

            fn len(&self) -> usize {
                match self {
                    $(Self::$variant(var) => var.len(),)*
                }
            }
        }
    };
}

#[cfg(feature = "pico")]
define_enum_of_arrays! {
    /// Type declaration for DAC pin arrays
    pub DACPins: Pin<DynPinId, FunctionPio0, PullNone> {
        BIT8(8),
        BIT10(10)
    }
}

#[cfg(feature = "pico")]
define_enum_of_arrays! {
    /// Type declaration for sample buffer sizes
    pub SampleBuffer: u32 {
        B256(256),
        B512(512),
        B1K(1024),
        B2K(2048),
        B4K(4096),
        B8K(8192),
        B16K(16_384),
        B32K(32_768),
        B64k(65_536)
    }
}

/// Available generator functions
#[derive(Debug)]
pub enum GeneratorFunction {
    SINE,
    PULSE,
    GAUSSIAN,
    SINC,
    EXPONENTIAL,
}

impl FromStr for GeneratorFunction {
    type Err = ();

    fn from_str(name: &str) -> Result<Self, Self::Err> {
        match name {
            "Sine" => Ok(GeneratorFunction::SINE),
            "Pulse" => Ok(GeneratorFunction::PULSE),
            "Gaussian" => Ok(GeneratorFunction::GAUSSIAN),
            "Sinc" => Ok(GeneratorFunction::SINC),
            "Exponential" => Ok(GeneratorFunction::EXPONENTIAL),
            _ => Err(()),
        }
    }
}

/// Arbitrary waveform specification
#[derive(Debug)]
pub struct Wave {
    /// Amplitude of the waveform
    pub amplitude: f32,

    /// Vertical offset of the waveform
    pub offset: f32,

    /// Phase shift of the waveform
    ///
    /// This will only have an effect when combining waveforms
    pub phase: f32,

    /// Specifies wheter the waveform will repeat itself
    pub replicate: i32,

    /// Optional parameters specific to some generator functions
    pub params: [Option<f32>; 3],

    /// Specifies the waveform’s generator function:
    /// - [Sine](sine)
    /// - [Pulse](pulse)
    /// - [Gaussian](gaussian)
    /// - [Sinc](sinc)
    /// - [Exponential](exponential)
    pub func: GeneratorFunction,
}

impl Default for Wave {
    /// Default sine wave specification
    fn default() -> Self {
        Self {
            amplitude: 0.48,
            offset: 0.5,
            phase: 0.0,
            replicate: 1,
            params: [None; 3],
            func: GeneratorFunction::SINE,
        }
    }
}

impl Wave {
    #[allow(unused)]
    pub fn set_amplitude(&mut self, amplitude: f32) {
        self.amplitude = amplitude;
    }

    #[allow(unused)]
    pub fn set_offset(&mut self, offset: f32) {
        self.offset = offset;
    }

    #[allow(unused)]
    pub fn set_phase(&mut self, phase: f32) {
        self.phase = phase;
    }

    #[allow(unused)]
    pub fn set_replicate(&mut self, replicate: i32) {
        self.replicate = replicate;
    }

    #[allow(unused)]
    pub fn set_params(&mut self, params: [Option<f32>; 3]) {
        self.params = params;
    }

    #[allow(unused)]
    pub fn set_func(&mut self, func: GeneratorFunction) {
        self.func = func;
    }

    /// Evaluate the content of a wave
    #[cfg(feature = "pico")]
    fn eval(&self, x: f32) -> f32 {
        eval(
            self.amplitude,
            self.offset,
            self.phase,
            self.replicate,
            self.params,
            &self.func,
            x,
        )
    }
}

fn eval(
    amplitude: f32,
    offset: f32,
    phase: f32,
    replicate: i32,
    params: [Option<f32>; 3],
    func: &GeneratorFunction,
    mut x: f32,
) -> f32 {
    x = fsub(fmul(x, int_to_float(replicate)), phase);
    x = fsub(x, floorf(x)); // Reduce x to 0.0-1.0 range

    let mut v = match func {
        GeneratorFunction::SINE => sine(x),
        GeneratorFunction::PULSE => pulse(x, params.map(|p| unwrap_param(p))),
        GeneratorFunction::GAUSSIAN => gaussian(x, unwrap_param(params[0])),
        GeneratorFunction::SINC => sinc(x, unwrap_param(params[0])),
        GeneratorFunction::EXPONENTIAL => exponential(x, unwrap_param(params[0])),
    };
    v = fmul(v, amplitude);
    fadd(v, offset)
}

fn unwrap_param(p: Option<f32>) -> f32 {
    match p {
        Some(p) => p,
        None => {
            #[cfg(debug_assertions)]
            defmt::error!("Expected parameter missing, replaced with 0.0");
            0.0
        }
    }
}

/// Arbitrary waveform generator configuration
#[cfg(feature = "pico")]
pub struct Config<CH1, CH2, P, I>
where
    CH1: SingleChannel,
    CH2: SingleChannel,
    P: PIOExt,
    I: StateMachineIndex,
{
    clk_freq: HertzU32,
    dac_res: DACResolution,
    dma: (CH1, CH2),
    sm: StateMachine<(P, I), Running>,
    tx_buf: Tx<(P, I)>,
}

#[cfg(feature = "pico")]
impl<CH1, CH2, P, I> Config<CH1, CH2, P, I>
where
    CH1: SingleChannel,
    CH2: SingleChannel,
    P: PIOExt,
    I: StateMachineIndex,
{
    /// Create a new AWG configuration
    pub fn create(
        clk_freq: HertzU32,
        dac_res: DACResolution,
        dac_pins: DACPins,
        dma: (CH1, CH2),
        pio: &mut PIO<P>,
        sm: UninitStateMachine<(P, I)>,
    ) -> Option<Self> {
        match setup_pio(pio, dac_pins, sm) {
            Some((sm, tx_buf)) => Some(Self {
                clk_freq,
                dac_res,
                dma,
                sm,
                tx_buf,
            }),
            None => None,
        }
    }

    pub fn setup(&mut self, wave: Wave, buf: &mut SampleBuffer, freq: HertzU32) -> (usize, u32) {
        let clk_freq = uint_to_float(self.clk_freq.raw());
        let freq = uint_to_float(freq.raw());
        let buf_size = uint_to_float(buf.len() as u32);

        let clk_div: f32;
        let clk_div_int: u32;
        let duplicate: u32;
        let buf_len: usize;

        // TODO: Fix breaking output in debug build when debug message is removed
        #[cfg(debug_assertions)]
        defmt::debug!("raw freq: {}", freq);

        // Get required clock division for maximum buffer size
        clk_div = fdiv(clk_freq, fmul(freq, buf_size));

        match fcmp(clk_div, 1.0) {
            -1 => {
                // clk_div < 1.0
                // Cannot speed up clock, duplicate wave instead
                clk_div_int = 1;
                duplicate = float_to_uint(fdiv(1.0, clk_div));

                // Force sample count to multiple of 4
                buf_len = float_to_uint(fdiv(
                    fadd(fmul(buf_size, fmul(clk_div, uint_to_float(duplicate))), 0.5),
                    4.0,
                )) as usize
                    * 4;
            }
            _ => {
                // Apply required clock division, extend to next larger integer
                clk_div_int = float_to_uint(clk_div) + 1;
                duplicate = 1;

                // Force sample count to multiple of 4
                buf_len = float_to_uint(fadd(
                    fmul(buf_size, fdiv(clk_div, uint_to_float(clk_div_int as u32))),
                    0.5,
                )) as usize
                    * 4;
            }
        }

        // Fill the buffer
        for index in 0..buf_len {
            let val = fmul(
                uint_to_float(self.dac_res.get_size()),
                wave.eval(fdiv(
                    fmul(
                        4.0, // Added factor of 4 to fix buffer issue
                        fmul(
                            uint_to_float(duplicate),
                            fadd(uint_to_float(index as u32), 0.5),
                        ),
                    ),
                    uint_to_float(buf_len as u32),
                )),
            ) as u32;

            // Make sure value fits into DAC range
            buf.insert(index, max(0, min(self.dac_res.get_max_value(), val)));
        }

        // Set the clock divisor
        // Only the int value is set, fractional clock division results in jitter.
        self.sm.clock_divisor_fixed_point(clk_div_int as u16, 0);

        // Start DMA
        self.start_dma(buf, buf_len as u32 / 4);

        let freq_out = float_to_uint(fmul(
            fdiv(
                fdiv(clk_freq, uint_to_float(clk_div_int)),
                uint_to_float(buf_len as u32),
            ),
            uint_to_float(duplicate),
        ));

        (buf_len, freq_out)
    }

    /// Start 2-channel chained DMA.
    /// Channel 1 does the transfer, channel 2 reconfigures.
    fn start_dma(&self, buf: &mut SampleBuffer, word_count: u32) {
        // Disable the DMAs to prevent corruption while writing
        self.stop_dma();

        // Setup fist DMA channel
        let wave_buf_ptr = buf as *mut SampleBuffer; // Convert mutable ref to a raw pointer and memory address, successively
        let tx_buf_addr = self.tx_buf.fifo_address() as u32;

        self.dma
            .0
            .ch()
            .ch_read_addr
            .write(|w| unsafe { w.bits(wave_buf_ptr as u32) });
        self.dma
            .0
            .ch()
            .ch_write_addr
            .write(|w| unsafe { w.bits(tx_buf_addr) });
        self.dma
            .0
            .ch()
            .ch_trans_count
            .write(|w| unsafe { w.bits(word_count) });

        let irq_quiet: u32 = 1; // Do not generate an interrupt
        let treq_sel: u32 = 0; // Wait for PIO0_TX0
        let chain_to: u32 = 3; // Start channel 3 when done
        let ring_sel: u32 = 0;
        let ring_size: u32 = 0; // No wrapping
        let incr_write: u32 = 0; // For write to array
        let incr_read: u32 = 1; // For read from array
        let data_size: u32 = 2; // 32-bit word transfer
        let high_priority: u32 = 1;
        let en: u32 = 1;
        let ch1_ctrl = (irq_quiet << 21)
            | (treq_sel << 15)
            | (chain_to << 11)
            | (ring_sel << 10)
            | (ring_size << 9)
            | (incr_write << 5)
            | (incr_read << 4)
            | (data_size << 2)
            | (high_priority << 1)
            | (en << 0);

        // Write configuration to register block
        self.dma
            .0
            .ch()
            .ch_al1_ctrl
            .write(|w| unsafe { w.bits(ch1_ctrl) });

        // Setup second DMA channel
        let wave_buf_ptr_addr = ptr::addr_of!(wave_buf_ptr) as u32;
        let ch0_read_addr = self.dma.0.ch().ch_read_addr.as_ptr() as u32;

        self.dma
            .1
            .ch()
            .ch_read_addr
            .write(|w| unsafe { w.bits(wave_buf_ptr_addr) });
        self.dma
            .1
            .ch()
            .ch_write_addr
            .write(|w| unsafe { w.bits(ch0_read_addr) });
        self.dma
            .1
            .ch()
            .ch_trans_count
            .write(|w| unsafe { w.bits(1) });

        let treq_sel: u32 = 0x3f; // No pacing
        let chain_to: u32 = 2; // Start channel 2 when done
        let incr_write: u32 = 0; // Single write
        let incr_read: u32 = 0; // Single read
        let ch2_ctrl = (irq_quiet << 21)
            | (treq_sel << 15)
            | (chain_to << 11)
            | (ring_sel << 10)
            | (ring_size << 9)
            | (incr_write << 5)
            | (incr_read << 4)
            | (data_size << 2)
            | (high_priority << 1)
            | (en << 0);

        // Write configuration to register block
        self.dma
            .1
            .ch()
            .ch_ctrl_trig
            .write(|w| unsafe { w.bits(ch2_ctrl) });
    }

    pub fn stop_dma(&self) {
        self.dma.0.ch().ch_al1_ctrl.write(|w| unsafe { w.bits(0) });
        self.dma.1.ch().ch_al1_ctrl.write(|w| unsafe { w.bits(0) });
    }
}

/// Setup PIO peripheral for AWG
#[cfg(feature = "pico")]
fn setup_pio<P, I>(
    pio: &mut PIO<P>,
    mut pins: DACPins,
    sm: UninitStateMachine<(P, I)>,
) -> Option<(StateMachine<(P, I), Running>, Tx<(P, I)>)>
where
    P: PIOExt,
    I: StateMachineIndex,
{
    let program = pio_proc::pio_asm!("out pins, 8"); // TODO: Allow for other bitrates

    // // Change pin state from dynamic to PIO pins
    // // TODO: Make generic regarding used PIO
    // for pin in pins.iter_mut() {
    //     pin.try_into_function::<FunctionPio0>();
    // }

    // Setup PIO
    let installed = pio.install(&program.program);
    let (int, frac) = (0, 0); // As slow as possible (0 is interpreted as 65536)

    match installed {
        Ok(program) => {
            let (mut sm, _, tx) = PIOBuilder::from_program(program)
                .out_pins(pins.get(0).id().num, pins.len() as u8)
                .clock_divisor_fixed_point(int, frac)
                .out_shift_direction(Right)
                .autopull(true)
                .pull_threshold(32)
                .build(sm);

            match pins {
                DACPins::BIT8(p) => {
                    let pin_dirs: [(u8, PinDir); 8] = array::from_fn(|i| {
                        (
                            match p.get(i) {
                                Some(p) => p.id().num,
                                None => 0,
                            },
                            PinDir::Output,
                        )
                    });
                    let pin_states: [(u8, PinState); 8] = array::from_fn(|i| {
                        (
                            match p.get(i) {
                                Some(p) => p.id().num,
                                None => 0,
                            },
                            PinState::Low,
                        )
                    });

                    sm.set_pindirs(pin_dirs);
                    sm.set_pins(pin_states);
                }

                DACPins::BIT10(p) => {
                    let pin_dirs: [(u8, PinDir); 10] = array::from_fn(|i| {
                        (
                            match p.get(i) {
                                Some(p) => p.id().num,
                                None => 0,
                            },
                            PinDir::Output,
                        )
                    });
                    let pin_states: [(u8, PinState); 10] = array::from_fn(|i| {
                        (
                            match p.get(i) {
                                Some(p) => p.id().num,
                                None => 0,
                            },
                            PinState::Low,
                        )
                    });

                    sm.set_pindirs(pin_dirs);
                    sm.set_pins(pin_states);
                }
            }

            Some((sm.start(), tx))
        }
        Err(e) => {
            #[cfg(debug_assertions)]
            defmt::error!(
                "Error during PIO program installation: {}",
                defmt::Debug2Format(&e)
            );

            None
        }
    }
}
