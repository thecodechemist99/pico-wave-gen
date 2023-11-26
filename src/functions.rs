//! Helper functions for the arbitrary waveform generator

// Low-level traits
use core::f32::consts::PI;

// Pico traits
use rp2040_hal::rom_data::float_funcs::{
    fadd, fdiv, fexp, float_to_int, float_to_uint, fmul, fsin, fsub, int_to_float,
};

/// Memory efficient flooring implementation
pub(crate) fn floorf(x: f32) -> f32 {
    let x = float_to_int(x);
    int_to_float(match x {
        0.. => x,
        _ => x - 1,
    })
}

/// Calculate the nth power of a floating point number
pub(crate) fn nth_power(x: f32, n: u32) -> f32 {
    match n {
        0 => 1.0,
        _ => fmul(x, nth_power(x, n - 1)),
    }
}

/// Sine waveform generator function
pub(crate) fn sine(x: f32) -> f32 {
    fsin(fmul(x, fmul(2.0, PI)))
}

/// Pulse waveform generator function
///
/// Parameters:
/// - Rise time
/// - High time
/// - Fall time
pub(crate) fn pulse(x: f32, p: [f32; 3]) -> f32 {
    if x < p[0] {
        fdiv(x, p[0])
    } else if x < fadd(p[0], p[1]) {
        1.0
    } else if x < fadd(p[0], fadd(p[1], p[2])) {
        fsub(1.0, fdiv(fsub(x, fsub(p[0], p[1])), p[2]))
    } else {
        1.0
    }
}

/// Gaussian waveform generator function
///
/// Parameters:
/// - Time
pub(crate) fn gaussian(x: f32, p: f32) -> f32 {
    fexp(nth_power(-(fdiv(fsub(x, 0.5), p)), 2))
}

/// Sinc waveform generator function
///
/// Parameters:
/// - Time
pub(crate) fn sinc(x: f32, p: f32) -> f32 {
    match float_to_uint(x) * 2 {
        1 => 1.0,
        _ => fdiv(fsin(fdiv(fsub(x, 0.5), p)), fdiv(fsub(x, 0.5), p)),
    }
}

/// Exponential waveform generator function
///
/// Parameters:
/// - Time
pub(crate) fn exponential(x: f32, p: f32) -> f32 {
    fexp(fdiv(-x, p))
}
