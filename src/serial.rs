//! Module for communication with remote control GUI for the
//! arbitrary waveform generator

// Low-level traits
#[cfg(feature = "pico")]
use cortex_m::singleton;

// Pico traits
#[cfg(feature = "pico")]
use rp2040_hal::{
    dma::SingleChannel,
    pio::{PIOExt, StateMachineIndex},
};
use serde::{Deserialize, Serialize};

// Math and number related traits
#[cfg(feature = "pico")]
use fugit::RateExtU32;

// Crate level traits
#[cfg(feature = "pico")]
use crate::{Config, SampleBuffer};
use crate::{GeneratorFunction, Wave};

// Setup consts
const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Response to remote UI
///
/// Either of type [ConnStatus] or [DeviceStatus]
#[derive(Debug)]
#[allow(unused)]
pub enum Response<'a> {
    ConnStatus(ConnStatus<'a>),
    DeviceStatus(DeviceStatus),
}

/// Connection status flag:
/// - [NotConnected](ConnStatusFlag::NotConnected): Not connected to remote UI
/// - [Connected](ConnStatusFlag::Connected): Connected to remote UI
#[derive(Serialize, Debug, Deserialize)]
#[allow(unused)]
pub enum ConnStatusFlag {
    NotConnected,
    Connected,
}

/// Device status flag:
/// - [Init](DeviceStatusFlag::Init): Intitialisation, generator not yet started using start button
/// - [CalcWave](DeviceStatusFlag::CalcWave): Generator setup is triggered, wave form is calculated, no further trigger is allowed
/// - [Running](DeviceStatusFlag::Running): Calculation finished, generator output active, new setup trigger is allowed
/// - [Stopped](DeviceStatusFlag::Stopped): Generator output stopped, new set up trigger is allowed, initialisation status
/// - [ConnReset](DeviceStatusFlag::ConnReset): Connection reset by remote UI
/// - [Error](DeviceStatusFlag::Error): An error occured
#[derive(Serialize, Debug, Deserialize)]
pub enum DeviceStatusFlag {
    Init,
    CalcWave,
    Running,
    Stopped,
    ConnReset,
    Error,
}

/// Connection status
#[derive(Serialize, Debug, Deserialize)]
pub struct ConnStatus<'a> {
    status: ConnStatusFlag,
    version: &'a str,
    cpu_freq: u32,
}

impl<'a> Default for ConnStatus<'a> {
    fn default() -> Self {
        Self {
            status: ConnStatusFlag::NotConnected,
            version: VERSION,
            cpu_freq: 0,
        }
    }
}

#[derive(Serialize, Debug, Deserialize)]
pub struct DeviceStatus {
    status: DeviceStatusFlag,
    buf_len: u32,
    freq_out: u32,
}

impl<'a> Default for DeviceStatus {
    fn default() -> Self {
        Self {
            status: DeviceStatusFlag::Init,
            buf_len: 0,
            freq_out: 0,
        }
    }
}

#[derive(Serialize, Deserialize)]
#[serde(remote = "GeneratorFunction")]
enum GeneratorFunctionDef {
    SINE,
    PULSE,
    GAUSSIAN,
    SINC,
    EXPONENTIAL,
}

#[derive(Serialize, Deserialize)]
#[serde(remote = "Wave")]
struct WaveDef {
    amplitude: f32,
    offset: f32,
    phase: f32,
    replicate: i32,
    params: [Option<f32>; 3],
    #[serde(with = "GeneratorFunctionDef")]
    func: GeneratorFunction,
}

#[derive(Serialize, Debug, Deserialize)]
pub struct Request<'a> {
    pub command: &'a str,
    pub freq: u32,
    pub buf_size: u32,
    #[serde(with = "WaveDef")]
    pub wave: Wave,
}

#[cfg(feature = "pico")]
pub fn receive<'a, CH1, CH2, P, I>(conf: &mut Config<CH1, CH2, P, I>, buf: &[u8]) -> Response<'a>
where
    CH1: SingleChannel,
    CH2: SingleChannel,
    P: PIOExt,
    I: StateMachineIndex,
{
    #[cfg(debug_assertions)]
    defmt::info!("receiving");

    let (request, _): (Request, usize) = serde_json_core::from_slice(&buf).unwrap();
    match request.command {
        "setup" => {
            // Setup waveform from received values
            let mut device_status = DeviceStatus::default();
            device_status.status = DeviceStatusFlag::CalcWave;

            let wave_buf = singleton!(: SampleBuffer =
                match request.buf_size {
                    256 => SampleBuffer::B256([0; 256]),
                    1024 => SampleBuffer::B1K([0; 1024]),
                    2048 => SampleBuffer::B2K([0; 2048]),
                    4096 => SampleBuffer::B4K([0; 4096]),
                    8192 => SampleBuffer::B8K([0; 8192]),
                    16_384 => SampleBuffer::B16K([0; 16_384]),
                    32_768 => SampleBuffer::B32K([0; 32_768]),
                    65_536 => SampleBuffer::B64k([0; 65_536]),
                    _ => SampleBuffer::B512([0; 512]),
            })
            .unwrap();

            let setup_status = conf.setup(request.wave, wave_buf, request.freq.Hz());

            device_status.status = DeviceStatusFlag::Running;
            device_status.buf_len = setup_status.0 as u32;
            device_status.freq_out = setup_status.1;

            // Send status to remote UI
            Response::DeviceStatus(device_status)
        }
        "stop" => {
            // Stop the generator output and send status to remote UI
            conf.stop_dma();

            Response::DeviceStatus(DeviceStatus {
                status: DeviceStatusFlag::Stopped,
                buf_len: 0,
                freq_out: 0,
            })
        }
        "disconnect" => {
            // Stop the generator output, send status to remote UI and restart the AWG
            conf.stop_dma();
            Response::DeviceStatus(DeviceStatus {
                status: DeviceStatusFlag::ConnReset,
                buf_len: 0,
                freq_out: 0,
            })
            // reset();
        }
        _ => {
            // Send status to remote UI
            Response::DeviceStatus(DeviceStatus {
                status: DeviceStatusFlag::Error,
                buf_len: 0,
                freq_out: 0,
            })
        }
    }
}
