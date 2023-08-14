//! Module for communication with remote control GUI for the
//! Arbitrary Waveform Generator

// Low-level traits
use core::{default::Default, option::Option};
use cortex_m::singleton;

// Pico traits
use rp_pico::hal::{
    dma::SingleChannel,
    pio::{PIOExt, StateMachineIndex},
};
use serde::{Deserialize, Serialize};

// Math and number related traits
use fugit::RateExtU32;

// Crate level traits
use crate::{Config, GeneratorFunction, SampleBuffer, Wave};

// Setup consts
const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Response to remote UI
///
/// Either of type [ConnStatus] or [DeviceStatus]
#[allow(unused)]
pub enum Response<'a> {
    ConnStatus(ConnStatus<'a>),
    DeviceStatus(DeviceStatus),
}

/// Connection status flag:
/// - [NotConnected](ConnStatusFlag::NotConnected): Not connected to remote UI
/// - [Connected](ConnStatusFlag::Connected): Connected to remote UI
#[derive(Serialize)]
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
#[derive(Serialize)]
#[allow(unused)]
pub enum DeviceStatusFlag {
    Init,
    CalcWave,
    Running,
    Stopped,
    ConnReset,
    Error,
}

/// Connection status
#[derive(Serialize)]
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

#[derive(Serialize)]
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

#[derive(Deserialize)]
#[serde(remote = "GeneratorFunction")]
enum GeneratorFunctionDef {
    SINE,
    PULSE,
    GAUSSIAN,
    SINC,
    EXPONENTIAL,
}

#[derive(Deserialize)]
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

#[derive(Deserialize)]
struct Request<'a> {
    command: &'a str,
    freq: u32,
    buf_size: u32,
    #[serde(with = "WaveDef")]
    wave: Wave,
}

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

            conf.setup(request.wave, wave_buf, request.freq.Hz());

            // let setup_status = setup_wave(wavbuf[0], self.wave);

            // setup_wave returned to main program

            // setup_wave returns awg_status, buf_len and freq_out
            // self.awg_status = AWGStatus {
            //     status: setup_status[0],
            //     buf_len: setup_status[1],
            //     freq_out: setup_status[2],
            // };

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

// except KeyboardInterrupt:

//     # set connection status and send to RemoteUI
//     Conn_stat["version"] = "0.0.0"
//     Conn_stat["connection"] = "closed"
//     send(Conn_stat)
//     connected = 0

// except Exception as e:
//     #print("0: mainloop crashed: ", e)
//     led.on()
//     Conn_stat["version"] = "mainloop crashed"
//     Conn_stat["connection"] = e
//     send(Conn_stat)
//     connected = 0

// finally:
//     print("0: finally: cleaning up")
//     stopDMA()
//     #soft_reset()
