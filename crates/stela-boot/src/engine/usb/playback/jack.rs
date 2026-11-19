//! Audio on an already admitted host HDA controller and sensed 3.5 mm pin.
use super::{Board, FRAMES_PER_MS, HdaHost, PREFILL, Pcm, RATE, RestoreRoot, host, output};
use alloc::{vec, vec::Vec};
use stela_drivers::audio::{Error, PcmFormat};
use stela_drivers::probe::native::hda::RetirementError;

const PERIOD_FRAMES: usize = 8 * FRAMES_PER_MS;
const PERIODS: usize = 4;
const PERIOD_BYTES: usize = PERIOD_FRAMES * 4;
const RING_FRAMES: usize = PERIOD_FRAMES * PERIODS;
const FORMAT: PcmFormat = PcmFormat {
    rate: RATE,
    bits: 16,
    channels: 2,
};

pub(super) struct JackPlayback {
    device: HdaHost,
    samples: Vec<[i32; 2]>,
    pcm: Vec<u8>,
    pending: usize,
    running: bool,
    next_sense: u64,
}

impl JackPlayback {
    pub(super) fn select(mut hosts: Vec<HdaHost>) -> Option<Self> {
        let selected = hosts.iter_mut().position(|device| {
            match device.prepare_jack(FORMAT, PERIOD_BYTES, PERIODS) {
                Ok(present) => present,
                Err(error) => {
                    output::arguments(format_args!(
                        "AUDIO 3.5 mm {} route unavailable: {error:?}\n",
                        device.location
                    ));
                    false
                }
            }
        });
        let device = selected.map(|index| hosts.remove(index));
        Vec::leak(hosts);
        device.map(|device| Self {
            device,
            samples: vec![[0; 2]; RING_FRAMES],
            pcm: vec![0; PERIOD_BYTES * PERIODS],
            pending: 0,
            running: false,
            next_sense: 0,
        })
    }

    pub(super) fn running(&self) -> bool {
        self.running
    }

    pub(super) fn step(&mut self, root: u64, board: &Board, now: u64) -> bool {
        let Some(Some(streaming)) = board.try_with(|board| {
            let speaker = board.usb_audio()?;
            if !speaker.streaming() {
                self.pending = 0;
                if speaker.queued() != 0 {
                    speaker.take(&mut self.samples);
                }
                return Some(false);
            }
            if self.pending == 0 && (self.running || speaker.queued() >= PREFILL) {
                let frames = if self.running {
                    PERIOD_FRAMES
                } else {
                    RING_FRAMES
                };
                let received = speaker.take(&mut self.samples[..frames]);
                self.pending = frames * 4;
                Pcm::I16.render(
                    &self.samples[..received],
                    &mut self.pcm[..self.pending],
                    false,
                );
            }
            Some(true)
        }) else {
            return true;
        };
        // The board lock is released. This is the captured host-driver root.
        let Some(_root) = (unsafe { RestoreRoot::try_enter(root) }) else {
            return true;
        };
        match self.service(streaming, now) {
            Ok(true) => true,
            Ok(false) => {
                self.stop();
                output::arguments(format_args!(
                    "AUDIO 3.5 mm unplugged; trying the next output\n"
                ));
                false
            }
            Err(RetirementError::Controller(Error::PlaybackUnderrun)) => {
                // The controller stopped the descriptor. Refill a fresh ring.
                self.running = false;
                self.pending = 0;
                true
            }
            Err(error) => {
                self.stop();
                output::arguments(format_args!("AUDIO 3.5 mm unavailable: {error:?}\n"));
                false
            }
        }
    }

    fn service(&mut self, streaming: bool, now: u64) -> Result<bool, RetirementError> {
        if now >= self.next_sense {
            self.next_sense = now + host::COUNTER_HZ / 10;
            if !self.device.jack_present()? {
                return Ok(false);
            }
        }
        if !streaming && self.running {
            self.device.stop_jack()?;
            self.running = false;
        } else if self.pending != 0 {
            let pcm = &self.pcm[..self.pending];
            if !self.running {
                self.device.start_jack(pcm)?;
                self.running = true;
                self.pending = 0;
                output::arguments(format_args!(
                    "AUDIO 3.5 mm playback started rate={RATE} bits=16 channels=2\n"
                ));
            } else if self.device.refill_jack(pcm)? {
                self.pending = 0;
            }
        }
        Ok(true)
    }

    pub(super) fn stop(&mut self) {
        if self.running {
            if let Err(error) = self.device.stop_jack() {
                output::arguments(format_args!("AUDIO 3.5 mm stop: {error:?}\n"));
            }
            self.running = false;
        }
    }
}
