//! Conventional HDA through one original-root/config/BAR/DMA lifetime.
use super::{Failure, NativeServices, ServiceError, map};
use crate::{
    Clock,
    audio::{
        self, Direction, PcmFormat,
        hda::{self, CodecFailure, CodecGraph, CodecSetup, PinDevice, PinOptions, StreamId},
    },
    bus::{
        mmio::{Mmio, owned::Mapping},
        pci::Address,
    },
    probe::Error,
};
use alloc::{boxed::Box, string::String, vec::Vec};
mod lifetime;
pub use lifetime::Error as RetirementError;
use lifetime::{Inputs, Native};

/// HD Audio BAR0 through its actual owned uncached mapping/native widths.
pub struct Bar0 {
    mmio: Mmio,
    bytes: usize,
    mapping: Mapping,
}
impl hda::Registers for Bar0 {
    fn region_bytes(&self) -> usize {
        self.bytes
    }
    fn read8(&mut self, o: usize) -> u8 {
        self.mmio.read8(o)
    }
    fn read16(&mut self, o: usize) -> u16 {
        self.mmio.read16(o)
    }
    fn read32(&mut self, o: usize) -> u32 {
        self.mmio.read32(o)
    }
    fn write8(&mut self, o: usize, v: u8) {
        self.mmio.write8(o, v);
    }
    fn write16(&mut self, o: usize, v: u16) {
        self.mmio.write16(o, v);
    }
    fn write32(&mut self, o: usize, v: u32) {
        self.mmio.write32(o, v);
    }
}
pub struct Codec {
    pub graph: CodecGraph,
    pub setup: Result<CodecSetup, audio::Error>,
}
/// The real register provider/core/DMA allocator never escape independently.
/// Discovery is metadata; a retained success alone is not guest audio readiness.
pub struct Host {
    pub location: String,
    owner: Native<Bar0, audio::Timing<Clock>>,
    jack: Option<Jack>,
    pub codecs: Vec<Codec>,
    pub codec_failures: Vec<CodecFailure>,
    pub discovery_error: Option<audio::Error>,
}

struct Jack {
    codec: usize,
    pin: u8,
    stream: StreamId,
}

impl Host {
    pub fn prepare_jack(
        &mut self,
        format: PcmFormat,
        period_bytes: usize,
        periods: usize,
    ) -> Result<bool, RetirementError> {
        if self.jack.is_some() {
            return self.jack_present();
        }
        for (codec, entry) in self
            .codecs
            .iter()
            .enumerate()
            .filter(|(_, entry)| entry.setup.is_ok())
        {
            let route = self.owner.playback(|controller, dma| {
                let Some(jack) = controller.connected_output_jack(&entry.graph, format)? else {
                    return Ok(None);
                };
                let stream = controller.allocate_stream(
                    Direction::Playback,
                    format,
                    period_bytes,
                    periods,
                    dma,
                )?;
                controller.configure_analog_route(
                    &entry.graph,
                    &jack.route,
                    stream,
                    0,
                    PinOptions::Playback {
                        headphone: jack.configuration.device() == PinDevice::Headphone,
                        eapd: None,
                    },
                )?;
                Ok(Some(Jack {
                    codec,
                    pin: jack.route.pin(),
                    stream,
                }))
            })?;
            if let Some(route) = route {
                self.jack = Some(route);
                return Ok(true);
            }
        }
        Ok(false)
    }

    pub fn jack_present(&mut self) -> Result<bool, RetirementError> {
        let Some(jack) = &self.jack else {
            return Ok(false);
        };
        self.owner.playback(|controller, _| {
            let sense = controller.poll_pin_sense(&self.codecs[jack.codec].graph, jack.pin)?;
            Ok(sense.present && sense.raw != u32::MAX)
        })
    }

    pub fn start_jack(&mut self, pcm: &[u8]) -> Result<(), RetirementError> {
        let stream = self
            .jack
            .as_ref()
            .ok_or(audio::Error::InvalidStream)?
            .stream;
        self.owner
            .playback(|controller, _| controller.start_analog_playback(stream, pcm))
    }

    pub fn refill_jack(&mut self, pcm: &[u8]) -> Result<bool, RetirementError> {
        let stream = self
            .jack
            .as_ref()
            .ok_or(audio::Error::InvalidStream)?
            .stream;
        self.owner
            .playback(|controller, _| Ok(controller.refill_analog_period(stream, pcm)?.is_some()))
    }

    pub fn stop_jack(&mut self) -> Result<(), RetirementError> {
        let stream = self
            .jack
            .as_ref()
            .ok_or(audio::Error::InvalidStream)?
            .stream;
        self.owner.playback(|controller, _| controller.stop(stream))
    }

    /// Explicit one-shot hardware/root/lease/Attachment/mapping retirement.
    /// Failure and unwind keep the complete graph and permanently close entry.
    pub fn shutdown(&mut self) -> Result<(), RetirementError> {
        self.owner.close()
    }
}
struct ColdOwner {
    requester: Option<crate::bus::pci::requester::preparation::PreparingRequester>,
    mapping: Option<Mapping>,
    minimum: Option<alloc::rc::Rc<dyn crate::bus::pci::power::Timer>>,
}
impl Drop for ColdOwner {
    fn drop(&mut self) {
        core::mem::forget(self.requester.take());
        core::mem::forget(self.mapping.take());
        core::mem::forget(self.minimum.take());
    }
}
/// # Safety
/// NativeServices ownership, calibrated clock and original D0/decode/power
/// hold. Conventional identity alone cannot exclude firmware/AML/SMM/DSP users.
/// Their original root/backing and competing-owner exclusions remain required.
/// The separate original IRQ/work retirement producer is currently unavailable:
/// this factory performs source containment, then retains/refuses before bind.
#[expect(
    clippy::result_large_err,
    reason = "A failed HDA attach keeps its typed cause and all original resource owners"
)]
pub(in crate::probe) unsafe fn attach(
    services: &dyn NativeServices,
    address: Address,
    clock: Clock,
    minimum: alloc::rc::Rc<dyn crate::bus::pci::power::Timer>,
) -> Result<Host, Failure> {
    minimum.check().map_err(|error| {
        (
            Error::HdaNative(RetirementError::Controller(audio::Error::MinimumDelay(
                error,
            ))),
            None,
        )
    })?;
    // Check the largest selected conventional-core wait before any config or
    // vendor effect. Generic clock metadata alone does not qualify a timer.
    clock
        .check_duration(1_000_000)
        .map_err(|error| (Error::HdaNative(RetirementError::Clock(error)), None))?;
    let requester = services
        .prepare_requester(address)
        .map_err(|e| (Error::Native(e), None))?;
    let mut cold = ColdOwner {
        requester: Some(requester),
        mapping: None,
        minimum: Some(minimum),
    };
    let bars = match cold.requester.as_mut().unwrap().function_mut().bars() {
        Ok(bars) => bars,
        Err(error) => return Err((error.into(), Some(Box::new(cold)))),
    };
    let mapping = match map(services, address, &bars, 0) {
        Ok(m) => m,
        Err(e) => return Err((Error::Native(e), Some(Box::new(cold)))),
    };
    cold.mapping = Some(mapping);
    let bar = bars[0].expect("map checked BAR0");
    let window = (|| {
        let mapping = cold.mapping.as_ref().unwrap();
        let bytes = mapping.bytes()?;
        Ok((mapping.address(0, bytes)?, bytes))
    })();
    let (window, bytes) = match window {
        Ok(v) => v,
        Err(e) => {
            return Err((
                Error::Native(ServiceError::Mapping(e)),
                Some(Box::new(cold)),
            ));
        }
    };
    let registers = Bar0 {
        // SAFETY: exactly this original owned UC BAR0; Native outer owner
        // keeps actual mapping across constructor error and callback unwind.
        mmio: unsafe { Mmio::new(window.as_ptr() as u64, bytes) },
        bytes,
        mapping: cold.mapping.take().unwrap(),
    };
    let inputs = Inputs {
        requester: cold.requester.take().unwrap(),
        registers,
        clock: audio::Timing::new(clock, cold.minimum.take().unwrap()),
        pages: services.pages(),
        bar,
        // No native original-owner IRQ/CPU/EOI/service/work completion producer
        // exists. Post-EBS serialization, source masks or polled operation do
        // not construct one; Native retains/refuses before root drain/bind.
        original_users: None,
    };
    let mut owner = match Native::new(inputs) {
        Ok(owner) => owner,
        Err(failure) => {
            let error = failure
                .construction_error()
                .cloned()
                .map_or(Error::HdaNative(failure.error), Error::Device);
            return Err((error, Some(Box::new(failure))));
        }
    };
    let discovery = match owner.discover_and_setup() {
        Ok(d) => d,
        Err(error) => return Err((Error::HdaNative(error), Some(Box::new(owner)))),
    };
    Ok(Host {
        location: alloc::format!("{address}"),
        owner,
        jack: None,
        codecs: discovery.codecs,
        codec_failures: discovery.codec_failures,
        discovery_error: discovery.discovery_error,
    })
}
