//! An in-memory XCP transport: ECU calibration memory as a plain map.
//!
//! [`MockTransport`] implements [`XcpTransport`] over a sparse per-page
//! address → byte map. It is the substrate for the crate's own tests and
//! for [`examples/calibrate.rs`](https://github.com/WyattAu/cal-model):
//! a calibration cycle — read, adjust, write, snapshot, diff — runs against
//! it with no hardware, no timing, and no flaky I/O.
//!
//! It also records the XCP frame sequence of every access
//! ([`MockTransport::requests`]) and can be told to fail
//! ([`MockTransport::fail_reads`] / [`MockTransport::fail_writes`]), which is
//! how the error-propagation paths are exercised.

use std::collections::BTreeMap;

use xcp_core::ByteOrder;

use crate::error::CalError;
use crate::session::XcpTransport;

/// Per-page calibration memory: address → byte, sparse (an unset address
/// reads as zero, exactly like a freshly erased flash).
type Page = BTreeMap<u32, u8>;

/// An in-memory ECU: pages of calibration memory behind the
/// [`XcpTransport`] contract.
#[derive(Debug, Clone)]
pub struct MockTransport {
    pages: BTreeMap<u32, Page>,
    page: u32,
    mode: crate::session::ResourceMode,
    byte_order: ByteOrder,
    fail_reads: bool,
    fail_writes: bool,
    requests: Vec<Vec<u8>>,
    page_switches: Vec<u32>,
}

impl MockTransport {
    /// A transport with `page_count` zeroed pages, page 0 active, and the
    /// CAL/PAG + DAQ resources a calibration tool expects.
    #[must_use]
    pub fn with_pages(page_count: u32) -> Self {
        Self {
            pages: (0..page_count.max(1)).map(|page| (page, Page::new())).collect(),
            page: 0,
            mode: crate::session::ResourceMode::CAL_PAGE_OR_DAQ,
            byte_order: ByteOrder::Intel,
            fail_reads: false,
            fail_writes: false,
            requests: Vec::new(),
            page_switches: Vec::new(),
        }
    }

    /// Advertise the slave's byte order.
    pub fn set_byte_order(&mut self, order: ByteOrder) {
        self.byte_order = order;
    }

    /// Make every subsequent read fail with a transport error.
    pub fn fail_reads(&mut self, fail: bool) {
        self.fail_reads = fail;
    }

    /// Make every subsequent write fail with a transport error.
    pub fn fail_writes(&mut self, fail: bool) {
        self.fail_writes = fail;
    }

    /// Write bytes into `page`'s memory without going through the session.
    ///
    /// A range that would run past the top of the address space is
    /// truncated — seeding fixtures is a test concern, and silently dropping
    /// the tail beats panicking inside a library the tests only drive.
    pub fn seed(&mut self, page: u32, address: u32, data: &[u8]) {
        let memory = self.pages.entry(page).or_default();
        for (offset, byte) in data.iter().enumerate() {
            let index = u32::try_from(offset).unwrap_or(u32::MAX);
            let Some(addr) = address.checked_add(index) else {
                break;
            };
            memory.insert(addr, *byte);
        }
    }

    /// Read `len` bytes of `page`'s memory directly, bypassing the session.
    /// Addresses past the top of the address space read as zero.
    #[must_use]
    pub fn peek(&self, page: u32, address: u32, len: usize) -> Vec<u8> {
        let empty = Page::new();
        let memory = self.pages.get(&page).unwrap_or(&empty);
        let mut bytes = Vec::with_capacity(len);
        for offset in 0..len {
            let index = u32::try_from(offset).unwrap_or(u32::MAX);
            let addr = address.checked_add(index);
            let byte = addr.and_then(|addr| memory.get(&addr)).copied().unwrap_or(0);
            bytes.push(byte);
        }
        bytes
    }

    /// The active page.
    #[must_use]
    pub const fn active_page(&self) -> u32 {
        self.page
    }

    /// The number of pages the transport holds.
    #[must_use]
    pub fn page_count(&self) -> usize {
        self.pages.len()
    }

    /// The frame sequence of every access, in order.
    #[must_use]
    pub fn requests(&self) -> &[Vec<u8>] {
        &self.requests
    }

    /// The pages that were switched to, in order (page 0 excluded).
    #[must_use]
    pub fn page_switches(&self) -> &[u32] {
        &self.page_switches
    }

    /// Forget the recorded requests.
    pub fn clear_requests(&mut self) {
        self.requests.clear();
    }

    /// Record an access's XCP frames. Called by the mock's own read/write
    /// so a caller can hand-verify the wire form the session produced.
    fn record(&mut self, frames: Vec<u8>) {
        self.requests.push(frames);
    }

}

impl XcpTransport for MockTransport {
    fn read(&mut self, addr: u32, len: u8) -> Result<Vec<u8>, CalError> {
        if self.fail_reads {
            return Err(CalError::Transport(format!(
                "mock read failure at 0x{addr:08X} ({len} bytes)"
            )));
        }
        let page = self.page;
        let bytes = self.peek(page, addr, usize::from(len));
        self.record(xcp_core::upload_with(self.byte_order, addr, 0x00, len));
        Ok(bytes)
    }

    fn write(&mut self, addr: u32, data: &[u8]) -> Result<(), CalError> {
        if self.fail_writes {
            return Err(CalError::Transport(format!(
                "mock write failure at 0x{addr:08X} ({} bytes)",
                data.len()
            )));
        }
        self.record(xcp_core::download_with(
            self.byte_order,
            addr,
            0x00,
            data,
        ));
        #[allow(clippy::cast_precision_loss)]
        addr.checked_add(u32::try_from(data.len()).unwrap_or(u32::MAX))
            .ok_or_else(|| CalError::OutOfBounds {
                name: format!("0x{addr:08X}"),
                value: data.len() as f64,
                lower: 0.0,
                upper: 4_294_967_295.0,
            })?;
        self.seed(self.page, addr, data);
        Ok(())
    }

    fn resource_mode(&self) -> crate::session::ResourceMode {
        self.mode
    }

    fn byte_order(&self) -> ByteOrder {
        self.byte_order
    }

    fn set_cal_page(&mut self, page: u32) -> Result<(), CalError> {
        if !self.pages.contains_key(&page) {
            return Err(CalError::Transport(format!(
                "mock transport has no calibration page {page}"
            )));
        }
        if page != 0 {
            self.page_switches.push(page);
        }
        self.page = page;
        Ok(())
    }
}

/// A [`XcpTransport`] that fails every access — the error-propagation
/// fixture.
#[derive(Debug, Clone, Default)]
pub struct FailingTransport {
    /// The message every access fails with.
    pub message: String,
}

impl FailingTransport {
    /// A transport that fails with `message`.
    #[must_use]
    pub fn new(message: impl Into<String>) -> Self {
        Self {
            message: message.into(),
        }
    }
}

impl XcpTransport for FailingTransport {
    fn read(&mut self, _addr: u32, _len: u8) -> Result<Vec<u8>, CalError> {
        Err(CalError::Transport(self.message.clone()))
    }

    fn write(&mut self, _addr: u32, _data: &[u8]) -> Result<(), CalError> {
        Err(CalError::Transport(self.message.clone()))
    }
}