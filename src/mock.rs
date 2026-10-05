//! An in-memory XCP transport: ECU calibration memory as a sparse map.
//!
//! [`MockTransport`] implements [`XcpTransport`](crate::XcpTransport) over
//! per-page address → byte maps. A full calibration cycle — read, adjust,
//! write, snapshot, diff — runs against it with no hardware, no timing, and
//! no flaky I/O, which is what makes this crate's session tests meaningful
//! rather than merely fast.
//!
//! It also records every access ([`requests`](MockTransport::requests)) and
//! can be told to fail ([`fail_reads`](MockTransport::fail_reads) /
//! [`fail_writes`](MockTransport::fail_writes)), which is how the
//! error-propagation paths are exercised.
//!
//! ```
//! use cal_model::mock::MockTransport;
//! use cal_model::{CalError, XcpTransport};
//!
//! let mut bench = MockTransport::seeded(&[(0x720100, vec![0x10, 0x00])]);
//! assert_eq!(bench.read(0x720100, 2)?, vec![0x10, 0x00]);
//! bench.write(0x720100, &[0x20, 0x00])?;
//! assert_eq!(bench.read(0x720100, 2)?, vec![0x20, 0x00]);
//! # Ok::<(), CalError>(())
//! ```

use crate::error::CalError;
use crate::session::XcpTransport;
use std::collections::BTreeMap;

/// One page of calibration memory: address → byte. Sparse: an address that
/// was never written reads as zero, exactly like freshly erased flash.
type Page = BTreeMap<u32, u8>;

/// One access a session made, in order.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Request {
    /// A read of `len` bytes at `addr`.
    Read {
        /// ECU address read.
        addr: u32,
        /// Byte count requested.
        len: u8,
    },
    /// A write of `len` bytes at `addr`.
    Write {
        /// ECU address written.
        addr: u32,
        /// Byte count written.
        len: u8,
    },
    /// A `SET_CAL_PAGE` switch to `page`.
    SetPage(u32),
}

/// An in-memory ECU: pages of calibration memory behind the
/// [`XcpTransport`] contract.
#[derive(Debug, Clone)]
pub struct MockTransport {
    pages: BTreeMap<u32, Page>,
    page: u32,
    requests: Vec<Request>,
    read_failure: Option<String>,
    write_failure: Option<String>,
}

impl MockTransport {
    /// An empty ECU on page 0.
    #[must_use]
    pub fn new() -> Self {
        Self {
            pages: BTreeMap::new(),
            page: 0,
            requests: Vec::new(),
            read_failure: None,
            write_failure: None,
        }
    }

    /// An ECU seeded with `(address, bytes)` pairs — a factory calibration.
    #[must_use]
    pub fn seeded(seeds: &[(u32, Vec<u8>)]) -> Self {
        let mut transport = Self::new();
        for (addr, bytes) in seeds {
            transport.seed(*addr, bytes);
        }
        transport
    }

    /// Make every read fail with `message` (a slave that is not answering).
    #[must_use]
    pub fn fail_reads(mut self, message: &str) -> Self {
        self.read_failure = Some(message.to_string());
        self
    }

    /// Make every write fail with `message` (a write-protected page).
    #[must_use]
    pub fn fail_writes(mut self, message: &str) -> Self {
        self.write_failure = Some(message.to_string());
        self
    }

    /// The active page.
    #[must_use]
    pub const fn page(&self) -> u32 {
        self.page
    }

    /// Every access so far, in order.
    #[must_use]
    pub fn requests(&self) -> &[Request] {
        &self.requests
    }

    /// Write `bytes` at `addr` on the active page, outside the transport
    /// contract (test setup).
    pub fn seed(&mut self, addr: u32, bytes: &[u8]) {
        let page = self.pages.entry(self.page).or_default();
        for (offset, &byte) in bytes.iter().enumerate() {
            let Some(address) = addr.checked_add(u32::try_from(offset).unwrap_or(u32::MAX)) else {
                continue;
            };
            page.insert(address, byte);
        }
    }

    /// The bytes at `addr..addr + len` on the active page, bypassing the
    /// transport contract (test assertions).
    #[must_use]
    pub fn peek(&self, addr: u32, len: u8) -> Vec<u8> {
        let empty = Page::new();
        let page = self.pages.get(&self.page).unwrap_or(&empty);
        let len = usize::from(len);
        let mut out = Vec::with_capacity(len);
        for offset in 0..len {
            let Some(address) = addr.checked_add(u32::try_from(offset).unwrap_or(u32::MAX)) else {
                out.push(0);
                continue;
            };
            out.push(page.get(&address).copied().unwrap_or(0));
        }
        out
    }

    /// The byte at `addr` on the active page.
    #[must_use]
    pub fn byte(&self, addr: u32) -> u8 {
        self.peek(addr, 1).first().copied().unwrap_or(0)
    }

    /// The byte at `addr` on page `page`, ignoring the active page.
    #[must_use]
    pub fn byte_on(&self, page: u32, addr: u32) -> u8 {
        self.pages
            .get(&page)
            .and_then(|p| p.get(&addr))
            .copied()
            .unwrap_or(0)
    }

    /// This mock, recovered from a `&dyn XcpTransport`.
    ///
    /// A [`CalibrationSession`](crate::CalibrationSession) owns its
    /// transport as a trait object; `as_any` is the sanctioned way back to
    /// the concrete type when a caller needs to inspect what the session did.
    #[must_use]
    pub fn from_transport(transport: &dyn XcpTransport) -> Option<&Self> {
        transport.as_any()?.downcast_ref::<Self>()
    }
}

impl Default for MockTransport {
    fn default() -> Self {
        Self::new()
    }
}

impl XcpTransport for MockTransport {
    fn read(&mut self, addr: u32, len: u8) -> Result<Vec<u8>, CalError> {
        self.requests.push(Request::Read { addr, len });
        if let Some(message) = &self.read_failure {
            return Err(CalError::Transport(message.clone()));
        }
        Ok(self.peek(addr, len))
    }

    fn write(&mut self, addr: u32, data: &[u8]) -> Result<(), CalError> {
        #[allow(clippy::cast_possible_truncation)]
        self.requests.push(Request::Write {
            addr,
            len: data.len() as u8,
        });
        if let Some(message) = &self.write_failure {
            return Err(CalError::Transport(message.clone()));
        }
        self.seed(addr, data);
        Ok(())
    }

    fn set_page(&mut self, page: u32) -> Result<(), CalError> {
        self.requests.push(Request::SetPage(page));
        self.pages.entry(page).or_default();
        self.page = page;
        Ok(())
    }

    fn as_any(&self) -> Option<&dyn std::any::Any> {
        Some(self)
    }
}
