//! An in-memory ECU, for tests, examples, and dry runs.
//!
//! A calibration tool is only as trustworthy as the session layer under it,
//! and a session layer is only trustworthy if it can be driven without a
//! bench. [`MockTransport`] is that bench: a paged, byte-addressable memory
//! with an optional failure schedule.
//!
//! It models the one thing an ECU does that a plain `HashMap` does not —
//! **pages**. `SET_CAL_PAGE` selects which memory an address resolves into,
//! so a write to page 1 must leave page 0 byte-for-byte untouched. That is
//! the property test `page_switch_isolates_memory` pins down, because it is
//! exactly the bug a naive implementation has.

use std::collections::BTreeMap;

use crate::error::CalError;

/// One ECU memory page: a base address and the bytes mapped from it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MemoryPage {
    /// The lowest address this page answers at.
    pub base: u32,
    /// The page's contents, `0` at index 0 addressing `base`.
    pub bytes: Vec<u8>,
}

impl MemoryPage {
    /// A page of `len` zero bytes at `base`.
    #[must_use]
    pub fn new(base: u32, len: usize) -> Self {
        Self {
            base,
            bytes: vec![0; len],
        }
    }

    /// A page holding `bytes` at `base`.
    #[must_use]
    pub fn with_bytes(base: u32, bytes: Vec<u8>) -> Self {
        Self { base, bytes }
    }
}

/// An in-memory XCP target.
///
/// # Reading and writing
///
/// Accesses resolve against the **current page**: a read outside the page's
/// mapped range is [`CalError::Transport`], because silently returning
/// zeroes would make a mis-mapped A2L look like a working ECU.
///
/// # Failure injection
///
/// [`MockTransport::fail_next_read`] and [`MockTransport::fail_next_write`]
/// arm exactly one failure, so a test can assert that an error propagates
/// out of a session rather than being swallowed.
#[derive(Debug, Clone, Default)]
pub struct MockTransport {
    pages: BTreeMap<u32, MemoryPage>,
    current_page: u32,
    fail_reads: usize,
    fail_writes: usize,
    /// Every write performed, in order: `(address, bytes)`.
    pub write_log: Vec<(u32, Vec<u8>)>,
    /// Every read performed, in order: `(address, length)`.
    pub read_log: Vec<(u32, u8)>,
}

impl MockTransport {
    /// An empty transport with a single zero-filled page 0 of `len` bytes
    /// at `base`.
    #[must_use]
    pub fn new() -> Self {
        let mut transport = Self::default();
        transport.insert_page(0, MemoryPage::new(0, 0));
        transport
    }

    /// An empty transport with no pages mapped at all — every access fails.
    #[must_use]
    pub fn empty() -> Self {
        Self::default()
    }

    /// Add or replace a page.
    pub fn insert_page(&mut self, page: u32, memory: MemoryPage) {
        self.pages.insert(page, memory);
    }

    /// Select the active page. Mirrors `SET_CAL_PAGE`; no protocol traffic is
    /// implied, because the transport contract is memory-level.
    pub fn select_page(&mut self, page: u32) {
        self.current_page = page;
    }

    /// The active page number.
    #[must_use]
    pub fn current_page(&self) -> u32 {
        self.current_page
    }

    /// Arm `count` consecutive read failures.
    pub fn fail_next_read(&mut self, count: usize) {
        self.fail_reads = count;
    }

    /// Arm `count` consecutive write failures.
    pub fn fail_next_write(&mut self, count: usize) {
        self.fail_writes = count;
    }

    /// The bytes of `page`, or `None` when that page is not mapped.
    #[must_use]
    pub fn page(&self, page: u32) -> Option<&MemoryPage> {
        self.pages.get(&page)
    }

    /// The bytes of the active page.
    #[must_use]
    pub fn bytes(&self) -> Option<&[u8]> {
        self.pages
            .get(&self.current_page)
            .map(|page| page.bytes.as_slice())
    }

    /// Mutable bytes of the active page.
    pub fn bytes_mut(&mut self) -> Option<&mut [u8]> {
        self.pages
            .get_mut(&self.current_page)
            .map(|page| page.bytes.as_mut_slice())
    }

    /// The active page's byte at `address`, for an out-of-band assertion.
    #[must_use]
    pub fn byte_at(&self, address: u32) -> Option<u8> {
        let page = self.pages.get(&self.current_page)?;
        let offset = address.checked_sub(page.base)? as usize;
        page.bytes.get(offset).copied()
    }

    /// Read `len` bytes from `addr` on the active page.
    ///
    /// # Errors
    ///
    /// [`CalError::Transport`] when a failure is armed, when the page is not
    /// mapped, or when the range is not fully mapped.
    pub fn read(&mut self, addr: u32, len: u8) -> Result<Vec<u8>, CalError> {
        self.read_log.push((addr, len));
        if self.fail_reads > 0 {
            self.fail_reads -= 1;
            return Err(CalError::Transport(format!(
                "mock: read of {len} byte(s) at 0x{addr:08X} failed"
            )));
        }
        let page = self.pages.get(&self.current_page).ok_or_else(|| {
            CalError::Transport(format!(
                "mock: page {} is not mapped",
                self.current_page
            ))
        })?;
        let offset = addr.checked_sub(page.base).ok_or_else(|| {
            CalError::Transport(format!(
                "mock: address 0x{addr:08X} is below the page base 0x{:08X}",
                page.base
            ))
        })?;
        let start = usize::try_from(offset)
            .map_err(|_| CalError::Transport("mock: address offset overflowed".to_owned()))?;
        let end = start
            .checked_add(usize::from(len))
            .ok_or_else(|| CalError::Transport("mock: read length overflowed".to_owned()))?;
        page.bytes
            .get(start..end)
            .map(<[u8]>::to_vec)
            .ok_or_else(|| {
                CalError::Transport(format!(
                    "mock: read of {len} byte(s) at 0x{addr:08X} runs past the mapped page"
                ))
            })
    }

    /// Write `data` at `addr` on the active page.
    ///
    /// # Errors
    ///
    /// [`CalError::Transport`] when a failure is armed, when the page is not
    /// mapped, or when the range is not fully mapped.
    pub fn write(&mut self, addr: u32, data: &[u8]) -> Result<(), CalError> {
        self.write_log.push((addr, data.to_vec()));
        if self.fail_writes > 0 {
            self.fail_writes -= 1;
            return Err(CalError::Transport(format!(
                "mock: write of {} byte(s) at 0x{addr:08X} failed",
                data.len()
            )));
        }
        let page = self.pages.get_mut(&self.current_page).ok_or_else(|| {
            CalError::Transport(format!(
                "mock: page {} is not mapped",
                self.current_page
            ))
        })?;
        let offset = addr.checked_sub(page.base).ok_or_else(|| {
            CalError::Transport(format!(
                "mock: address 0x{addr:08X} is below the page base 0x{:08X}",
                page.base
            ))
        })?;
        let start = usize::try_from(offset)
            .map_err(|_| CalError::Transport("mock: address offset overflowed".to_owned()))?;
        let end = start
            .checked_add(data.len())
            .ok_or_else(|| CalError::Transport("mock: write length overflowed".to_owned()))?;
        let target = page.bytes.get_mut(start..end).ok_or_else(|| {
            CalError::Transport(format!(
                "mock: write of {} byte(s) at 0x{addr:08X} runs past the mapped page",
                data.len()
            ))
        })?;
        target.copy_from_slice(data);
        Ok(())
    }
}

impl crate::session::XcpTransport for MockTransport {
    fn read(&mut self, addr: u32, len: u8) -> Result<Vec<u8>, CalError> {
        Self::read(self, addr, len)
    }

    fn write(&mut self, addr: u32, data: &[u8]) -> Result<(), CalError> {
        Self::write(self, addr, data)
    }
}
