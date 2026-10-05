//! The calibration session: read-modify-write against an ECU, plus snapshot,
//! restore, and diff.
//!
//! # Why the transport is two methods wide
//!
//! [`XcpTransport`] is `read`/`write` and nothing else. That is narrower than
//! XCP, deliberately: XCP addresses every access through the MTA, so the
//! protocol's verbosity (SET_MTA, then UPLOAD or DOWNLOAD, then the response
//! parse) is a *framing* concern, not a calibration concern. Keeping it out
//! of the session signature is what lets
//! [`MockTransport`](crate::MockTransport) — plain memory — stand in for a
//! real bus, and what lets [`CalibrationSession::set_cal_page`] be tested
//! without a protocol stack.
//!
//! The `xcp-core` framing is still used, by [`CalibrationSession::upload_frames`]
//! and friends: those hand a real bridge exactly the frames to transmit,
//! already split into MTA and transfer packets. Nothing in the calibration
//! path has to reimplement the protocol, and nothing outside it has to know
//! it.

use std::fmt;

use xcp_core::{ByteOrder, ResourceMode};

use crate::error::CalError;
use crate::project::CalibrationProject;

/// `MAX_CTO` for classic XCP on CAN: the CTO packet size every CAN slave
/// uses, and so the frame size the download sequence chunks at.
const CAN_MAX_CTO: usize = 8;

/// A memory-level transport to an ECU.
///
/// Implementations wrap whatever carries the bytes: a CAN bridge, a UDP
/// gateway, or [`MockTransport`](crate::MockTransport). Address resolution
/// and paging belong to the session, so an implementation need not know
/// about either.
pub trait XcpTransport {
    /// Read `len` bytes starting at ECU address `addr`.
    ///
    /// # Errors
    ///
    /// [`CalError::Transport`] on a bus or link failure, and on an address
    /// the target does not map.
    fn read(&mut self, addr: u32, len: u8) -> Result<Vec<u8>, CalError>;

    /// Write `data` starting at ECU address `addr`.
    ///
    /// # Errors
    ///
    /// [`CalError::Transport`] on a bus or link failure, and on an address
    /// the target does not map.
    fn write(&mut self, addr: u32, data: &[u8]) -> Result<(), CalError>;
}

/// One characteristic's captured value.
#[derive(Debug, Clone, PartialEq)]
pub struct SnapshotEntry {
    /// The module the characteristic lives in.
    pub module: String,
    /// The characteristic name.
    pub name: String,
    /// The physical value read at capture time.
    pub value: f64,
    /// The raw memory bit pattern, so a restore is bit-exact.
    pub raw: u64,
}

/// A named set of characteristic values captured from an ECU.
///
/// `PartialEq` compares by value, so "the ECU is back where it started" is a
/// one-line assertion — and a property test can assert it over generated
/// value sets.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Snapshot {
    /// Captured values, in capture order.
    pub entries: Vec<SnapshotEntry>,
}

impl Snapshot {
    /// An empty snapshot.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// The captured value of `name`.
    #[must_use]
    pub fn value(&self, name: &str) -> Option<f64> {
        self.entries
            .iter()
            .find(|entry| entry.name == name)
            .map(|entry| entry.value)
    }

    /// The captured raw pattern of `name`.
    #[must_use]
    pub fn raw(&self, name: &str) -> Option<u64> {
        self.entries
            .iter()
            .find(|entry| entry.name == name)
            .map(|entry| entry.raw)
    }

    /// Number of captured characteristics.
    #[must_use]
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// `true` when nothing was captured.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// Add an entry.
    pub fn push(&mut self, entry: SnapshotEntry) {
        self.entries.push(entry);
    }

    /// A readable report of `other` measured against this snapshot.
    ///
    /// Lists **every** characteristic both snapshots share, marking the
    /// changed ones, because a calibration report that silently omits an
    /// unchanged parameter is the report an engineer stops trusting.
    /// [`render_deltas`] is the changed-only form.
    #[must_use]
    pub fn report(&self, other: &Snapshot) -> String {
        let mut lines = Vec::new();
        let width = self
            .entries
            .iter()
            .chain(other.entries.iter())
            .map(|entry| entry.name.len())
            .max()
            .unwrap_or(0);
        for entry in &self.entries {
            let Some(counterpart) = other.entries.iter().find(|candidate| {
                candidate.module == entry.module && candidate.name == entry.name
            }) else {
                continue;
            };
            #[allow(clippy::cast_precision_loss)]
            let delta = counterpart.value - entry.value;
            if delta == 0.0 {
                continue;
            }
            #[allow(clippy::cast_precision_loss)]
            let _ = lines.push(format!(
                "{:width$}  {:>12.4} -> {:<12.4} ({:+11.4})",
                entry.name, entry.value, counterpart.value, delta,
                width = width
            ));
        }
        if lines.is_empty() {
            return "no calibration changes".to_owned();
        }
        lines.join("\n")
    }
}

/// One characteristic that changed between two snapshots.
#[derive(Debug, Clone, PartialEq)]
pub struct CalibrationDelta {
    /// The characteristic name.
    pub name: String,
    /// The value in the earlier snapshot.
    pub before: f64,
    /// The value in the later snapshot.
    pub after: f64,
    /// `after - before`.
    pub delta: f64,
    /// `true` when the **new** value sits inside the characteristic's
    /// declared limits.
    pub within_limits: bool,
}

/// Render changed characteristics as an aligned report.
///
/// Unchanged characteristics are omitted; [`Snapshot::report`] is the
/// form that lists everything.
#[must_use]
pub fn render_deltas(deltas: &[CalibrationDelta]) -> String {
    if deltas.is_empty() {
        return "no calibration changes".to_owned();
    }
    let width = deltas
        .iter()
        .map(|delta| delta.name.len())
        .max()
        .unwrap_or(0);
    let mut lines = Vec::with_capacity(deltas.len());
    for delta in deltas {
        #[allow(clippy::cast_precision_loss)]
        let flag = if delta.within_limits {
            ""
        } else {
            "  OUT OF LIMITS"
        };
        #[allow(clippy::cast_precision_loss)]
        lines.push(format!(
            "{:width$}  {:>12.4} -> {:<12.4} ({:+11.4}){flag}",
            delta.name,
            delta.before,
            delta.after,
            delta.delta,
            width = width
        ));
    }
    lines.join("\n")
}

/// A live calibration session against one ECU.
pub struct CalibrationSession<'a> {
    project: &'a CalibrationProject,
    transport: Box<dyn XcpTransport>,
    page: u32,
    resource: ResourceMode,
    byte_order: ByteOrder,
    /// The most recent capture, the reference point for
    /// [`CalibrationSession::diff`].
    baseline: Option<Snapshot>,
}

impl fmt::Debug for CalibrationSession<'_> {
    /// Hand-written so the trait object need not be `Debug`: a session holds
    /// a boxed transport whose internals are the caller's business, and
    /// printing them would leak bus details into a log.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("CalibrationSession")
            .field("page", &self.page)
            .field("resource", &self.resource)
            .field("byte_order", &self.byte_order)
            .field("baseline_entries", &self.baseline.as_ref().map_or(0, Snapshot::len))
            .finish_non_exhaustive()
    }
}

impl<'a> CalibrationSession<'a> {
    /// Open a session on `transport`, requesting `mode`'s resources.
    ///
    /// The requested mask is recorded and reported by
    /// [`CalibrationSession::resource`]. A real bridge enforces it against
    /// the CONNECT response; a mock has no protocol to negotiate.
    ///
    /// `CONNECT_NORMAL` and `CONNECT_USER_DEFINED` are mode bytes rather than
    /// resource sets, so a session opened with one is treated as offering
    /// `CAL/PAG` — the resource every calibration session needs.
    ///
    /// # Errors
    ///
    /// Never today; the signature is fallible so a transport that *does*
    /// negotiate at connect time (and may refuse) fits without a break.
    pub fn connect(
        project: &'a CalibrationProject,
        transport: Box<dyn XcpTransport>,
        mode: ResourceMode,
    ) -> Result<Self, CalError> {
        Ok(Self {
            project,
            transport,
            page: 0,
            resource: mode,
            byte_order: ByteOrder::Intel,
            baseline: None,
        })
    }

    /// The resource mask this session was opened with.
    #[must_use]
    pub fn resource(&self) -> ResourceMode {
        self.resource
    }

    /// The current calibration page.
    #[must_use]
    pub fn page(&self) -> u32 {
        self.page
    }

    /// The byte order multi-byte deposits are read and written in.
    ///
    /// Intel (little-endian) by default, matching `xcp-core`'s short-form
    /// builders and the overwhelming majority of XCP slaves.
    #[must_use]
    pub fn byte_order(&self) -> ByteOrder {
        self.byte_order
    }

    /// Set the byte order, as the CONNECT response's `COMM_MODE_BASIC`
    /// advertises it.
    pub fn set_byte_order(&mut self, byte_order: ByteOrder) {
        self.byte_order = byte_order;
    }

    /// Switch the active calibration page.
    ///
    /// Mirrors `SET_CAL_PAGE`: subsequent addresses resolve into the new
    /// page and the previous page's contents are untouched.
    ///
    /// # Errors
    ///
    /// [`CalError::Transport`] when the session was opened without the
    /// `CAL/PAG` resource, which [`ResourceMode::cal_page`] detects.
    pub fn set_cal_page(&mut self, page: u32) -> Result<(), CalError> {
        if !self.resource.cal_page() && !is_connect_mode(self.resource) {
            return Err(CalError::Transport(
                "the slave does not offer the CAL/PAG resource".to_owned(),
            ));
        }
        self.page = page;
        Ok(())
    }

    /// Read a characteristic's physical value from the ECU.
    ///
    /// # Errors
    ///
    /// [`CalError::UnknownModule`], [`CalError::UnknownCharacteristic`],
    /// [`CalError::UnsupportedConversion`] for a block deposit read as a
    /// scalar, and [`CalError::Transport`] on a bus failure.
    pub fn read_characteristic(&mut self, module: &str, name: &str) -> Result<f64, CalError> {
        let (address, size) = self.scalar_deposit(module, name)?;
        let bytes = self.transport.read(address, size)?;
        let raw = decode(&bytes, self.byte_order)
            .ok_or_else(|| CalError::Transport("the transport returned no bytes".to_owned()))?;
        Ok(self
            .project
            .characteristic(module, name)?
            .to_physical(raw))
    }

    /// Read a characteristic's raw memory bit pattern from the ECU.
    ///
    /// # Errors
    ///
    /// As [`CalibrationSession::read_characteristic`].
    pub fn read_raw(&mut self, module: &str, name: &str) -> Result<u64, CalError> {
        let (address, size) = self.scalar_deposit(module, name)?;
        let bytes = self.transport.read(address, size)?;
        decode(&bytes, self.byte_order)
            .ok_or_else(|| CalError::Transport("the transport returned no bytes".to_owned()))
    }

    /// Write a characteristic's physical value to the ECU.
    ///
    /// The value is converted, limit-checked, and only then deposited — so an
    /// out-of-range request never reaches the bus. This is the
    /// read-modify-write cycle's write half, and it is the step that must
    /// never be best-effort.
    ///
    /// # Errors
    ///
    /// As [`CalibrationProject::to_raw`], plus [`CalError::Transport`] on a
    /// bus failure.
    pub fn write_characteristic(
        &mut self,
        module: &str,
        name: &str,
        physical: f64,
    ) -> Result<(), CalError> {
        let (address, size) = self.scalar_deposit(module, name)?;
        // Convert first: a rejected value must not reach the transport.
        let raw = self.project.to_raw(module, name, physical)?;
        self.transport
            .write(address, &encode(raw, usize::from(size), self.byte_order))
    }

    /// Capture the current value of each named characteristic.
    ///
    /// The module is inferred: a name is resolved against every module, and
    /// a name that appears in more than one is
    /// [`CalError::AmbiguousSignal`]-shaped ambiguity — reported as
    /// [`CalError::UnknownCharacteristic`] would hide the real cause, so this
    /// returns a [`CalError::Transport`] naming both modules instead. Use
    /// [`CalibrationSession::snapshot_module`] to name the module explicitly
    /// and skip the search.
    ///
    /// # Errors
    ///
    /// [`CalError::UnknownModule`], [`CalError::UnknownCharacteristic`],
    /// [`CalError::Transport`] when a name is ambiguous, and
    /// [`CalError::Transport`] on a bus failure.
    pub fn snapshot(&mut self, names: &[&str]) -> Result<Snapshot, CalError> {
        let mut entries = Vec::with_capacity(names.len());
        for name in names {
            let module = self.locate(name)?;
            let value = self.read_characteristic(&module, name)?;
            let raw = self.read_raw(&module, name)?;
            entries.push(SnapshotEntry {
                module,
                name: (*name).to_owned(),
                value,
                raw,
            });
        }
        let snapshot = Snapshot { entries };
        self.baseline = Some(snapshot.clone());
        Ok(snapshot)
    }

    /// Capture the current value of each named characteristic in `module`.
    ///
    /// # Errors
    ///
    /// [`CalError::UnknownModule`], [`CalError::UnknownCharacteristic`], and
    /// [`CalError::Transport`] on a bus failure.
    pub fn snapshot_module(
        &mut self,
        module: &str,
        names: &[&str],
    ) -> Result<Snapshot, CalError> {
        // Resolve every name before any bus traffic, so a typo in the last
        // entry does not leave the ECU half-read.
        for name in names {
            self.project.characteristic(module, name)?;
        }
        let mut entries = Vec::with_capacity(names.len());
        for name in names {
            let value = self.read_characteristic(module, name)?;
            let raw = self.read_raw(module, name)?;
            entries.push(SnapshotEntry {
                module: module.to_owned(),
                name: (*name).to_owned(),
                value,
                raw,
            });
        }
        let snapshot = Snapshot { entries };
        self.baseline = Some(snapshot.clone());
        Ok(snapshot)
    }

    /// Write a snapshot's values back to the ECU.
    ///
    /// Restoring the captured **raw** pattern — not the physical value — is
    /// what makes this exact: the round trip through the conversion is not
    /// repeated on the way back, so even a value that does not survive a
    /// float quantisation is restored bit-for-bit.
    ///
    /// # Errors
    ///
    /// [`CalError::UnknownModule`], [`CalError::UnknownCharacteristic`],
    /// [`CalError::UnsupportedConversion`] for a block deposit, and
    /// [`CalError::Transport`] on a bus failure.
    pub fn restore(&mut self, snapshot: &Snapshot) -> Result<(), CalError> {
        for entry in &snapshot.entries {
            let size = self.scalar_deposit(&entry.module, &entry.name)?.1;
            self.transport.write(
                self.project
                    .characteristic(&entry.module, &entry.name)?
                    .address,
                &encode(entry.raw, usize::from(size), self.byte_order),
            )?;
        }
        Ok(())
    }

    /// The characteristics that differ between this session's baseline and
    /// `other`.
    ///
    /// The baseline is the most recent snapshot the session captured, so
    /// `before` is what the ECU held then and `after` is what `other` says
    /// it holds now. A characteristic present in one snapshot and absent
    /// from the other cannot be compared and is omitted; an unchanged value
    /// is omitted too, so the result is a work list rather than a
    /// transcript.
    ///
    /// `within_limits` reports whether the **new** value sits inside the
    /// characteristic's declared limits — the flag an engineer scans for.
    ///
    /// A session that has not captured a baseline yields no deltas; use
    /// [`diff_snapshots`] to compare two snapshots directly.
    #[must_use]
    pub fn diff(&self, other: &Snapshot) -> Vec<CalibrationDelta> {
        match &self.baseline {
            Some(baseline) => diff_snapshots(baseline, other, self.project),
            None => Vec::new(),
        }
    }

    /// The most recent snapshot this session captured, if any.
    #[must_use]
    pub fn baseline(&self) -> Option<&Snapshot> {
        self.baseline.as_ref()
    }

    /// The XCP frames a bridge should send to upload `len` bytes from
    /// `addr`: `SET_MTA` then `UPLOAD`.
    ///
    /// # Errors
    ///
    /// [`CalError::UnsupportedConversion`] when the request is larger than
    /// one transfer can carry, which a bridge must not silently split.
    pub fn upload_frames(
        &self,
        addr: u32,
        extension: u8,
        len: u8,
    ) -> Result<Vec<Vec<u8>>, CalError> {
        Ok(vec![
            xcp_core::set_mta(self.byte_order, addr, extension),
            xcp_core::upload_with(self.byte_order, addr, extension, len),
        ])
    }

    /// The XCP frames a bridge should send to download `data` to `addr`:
    /// `SET_MTA` then a block-mode `DOWNLOAD` / `DOWNLOAD_NEXT` chain,
    /// chunked to the slave's `MAX_CTO`.
    ///
    /// # Errors
    ///
    /// [`CalError::UnsupportedConversion`] when `data` is longer than the
    /// protocol's absolute cap, and [`CalError::Transport`] when it is empty
    /// — a `DOWNLOAD` of zero elements is a protocol error, not a no-op.
    pub fn download_frames(
        &self,
        addr: u32,
        extension: u8,
        data: &[u8],
    ) -> Result<Vec<Vec<u8>>, CalError> {
        if data.is_empty() {
            return Err(CalError::Transport(
                "refusing to build a DOWNLOAD of zero elements".to_owned(),
            ));
        }
        if data.len() > xcp_core::DOWNLOAD_MAX_CHUNK + xcp_core::DOWNLOAD_NEXT_MAX_CHUNK {
            return Err(CalError::UnsupportedConversion {
                name: format!("0x{addr:08X}"),
                detail: "the block transfer exceeds the protocol's element cap",
            });
        }
        Ok(xcp_core::download_with(self.byte_order, addr, extension, data)
            .chunks(CAN_MAX_CTO)
            .map(<[u8]>::to_vec)
            .collect())
    }

    /// The frames that select calibration page `page` in `segment`.
    #[must_use]
    pub fn cal_page_frames(&self, page: u32, segment: u16) -> Vec<u8> {
        #[allow(clippy::cast_possible_truncation)]
        let mode = (page & 0xFF) as u8;
        xcp_core::set_cal_page(self.byte_order, mode, segment)
    }

    /// The project this session reads through.
    #[must_use]
    pub fn project(&self) -> &'a CalibrationProject {
        self.project
    }

    /// The deposit `(address, element size)` for a scalar characteristic.
    fn scalar_deposit(&self, module: &str, name: &str) -> Result<(u32, u8), CalError> {
        let characteristic = self.project.characteristic(module, name)?;
        let size = characteristic
            .deposit_size()
            .filter(|_| !characteristic.kind.is_block())
            .ok_or_else(|| CalError::UnsupportedConversion {
                name: characteristic.name.clone(),
                detail: "a block deposit cannot be accessed as a scalar",
            })?;
        let size = u8::try_from(size).map_err(|_| CalError::UnsupportedConversion {
            name: characteristic.name.clone(),
            detail: "the deposit element is larger than one XCP transfer",
        })?;
        Ok((characteristic.address, size))
    }

    /// Find the single module declaring `name`.
    fn locate(&self, name: &str) -> Result<String, CalError> {
        let mut found: Option<&str> = None;
        for module in self.project.modules() {
            if module.characteristic(name).is_some() {
                if let Some(previous) = found {
                    return Err(CalError::Transport(format!(
                        "characteristic `{name}` is declared in both module `{previous}` and `{}`",
                        module.name
                    )));
                }
                found = Some(&module.name);
            }
        }
        found
            .map(str::to_owned)
            .ok_or_else(|| CalError::UnknownCharacteristic((*name).to_owned()))
    }
}

/// `true` when `resource` is a CONNECT mode byte rather than a resource set.
fn is_connect_mode(resource: ResourceMode) -> bool {
    resource == ResourceMode::CONNECT_NORMAL
        || resource == ResourceMode::CONNECT_USER_DEFINED
}

/// Decode a raw count from a deposit buffer in `byte_order`.
///
/// # Total
///
/// A buffer shorter than the count's width is zero-extended and a longer one
/// is truncated, matching `xcp-core`'s short-form builders. An empty buffer
/// is `None`, because there is no value to report.
#[must_use]
fn decode(bytes: &[u8], byte_order: ByteOrder) -> Option<u64> {
    // An empty buffer is `None`: the caller asked for a count and there is
    // no value to report.
    bytes.first()?;
    let width = bytes.len().min(8);
    let mut value = 0u64;
    for (index, byte) in bytes.iter().take(8).enumerate() {
        let shift = if byte_order == ByteOrder::Motorola {
            8 * u32::try_from(width - 1 - index).unwrap_or(0)
        } else {
            8 * u32::try_from(index).unwrap_or(0)
        };
        value |= u64::from(*byte).wrapping_shl(shift);
    }
    Some(value)
}

/// Encode a raw count into a little- or big-endian buffer of `size` bytes.
///
/// # Total
///
/// A `size` of 0 yields an empty buffer and a `size` above 8 truncates the
/// high bytes, which is what a deposit wider than the transfer implies.
#[must_use]
fn encode(value: u64, size: usize, byte_order: ByteOrder) -> Vec<u8> {
    let width = size.min(8);
    (0..size)
        .map(|index| {
            if index >= 8 {
                return 0;
            }
            #[allow(clippy::cast_possible_truncation)]
            let shift = if byte_order == ByteOrder::Motorola {
                8 * (width - 1 - index.min(width - 1))
            } else {
                8 * index
            };
            #[allow(clippy::cast_possible_truncation)]
            {
                (value >> shift.min(56)) as u8
            }
        })
        .collect()
}

/// Compare two snapshots, flagging which changes land inside declared limits.
///
/// `before` supplies the earlier state and `after` the later one; `project`
/// supplies the limits. A characteristic `project` does not declare is
/// reported as within limits — there is no limit to violate, and reporting
/// otherwise would train an engineer to ignore the flag.
///
/// An unchanged value is omitted, so the result is a work list rather than a
/// transcript. [`CalibrationSession::diff`] is the session-bound form of this.
#[must_use]
pub fn diff_snapshots(
    before: &Snapshot,
    after: &Snapshot,
    project: &CalibrationProject,
) -> Vec<CalibrationDelta> {
    let mut deltas = Vec::new();
    for entry in &before.entries {
        let Some(counterpart) = after
            .entries
            .iter()
            .find(|candidate| candidate.module == entry.module && candidate.name == entry.name)
        else {
            continue;
        };
        #[allow(clippy::cast_precision_loss)]
        let delta = counterpart.value - entry.value;
        if delta == 0.0 {
            continue;
        }
        let within_limits = project
            .characteristic(&entry.module, &entry.name)
            .is_ok_and(|characteristic| {
                characteristic.lower_limit <= counterpart.value
                    && counterpart.value <= characteristic.upper_limit
            });
        deltas.push(CalibrationDelta {
            name: entry.name.clone(),
            before: entry.value,
            after: counterpart.value,
            delta,
            within_limits,
        });
    }
    deltas
}
