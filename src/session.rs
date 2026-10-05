//! The calibration session: read-modify-write over an XCP transport, with
//! page switching, snapshots, restore, and diffs.
//!
//! # The transport is a trait, not a wire protocol
//!
//! [`XcpTransport`] is two methods — read bytes at an ECU address, write
//! bytes at an ECU address — plus an optional
//! [`set_page`](XcpTransport::set_page). Everything above it (addressing,
//! element coding, conversions, limits, paging) is this crate's job;
//! everything below it (CTO framing, block mode, CAN IDs, transport-layer
//! timing) is `xcp_core`'s and the caller's. A bench mock, a UDP XCP slave,
//! and a SocketCAN bridge all satisfy the same three methods, which is what
//! makes the session testable without an ECU — see [`crate::mock`].
//!
//! [`CalibrationSession`] also exposes the `xcp_core` command frames for
//! each operation ([`upload_frames`](CalibrationSession::upload_frames),
//! [`download_frames`](CalibrationSession::download_frames),
//! [`set_cal_page_frame`](CalibrationSession::set_cal_page_frame)), so a
//! caller that *is* a wire bridge builds the right frames instead of
//! inventing them.
//!
//! # Writes are verified, not trusted
//!
//! `write_characteristic` checks the A2L limits, inverts the COMPU_METHOD,
//! checks that the deposit can represent the result, reads back what it
//! wrote, and fails if the read-back differs. A calibration tool that
//! reports a write it cannot confirm is worse than one that reports an
//! error.

use crate::error::CalError;
use crate::project::CalibrationProject;
use std::fmt;

/// Byte-level ECU memory access, as a calibration session needs it.
pub trait XcpTransport {
    /// Read `len` bytes at ECU address `addr`.
    ///
    /// # Errors
    ///
    /// [`CalError::Transport`] when the slave or the link failed; the
    /// session surfaces the transport's own message unchanged.
    fn read(&mut self, addr: u32, len: u8) -> Result<Vec<u8>, CalError>;

    /// Write `data` at ECU address `addr`.
    ///
    /// # Errors
    ///
    /// [`CalError::Transport`] when the slave or the link failed.
    fn write(&mut self, addr: u32, data: &[u8]) -> Result<(), CalError>;

    /// Switch the slave's active calibration page (`SET_CAL_PAGE`).
    ///
    /// The default implementation accepts the switch without doing anything:
    /// a transport over an ECU with a single calibration page has nothing to
    /// do. A transport that multiplexes pages (or that forwards the command
    /// to a real slave) overrides it.
    ///
    /// # Errors
    ///
    /// [`CalError::Transport`] when the slave rejected the switch.
    fn set_page(&mut self, _page: u32) -> Result<(), CalError> {
        Ok(())
    }

    /// This transport as `Any`, for a caller that wants to reach a concrete
    /// implementation through the trait object (a test asserting on
    /// [`MockTransport`](crate::mock::MockTransport)'s recorded traffic, say).
    ///
    /// The default returns `None` — an implementation opts in by overriding
    /// it with `Some(self)`, so the trait stays object-safe and adding this
    /// was never a breaking change.
    fn as_any(&self) -> Option<&dyn std::any::Any> {
        None
    }
}

/// One captured value of one characteristic element.
#[derive(Debug, Clone, PartialEq)]
pub struct SnapshotValue {
    /// The memory bit pattern.
    pub raw: u64,
    /// The physical value the conversion produced.
    pub physical: f64,
}

/// One characteristic as captured: every element it deposits.
#[derive(Debug, Clone, PartialEq)]
pub struct SnapshotEntry {
    /// The module the characteristic lives in.
    pub module: String,
    /// The characteristic name.
    pub name: String,
    /// Its elements, in deposit order.
    pub elements: Vec<SnapshotValue>,
}

impl SnapshotEntry {
    /// The physical value of element `index`.
    #[must_use]
    pub fn physical(&self, index: usize) -> Option<f64> {
        self.elements.get(index).map(|v| v.physical)
    }

    /// `name` or `name[index]`, the way a report refers to one element.
    #[must_use]
    pub fn element_name(&self, index: usize) -> String {
        element_name(&self.name, self.elements.len(), index)
    }
}

impl fmt::Display for SnapshotEntry {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "  {}.{} ({} element{})",
            self.module,
            self.name,
            self.elements.len(),
            if self.elements.len() == 1 { "" } else { "s" }
        )?;
        for (index, value) in self.elements.iter().enumerate() {
            write!(
                f,
                "\n    [{index}] raw 0x{:X} physical {:.6}",
                value.raw, value.physical
            )?;
        }
        Ok(())
    }
}

/// A captured calibration set: the values of every named characteristic,
/// with the page they were read on.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Snapshot {
    /// The calibration page that was active when the capture started.
    pub page: u32,
    /// The captured characteristics, in capture order.
    pub entries: Vec<SnapshotEntry>,
}

impl Snapshot {
    /// An empty snapshot on page `page`.
    #[must_use]
    pub fn empty(page: u32) -> Self {
        Self {
            page,
            entries: Vec::new(),
        }
    }

    /// The entry for `name` in `module`.
    #[must_use]
    pub fn entry(&self, module: &str, name: &str) -> Option<&SnapshotEntry> {
        self.entries
            .iter()
            .find(|e| e.module == module && e.name == name)
    }

    /// The entry for `name`, searched across every module.
    #[must_use]
    pub fn find(&self, name: &str) -> Option<&SnapshotEntry> {
        self.entries.iter().find(|e| e.name == name)
    }

    /// The physical value of `name`'s first element.
    #[must_use]
    pub fn physical(&self, name: &str) -> Option<f64> {
        self.find(name).and_then(|entry| entry.physical(0))
    }

    /// Total number of elements captured.
    #[must_use]
    pub fn element_count(&self) -> usize {
        self.entries.iter().map(|e| e.elements.len()).sum()
    }
}

impl fmt::Display for Snapshot {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(
            f,
            "calibration snapshot: page {}, {} characteristic(s), {} element(s)",
            self.page,
            self.entries.len(),
            self.element_count()
        )?;
        for entry in &self.entries {
            writeln!(f, "{entry}")?;
        }
        Ok(())
    }
}

/// One change between two calibration states.
#[derive(Debug, Clone, PartialEq)]
pub struct CalibrationDelta {
    /// The changed parameter: `characteristic`, or `characteristic[index]`
    /// for a multi-element deposit.
    pub name: String,
    /// The value before the change (`NaN` when the parameter was not in the
    /// "before" snapshot).
    pub before: f64,
    /// The value after the change.
    pub after: f64,
    /// `after - before` (`NaN` when `before` is `NaN`).
    pub delta: f64,
    /// Whether `after` is inside the A2L limits of the characteristic.
    pub within_limits: bool,
}

impl CalibrationDelta {
    /// `true` when this parameter was absent from the "before" snapshot.
    #[must_use]
    pub fn is_new(&self) -> bool {
        self.before.is_nan()
    }

    /// `true` when nothing moved.
    #[must_use]
    pub fn is_unchanged(&self) -> bool {
        !self.is_new() && self.delta == 0.0
    }
}

impl fmt::Display for CalibrationDelta {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let verdict = if self.within_limits {
            "ok"
        } else {
            "OUT OF LIMITS"
        };
        if self.is_new() {
            write!(
                f,
                "{:<28} {:>14} {:>14.6} {:>14}  {verdict} (new)",
                self.name, "-", self.after, "-"
            )
        } else {
            write!(
                f,
                "{:<28} {:>14.6} {:>14.6} {:>+14.6}  {verdict}",
                self.name, self.before, self.after, self.delta
            )
        }
    }
}

/// Render a diff report: one header line, then one row per delta, then a
/// summary.
#[must_use]
pub fn render_deltas(deltas: &[CalibrationDelta]) -> String {
    use std::fmt::Write as _;

    let mut out = String::new();
    let _ = writeln!(
        out,
        "{:<28} {:>14} {:>14} {:>14}  limits",
        "parameter", "before", "after", "delta"
    );
    for delta in deltas {
        let _ = writeln!(out, "{delta}");
    }
    let changed = deltas.iter().filter(|d| !d.is_unchanged()).count();
    let out_of_limits = deltas.iter().filter(|d| !d.within_limits).count();
    let _ = writeln!(
        out,
        "-- {changed} changed, {} unchanged, {out_of_limits} out of limits",
        deltas.len() - changed
    );
    out
}

/// Diff two calibration states: `before` → `after`, one entry per element,
/// limit-checked against `project`'s current A2L limits.
///
/// A parameter present in `after` but not in `before` is reported with
/// `before == NaN` ([`CalibrationDelta::is_new`]); a parameter only in
/// `before` (removed between the two captures) is not reported, because the
/// report describes the state being moved to.
#[must_use]
pub fn diff_snapshots(
    before: Option<&Snapshot>,
    after: &Snapshot,
    project: &CalibrationProject,
) -> Vec<CalibrationDelta> {
    let mut deltas = Vec::new();
    for entry in &after.entries {
        let before_entry = before.and_then(|b| b.entry(&entry.module, &entry.name));
        let characteristic = project.characteristic(&entry.module, &entry.name).ok();
        for (index, value) in entry.elements.iter().enumerate() {
            let before_value = before_entry
                .and_then(|b| b.elements.get(index))
                .map_or(f64::NAN, |v| v.physical);
            let delta = if before_value.is_nan() {
                f64::NAN
            } else {
                value.physical - before_value
            };
            let within_limits = characteristic.is_none_or(|c| c.within_limits(value.physical));
            deltas.push(CalibrationDelta {
                name: entry.element_name(index),
                before: before_value,
                after: value.physical,
                delta,
                within_limits,
            });
        }
    }
    deltas
}

/// A calibration session against one ECU, over one transport.
pub struct CalibrationSession<'a> {
    project: &'a CalibrationProject,
    transport: Box<dyn XcpTransport + 'a>,
    page: u32,
    resources: xcp_core::ResourceMode,
    baseline: Option<Snapshot>,
}

impl<'a> CalibrationSession<'a> {
    /// Open a session on `project` over `transport`.
    ///
    /// `mode` is the slave's resource bitmask. When it is non-empty the
    /// session requires [`ResourceMode::CAL_PAGE`](xcp_core::ResourceMode::CAL_PAGE)
    /// and refuses to connect otherwise — a slave that advertises no CAL/PAG
    /// resource cannot be calibrated, and finding that out at connect time
    /// is much cheaper than finding it out at the first write. Pass the empty
    /// mask ([`ResourceMode::CONNECT_NORMAL`]) to skip the check, which is
    /// what a caller does when the transport is not completing a real CONNECT
    /// handshake.
    ///
    /// # Errors
    ///
    /// [`CalError::Xcp`] — [`xcp_core::XcpError::AccessDenied`] — when the
    /// resource mask excludes CAL/PAG.
    pub fn connect(
        project: &'a CalibrationProject,
        transport: Box<dyn XcpTransport + 'a>,
        mode: xcp_core::ResourceMode,
    ) -> Result<Self, CalError> {
        if !mode.is_empty() && !mode.cal_page() {
            return Err(CalError::Xcp(xcp_core::XcpError::AccessDenied));
        }
        Ok(Self {
            project,
            transport,
            page: 0,
            resources: mode,
            baseline: None,
        })
    }

    /// The project this session calibrates.
    #[must_use]
    pub const fn project(&self) -> &'a CalibrationProject {
        self.project
    }

    /// The transport, borrowed.
    #[must_use]
    pub fn transport(&self) -> &dyn XcpTransport {
        self.transport.as_ref()
    }

    /// The transport, mutably borrowed.
    pub fn transport_mut(&mut self) -> &mut (dyn XcpTransport + 'a) {
        self.transport.as_mut()
    }

    /// The negotiated resource mask.
    #[must_use]
    pub const fn resources(&self) -> xcp_core::ResourceMode {
        self.resources
    }

    /// The active calibration page.
    #[must_use]
    pub const fn cal_page(&self) -> u32 {
        self.page
    }

    /// Switch the slave's active calibration page.
    ///
    /// # Errors
    ///
    /// [`CalError::Transport`] when the transport rejected the switch; the
    /// session's page is left unchanged in that case, so a failed switch
    /// cannot silently redirect a later write to the wrong page.
    pub fn set_cal_page(&mut self, page: u32) -> Result<(), CalError> {
        self.transport.set_page(page)?;
        self.page = page;
        Ok(())
    }

    /// The `SET_MTA` + `UPLOAD` frames for a read of `len` bytes at `addr`,
    /// in the project's deposit byte order.
    #[must_use]
    pub fn upload_frames(&self, addr: u32, len: u8) -> Vec<u8> {
        xcp_core::upload_with(self.project.deposit_byte_order(), addr, 0, len)
    }

    /// The `SET_MTA` + `DOWNLOAD` frames for a write of `data` at `addr`, in
    /// the project's deposit byte order.
    #[must_use]
    pub fn download_frames(&self, addr: u32, data: &[u8]) -> Vec<u8> {
        xcp_core::download_with(self.project.deposit_byte_order(), addr, 0, data)
    }

    /// The `SET_CAL_PAGE` frame for `page` (switch on the ECU, all
    /// segments).
    #[must_use]
    pub fn set_cal_page_frame(&self, page: u32) -> Vec<u8> {
        xcp_core::set_cal_page(self.project.deposit_byte_order(), 0x03, page as u16)
    }

    /// Read a characteristic and return the physical value of its first
    /// element.
    ///
    /// # Errors
    ///
    /// [`CalError::Transport`] on a bus failure, [`CalError::Unsupported`]
    /// for an unresolvable conversion or a deposit the UPLOAD cannot carry,
    /// plus the lookup errors.
    pub fn read_characteristic(&mut self, module: &str, name: &str) -> Result<f64, CalError> {
        let raws = self.read_characteristic_raw(module, name)?;
        match raws.first() {
            Some(&raw) => self.project.to_physical(module, name, raw),
            // `elements == 0` is not rejected at declare_elements time (a
            // zero-element curve is a legal if useless declaration), so the
            // read path refuses it rather than indexing.
            None => Err(CalError::Unsupported {
                subject: format!("characteristic `{}`", qualified(module, name)),
                reason: "the deposit declares no elements".to_string(),
            }),
        }
    }

    /// Read every element of a characteristic and return their physical
    /// values — the curve or map as one read.
    ///
    /// # Errors
    ///
    /// As [`read_characteristic`](Self::read_characteristic).
    pub fn read_characteristic_elements(
        &mut self,
        module: &str,
        name: &str,
    ) -> Result<Vec<f64>, CalError> {
        let raws = self.read_characteristic_raw(module, name)?;
        let characteristic = self.project.characteristic(module, name)?;
        self.project.physical_values(characteristic, &raws)
    }

    /// Read every element of a characteristic and return its raw bit
    /// patterns.
    ///
    /// # Errors
    ///
    /// [`CalError::Transport`], [`CalError::Unsupported`] (including a
    /// deposit larger than one 255-byte UPLOAD), plus the lookup errors.
    pub fn read_characteristic_raw(
        &mut self,
        module: &str,
        name: &str,
    ) -> Result<Vec<u64>, CalError> {
        let characteristic = self.project.characteristic(module, name)?;
        let len = upload_len(characteristic)?;
        let data = self.transport.read(characteristic.address, len)?;
        self.project.read_raw(characteristic, &data)
    }

    /// Write a physical value to a characteristic's first element, and read
    /// it back to confirm the ECU latched it.
    ///
    /// The value is limit-checked and inverted through the COMPU_METHOD
    /// before anything reaches the bus, so a rejected write costs no
    /// traffic.
    ///
    /// # Errors
    ///
    /// [`CalError::LimitViolation`] for an out-of-limit value,
    /// [`CalError::OutOfBounds`] when the deposit cannot represent it,
    /// [`CalError::Transport`] on a bus failure or a read-back mismatch,
    /// plus the lookup errors.
    pub fn write_characteristic(
        &mut self,
        module: &str,
        name: &str,
        physical: f64,
    ) -> Result<(), CalError> {
        self.write_characteristic_element(module, name, 0, physical)
    }

    /// Write a physical value to one element of a characteristic and read
    /// it back to confirm.
    ///
    /// # Errors
    ///
    /// As [`write_characteristic`](Self::write_characteristic).
    pub fn write_characteristic_element(
        &mut self,
        module: &str,
        name: &str,
        index: usize,
        physical: f64,
    ) -> Result<(), CalError> {
        let raw = self.project.to_raw(module, name, physical)?;
        self.write_characteristic_raw(module, name, index, raw)?;
        let confirm = self.read_characteristic_raw(module, name)?;
        let latched = confirm.get(index).copied().unwrap_or(u64::MAX);
        if latched != raw {
            return Err(CalError::Transport(format!(
                "write-back mismatch for `{}.{}` element {index}: wrote 0x{raw:X}, read 0x{latched:X}",
                module, name
            )));
        }
        Ok(())
    }

    /// Write a raw bit pattern to one element, without limit checking or
    /// read-back — the path a snapshot restore takes, where the value was
    /// already validated when it was captured.
    ///
    /// A multi-element deposit is read, one element replaced, and written
    /// back whole: a calibration engineer adjusting curve point 3 must not
    /// lose points 1, 2, and 4.
    ///
    /// # Errors
    ///
    /// [`CalError::OutOfBounds`] when `index` is past the end of the
    /// deposit, [`CalError::Transport`] on a bus failure, plus the lookup
    /// errors.
    pub fn write_characteristic_raw(
        &mut self,
        module: &str,
        name: &str,
        index: usize,
        raw: u64,
    ) -> Result<(), CalError> {
        let characteristic = self.project.characteristic(module, name)?;
        let elements = characteristic.elements.max(1);
        if index >= elements {
            return Err(CalError::OutOfBounds {
                name: qualified(module, name),
                value: index as f64,
                lower: 0.0,
                upper: (elements - 1) as f64,
            });
        }
        let mut raws = if characteristic.elements > 1 {
            self.read_characteristic_raw(module, name)?
        } else {
            vec![0_u64; elements]
        };
        raws.resize(elements, 0);
        if let Some(slot) = raws.get_mut(index) {
            *slot = raw;
        }
        let data = self.project.write_raw(characteristic, &raws)?;
        self.transport.write(characteristic.address, &data)
    }

    /// Read a measurement and return its physical value.
    ///
    /// # Errors
    ///
    /// [`CalError::Transport`] on a bus failure, [`CalError::Unsupported`]
    /// for an unresolvable conversion, plus the lookup errors.
    pub fn read_measurement(&mut self, module: &str, name: &str) -> Result<f64, CalError> {
        let measurement = self.project.measurement(module, name)?;
        let len = measurement.size_bytes();
        if len == 0 || len > usize::from(u8::MAX) {
            return Err(CalError::Unsupported {
                subject: format!("measurement `{}.{}`", module, name),
                reason: format!("{len}-byte measurement cannot be read in one UPLOAD"),
            });
        }
        #[allow(clippy::cast_possible_truncation)]
        let data = self.transport.read(measurement.address, len as u8)?;
        let raw = decode_scalar(&data, measurement.is_signed());
        let conversion = measurement.conversion.resolved(self.project.tables())?;
        Ok(conversion.apply(raw))
    }

    /// Capture the named characteristics, searching every module for each
    /// name (first declaration wins).
    ///
    /// The snapshot records the page it was taken on, so
    /// [`restore`](Self::restore) puts the ECU back where it found it.
    ///
    /// # Errors
    ///
    /// As [`read_characteristic_raw`](Self::read_characteristic_raw), and
    /// [`CalError::UnknownCharacteristic`] for a name no module declares.
    pub fn snapshot(&mut self, names: &[&str]) -> Result<Snapshot, CalError> {
        let mut entries = Vec::with_capacity(names.len());
        for name in names {
            let (module_name, characteristic) = self.project.locate(name)?;
            let module = module_name.to_string();
            let raws = self.read_characteristic_raw(module_name, name)?;
            entries.push(SnapshotEntry {
                module,
                name: characteristic.name.clone(),
                elements: self.snapshot_values(characteristic, &raws)?,
            });
        }
        Ok(Snapshot {
            page: self.page,
            entries,
        })
    }

    /// Capture every characteristic of one module.
    ///
    /// # Errors
    ///
    /// As [`snapshot`](Self::snapshot).
    pub fn snapshot_module(&mut self, module: &str) -> Result<Snapshot, CalError> {
        let declared = self.project.module(module)?;
        let names: Vec<String> = declared.characteristic_names().map(String::from).collect();
        let borrowed: Vec<&str> = names.iter().map(String::as_str).collect();
        let mut snapshot = self.snapshot(&borrowed)?;
        for entry in &mut snapshot.entries {
            entry.module = module.to_string();
        }
        Ok(snapshot)
    }

    /// Restore a captured calibration set: switch to the snapshot's page,
    /// then write every captured raw value back verbatim.
    ///
    /// Raws are written, not physical values, so a restore is bit-exact and
    /// cannot drift through a second round of inversion.
    ///
    /// # Errors
    ///
    /// [`CalError::Transport`], [`CalError::UnknownModule`] /
    /// [`CalError::UnknownCharacteristic`] when the project no longer
    /// declares a captured parameter, plus the write errors.
    pub fn restore(&mut self, snapshot: &Snapshot) -> Result<(), CalError> {
        self.set_cal_page(snapshot.page)?;
        for entry in &snapshot.entries {
            let characteristic = self.project.characteristic(&entry.module, &entry.name)?;
            let raws: Vec<u64> = entry.elements.iter().map(|v| v.raw).collect();
            let data = self.project.write_raw(characteristic, &raws)?;
            self.transport.write(characteristic.address, &data)?;
        }
        Ok(())
    }

    /// Diff this session's **baseline** snapshot against `other`, one entry
    /// per element.
    ///
    /// `&self` cannot read the bus, so the "before" side is the baseline
    /// installed by [`set_baseline`](Self::set_baseline) (or
    /// [`capture_baseline`](Self::capture_baseline)). With no baseline set,
    /// every parameter is reported as new. Use [`diff_snapshots`] to diff two
    /// snapshots you already hold.
    ///
    /// `within_limits` is evaluated against the project's current A2L
    /// limits, so a baseline captured before a limit was tightened is
    /// reported against the limit that now applies.
    #[must_use]
    pub fn diff(&self, other: &Snapshot) -> Vec<CalibrationDelta> {
        diff_snapshots(self.baseline.as_ref(), other, self.project)
    }

    /// Install the baseline every [`diff`](Self::diff) measures against.
    pub fn set_baseline(&mut self, snapshot: Snapshot) {
        self.baseline = Some(snapshot);
    }

    /// The installed baseline, when one is set.
    #[must_use]
    pub fn baseline(&self) -> Option<&Snapshot> {
        self.baseline.as_ref()
    }

    /// Capture a baseline over every characteristic of every module and
    /// install it.
    ///
    /// # Errors
    ///
    /// [`CalError::Transport`], [`CalError::UnknownModule`] /
    /// [`CalError::UnknownCharacteristic`], [`CalError::Unsupported`] — as
    /// [`snapshot`](Self::snapshot).
    pub fn capture_baseline(&mut self) -> Result<(), CalError> {
        let modules: Vec<String> = self.project.module_names().map(String::from).collect();
        let mut entries = Vec::new();
        let page = self.page;
        for module in modules {
            let mut snapshot = self.snapshot_module(&module)?;
            entries.append(&mut snapshot.entries);
        }
        self.baseline = Some(Snapshot { page, entries });
        Ok(())
    }

    /// Convert raws into snapshot values.
    fn snapshot_values(
        &self,
        characteristic: &crate::model::Characteristic,
        raws: &[u64],
    ) -> Result<Vec<SnapshotValue>, CalError> {
        let conversion = self.project.resolved_conversion(characteristic)?;
        Ok(raws
            .iter()
            .map(|&raw| SnapshotValue {
                raw,
                physical: conversion.apply(characteristic.raw_value(raw)),
            })
            .collect())
    }
}

impl fmt::Debug for CalibrationSession<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("CalibrationSession")
            .field("page", &self.page)
            .field("resources", &self.resources.to_string())
            .finish_non_exhaustive()
    }
}

/// UPLOAD length for a characteristic's whole deposit record.
///
/// The record includes the bytes *before* `FNC_VALUES POSITION`, which the
/// layout reserves for other members of the record — the same span
/// [`CalibrationProject::write_raw`] produces, so a read and a write of the
/// same characteristic are the same length on the wire.
fn upload_len(characteristic: &crate::model::Characteristic) -> Result<u8, CalError> {
    let size = usize::from(characteristic.deposit_position) + characteristic.size_bytes();
    if size == 0 || size > usize::from(u8::MAX) {
        return Err(CalError::Unsupported {
            subject: format!("characteristic `{}`", characteristic.name),
            reason: format!("deposit of {size} bytes cannot be read in one UPLOAD"),
        });
    }
    #[allow(clippy::cast_possible_truncation)]
    Ok(size as u8)
}

/// Decode one signed or unsigned integer from a measurement payload.
fn decode_scalar(data: &[u8], is_signed: bool) -> f64 {
    let mut value = 0_u64;
    for (index, &byte) in data.iter().enumerate() {
        value |= u64::from(byte) << (index * 8);
    }
    let bits = u32::try_from(data.len() * 8).unwrap_or(32);
    if is_signed && bits > 0 && bits < 64 && value & (1_u64 << (bits - 1)) != 0 {
        #[allow(clippy::cast_possible_wrap)]
        let extended = (value | (u64::MAX << bits)) as i64;
        extended as f64
    } else {
        #[allow(clippy::cast_precision_loss)]
        {
            value as f64
        }
    }
}

/// `name` for a single element, `name[index]` for a multi-element one.
fn element_name(name: &str, elements: usize, index: usize) -> String {
    if elements <= 1 {
        name.to_string()
    } else {
        format!("{name}[{index}]")
    }
}

/// `module.name`.
fn qualified(module: &str, name: &str) -> String {
    format!("{module}.{name}")
}
