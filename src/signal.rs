//! Binding a CHARACTERISTIC to the CAN signal that carries it, and the
//! bit-level extraction that binding implies.
//!
//! # Why bit extraction lives here
//!
//! `dbc-parse` exposes [`Signal::decode_raw`](dbc_parse::Signal::decode_raw),
//! which returns the **scaled** value: it extracts the bits, sign-extends,
//! then applies `raw * scale + offset`. A calibration host needs the
//! *unscaled count*, because that is what the A2L `COMPU_METHOD` consumes.
//!
//! So the two halves compose as
//!
//! ```text
//!   frame bytes ──extract_raw──► raw counts ──COMPU_METHOD──► physical units
//!                   (here)          (dbc-parse's scale/offset
//!                                     is deliberately not applied)
//! ```
//!
//! and the DBC scale is only used to *cross-check*: inverting it must
//! reproduce the count this module extracts. The `dbc_integration` test
//! does exactly that.
//!
//! # Bit numbering
//!
//! DBC numbers payload bits **LSB-first within each byte**: bit `b` lives in
//! byte `b / 8` at in-byte mask `1 << (b % 8)`. That numbering is the same
//! for both byte orders; what differs is which bit `start_bit` names and the
//! direction of travel:
//!
//! - **Intel** (`@1`, [`ByteOrder::Intel`]) — `start_bit` names the **least
//!   significant** bit. The signal's bits ascend linearly, straight through
//!   byte boundaries, so `12|16@1` reads bytes 1–2 little-endian.
//! - **Motorola** (`@0`, [`ByteOrder::Motorola`]) — `start_bit` names the
//!   **most significant** bit. Extraction descends within each byte and then
//!   jumps `+15` from bit 0 to the next byte's bit 7 — the "sawtooth" that
//!   reproduces `CANdb++` and `cantools` bit for bit, including the layouts
//!   that straddle three bytes.
//!
//! Both are implemented by walking the bit positions in significance order
//! and accumulating, which is the clearest form of the two layouts and the
//! one a naive bit-reversal reference can be checked against.

use dbc_parse::ByteOrder;

use crate::error::CalError;

/// A CHARACTERISTIC bound to the CAN signal that carries it.
///
/// The binding is what lets a calibration tool tie a calibration parameter to
/// what the rest of the vehicle sees on the bus: the same number, carried
/// two different ways.
#[derive(Debug, Clone, PartialEq)]
pub struct SignalBinding {
    /// The CHARACTERISTIC this signal carries.
    pub characteristic: String,
    /// CAN identifier of the carrying message.
    pub can_id: u32,
    /// DBC start bit of the signal.
    pub start_bit: u8,
    /// Signal width in bits.
    pub length: u8,
    /// Bit layout order.
    pub byte_order: ByteOrder,
    /// DBC scale factor: `physical = raw * factor + offset`.
    pub factor: f64,
    /// DBC offset: `physical = raw * factor + offset`.
    pub offset: f64,
}

impl SignalBinding {
    /// Bind `characteristic` to the signal layout `signal` of `can_id`.
    #[must_use]
    pub fn new(
        characteristic: impl Into<String>,
        can_id: u32,
        start_bit: u16,
        length: u8,
        byte_order: ByteOrder,
        factor: f64,
        offset: f64,
    ) -> Self {
        Self {
            characteristic: characteristic.into(),
            can_id,
            #[allow(clippy::cast_possible_truncation)]
            start_bit: start_bit.min(511) as u8,
            length: length.clamp(0, 64),
            byte_order,
            factor,
            offset,
        }
    }

    /// Extract the signal's **raw count** from a CAN payload.
    ///
    /// Bits beyond `data` read as zero, matching `dbc-parse`'s documented
    /// partial-payload behaviour, so a truncated frame decodes rather than
    /// failing. A zero- or 64-bit width both yield `0` / all-ones-free
    /// `0` respectively and never panic.
    #[must_use]
    pub fn extract_raw(&self, data: &[u8]) -> u64 {
        extract_raw(
            u32::from(self.start_bit),
            self.length,
            self.byte_order,
            data,
        )
    }

    /// Write `raw` into a CAN payload at this signal's layout.
    ///
    /// Bits outside the buffer are dropped rather than panicking, so a
    /// short frame is written partially — which is what a real bus driver
    /// does with a DLC that exceeds the mapped memory.
    ///
    /// A `raw` too wide for `length` is truncated to `length` bits, because
    /// the alternative — refusing — would make a signal whose scaled value
    /// rounds to an edge case unwritable.
    pub fn insert_raw(&self, data: &mut [u8], raw: u64) -> Result<(), CalError> {
        let width = u32::from(self.length);
        if width == 0 {
            return Ok(());
        }
        let mask = if width >= 64 { u64::MAX } else { (1u64 << width) - 1 };
        write_bits(
            u32::from(self.start_bit),
            width,
            self.byte_order,
            data,
            raw & mask,
        );
        Ok(())
    }

    /// The signal's physical value as the bus presents it: the count this
    /// binding extracts, scaled the DBC way.
    ///
    /// This is the quantity a bus observer sees; the count itself is what the
    /// A2L `COMPU_METHOD` consumes.
    #[must_use]
    pub fn to_physical(&self, data: &[u8]) -> f64 {
        let raw = self.extract_raw(data);
        let value = raw as f64;
        value * self.factor + self.offset
    }

    /// Recover the raw count from a bus-observed physical value, by
    /// inverting the DBC factor.
    ///
    /// # Errors
    ///
    /// [`CalError::UnsupportedConversion`] when `factor` is zero, so the
    /// inverse does not exist.
    pub fn from_physical(&self, physical: f64) -> Result<u64, CalError> {
        if self.factor == 0.0 {
            return Err(CalError::UnsupportedConversion {
                name: self.characteristic.clone(),
                detail: "the DBC signal declares a zero scale factor",
            });
        }
        Ok(crate::conversion::round_half_away((physical - self.offset) / self.factor)
            as u64)
    }

    /// `true` when the CAN id is a standard (11-bit) identifier.
    #[must_use]
    pub fn is_standard_id(&self) -> bool {
        self.can_id <= 0x7FF
    }
}

/// Extract a signal's raw count from a CAN payload.
///
/// The single implementation of the DBC bit layouts, shared by
/// [`SignalBinding::extract_raw`] and the tests. See the module docs for the
/// numbering.
///
/// # Total
///
/// `start_bit` above 511 (the CAN FD limit) and bits beyond `data` both read
/// as zero; the loop is bounded by `length` (clamped to 64), so no input can
/// panic or fail to terminate.
#[must_use]
pub fn extract_raw(start_bit: u32, length: u8, byte_order: ByteOrder, data: &[u8]) -> u64 {
    let width = u32::from(length.min(64));
    if width == 0 {
        return 0;
    }
    // Accumulate MSB-first along the layout's significance order, so one
    // loop serves both byte orders.
    let mut value = 0u64;
    for offset in 0..width {
        let bit = read_bit(data, bit_position(start_bit, width, offset, byte_order));
        value = (value << 1) | u64::from(bit);
    }
    value
}

/// The absolute DBC bit index carrying the `offset`-th **most significant**
/// bit of a `width`-bit signal that starts at `start_bit`.
///
/// Intel: `start_bit` is the LSB, so the MSB sits `width - 1` above it and
/// the walk *descends* from there. Motorola: `start_bit` is the MSB, so the
/// walk follows the sawtooth — descending within a byte, then jumping
/// `+15` from bit 0 to the next byte's bit 7.
#[must_use]
pub const fn bit_position(start_bit: u32, width: u32, offset: u32, byte_order: ByteOrder) -> u32 {
    match byte_order {
        ByteOrder::Intel => start_bit
            .saturating_add(width.saturating_sub(1))
            .saturating_sub(offset),
        ByteOrder::Motorola => sawtooth(start_bit, offset),
    }
}

/// The sawtooth walk: `steps` positions along the Motorola layout.
const fn sawtooth(start_bit: u32, steps: u32) -> u32 {
    let mut pos = start_bit;
    let mut i = 0;
    while i < steps {
        pos = if pos % 8 == 0 { pos + 15 } else { pos - 1 };
        i += 1;
    }
    pos
}

/// Read one absolute DBC bit index from `data`; out-of-range reads as zero.
///
/// DBC numbers bits LSB-first within each byte, so the in-byte mask is
/// `1 << (position % 8)`.
#[must_use]
fn read_bit(data: &[u8], position: u32) -> u8 {
    let index = usize::try_from(position / 8).unwrap_or(usize::MAX);
    match data.get(index) {
        Some(byte) => (byte >> (position % 8)) & 1,
        None => 0,
    }
}

/// Write `value`'s low `width` bits into `data` at the signal's layout.
///
/// Walks the same significance order as [`extract_raw`], so this is its
/// exact inverse for both byte orders.
fn write_bits(start_bit: u32, width: u32, byte_order: ByteOrder, data: &mut [u8], value: u64) {
    for offset in 0..width {
        let position = bit_position(start_bit, width, offset, byte_order);
        // `offset` counts from the MSB, so the bit to place is
        // `width - 1 - offset` from the bottom.
        let shift = width.saturating_sub(1).saturating_sub(offset);
        let bit = ((value >> shift) & 1) as u8;
        let index = usize::try_from(position / 8).unwrap_or(usize::MAX);
        if let Some(byte) = data.get_mut(index) {
            let mask = 1u8 << (position % 8);
            *byte &= !mask;
            *byte |= bit << (position % 8);
        }
    }
}

/// Sign-extend a raw count of `width` bits from its two's-complement form.
///
/// # Total
///
/// A `width` of 0 or ≥ 64 returns `raw` reinterpreted, so no shift by 64 can
/// occur.
#[must_use]
pub fn sign_extend(raw: u64, width: u8) -> i64 {
    #[allow(clippy::cast_possible_wrap)]
    if width == 0 || width >= 64 {
        return raw as i64;
    }
    let bits = u32::from(width);
    let sign = 1u64 << (bits - 1);
    #[allow(clippy::cast_possible_wrap)]
    if raw & sign != 0 {
        (raw | (u64::MAX << bits)) as i64
    } else {
        raw as i64
    }
}
