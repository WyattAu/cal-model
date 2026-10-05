//! CAN signal bindings: the bridge from an A2L CHARACTERISTIC to a DBC
//! message signal.
//!
//! A calibration engineer routinely needs the *bus* view of a parameter —
//! "which frame carries the boost target, and at which bit?" — as well as
//! its ECU-memory view. [`attach_dbc`](crate::CalibrationProject::attach_dbc)
//! produces one [`SignalBinding`] per characteristic that has a
//! same-named signal, and this module is where its bits are coded: the
//! Motorola sawtooth and the Intel linear layout both come from
//! `dbc_parse::bits`, the estate's single implementation of the two DBC
//! bit-numbering systems.

use crate::CalError;
use dbc_parse::ByteOrder;

/// A resolved binding between a CHARACTERISTIC and a DBC signal.
#[derive(Debug, Clone, PartialEq)]
pub struct SignalBinding {
    /// The CHARACTERISTIC this binding describes.
    pub characteristic: String,
    /// The message's CAN identifier, widened to `u32`
    /// (`dbc_parse::CanId::raw`).
    pub can_id: u32,
    /// DBC start bit: the MSB for [`ByteOrder::Motorola`], the LSB for
    /// [`ByteOrder::Intel`]. `u16` because DBC numbers up to 511 (CAN FD
    /// payloads reach byte 64).
    pub start_bit: u16,
    /// Signal width in bits (1–64).
    pub length: u8,
    /// Bit layout order (`@0`/`@1` in the DBC grammar).
    pub byte_order: ByteOrder,
    /// Physical value = `raw * factor + offset`.
    pub factor: f64,
    /// Physical value = `raw * factor + offset`.
    pub offset: f64,
}

impl SignalBinding {
    /// Build a binding from a DBC signal in message `can_id`.
    #[must_use]
    pub fn from_signal(characteristic: &str, can_id: u32, signal: &dbc_parse::Signal) -> Self {
        Self {
            characteristic: characteristic.to_string(),
            can_id,
            start_bit: signal.start_bit,
            length: signal.bit_length,
            byte_order: signal.byte_order,
            factor: signal.scale,
            offset: signal.offset,
        }
    }

    /// Bytes a payload must have for this layout.
    #[must_use]
    pub fn payload_len(&self) -> usize {
        dbc_parse::bits::max_bit_position(self.start_bit, self.length, self.byte_order) / 8 + 1
    }

    /// The raw bit pattern the signal occupies in `data`.
    ///
    /// Total: positions beyond the buffer read as zero bits (the same
    /// documented semantics `dbc_parse` uses), so a truncated capture
    /// decodes rather than panicking.
    #[must_use]
    pub fn extract_raw(&self, data: &[u8]) -> u64 {
        dbc_parse::bits::extract_raw(self.start_bit, self.length, self.byte_order, data)
    }

    /// Decode the signal's physical value from a payload.
    #[must_use]
    pub fn decode(&self, data: &[u8]) -> f64 {
        #[allow(clippy::cast_precision_loss)]
        {
            self.extract_raw(data) as f64 * self.factor + self.offset
        }
    }

    /// Encode a physical value into `data` at this signal's bits — the
    /// exact inverse of [`decode`](Self::decode), modulo the rounding to
    /// the signal's raw resolution.
    ///
    /// # Errors
    ///
    /// [`CalError::Unsupported`] when `factor` is zero (nothing can be
    /// inverted), [`CalError::Dbc`] when the raw value does not fit the
    /// signal's width or the buffer is too short.
    pub fn encode(&self, data: &mut [u8], physical: f64) -> Result<(), CalError> {
        if self.factor == 0.0 || !self.factor.is_finite() {
            return Err(CalError::Unsupported {
                subject: format!("signal `{}`", self.characteristic),
                reason: format!("factor {} is not invertible", self.factor),
            });
        }
        // f64::round is half-away-from-zero, the convention CAN tooling uses.
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let raw = ((physical - self.offset) / self.factor).round() as u64;
        if raw >= (1_u128 << u32::from(self.length)) && u32::from(self.length) < 64 {
            return Err(CalError::Dbc(dbc_parse::DbcError::RawOutOfRange {
                raw: raw as f64,
                bit_length: self.length,
            }));
        }
        dbc_parse::bits::insert_raw(self.start_bit, self.length, self.byte_order, data, raw)
            .map_err(CalError::Dbc)
    }

    /// Decode every binding of `bindings` from one payload, as
    /// `(can_id, name, physical)` triples in the order given.
    #[must_use]
    pub fn decode_all(bindings: &[Self], data: &[u8]) -> Vec<(u32, String, f64)> {
        bindings
            .iter()
            .map(|b| (b.can_id, b.characteristic.clone(), b.decode(data)))
            .collect()
    }
}

impl std::fmt::Display for SignalBinding {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let order = match self.byte_order {
            ByteOrder::Motorola => "motorola",
            ByteOrder::Intel => "intel",
        };
        write!(
            f,
            "0x{:03X} {} start_bit {} len {} {order} factor {} offset {}",
            self.can_id,
            self.characteristic,
            self.start_bit,
            self.length,
            self.factor,
            self.offset
        )
    }
}
