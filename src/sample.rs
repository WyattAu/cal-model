//! A worked example: the two-ECU powertrain description this crate's tests,
//! documentation, and `powertrain` binary all run against.
//!
//! Embedding the A2L and DBC text means the examples and doctests cannot
//! drift from a file on disk, and that `cargo test` needs no fixtures. The
//! same text is written to `data/powertrain.a2l` for a human to read.

/// The sample A2L description: two modules, sixteen characteristics spanning
/// all five `CHARACTERISTIC` types, six measurements, and a `COMPU_METHOD`
/// set covering every conversion type — `LINEAR`, `RAT_FUNC` (both the
/// reducible-linear and the genuinely quadratic forms), `TABLE` (with
/// `TAB_INTP` and `TAB_VERB` tabs), and `IDENTITY`.
pub const SAMPLE_A2L: &str = include_str!("../data/powertrain.a2l");

/// The sample CAN database, binding the engine and transmission
/// characteristics to the signals that carry them.
pub const SAMPLE_DBC: &str = include_str!("../data/powertrain.dbc");

/// One seed value: the raw count an ECU ships with, for one characteristic.
struct Seed(&'static str, u64);

/// The engine ECU's shipped calibration page (page 0).
///
/// Chosen so every converted value is a round number in its declared unit,
/// which is what makes a read-modify-write assertion exact rather than
/// approximate.
const ENGINE_SEEDS: &[Seed] = &[
    // 600 counts * 0.1 %/count - 10 % = 50.0 %
    Seed("eng_load", 600),
    // (0.5 * 2000 + 100) / 2 = 550 Nm
    Seed("eng_torque_max", 2000),
    // 3200 counts * 0.25 rpm/count = 800 rpm
    Seed("idle_target_rpm", 3200),
    // 0.0001 * 400^2 + 0.5 * 400 = 216 kPa
    Seed("boost_target", 400),
    // identity
    Seed("rev_limit_cut", 5),
];

/// The transmission ECU's shipped calibration page (page 1).
const TRANSMISSION_SEEDS: &[Seed] = &[
    // 12000 counts * 1e-4 = 1.2
    Seed("primary_ratio", 12000),
    // 200 / 4 = 50 kPa
    Seed("shift_pressure", 200),
    // 0.5 * 120 - 20 = 40 ms
    Seed("shift_time_ms", 120),
    // 0.0005 * 100^2 + 0.2 * 100 = 25 N
    Seed("clutch_force", 100),
    // TAB_INTP over gear_pos_tab, input 25 -> 2
    Seed("gear_code", 25),
];

/// A seed deposit: a characteristic's raw count, for the mock transport.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SeedValue {
    /// The module the characteristic lives in.
    pub module: &'static str,
    /// The characteristic name.
    pub name: &'static str,
    /// The raw count the ECU ships with.
    pub raw: u64,
}

/// Every seed value, engine first.
#[must_use]
pub fn seed_values() -> Vec<SeedValue> {
    ENGINE_SEEDS
        .iter()
        .map(|seed| SeedValue {
            module: "engine",
            name: seed.0,
            raw: seed.1,
        })
        .chain(TRANSMISSION_SEEDS.iter().map(|seed| SeedValue {
            module: "transmission",
            name: seed.0,
            raw: seed.1,
        }))
        .collect()
}

/// The base address of the engine ECU's calibration page.
pub const ENGINE_BASE: u32 = 0x0072_0000;
/// The base address of the transmission ECU's calibration page.
pub const TRANSMISSION_BASE: u32 = 0x0073_0000;
/// The page number the engine's calibration lives on.
pub const ENGINE_PAGE: u32 = 0;
/// The page number the transmission's calibration lives on.
pub const TRANSMISSION_PAGE: u32 = 1;
/// Bytes mapped per page — enough for every characteristic in the module.
pub const PAGE_LEN: usize = 0x400;

/// A bench transport seeded with [`seed_values`], one page per ECU module.
///
/// Page 0 carries the engine addresses and page 1 the transmission ones, so
/// switching pages in a test exercises the same isolation a real
/// `SET_CAL_PAGE` does.
///
/// # Errors
///
/// [`CalError::UnknownModule`] or [`CalError::UnknownCharacteristic`] when a
/// seed names something the description does not declare — a fixture that
/// has drifted out of sync with its A2L, which must fail loudly.
///
/// # Panics
///
/// Never: every path returns a typed error.
pub fn seeded_transport(
    project: &crate::CalibrationProject,
) -> Result<crate::MockTransport, crate::CalError> {
    let mut transport = crate::MockTransport::empty();
    transport.insert_page(
        ENGINE_PAGE,
        crate::mock::MemoryPage::new(ENGINE_BASE, PAGE_LEN),
    );
    transport.insert_page(
        TRANSMISSION_PAGE,
        crate::mock::MemoryPage::new(TRANSMISSION_BASE, PAGE_LEN),
    );
    for seed in seed_values() {
        let characteristic = project.characteristic(seed.module, seed.name)?;
        let size = characteristic.deposit_size().unwrap_or(2);
        let offset = usize::try_from(characteristic.address - page_base(seed.module))
            .map_err(|_| crate::CalError::Transport("seed address underflow".to_owned()))?;
        let page = if seed.module == "engine" {
            ENGINE_PAGE
        } else {
            TRANSMISSION_PAGE
        };
        let bytes = to_little_endian(seed.raw, size);
        transport.select_page(page);
        transport.write(characteristic.address, &bytes)?;
        let _ = offset;
    }
    transport.select_page(ENGINE_PAGE);
    Ok(transport)
}

/// The base address of a module's calibration page.
#[must_use]
pub fn page_base(module: &str) -> u32 {
    if module == "transmission" {
        TRANSMISSION_BASE
    } else {
        ENGINE_BASE
    }
}

/// The page number a module's calibration lives on.
#[must_use]
pub fn page_of(module: &str) -> u32 {
    if module == "transmission" {
        TRANSMISSION_PAGE
    } else {
        ENGINE_PAGE
    }
}

/// A raw count as a little-endian buffer of `size` bytes.
fn to_little_endian(value: u64, size: usize) -> Vec<u8> {
    (0..size)
        .map(|index| {
            if index >= 8 {
                return 0;
            }
            #[allow(clippy::cast_possible_truncation)]
            let shift = 8 * u32::try_from(index).unwrap_or(0);
            #[allow(clippy::cast_possible_truncation)]
            {
                (value >> shift) as u8
            }
        })
        .collect()
}
