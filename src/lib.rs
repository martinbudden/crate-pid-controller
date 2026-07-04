#![doc = include_str!("../README.md")]
#![no_std]
#![deny(clippy::unwrap_used)]
#![deny(clippy::expect_used)]
#![deny(clippy::panic)]
#![deny(missing_docs)]
#![deny(
    missing_copy_implementations,
    missing_debug_implementations,
    trivial_casts,
    trivial_numeric_casts,
    unused_must_use,
    unused_extern_crates,
    unused_import_braces,
    unused_qualifications,
    unused_results
)]
#![warn(unused_results)]
#![warn(clippy::pedantic)]
#![warn(clippy::doc_paragraphs_missing_punctuation)]

mod pidsk_controller;

pub use pidsk_controller::{PidController, PidControllerf32, PidControllerf64};
pub use pidsk_controller::{PidErrorf32, PidErrorf64, PidErrors};
pub use pidsk_controller::{PidGains, PidGainsf32, PidGainsf64};
pub use pidsk_controller::{PidLimits, PidLimitsf32, PidLimitsf64};
