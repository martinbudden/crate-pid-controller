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

mod pid_errors;
mod pid_gains;
mod pid_limits;
mod pidsk_controller;

pub use pid_errors::{PidErrors, PidErrorsf32, PidErrorsf64};
pub use pid_gains::{PidGains, PidGainsf32, PidGainsf64};
pub use pid_limits::{PidLimits, PidLimitsf32, PidLimitsf64};
pub use pidsk_controller::{PidController, PidControllerf32, PidControllerf64};
