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

mod pid_limits;

mod pidsk_controller;
mod pidsk_errors;
mod pidsk_gains;

mod pid_controller;
mod pid_errors;
mod pid_gains;

mod p_controller;
mod p_errors;
mod p_gains;

mod pd_controller;
mod pd_errors;
mod pd_gains;

pub use pid_limits::{PidLimits, PidLimitsf32, PidLimitsf64};

pub use pidsk_controller::{PidskController, PidskControllerf32, PidskControllerf64};
pub use pidsk_errors::{PidskErrors, PidskErrorsf32, PidskErrorsf64};
pub use pidsk_gains::{PidskGains, PidskGainsf32, PidskGainsf64};

pub use pid_controller::{PidController, PidControllerf32, PidControllerf64};
pub use pid_errors::{PidErrors, PidErrorsf32, PidErrorsf64};
pub use pid_gains::{PidGains, PidGainsf32, PidGainsf64};

pub use p_controller::{PController, PControllerf32, PControllerf64};
pub use p_errors::{PErrors, PErrorsf32, PErrorsf64};
pub use p_gains::{PGains, PGainsf32, PGainsf64};

pub use pd_controller::{PdController, PdControllerf32, PdControllerf64};
pub use pd_errors::{PdErrors, PdErrorsf32, PdErrorsf64};
pub use pd_gains::{PdGains, PdGainsf32, PdGainsf64};
