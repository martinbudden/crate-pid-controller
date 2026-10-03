use core::ops::AddAssign;
use num_traits::{ConstOne, ConstZero, float::FloatCore};

use crate::PGains;

#[cfg(feature = "storage")]
use sequential_storage::map::PostcardValue;
#[cfg(feature = "serde")]
use {
    postcard::experimental::max_size::MaxSize,
    serde::{Deserialize, Serialize},
};

/// P-controller using `f32` values.
pub type PControllerf32 = PController<f32>;

/// P-controller using `f64` values.
pub type PControllerf64 = PController<f64>;

/// Pure P-controller.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize, MaxSize))]
pub struct PController<T> {
    gains: PGains<T>,

    setpoint: T,

    error: T,
}

#[cfg(feature = "storage")]
impl<T> PostcardValue<'_> for PController<T> where T: Serialize + MaxSize + for<'de> Deserialize<'de> {}

/// Default `PController`.
/// ```
/// # use pidsk_controller::PControllerf32;
/// # use num_traits::Zero;
///
/// let p_controller = PControllerf32::default();
///
/// assert_eq!(1.0, p_controller.gains().kp);
/// ```
impl<T: FloatCore + AddAssign + ConstZero + ConstOne> Default for PController<T> {
    fn default() -> Self {
        Self::new()
    }
}

// Constructors
impl<T: FloatCore + AddAssign + ConstZero + ConstOne> PController<T> {
    /// Constructor.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            gains: PGains::new(),
            setpoint: T::ZERO,
            error: T::ZERO,
        }
    }
    /// Set the gains of a newly constructed P-controller.
    #[must_use]
    pub fn with_gains(mut self, gains: PGains<T>) -> Self {
        self.set_gains(gains);
        self
    }

    /// Set the `kp` of a newly constructed P-controller.
    #[must_use]
    pub fn with_kp(mut self, kp: T) -> Self {
        self.gains.kp = kp;
        self
    }
}

impl<T: FloatCore + AddAssign> PController<T> {
    /// P-controller update.
    /// ```
    /// # use pidsk_controller::{PControllerf32, PGainsf32};
    /// let mut p_controller = PControllerf32::new().with_kp(0.1);
    ///
    /// p_controller.set_setpoint(8.7);
    ///
    /// let measurement:f32 = 9.2;
    /// let output = p_controller.update(measurement);
    ///
    /// assert_eq!(-0.05, output);
    /// ```
    #[inline]
    pub fn update(&mut self, measurement: T) -> T {
        self.error = self.setpoint - measurement;

        self.gains.kp * self.error
    }

    /// Set the setpoint.
    pub fn set_setpoint(&mut self, setpoint: T) {
        self.setpoint = setpoint;
    }

    /// Return the setpoint.
    pub fn setpoint(&self) -> T {
        self.setpoint
    }

    /// Safely update gains on the fly without causing an output bump.
    pub fn update_gains(&mut self, new_gains: PGains<T>) {
        self.set_gains(new_gains);
    }
}

impl<T: FloatCore> PController<T> {
    /// Return the gains.
    pub fn gains(&self) -> PGains<T> {
        PGains { kp: self.gains.kp }
    }

    /// Set the gains.
    pub fn set_gains(&mut self, gains: PGains<T>) {
        self.gains = gains;
    }
}

impl<T: FloatCore + Default> From<PGains<T>> for PController<T> {
    fn from(pid: PGains<T>) -> Self {
        Self {
            gains: PGains { kp: pid.kp },
            setpoint: T::default(),
            error: T::default(),
        }
    }
}

/// P error as calculated by P-controller.<br><br>
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize, MaxSize))]
#[allow(missing_docs)]
pub struct PErrors<T> {
    pub p: T,
}

#[cfg(feature = "storage")]
impl<T> PostcardValue<'_> for PErrors<T> where T: Serialize + MaxSize + for<'de> Deserialize<'de> {}

impl<T: FloatCore> Default for PErrors<T> {
    fn default() -> Self {
        Self::new(T::zero())
    }
}

impl<T: FloatCore> PErrors<T> {
    /// Constructor.
    #[allow(clippy::many_single_char_names)]
    pub const fn new(p: T) -> Self {
        Self { p }
    }
}

/// Accessor functions to obtain error values.
impl<T: FloatCore> PController<T> {
    /// Returns the PID errors, multiplied by the PID gains.
    pub fn error(&self) -> PErrors<T> {
        PErrors {
            p: self.error * self.gains.kp,
        }
    }

    /// Returns the raw values of the P-errors, ie NOT multiplied by the P-gain.
    pub fn error_raw(&self) -> PErrors<T> {
        PErrors { p: self.error }
    }

    /// Returns the error, for test code.
    pub fn previous_error(&self) -> T {
        self.error
    }

    /// Clears historical record to allow bumpless transfer.
    pub fn reset(&mut self) {
        self.error = T::zero();
    }

    /// Reset all, for test code.
    pub fn reset_all(&mut self) {
        self.setpoint = T::zero();
        self.error = T::zero();
    }
}
