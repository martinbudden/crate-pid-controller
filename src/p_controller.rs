use core::ops::AddAssign;
use num_traits::{ConstOne, ConstZero, float::FloatCore};

use crate::{PErrors, PGains};

#[cfg(feature = "storage")]
use sequential_storage::map::PostcardValue;
#[cfg(feature = "serde")]
use {
    postcard::experimental::max_size::MaxSize,
    serde::{Deserialize, Serialize},
};

/// `PController` using `f32` values.
pub type PControllerf32 = PController<f32>;

/// `PController` using `f64` values.
pub type PControllerf64 = PController<f64>;

/// Pure P-controller.
/// `PControllerf32` and `PControllerf64` aliases are available.<br>
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

    /// Set the gains of a newly constructed `PController`.
    #[must_use]
    pub fn with_gains(mut self, gains: PGains<T>) -> Self {
        self.set_gains(gains);
        self
    }

    /// Set the `kp` of a newly constructed `PController`.
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

    /// Safely update gains on the fly without causing an output bump.
    pub fn update_gains(&mut self, new_gains: PGains<T>) {
        self.set_gains(new_gains);
    }
}

impl<T: FloatCore> PController<T> {
    /// Set the setpoint.
    #[inline]
    pub fn set_setpoint(&mut self, setpoint: T) {
        self.setpoint = setpoint;
    }

    /// Return the setpoint.
    #[inline]
    pub fn setpoint(&self) -> T {
        self.setpoint
    }

    /// Return the gains.
    #[inline]
    pub fn gains(&self) -> PGains<T> {
        PGains { kp: self.gains.kp }
    }

    /// Set the gains.
    #[inline]
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

/// Accessor functions to obtain error values.
impl<T: FloatCore> PController<T> {
    /// Returns the error multiplied by the gain.
    pub fn error(&self) -> PErrors<T> {
        PErrors {
            p: self.error * self.gains.kp,
        }
    }

    /// Returns the raw values of the error, ie NOT multiplied by the gain.
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

#[cfg(test)]
mod test_traits {
    use super::*;

    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}
    #[cfg(feature = "serde")]
    fn is_serde<T: Serialize + MaxSize + for<'a> Deserialize<'a>>() {}
    #[cfg(feature = "storage")]
    fn is_storage<T: for<'a> PostcardValue<'a>>() {}

    #[test]
    fn normal_types() {
        is_full::<PControllerf32>();
        #[cfg(feature = "serde")]
        is_serde::<PControllerf32>();
        #[cfg(feature = "storage")]
        is_storage::<PControllerf32>();
    }
}
