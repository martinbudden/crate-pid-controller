use core::ops::AddAssign;
use num_traits::{ConstOne, ConstZero, float::FloatCore};

use crate::{PdErrors, PdGains};

#[cfg(feature = "storage")]
use sequential_storage::map::PostcardValue;
#[cfg(feature = "serde")]
use {
    postcard::experimental::max_size::MaxSize,
    serde::{Deserialize, Serialize},
};

/// `PdController` using `f32` values.
pub type PdControllerf32 = PdController<f32>;

/// `PdController` using `f64` values.
pub type PdControllerf64 = PdController<f64>;

/// PD controller.<br>
/// `PdControllerf32` and `PdControllerf64` aliases are available.<br>
///
/// Uses "independent PID" notation, where the gains are denoted as kp and kd.<br>
/// (In the "dependent PID" notation `kc` and `tau_d` parameters are used, where `kp = kc` and `kd = kc*tau_d`).
///
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize, MaxSize))]
pub struct PdController<T> {
    gains: PdGains<T>,
    measurement_previous: T,

    setpoint: T,

    error: T,
    error_derivative: T,
}

#[cfg(feature = "storage")]
impl<T> PostcardValue<'_> for PdController<T> where T: Serialize + MaxSize + for<'de> Deserialize<'de> {}

/// Default `PdController`.
/// ```
/// # use pidsk_controller::PdControllerf32;
/// # use num_traits::Zero;
///
/// let pd_controller = PdControllerf32::default();
///
/// assert_eq!(1.0, pd_controller.gains().kp);
/// ```
impl<T: FloatCore + AddAssign + ConstZero + ConstOne> Default for PdController<T> {
    fn default() -> Self {
        Self::new()
    }
}

// Constructors
impl<T: FloatCore + AddAssign + ConstZero + ConstOne> PdController<T> {
    /// Constructor.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            gains: PdGains::new(),
            measurement_previous: T::ZERO,
            setpoint: T::ZERO,
            error: T::ZERO,
            error_derivative: T::ZERO,
        }
    }

    /// Set the gains of a newly constructed `PdController`.
    #[must_use]
    pub fn with_gains(mut self, gains: PdGains<T>) -> Self {
        self.set_gains(gains);
        self
    }

    /// Set the `kp` of a newly constructed `PdController`.
    #[must_use]
    pub fn with_kp(mut self, kp: T) -> Self {
        self.gains.kp = kp;
        self
    }

    /// Set the `kd` of a newly constructed `PdController`.
    #[must_use]
    pub fn with_kd(mut self, kd: T) -> Self {
        self.gains.kd = kd;
        self
    }
}

impl<T: FloatCore + AddAssign> PdController<T> {
    /// PD-controller update.
    /// ```
    /// # use pidsk_controller::{PdControllerf32, PdGainsf32};
    /// let dt: f32 = 0.01;
    /// let mut pd_controller = PdControllerf32::new().with_kp(0.1);
    ///
    /// pd_controller.set_setpoint(8.7);
    ///
    /// let measurement:f32 = 9.2;
    /// let output = pd_controller.update(measurement, dt);
    ///
    /// assert_eq!(-0.05, output);
    /// ```
    #[inline]
    pub fn update(&mut self, measurement: T, dt: T) -> T {
        self.update_delta(measurement, measurement - self.measurement_previous, dt)
    }

    /// Update with `measurement_delta` specified.
    /// This allows the user to filter `measurement_delta` with a filter of their choice.
    ///
    /// ```
    /// # use pidsk_controller::{PdControllerf32, PdGainsf32};
    /// # use signal_filters::{Pt1Filterf32, UpdateFilter};
    /// let dt: f32 = 0.01;
    /// let mut pd_controller = PdControllerf32::new().with_kp(0.1).with_kd(0.01);
    /// let mut dterm_filter = Pt1Filterf32::new().with_k(1.0);
    ///
    /// pd_controller.set_setpoint(2.1);
    ///
    /// let measurement:f32 = 0.2;
    ///
    /// let output = pd_controller.update_delta(measurement,
    ///     (measurement - pd_controller.previous_measurement()).filter_using(&mut dterm_filter),
    ///     dt,
    /// );
    ///
    /// assert_eq!(-0.010_000_005, output);
    ///
    pub fn update_delta(&mut self, measurement: T, measurement_delta: T, dt: T) -> T {
        debug_assert!(dt != T::zero());

        self.measurement_previous = measurement;

        self.error = self.setpoint - measurement;
        self.error_derivative = -measurement_delta / dt; // note minus sign, error delta has reverse polarity to measurement delta

        // The PD calculation
        self.gains.kp * self.error + self.gains.kd * self.error_derivative
    }

    /// Optimized form of `update` that assumes all gains except `kp` are zero.
    #[inline]
    pub fn update_p(&mut self, measurement: T) -> T {
        self.measurement_previous = measurement;
        self.error = self.setpoint - measurement;

        // The P (no I, no D) term
        //         P
        self.gains.kp * self.error
    }

    /// Safely update gains on the fly without causing an output bump.
    pub fn update_gains(&mut self, new_gains: PdGains<T>) {
        self.set_gains(new_gains);
    }
}

impl<T: FloatCore> PdController<T> {
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

    /// Sets the previous measurement.
    #[inline]
    pub fn set_initial_measurement(&mut self, measurement: T) {
        self.measurement_previous = measurement;
    }

    /// Returns the previous measurement, useful for `Dterm` filtering.
    #[inline]
    pub fn previous_measurement(&self) -> T {
        self.measurement_previous
    }

    /// Return the gains.
    #[inline]
    pub fn gains(&self) -> PdGains<T> {
        PdGains {
            kp: self.gains.kp,
            kd: self.gains.kd,
        }
    }

    /// Set the gains.
    #[inline]
    pub fn set_gains(&mut self, gains: PdGains<T>) {
        self.gains = gains;
    }
}

impl<T: FloatCore + Default> From<PdGains<T>> for PdController<T> {
    fn from(pid: PdGains<T>) -> Self {
        Self {
            gains: PdGains { kp: pid.kp, kd: pid.kd },
            measurement_previous: T::default(),
            setpoint: T::default(),
            error_derivative: T::default(),
            error: T::default(),
        }
    }
}

/// Accessor functions to obtain error values.
impl<T: FloatCore> PdController<T> {
    /// Returns the errors multiplied by the gains.
    pub fn error(&self) -> PdErrors<T> {
        PdErrors {
            p: self.error * self.gains.kp,
            d: self.error_derivative * self.gains.kd,
        }
    }

    /// Returns the raw values of the errors, ie NOT multiplied by the gains.
    pub fn error_raw(&self) -> PdErrors<T> {
        PdErrors {
            p: self.error,
            d: self.error_derivative,
        }
    }

    /// Returns the error, for test code.
    pub fn previous_error(&self) -> T {
        self.error
    }

    /// Clears historical record to allow bumpless transfer.
    pub fn reset(&mut self) {
        self.measurement_previous = self.setpoint;
        self.error = T::zero();
        self.error_derivative = T::zero();
    }

    /// Reset all, for test code.
    pub fn reset_all(&mut self) {
        self.measurement_previous = T::zero();
        self.setpoint = T::zero();
        self.error = T::zero();
        self.error_derivative = T::zero();
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
        is_full::<PdControllerf32>();
        #[cfg(feature = "serde")]
        is_serde::<PdControllerf32>();
        #[cfg(feature = "storage")]
        is_storage::<PdControllerf32>();
    }
}
