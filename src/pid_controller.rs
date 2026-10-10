use core::ops::AddAssign;
use num_traits::{ConstOne, ConstZero, float::FloatCore};

use crate::{PidErrors, PidGains, PidLimits};

#[cfg(feature = "storage")]
use sequential_storage::map::PostcardValue;
#[cfg(feature = "serde")]
use {
    postcard::experimental::max_size::MaxSize,
    serde::{Deserialize, Serialize},
};

/// `PidController` using `f32` values.
pub type PidControllerf32 = PidController<f32>;

/// `PidController` using `f64` values.
pub type PidControllerf64 = PidController<f64>;

/// PID controller.<br>
/// `PidControllerf32` and `PidControllerf64` aliases are available.<br>
///
/// Uses "independent PID" notation, where the gains are denoted as kp, ki, kd etc.<br>
/// (In the "dependent PID" notation `kc`, `tau_i`, and `tau_d` parameters are used, where `kp = kc`, `ki = kc/tau_i`, `kd = kc*tau_d`).
///
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize, MaxSize))]
pub struct PidController<T> {
    gains: PidGains<T>,
    limits: PidLimits<T>,
    /// saved value of pid.ki, so integration can be switched on and off.
    ki_saved: T,
    measurement_previous: T,

    setpoint: T,
    setpoint_previous: T,
    setpoint_derivative: T,

    error: T,
    error_integral: T,
    error_derivative: T,
}

#[cfg(feature = "storage")]
impl<T> PostcardValue<'_> for PidController<T> where T: Serialize + MaxSize + for<'de> Deserialize<'de> {}

/// Default `PidController`.
/// ```
/// # use pidsk_controller::PidControllerf32;
/// # use num_traits::Zero;
///
/// let pid = PidControllerf32::default();
///
/// assert_eq!(1.0, pid.gains().kp);
/// ```
impl<T: FloatCore + AddAssign + ConstZero + ConstOne> Default for PidController<T> {
    fn default() -> Self {
        Self::new()
    }
}

// Constructors
impl<T: FloatCore + AddAssign + ConstZero + ConstOne> PidController<T> {
    /// Constructor.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            gains: PidGains::new(),
            limits: PidLimits::new(),
            ki_saved: T::ZERO,
            measurement_previous: T::ZERO,
            setpoint: T::ZERO,
            setpoint_previous: T::ZERO,
            setpoint_derivative: T::ZERO,
            error: T::ZERO,
            error_integral: T::ZERO,
            error_derivative: T::ZERO,
        }
    }

    /// Set the gains of a newly constructed `PidController`.
    #[must_use]
    pub fn with_gains(mut self, gains: PidGains<T>) -> Self {
        self.set_gains(gains);
        self
    }

    /// Set the `kp` of a newly constructed `PidController`.
    #[must_use]
    pub fn with_kp(mut self, kp: T) -> Self {
        self.gains.kp = kp;
        self
    }

    /// Set the `ki` of a newly constructed `PidController`.
    #[must_use]
    pub fn with_ki(mut self, ki: T) -> Self {
        self.gains.ki = ki;
        self
    }

    /// Set the `kd` of a newly constructed `PidController`.
    #[must_use]
    pub fn with_kd(mut self, kd: T) -> Self {
        self.gains.kd = kd;
        self
    }

    /// Set the limits of a newly constructed `PidController`.
    #[must_use]
    pub fn with_limits(mut self, limits: PidLimits<T>) -> Self {
        self.set_limits(limits);
        self
    }

    /// Set the limits of a newly constructed `PidController`.
    #[must_use]
    pub fn with_integral_limits(mut self, integral_max: T, integral_min: T) -> Self {
        self.limits.integral_max = Some(integral_max);
        self.limits.integral_min = Some(integral_min);
        self
    }

    /// Set the the max and min integral limits of a newly constructed `PidController` to +/- `integral_limit`.
    #[must_use]
    pub fn with_integral_limit(mut self, integral_limit: T) -> Self {
        self.limits.integral_max = Some(integral_limit);
        self.limits.integral_min = Some(-integral_limit);
        self
    }

    /// Set the output saturation value of a newly constructed `PidController`.
    #[must_use]
    pub fn with_output_saturation(mut self, output_saturation: T) -> Self {
        self.limits.output_saturation = Some(output_saturation);
        self
    }
}

impl<T: FloatCore + AddAssign> PidController<T> {
    /// PID-controller update.
    /// ```
    /// # use pidsk_controller::{PidControllerf32, PidGainsf32};
    /// let dt: f32 = 0.01;
    /// let mut pid_controller = PidControllerf32::new().with_kp(0.1);
    ///
    /// pid_controller.set_setpoint(8.7);
    ///
    /// let measurement:f32 = 9.2;
    /// let output = pid_controller.update(measurement, dt);
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
    /// # use pidsk_controller::{PidControllerf32, PidGainsf32};
    /// # use signal_filters::{Pt1Filterf32, UpdateFilter};
    /// let dt: f32 = 0.01;
    /// let mut pid_controller = PidControllerf32::new().with_kp(0.1).with_kd(0.01);
    /// let mut dterm_filter = Pt1Filterf32::new().with_k(1.0);
    ///
    /// pid_controller.set_setpoint(2.1);
    ///
    /// let measurement:f32 = 0.2;
    ///
    /// let output = pid_controller.update_delta(measurement,
    ///     (measurement - pid_controller.previous_measurement()).filter_using(&mut dterm_filter),
    ///     dt,
    /// );
    ///
    /// assert_eq!(-0.010_000_005, output);
    ///
    #[inline]
    pub fn update_delta(&mut self, measurement: T, measurement_delta: T, dt: T) -> T {
        self.update_delta_iterm(measurement, measurement_delta, self.setpoint - measurement, dt)
    }

    /// PID update with `measurement_delta` and `iterm_error` specified.
    /// This allows the user to:
    /// 1. filter `measurement_delta` with a filter of their choice.
    /// 2. dynamically alter the Iterm.
    ///
    pub fn update_delta_iterm(&mut self, measurement: T, measurement_delta: T, iterm_error: T, dt: T) -> T {
        debug_assert!(dt != T::zero());

        self.measurement_previous = measurement;

        self.error = self.setpoint - measurement;
        self.error_derivative = -measurement_delta / dt; // note minus sign, error delta has reverse polarity to measurement delta

        // Perform Euler integration
        self.error_integral += self.gains.ki * iterm_error * dt;

        // Anti-windup via integral clamping
        if let Some(max) = self.limits.integral_max {
            self.error_integral = self.error_integral.min(max);
        }
        if let Some(min) = self.limits.integral_min {
            self.error_integral = self.error_integral.max(min);
        }

        let partial_sum = self.partial_sum();

        // Anti-windup clamping based on output saturation limit
        if let Some(output_saturation) = self.limits.output_saturation {
            // Determine dynamic upper and lower boundaries for the integral term
            let max_allowed_integral = output_saturation - partial_sum;
            let min_allowed_integral = -output_saturation - partial_sum;

            // Clamp the accumulator within the calculated window
            self.error_integral = self.error_integral.clamp(min_allowed_integral, max_allowed_integral);
        }

        // The PID calculation with additional S setpoint(openloop) and K kick(setpoint derivative) terms
        // P+D+S+K  + I
        partial_sum + self.error_integral
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

    /// Partial PID sum helper function, excludes `Iterm`,
    /// has additional S setpoint(openloop) and K kick(setpoint derivative) terms.
    #[inline]
    pub fn partial_sum(&self) -> T {
        self.gains.kp * self.error + self.gains.kd * self.error_derivative
    }

    /// Reset the PID integral to zero.
    #[inline]
    pub fn reset_integral(&mut self) {
        self.error_integral = T::zero();
    }

    /// Switch PID integration off by saving `ki` and then setting `ki` to zero.
    #[inline]
    pub fn switch_integration_off(&mut self) {
        self.ki_saved = self.gains.ki;
        self.gains.ki = T::zero();
        self.error_integral = T::zero();
    }

    /// Switch PID integration back on, restoring the previously saved value of `ki`.
    #[inline]
    pub fn switch_integration_on(&mut self) {
        self.gains.ki = self.ki_saved;
        self.error_integral = T::zero();
    }

    /// Safely update gains on the fly without causing an output bump.
    pub fn update_gains(&mut self, new_gains: PidGains<T>) {
        // Calculate the current components using OLD gains
        let old_partial_sum = self.partial_sum();
        let old_error_integral = self.error_integral;

        // Commit the new gains to the struct
        self.set_gains(new_gains);
        if self.error_integral == T::zero() {
            return;
        }

        // Calculate partial_sum using NEW gains
        let new_partial_sum = self.partial_sum();

        self.error_integral = old_error_integral + old_partial_sum - new_partial_sum;

        // Force-clamp the newly calculated accumulator within the Option bounds
        if let Some(output_saturation) = self.limits.output_saturation {
            let max = output_saturation - new_partial_sum;
            let min = -output_saturation - new_partial_sum;
            self.error_integral = self.error_integral.clamp(min, max);
        }

        if let Some(max) = self.limits.integral_max {
            self.error_integral = self.error_integral.min(max);
        }
        if let Some(min) = self.limits.integral_min {
            self.error_integral = self.error_integral.max(min);
        }
    }
}

impl<T: FloatCore> PidController<T> {
    /// Set the setpoint, saving the previous setpoint.
    #[inline]
    pub fn set_setpoint(&mut self, setpoint: T) {
        self.setpoint_previous = self.setpoint;
        self.setpoint = setpoint;
    }

    /// Directly set the setpoint derivative.
    #[inline]
    pub fn set_setpoint_derivative(&mut self, setpoint_derivative: T) {
        self.setpoint_derivative = setpoint_derivative;
    }

    /// Set the setpoint and calculate the setpoint derivative.
    #[inline]
    pub fn set_setpoint_for_delta_t(&mut self, setpoint: T, dt: T) {
        debug_assert!(dt != T::zero());
        self.setpoint_previous = self.setpoint;
        self.setpoint = setpoint;
        self.setpoint_derivative = (self.setpoint - self.setpoint_previous) / dt;
    }

    /// Return the setpoint.
    #[inline]
    pub fn setpoint(&self) -> T {
        self.setpoint
    }

    /// Returns the previous setpoint.
    pub fn previous_setpoint(&self) -> T {
        self.setpoint_previous
    }

    /// Returns the difference between the setpoint and the previous setpoint.
    pub fn setpoint_delta(&self) -> T {
        self.setpoint - self.setpoint_previous
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

    /// Return the gains. The set value of ki is returned, whether integration is turned on or not.
    #[inline]
    pub fn gains(&self) -> PidGains<T> {
        PidGains {
            kp: self.gains.kp,
            ki: self.ki_saved,
            kd: self.gains.kd,
        }
    }

    /// Set the gains, setting `error_integral` to zero if `ki` is zero.
    #[inline]
    pub fn set_gains(&mut self, gains: PidGains<T>) {
        self.gains = gains;
        self.ki_saved = self.gains.ki;
        if self.gains.ki.abs() <= T::epsilon() {
            // If the new Ki is zero, the integral term cannot contribute to the output.
            // Clear the accumulator to prevent massive hidden windup if Ki is re-enabled later.
            self.error_integral = T::zero();
        }
    }

    /// Set the `kp` gain.
    #[inline]
    pub fn set_kp(&mut self, kp: T) {
        self.gains.kp = kp;
    }

    /// Set the `ki` gain, setting `error_integral` to zero if `ki` is zero.
    #[inline]
    pub fn set_ki(&mut self, ki: T) {
        self.gains.ki = ki;
        self.ki_saved = self.gains.ki;
        if self.gains.ki.abs() <= T::epsilon() {
            // If the new Ki is zero, the integral term cannot contribute to the output.
            // Clear the accumulator to prevent massive hidden windup if Ki is re-enabled later.
            self.error_integral = T::zero();
        }
    }

    /// Set the `kd` gain.
    #[inline]
    pub fn set_kd(&mut self, kd: T) {
        self.gains.kd = kd;
    }
}

impl<T: FloatCore + Default> From<PidGains<T>> for PidController<T> {
    fn from(pid: PidGains<T>) -> Self {
        Self {
            gains: PidGains {
                kp: pid.kp,
                ki: pid.ki,
                kd: pid.kd,
            },
            limits: PidLimits {
                integral_max: None,
                integral_min: None,
                output_saturation: None,
            },
            ki_saved: pid.ki,
            measurement_previous: T::default(),
            setpoint: T::default(),
            setpoint_previous: T::default(),
            setpoint_derivative: T::default(),
            error_derivative: T::default(),
            error_integral: T::default(),
            error: T::default(),
        }
    }
}

#[allow(missing_docs)]
impl<T: FloatCore> PidController<T> {
    pub fn limits(&self) -> PidLimits<T> {
        self.limits
    }

    pub fn set_limits(&mut self, limits: PidLimits<T>) {
        self.limits = limits;
    }

    pub fn set_integral_max(&mut self, integral_max: T) {
        self.limits.integral_max = Some(integral_max);
    }

    pub fn set_integral_min(&mut self, integral_min: T) {
        self.limits.integral_min = Some(integral_min);
    }

    pub fn set_integral_limit(&mut self, integral_limit: T) {
        self.limits.integral_max = Some(integral_limit);
        self.limits.integral_min = Some(-integral_limit);
    }

    pub fn set_output_saturation(&mut self, output_saturation: T) {
        self.limits.output_saturation = Some(output_saturation);
    }
}

/// Accessor functions to obtain error values.
impl<T: FloatCore> PidController<T> {
    /// Returns the errors, multiplied by the gains.
    pub fn error(&self) -> PidErrors<T> {
        PidErrors {
            p: self.error * self.gains.kp,
            i: self.error_integral, // error_integral is already multiplied by self.gains.ki
            d: self.error_derivative * self.gains.kd,
        }
    }

    /// Returns the raw values of the errors, ie NOT multiplied by the gains.
    pub fn error_raw(&self) -> PidErrors<T> {
        PidErrors {
            p: self.error,
            i: if self.gains.ki == T::zero() {
                T::zero()
            } else {
                self.error_integral / self.gains.ki
            },
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
        self.setpoint_derivative = T::zero();
        self.error = T::zero();
        self.error_integral = T::zero();
        self.error_derivative = T::zero();
    }

    /// Reset all, for test code.
    pub fn reset_all(&mut self) {
        self.measurement_previous = T::zero();
        self.setpoint = T::zero();
        self.setpoint_derivative = T::zero();
        self.error = T::zero();
        self.error_integral = T::zero();
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
        is_full::<PidControllerf32>();
        #[cfg(feature = "serde")]
        is_serde::<PidControllerf32>();
        #[cfg(feature = "storage")]
        is_storage::<PidControllerf32>();
    }
}
