use core::ops::{Add, Div, Mul, Neg, Sub};
use num_traits::{One, Signed, Zero};

use crate::pid::Pid;

pub type PidControllerf32 = PidController<f32>;
pub type PidConstantsf32 = PidConstants<f32>;
pub type PidErrorf32 = PidError<f32>;

pub type PidControllerf64 = PidController<f64>;
pub type PidConstantsf64 = PidConstants<f64>;
pub type PidErrorf64 = PidError<f64>;

/// PID controller with open loop control.
/// This includes setpoint gain (classical feed forward) and setpoint derivative gain (kick - called feedforward by Betaflight).
///
/// Uses "independent PID" notation, where the gains are denoted as kp, ki, kd etc.
///
/// (In the "dependent PID" notation `kc`, `tau_i`, and `tau_d` parameters are used, where `kp = kc`, `ki = kc/tau_i`, `kd = kc*tau_d`).
///
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PidConstants<T> {
    pub kp: T, // proportional gain
    pub ki: T, // integral gain
    pub kd: T, // derivative gain
    pub ks: T, // setpoint gain
    pub kk: T, // setpoint derivative gain ('kick')
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PidError<T> {
    pub p: T,
    pub i: T,
    pub d: T,
    pub s: T,
    pub k: T,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PidController<T> {
    pid: PidConstants<T>,
    /// saved value of pid.ki, so integration can be switched on and off.
    ki_saved: T,
    measurement_previous: T,

    setpoint: T,
    setpoint_previous: T,
    setpoint_derivative: T,

    error_derivative: T,
    error_integral: T,
    error_previous: T,

    // integral anti-windup parameters
    /// Integral windup limit for positive integral.
    integral_max: T,
    /// Integral windup limit for negative integral.
    integral_min: T,
    /// Threshold for PID integration. Can be set to avoid integral wind-up due to movement in motor's backlash zone.
    integral_threshold: T,
    /// For integral windup control.
    output_saturation_value: T,
}

/// Default `PidController`.
/// ```
/// # use pidsk_controller::PidControllerf32;
/// # use num_traits::Zero;
///
/// let pid = PidControllerf32::default();
///
/// assert_eq!(1.0, pid.kp());
/// ```
impl<T> Default for PidController<T>
where
    T: Copy + Default + Zero + One,
{
    #[inline(always)]
    fn default() -> Self {
        Self::new(T::one(), T::zero(), T::zero())
    }
}

impl<T> PidController<T>
where
    T: Copy + Default,
{
    pub fn new(kp: T, ki: T, kd: T) -> Self {
        Self {
            pid: PidConstants {
                kp,
                ki,
                kd,
                ks: T::default(),
                kk: T::default(),
            },
            ki_saved: ki,
            measurement_previous: T::default(),
            setpoint: T::default(),
            setpoint_previous: T::default(),
            setpoint_derivative: T::default(),
            error_derivative: T::default(),
            error_integral: T::default(),
            error_previous: T::default(),
            integral_max: T::default(),
            integral_min: T::default(),
            integral_threshold: T::default(),
            output_saturation_value: T::default(),
        }
    }
}

impl<T>Pid<T> for  PidController<T>
where
    T: Copy + Zero + Signed + PartialOrd + Add<Output = T>  + Mul<Output = T>,
{
    /// PID update.
    /// ```
    /// # use pidsk_controller::{PidControllerf32,Pid};
    /// let delta_t: f32 = 0.01;
    /// let mut pid = PidControllerf32::new(0.1, 0.0, 0.0);
    ///
    /// pid.set_setpoint(8.7);
    ///
    /// let measurement:f32 = 9.2;
    /// let output = pid.update(measurement, delta_t);
    ///
    /// assert_eq!(-0.05, output);
    /// ```
    fn update(&mut self, measurement: T, delta_t: T) -> T {
        self.update_delta(measurement, measurement - self.measurement_previous, delta_t)
    }

    /// PID update with `measurement_delta` specified.
    /// This allows the user to filter `measurement_delta` with a filter of their choice.
    ///
    /// ```
    /// # use pidsk_controller::{PidControllerf32,Pid};
    /// # use signal_filters::{Pt1Filterf32,SignalFilter};
    /// let delta_t: f32 = 0.01;
    /// let mut pid = PidControllerf32::new(0.1, 0.0, 0.01);
    /// let mut filter = Pt1Filterf32::new(1.0);
    ///
    /// pid.set_setpoint(2.1);
    ///
    /// let measurement:f32 = 0.2;
    /// let measurement_delta = measurement - pid.previous_measurement();
    /// let measurement_delta_filtered = filter.update(measurement_delta);
    ///
    /// let output = pid.update_delta(measurement, measurement_delta_filtered, delta_t);
    ///
    /// assert_eq!(-0.010_000_005, output);
    ///
    fn update_delta(&mut self, measurement: T, measurement_delta: T, delta_t: T) -> T {
        self.update_delta_iterm(measurement, measurement_delta, self.setpoint - measurement, delta_t)
    }

    fn update_delta_iterm(&mut self, measurement: T, measurement_delta: T, i_term_error: T, delta_t: T) -> T {
        self.measurement_previous = measurement;
        let error = self.setpoint - measurement;
        self.error_derivative = -measurement_delta / delta_t; // note minus sign, error delta has reverse polarity to measurement delta
        // Partial PID sum, excludes ITerm
        // has additional S setpoint(openloop) and K kick(setpoint derivative) terms
        let partial_sum = self.pid.kp * error
            + self.pid.kd * self.error_derivative
            + self.pid.ks * self.setpoint
            + self.pid.kk * self.setpoint_derivative;

        if self.integral_threshold == T::zero() || (error).abs() >= self.integral_threshold {
            // "integrate" the error
            self.error_integral = self.error_integral + self.pid.ki * i_term_error * delta_t; // Euler integration
            //self.error_integral += self.pid.ki*0.5F*(iTermError + _errorPrevious)*delta_t; // integration using trapezoid rule
            // Anti-windup via integral clamping
            if self.integral_max > T::zero() && self.error_integral > self.integral_max {
                self.error_integral = self.integral_max;
            } else if self.integral_min < T::zero() && self.error_integral < self.integral_min {
                self.error_integral = self.integral_min;
            }
        }
        self.error_previous = error;

        if self.output_saturation_value > T::zero() {
            // Anti-windup by avoiding output saturation.
            // Check if partialSum + self.error_integral saturates the output
            // If so, the excess value above saturation does not help convergence to the setpoint and will result in
            // overshoot when the P value eventually comes down.
            // So limit the self.error_integral to a value that avoids output saturation.
            if self.error_integral > self.output_saturation_value - partial_sum {
                self.error_integral = self.output_saturation_value - partial_sum;
                if self.error_integral < T::zero() {
                    self.error_integral = T::zero();
                }
            } else if self.error_integral < -self.output_saturation_value - partial_sum {
                self.error_integral = -self.output_saturation_value - partial_sum;
                if self.error_integral > T::zero() {
                    self.error_integral = T::zero();
                }
            }
        }

        // The PID calculation with additional S setpoint(openloop) and K kick(setpoint derivative) terms
        //  P+D+S+K +  I
        partial_sum + self.error_integral
    }

    fn update_sp(&mut self, measurement: T) -> T {
        self.measurement_previous = measurement;
        let error = self.setpoint - measurement;
        self.error_previous = error;

        // The P (no I, no D) calculation with additional S setpoint(openloop) term
        //       P                   + S
        self.pid.kp * error + self.pid.ks * self.setpoint
    }

    fn update_spi(&mut self, measurement: T, delta_t: T) -> T {
        self.measurement_previous = measurement;
        let error = self.setpoint - measurement;
        let partial_sum: T = self.pid.kp * error + self.pid.ks * self.setpoint;

        if self.integral_threshold == T::zero() || error.abs() >= self.integral_threshold {
            // "integrate" the error
            self.error_integral = self.error_integral + self.pid.ki * error * delta_t; // Euler integration

            //_error_integral += self.pid.ki*0.5F*(error + self.errorPrevious)*delta_t; // integration using trapezoid rule
            // Anti-windup via integral clamping
            if self.integral_max > T::zero() && self.error_integral > self.integral_max {
                self.error_integral = self.integral_max;
            } else if self.integral_min < T::zero() && self.error_integral < self.integral_min {
                self.error_integral = self.integral_min;
            }
        }
        self.error_previous = error;

        if self.output_saturation_value > T::zero() {
            // Anti-windup by avoiding output saturation.
            // Check if partial_sum + self.error_integral saturates the output
            // If so, the excess value above saturation does not help convergence to the setpoint and will result in
            // overshoot when the P value eventually comes down.
            // So limit the self.error_integral to a value that avoids output saturation.
            if self.error_integral > self.output_saturation_value - partial_sum {
                self.error_integral = self.output_saturation_value - partial_sum;
                if self.error_integral < T::zero() {
                    self.error_integral = T::zero();
                }
            } else if self.error_integral < -self.output_saturation_value - partial_sum {
                self.error_integral = -self.output_saturation_value - partial_sum;
                if self.error_integral < T::zero() {
                    self.error_integral = T::zero();
                }
            }
        }

        // The PI (no D) calculation with additional S setpoint(openloop) term
        // P + S    + I
        partial_sum + self.error_integral
    }

    fn update_skpi(&mut self, measurement: T, delta_t: T) -> T {
        self.update_spi(measurement, delta_t) + self.pid.kk * self.setpoint_derivative
    }

    fn update_spd(&mut self, measurement: T, measurement_delta: T, delta_t: T) -> T {
        self.measurement_previous = measurement;
        let error = self.setpoint - measurement;
        self.error_previous = error;

        self.error_derivative = -measurement_delta / delta_t; // note minus sign, error delta has reverse polarity to measurement delta

        // The PD (no I) calculation with additional S setpoint(openloop) term
        //       P                   + D                                   + S
        self.pid.kp * error + self.pid.kd * self.error_derivative + self.pid.ks * self.setpoint
    }

    fn update_skpd(&mut self, measurement: T, measurement_delta: T, delta_t: T) -> T {
        self.update_spd(measurement, measurement_delta, delta_t) + self.pid.kk * self.setpoint_derivative
    }
}

impl<T> PidController<T>
where
    T: Copy + Zero + Neg<Output = T>,
{
    pub fn set_kp(&mut self, kp: T) {
        self.pid.kp = kp;
    }

    pub fn set_ki(&mut self, ki: T) {
        self.pid.ki = ki;
        self.ki_saved = self.pid.ki;
    }

    pub fn set_kd(&mut self, kd: T) {
        self.pid.kd = kd;
    }

    pub fn set_ks(&mut self, ks: T) {
        self.pid.ks = ks;
    }

    pub fn set_kk(&mut self, kk: T) {
        self.pid.kk = kk;
    }

    pub fn set_pid(&mut self, pid: PidConstants<T>) {
        self.pid = pid;
        self.ki_saved = self.pid.ki;
    }

    pub fn kp(&self) -> T {
        self.pid.kp
    }

    /// Return the set value of ki, whether integration is turned on or not.
    pub fn ki(&self) -> T {
        self.ki_saved
    }

    pub fn kd(&self) -> T {
        self.pid.kd
    }

    pub fn ks(&self) -> T {
        self.pid.ks
    }

    pub fn kk(&self) -> T {
        self.pid.kk
    }

    /// Return the set value of ki, whether integration is turned on or not.
    pub fn pid_constants(&self) -> PidConstants<T> {
        PidConstants {
            kp: self.pid.kp,
            ki: self.ki_saved,
            kd: self.pid.kd,
            ks: self.pid.ks,
            kk: self.pid.kk,
        }
    }

    pub fn reset_integral(&mut self) {
        self.error_integral = T::zero();
    }

    pub fn switch_integration_off(&mut self) {
        self.ki_saved = self.pid.ki;
        self.pid.ki = T::zero();
        self.error_integral = T::zero();
    }

    pub fn switch_integration_on(&mut self) {
        self.pid.ki = self.ki_saved;
        self.error_integral = T::zero();
    }

    pub fn set_integral_max(&mut self, integral_max: T) {
        self.integral_max = integral_max;
    }

    pub fn set_integral_min(&mut self, integral_min: T) {
        self.integral_min = integral_min;
    }

    pub fn set_integral_limit(&mut self, integral_limit: T) {
        self.integral_max = integral_limit;
        self.integral_min = -integral_limit;
    }

    pub fn set_integral_threshold(&mut self, integral_threshold: T) {
        self.integral_threshold = integral_threshold;
    }

    pub fn set_output_saturation_value(&mut self, output_saturation_value: T) {
        self.output_saturation_value = output_saturation_value;
    }

    pub fn set_setpoint(&mut self, setpoint: T) {
        self.setpoint_previous = self.setpoint;
        self.setpoint = setpoint;
    }

    pub fn set_setpoint_derivative(&mut self, setpoint_derivative: T) {
        self.setpoint_derivative = setpoint_derivative;
    }

    pub fn setpoint(&self) -> T {
        self.setpoint
    }

    pub fn previous_setpoint(&self) -> T {
        self.setpoint_previous
    }

    /// previous measurement, useful for `Dterm` filtering.
    pub fn previous_measurement(&self) -> T {
        self.measurement_previous
    }
}

impl<T> PidController<T>
where
    T: Copy + Sub<Output = T> + Div<Output = T>,
{
    pub fn setpoint_delta(&self) -> T {
        self.setpoint - self.setpoint_previous
    }

    pub fn set_setpoint_for_delta_t(&mut self, setpoint: T, delta_t: T) {
        self.setpoint_previous = self.setpoint;
        self.setpoint = setpoint;
        self.setpoint_derivative = (self.setpoint - self.setpoint_previous) / delta_t;
    }
}

impl<T> PidController<T>
where
    T: Copy + Zero + Signed+ PartialEq + Add<Output = T>  + Mul<Output = T>,
{
    // accessor functions to obtain error values
    pub fn error(&self) -> PidError<T> {
        PidError {
            p: self.error_previous * self.pid.kp,
            i: self.error_integral, // _error_integral is already multiplied by self.pid.ki
            d: self.error_derivative * self.pid.kd,
            s: self.setpoint * self.pid.ks,
            k: self.setpoint_derivative * self.pid.kk,
        }
    }

    pub fn error_raw(&self) -> PidError<T> {
        PidError {
            p: self.error_previous,
            i: if self.pid.ki == T::zero() {
                self.pid.ki
            } else {
                self.error_integral / self.pid.ki
            },
            d: self.error_derivative,
            s: self.setpoint,
            k: self.setpoint_derivative,
        }
    }

    pub fn error_p(&self) -> T {
        self.error_previous * self.pid.kp
    }

    pub fn error_i(&self) -> T {
        // self.error_integral is already multiplied by self.pid.ki
        self.error_integral
    }

    pub fn error_d(&self) -> T {
        self.error_derivative * self.pid.kd
    }

    pub fn error_s(&self) -> T {
        self.setpoint * self.pid.ks
    }

    pub fn error_k(&self) -> T {
        self.setpoint_derivative * self.pid.kk
    }

    pub fn error_raw_p(&self) -> T {
        self.error_previous
    }

    pub fn error_raw_i(&self) -> T {
        if self.pid.ki == T::zero() {
            self.pid.ki
        } else {
            self.error_integral / self.pid.ki
        }
    }

    pub fn error_raw_d(&self) -> T {
        self.error_derivative
    }

    pub fn error_raw_s(&self) -> T {
        self.setpoint
    }

    pub fn error_raw_k(&self) -> T {
        self.setpoint_derivative
    }

    /// get previous error, for test code.
    pub fn previous_error(&self) -> T {
        self.error_previous
    }

    /// reset all, for test code.
    pub fn reset_all(&mut self) {
        self.measurement_previous = T::zero();
        self.setpoint = T::zero();
        self.setpoint_previous = T::zero();
        self.setpoint_derivative = T::zero();
        self.error_derivative = T::zero();
        self.error_integral = T::zero();
        self.error_previous = T::zero();
    }
}

impl<T> From<PidConstants<T>> for PidController<T>
where
    T: Zero + Copy,
{
    fn from(pid: PidConstants<T>) -> Self {
        Self {
            pid: PidConstants {
                kp: pid.kp,
                ki: pid.ki,
                kd: pid.kd,
                ks: pid.ks,
                kk: pid.kk,
            },
            ki_saved: pid.ki,
            measurement_previous: T::zero(),
            setpoint: T::zero(),
            setpoint_previous: T::zero(),
            setpoint_derivative: T::zero(),
            error_derivative: T::zero(),
            error_integral: T::zero(),
            error_previous: T::zero(),
            integral_max: T::zero(),
            integral_min: T::zero(),
            integral_threshold: T::zero(),
            output_saturation_value: T::zero(),
        }
    }
}
