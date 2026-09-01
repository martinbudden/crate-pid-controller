use num_traits::{ConstOne, ConstZero, float::FloatCore};

#[cfg(feature = "serde")]
use {
    postcard::experimental::max_size::MaxSize,
    sequential_storage::map::PostcardValue,
    serde::{Deserialize, Serialize},
};

/// `PidGains` using `f32` values.
pub type PidGainsf32 = PidGains<f32>;
/// `PidGains` using `f64` values.
pub type PidGainsf64 = PidGains<f64>;

/// Gains for PID controller.
/// Includes classical PID (`kp`, `ki`, and `kd`) gains and also<br>
/// setpoint gain (`ks` - classical feed forward) and<br>
/// setpoint derivative gain (`kk`. kick - called feedforward by Betaflight).<br>
///
/// Uses "independent PID" notation, where the gains are denoted as kp, ki, kd etc.<br>
/// (In the "dependent PID" notation `kc`, `tau_i`, and `tau_d` parameters are used, where `kp = kc`, `ki = kc/tau_i`, `kd = kc*tau_d`).
///
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize, MaxSize))]
pub struct PidGains<T> {
    /// proportional gain.
    pub kp: T,
    /// integral gain.
    pub ki: T,
    /// derivative gain.
    pub kd: T,
    /// setpoint gain.
    pub ks: T,
    /// setpoint derivative gain ('kick').
    pub kk: T,
}

#[cfg(feature = "serde")]
impl<T> PostcardValue<'_> for PidGains<T> where T: Serialize + MaxSize + for<'de> Deserialize<'de> {}

impl<T: FloatCore> Default for PidGains<T>
where
    T: ConstOne + ConstZero,
{
    fn default() -> Self {
        Self::new()
    }
}

impl<T: FloatCore> PidGains<T>
where
    T: ConstOne + ConstZero,
{
    /// Constructor.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            kp: T::ONE,
            ki: T::ZERO,
            kd: T::ZERO,
            ks: T::ZERO,
            kk: T::ZERO,
        }
    }

    /// Set the `kp` of a newly constructed `PidGains`.
    #[must_use]
    pub fn with_kp(mut self, kp: T) -> Self {
        self.kp = kp;
        self
    }

    /// Set the `ki` of a newly constructed `PidGains`.
    #[must_use]
    pub fn with_ki(mut self, ki: T) -> Self {
        self.ki = ki;
        self
    }

    /// Set the `kd` of a newly constructed `PidGains`.
    #[must_use]
    pub fn with_kd(mut self, kd: T) -> Self {
        self.kd = kd;
        self
    }

    /// Set the `ks` of a newly constructed `PidGains`.
    #[must_use]
    pub fn with_ks(mut self, ks: T) -> Self {
        self.ks = ks;
        self
    }

    /// Set the `kk` of a newly constructed `PidGains`.
    #[must_use]
    pub fn with_kk(mut self, kk: T) -> Self {
        self.kk = kk;
        self
    }
}

#[cfg(test)]
mod test_traits {
    use super::*;

    #[cfg(feature = "serde")]
    use serde::{Deserialize, Serialize};

    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}
    #[cfg(feature = "serde")]
    fn is_config<T: Serialize + MaxSize + for<'a> Deserialize<'a> + for<'a> PostcardValue<'a>>() {}

    #[test]
    fn normal_types() {
        is_full::<PidGainsf32>();
        #[cfg(feature = "serde")]
        is_config::<PidGainsf32>();
    }
}
