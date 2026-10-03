use num_traits::{ConstOne, ConstZero, float::FloatCore};

#[cfg(feature = "storage")]
use sequential_storage::map::PostcardValue;
#[cfg(feature = "serde")]
use {
    postcard::experimental::max_size::MaxSize,
    serde::{Deserialize, Serialize},
};

/// `PdGains` using `f32` values.
pub type PdGainsf32 = PdGains<f32>;
/// `PdGains` using `f64` values.
pub type PdGainsf64 = PdGains<f64>;

/// Gains for PD controller.
///
/// Uses "independent PID" notation, where the gains are denoted as kp and kd.<br>
/// (In the "dependent PID" notation `kc` and `tau_d` parameters are used, where `kp = kc` and `kd = kc*tau_d`).
///
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize, MaxSize))]
pub struct PdGains<T> {
    /// proportional gain.
    pub kp: T,
    /// derivative gain.
    pub kd: T,
}

#[cfg(feature = "storage")]
impl<T> PostcardValue<'_> for PdGains<T> where T: Serialize + MaxSize + for<'de> Deserialize<'de> {}

impl<T: FloatCore> Default for PdGains<T>
where
    T: ConstOne + ConstZero,
{
    fn default() -> Self {
        Self::new()
    }
}

impl<T: FloatCore> PdGains<T>
where
    T: ConstOne + ConstZero,
{
    /// Constructor.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            kp: T::ONE,
            kd: T::ZERO,
        }
    }

    /// Set the `kp` of a newly constructed `PdGains`.
    #[must_use]
    pub fn with_kp(mut self, kp: T) -> Self {
        self.kp = kp;
        self
    }

    /// Set the `kd` of a newly constructed `PdGains`.
    #[must_use]
    pub fn with_kd(mut self, kd: T) -> Self {
        self.kd = kd;
        self
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
        is_full::<PdGainsf32>();
        #[cfg(feature = "serde")]
        is_serde::<PdGainsf32>();
        #[cfg(feature = "storage")]
        is_storage::<PdGainsf32>();
    }
}
