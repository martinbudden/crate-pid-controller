use num_traits::Zero;

use crate::GainOne;
#[cfg(feature = "serde")]
use ::serde::{Deserialize, Serialize};

#[cfg(feature = "storage")]
use {postcard::experimental::max_size::MaxSize, sequential_storage::map::PostcardValue};

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
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "storage", derive(MaxSize))]
pub struct PdGains<T> {
    /// proportional gain.
    pub kp: T,
    /// derivative gain.
    pub kd: T,
}

#[cfg(feature = "storage")]
impl<T> PostcardValue<'_> for PdGains<T> where T: Serialize + MaxSize + for<'de> Deserialize<'de> {}

impl<T> Default for PdGains<T>
where
    T: Zero + GainOne,
{
    fn default() -> Self {
        Self::new()
    }
}

impl<T> PdGains<T> {
    /// Constructor.
    #[must_use]
    pub fn new() -> Self
    where
        T: Zero + GainOne,
    {
        Self {
            kp: T::gain_one(),
            kd: T::zero(),
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
    fn is_serde<T: Serialize + for<'a> Deserialize<'a>>() {}
    #[cfg(feature = "storage")]
    fn is_storage<T: MaxSize + for<'a> PostcardValue<'a>>() {}

    #[test]
    fn normal_types() {
        is_full::<PdGainsf32>();
        #[cfg(feature = "serde")]
        is_serde::<PdGainsf32>();
        #[cfg(feature = "storage")]
        is_storage::<PdGainsf32>();
    }
}
