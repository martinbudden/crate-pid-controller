use crate::GainOne;

#[cfg(feature = "serde")]
use ::serde::{Deserialize, Serialize};

#[cfg(feature = "storage")]
use {postcard::experimental::max_size::MaxSize, sequential_storage::map::PostcardValue};

/// `PGains` using `f32` values.
pub type PGainsf32 = PGains<f32>;
/// `PGains` using `f64` values.
pub type PGainsf64 = PGains<f64>;

/// Gains for a pure P-controller.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "storage", derive(MaxSize))]
pub struct PGains<T> {
    /// proportional gain.
    pub kp: T,
}

#[cfg(feature = "storage")]
impl<T> PostcardValue<'_> for PGains<T> where T: Serialize + MaxSize + for<'de> Deserialize<'de> {}

impl<T> Default for PGains<T>
where
    T: GainOne,
{
    fn default() -> Self {
        Self::new()
    }
}

impl<T> PGains<T> {
    /// Constructor.
    #[must_use]
    pub fn new() -> Self
    where
        T: GainOne,
    {
        Self { kp: T::gain_one() }
    }

    /// Set the `kp` of a newly constructed `PGains`.
    #[must_use]
    pub fn with_kp(mut self, kp: T) -> Self {
        self.kp = kp;
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
        is_full::<PGainsf32>();
        #[cfg(feature = "serde")]
        is_serde::<PGainsf32>();
        #[cfg(feature = "storage")]
        is_storage::<PGainsf32>();
    }
}
