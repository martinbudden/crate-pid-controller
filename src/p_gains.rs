use num_traits::{ConstOne, ConstZero, float::FloatCore};

#[cfg(feature = "storage")]
use sequential_storage::map::PostcardValue;
#[cfg(feature = "serde")]
use {
    postcard::experimental::max_size::MaxSize,
    serde::{Deserialize, Serialize},
};

/// `PGains` using `f32` values.
pub type PGainsf32 = PGains<f32>;
/// `PGains` using `f64` values.
pub type PGainsf64 = PGains<f64>;

/// Gains for a pure P-controller.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize, MaxSize))]
pub struct PGains<T> {
    /// proportional gain.
    pub kp: T,
}

#[cfg(feature = "storage")]
impl<T> PostcardValue<'_> for PGains<T> where T: Serialize + MaxSize + for<'de> Deserialize<'de> {}

impl<T: FloatCore> Default for PGains<T>
where
    T: ConstOne + ConstZero,
{
    fn default() -> Self {
        Self::new()
    }
}

impl<T: FloatCore> PGains<T>
where
    T: ConstOne + ConstZero,
{
    /// Constructor.
    #[must_use]
    pub const fn new() -> Self {
        Self { kp: T::ONE }
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
    fn is_serde<T: Serialize + MaxSize + for<'a> Deserialize<'a>>() {}
    #[cfg(feature = "storage")]
    fn is_storage<T: for<'a> PostcardValue<'a>>() {}

    #[test]
    fn normal_types() {
        is_full::<PGainsf32>();
        #[cfg(feature = "serde")]
        is_serde::<PGainsf32>();
        #[cfg(feature = "storage")]
        is_storage::<PGainsf32>();
    }
}
