use num_traits::Zero;

#[cfg(feature = "storage")]
use sequential_storage::map::PostcardValue;
#[cfg(feature = "serde")]
use {
    postcard::experimental::max_size::MaxSize,
    serde::{Deserialize, Serialize},
};

/// `PidskErrors` using `f32` values.
pub type PidskErrorsf32 = PidskErrors<f32>;
/// `PidskErrors` using `f64` values.
pub type PidskErrorsf64 = PidskErrors<f64>;

/// P, I, D, S, and K errors as calculated by PID controller.<br><br>
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize, MaxSize))]
pub struct PidskErrors<T> {
    /// P-term error.
    pub p: T,
    /// I-term error.
    pub i: T,
    /// D-term error.
    pub d: T,
    /// S-term error.
    pub s: T,
    /// K-term error.
    pub k: T,
}

#[cfg(feature = "storage")]
impl<T> PostcardValue<'_> for PidskErrors<T> where T: Serialize + MaxSize + for<'de> Deserialize<'de> {}

impl<T: Zero> Default for PidskErrors<T> {
    fn default() -> Self {
        Self::new(T::zero(), T::zero(), T::zero(), T::zero(), T::zero())
    }
}

impl<T: Zero> PidskErrors<T> {
    /// Constructor.
    #[allow(clippy::many_single_char_names)]
    pub const fn new(p: T, i: T, d: T, s: T, k: T) -> Self {
        Self { p, i, d, s, k }
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
        is_full::<PidskErrorsf32>();
        #[cfg(feature = "serde")]
        is_serde::<PidskErrorsf32>();
        #[cfg(feature = "storage")]
        is_storage::<PidskErrorsf32>();
    }
}
