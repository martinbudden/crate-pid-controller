use num_traits::Zero;

#[cfg(feature = "serde")]
use ::serde::{Deserialize, Serialize};
#[cfg(feature = "storage")]
use {postcard::experimental::max_size::MaxSize, sequential_storage::map::PostcardValue};

/// `PdErrors` using `f32` values.
pub type PdErrorsf32 = PdErrors<f32>;
/// `PdErrors` using `f64` values.
pub type PdErrorsf64 = PdErrors<f64>;

/// P and D errors as calculated by PD-controller.<br><br>
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "storage", derive(MaxSize))]
pub struct PdErrors<T> {
    /// P-term error.
    pub p: T,
    /// D-term error.
    pub d: T,
}

#[cfg(feature = "storage")]
impl<T> PostcardValue<'_> for PdErrors<T> where T: Serialize + MaxSize + for<'de> Deserialize<'de> {}

impl<T: Zero> Default for PdErrors<T> {
    fn default() -> Self {
        Self::new(T::zero(), T::zero())
    }
}

impl<T: Zero> PdErrors<T> {
    /// Constructor.
    #[allow(clippy::many_single_char_names)]
    pub const fn new(p: T, d: T) -> Self {
        Self { p, d }
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
        is_full::<PdErrorsf32>();
        #[cfg(feature = "serde")]
        is_serde::<PdErrorsf32>();
        #[cfg(feature = "storage")]
        is_storage::<PdErrorsf32>();
    }
}
