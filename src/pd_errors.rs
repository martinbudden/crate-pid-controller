use num_traits::float::FloatCore;

#[cfg(feature = "storage")]
use sequential_storage::map::PostcardValue;
#[cfg(feature = "serde")]
use {
    postcard::experimental::max_size::MaxSize,
    serde::{Deserialize, Serialize},
};

/// `PdErrors` using `f32` values.
pub type PdErrorsf32 = PdErrors<f32>;
/// `PdErrors` using `f64` values.
pub type PdErrorsf64 = PdErrors<f64>;

/// P and D errors as calculated by PD-controller.<br><br>
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize, MaxSize))]
#[allow(missing_docs)]
pub struct PdErrors<T> {
    pub p: T,
    pub d: T,
}

#[cfg(feature = "storage")]
impl<T> PostcardValue<'_> for PdErrors<T> where T: Serialize + MaxSize + for<'de> Deserialize<'de> {}

impl<T: FloatCore> Default for PdErrors<T> {
    fn default() -> Self {
        Self::new(T::zero(), T::zero())
    }
}

impl<T: FloatCore> PdErrors<T> {
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
    fn is_serde<T: Serialize + MaxSize + for<'a> Deserialize<'a>>() {}
    #[cfg(feature = "storage")]
    fn is_storage<T: for<'a> PostcardValue<'a>>() {}

    #[test]
    fn normal_types() {
        is_full::<PdErrorsf32>();
        #[cfg(feature = "serde")]
        is_serde::<PdErrorsf32>();
        #[cfg(feature = "storage")]
        is_storage::<PdErrorsf32>();
    }
}
