use num_traits::float::FloatCore;

#[cfg(feature = "storage")]
use sequential_storage::map::PostcardValue;
#[cfg(feature = "serde")]
use {
    postcard::experimental::max_size::MaxSize,
    serde::{Deserialize, Serialize},
};

/// `PidError` using `f32` values.
pub type PErrorsf32 = PErrors<f32>;
/// `PidError` using `f64` values.
pub type PErrorsf64 = PErrors<f64>;

/// P error as calculated by P-controller.<br><br>
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize, MaxSize))]
#[allow(missing_docs)]
pub struct PErrors<T> {
    pub p: T,
}

#[cfg(feature = "storage")]
impl<T> PostcardValue<'_> for PErrors<T> where T: Serialize + MaxSize + for<'de> Deserialize<'de> {}

impl<T: FloatCore> Default for PErrors<T> {
    fn default() -> Self {
        Self::new(T::zero())
    }
}

impl<T: FloatCore> PErrors<T> {
    /// Constructor.
    #[allow(clippy::many_single_char_names)]
    pub const fn new(p: T) -> Self {
        Self { p }
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
        is_full::<PErrorsf32>();
        #[cfg(feature = "serde")]
        is_serde::<PErrorsf32>();
        #[cfg(feature = "storage")]
        is_storage::<PErrorsf32>();
    }
}
