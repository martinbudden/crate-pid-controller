use num_traits::float::FloatCore;

#[cfg(feature = "serde")]
use {
    postcard::experimental::max_size::MaxSize,
    sequential_storage::map::PostcardValue,
    serde::{Deserialize, Serialize},
};

/// `PidError` using `f32` values.
pub type PidErrorsf32 = PidErrors<f32>;
/// `PidError` using `f64` values.
pub type PidErrorsf64 = PidErrors<f64>;

/// P, I, D, S, and K errors as calculated by PID controller.<br><br>
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize, MaxSize))]
#[allow(missing_docs)]
pub struct PidErrors<T> {
    pub p: T,
    pub i: T,
    pub d: T,
    pub s: T,
    pub k: T,
}

#[cfg(feature = "serde")]
impl<T> PostcardValue<'_> for PidErrors<T> where T: Serialize + MaxSize + for<'de> Deserialize<'de> {}

impl<T: FloatCore> Default for PidErrors<T> {
    fn default() -> Self {
        Self::new(T::zero(), T::zero(), T::zero(), T::zero(), T::zero())
    }
}

impl<T: FloatCore> PidErrors<T> {
    /// Constructor.
    #[allow(clippy::many_single_char_names)]
    pub const fn new(p: T, i: T, d: T, s: T, k: T) -> Self {
        Self { p, i, d, s, k }
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
        is_full::<PidErrorsf32>();
        #[cfg(feature = "serde")]
        is_config::<PidErrorsf32>();
    }
}
