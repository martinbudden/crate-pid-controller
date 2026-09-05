use num_traits::{ConstZero, float::FloatCore};

#[cfg(feature = "serde")]
use {
    postcard::experimental::max_size::MaxSize,
    sequential_storage::map::PostcardValue,
    serde::{Deserialize, Serialize},
};

/// `PidLimits` using `f32` values.
pub type PidLimitsf32 = PidLimits<f32>;
/// `PidLimits` using `f32` values.
pub type PidLimitsf64 = PidLimits<f64>;

/// Pid integral anti-windup parameters.<br>
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize, MaxSize))]
pub struct PidLimits<T> {
    /// Integral windup limit for positive integral.
    pub integral_max: Option<T>,
    /// Integral windup limit for negative integral.
    pub integral_min: Option<T>,
    /// Output saturation value, for integral windup control.
    pub output_saturation: Option<T>,
}

#[cfg(feature = "serde")]
impl<T> PostcardValue<'_> for PidLimits<T> where T: Serialize + MaxSize + for<'de> Deserialize<'de> {}

impl<T: FloatCore + ConstZero> Default for PidLimits<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: FloatCore + ConstZero> PidLimits<T> {
    #[must_use]
    /// Constructor.
    pub const fn new() -> Self {
        Self {
            integral_max: None,
            integral_min: None,
            output_saturation: None,
        }
    }

    /// Set the `integral_max` of a newly constructed `PidLimits`.
    #[must_use]
    pub fn with_integral_max(mut self, integral_max: T) -> Self {
        self.integral_max = Some(integral_max);
        self
    }

    /// Set the `integral_min` of a newly constructed `PidLimits`.
    #[must_use]
    pub fn with_integral_min(mut self, integral_min: T) -> Self {
        self.integral_min = Some(integral_min);
        self
    }

    /// Set the `output_saturation` of a newly constructed `PidLimits`.
    #[must_use]
    pub fn with_output_saturation(mut self, output_saturation: T) -> Self {
        self.output_saturation = Some(output_saturation);
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
        is_full::<PidLimitsf32>();
        #[cfg(feature = "serde")]
        is_config::<PidLimitsf32>();
    }
}
