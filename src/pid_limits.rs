#[cfg(feature = "serde")]
use ::serde::{Deserialize, Serialize};
#[cfg(feature = "storage")]
use {postcard::experimental::max_size::MaxSize, sequential_storage::map::PostcardValue};

/// `PidLimits` using `f32` values.
pub type PidLimitsf32 = PidLimits<f32>;
/// `PidLimits` using `f32` values.
pub type PidLimitsf64 = PidLimits<f64>;

/// Pid integral anti-windup parameters.<br>
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "storage", derive(MaxSize))]
pub struct PidLimits<T> {
    /// Integral windup limit for positive integral.
    pub integral_max: Option<T>,
    /// Integral windup limit for negative integral.
    pub integral_min: Option<T>,
    /// Output saturation value, for integral windup control.
    pub output_saturation: Option<T>,
}

#[cfg(feature = "storage")]
impl<T> PostcardValue<'_> for PidLimits<T> where T: Serialize + MaxSize + for<'de> Deserialize<'de> {}

impl<T> Default for PidLimits<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T> PidLimits<T> {
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

    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}
    #[cfg(feature = "serde")]
    fn is_serde<T: Serialize + for<'a> Deserialize<'a>>() {}
    #[cfg(feature = "storage")]
    fn is_storage<T: MaxSize + for<'a> PostcardValue<'a>>() {}

    #[test]
    fn normal_types() {
        is_full::<PidLimitsf32>();
        #[cfg(feature = "serde")]
        is_serde::<PidLimitsf32>();
        #[cfg(feature = "storage")]
        is_storage::<PidLimitsf32>();
    }
}
