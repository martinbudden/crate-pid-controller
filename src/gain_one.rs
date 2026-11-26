/// Trait to allow implementation of gain of One for PID gains.
pub trait GainOne {
    /// Gain of One.
    fn gain_one() -> Self;
}

#[cfg(feature = "nalgebra")]
use nalgebra::{Vector2, Vector3, Vector4};

#[cfg(feature = "vqm")]
use {
    num_traits::One,
    vqm::{Vector2, Vector3, Vector4},
};

impl GainOne for f32 {
    fn gain_one() -> Self {
        1.0
    }
}

impl GainOne for f64 {
    fn gain_one() -> Self {
        1.0
    }
}

#[cfg(feature = "vqm")]
impl GainOne for Vector2<f32> {
    fn gain_one() -> Self {
        Self::one()
    }
}

#[cfg(feature = "vqm")]
impl GainOne for Vector3<f32> {
    fn gain_one() -> Self {
        Self::one()
    }
}

#[cfg(feature = "vqm")]
impl GainOne for Vector4<f32> {
    #[cfg(feature = "vqm")]
    fn gain_one() -> Self {
        Self::one()
    }
}

#[cfg(feature = "nalgebra")]
impl GainOne for Vector2<f32> {
    fn gain_one() -> Self {
        Self::repeat(1.0)
    }
}

#[cfg(feature = "nalgebra")]
impl GainOne for Vector3<f32> {
    fn gain_one() -> Self {
        Self::repeat(1.0)
    }
}

#[cfg(feature = "nalgebra")]
impl GainOne for Vector4<f32> {
    fn gain_one() -> Self {
        Self::repeat(1.0)
    }
}
