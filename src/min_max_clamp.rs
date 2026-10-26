#[cfg(feature = "vqm")]
use vqm::{Vector2f32, Vector2f64, Vector3f32, Vector3f64, Vector4f32, Vector4f64};
/// Trait for min, max, and clamp functions.
pub trait MinMaxClamp {
    /// Returns the minimum of the two values.
    #[must_use]
    fn min(self, other: Self) -> Self;

    /// Returns the maximum of the two values.
    #[must_use]
    fn max(self, other: Self) -> Self;

    /// Returns value bounded by a minimum and a maximum.
    ///
    ///  Returns min if input is less than min.
    ///  Returns max if input is greater than max.
    ///  Otherwise returns input.
    #[must_use]
    fn clamp(self, min: Self, max: Self) -> Self;
}

impl MinMaxClamp for f32 {
    #[inline]
    fn min(self, other: Self) -> Self {
        self.min(other)
    }
    #[inline]
    fn max(self, other: Self) -> Self {
        self.max(other)
    }
    #[inline]
    fn clamp(self, min: Self, max: Self) -> Self {
        self.clamp(min, max)
    }
}

impl MinMaxClamp for f64 {
    #[inline]
    fn min(self, other: Self) -> Self {
        self.min(other)
    }
    #[inline]
    fn max(self, other: Self) -> Self {
        self.max(other)
    }
    #[inline]
    fn clamp(self, min: Self, max: Self) -> Self {
        self.clamp(min, max)
    }
}

#[cfg(feature = "vqm")]
impl MinMaxClamp for Vector2f32 {
    #[inline]
    fn min(self, other: Self) -> Self {
        self.min(other)
    }
    #[inline]
    fn max(self, other: Self) -> Self {
        self.max(other)
    }
    #[inline]
    fn clamp(self, min: Self, max: Self) -> Self {
        self.clamp(min, max)
    }
}

#[cfg(feature = "vqm")]
impl MinMaxClamp for Vector3f32 {
    #[inline]
    fn min(self, other: Self) -> Self {
        self.min(other)
    }
    #[inline]
    fn max(self, other: Self) -> Self {
        self.max(other)
    }
    #[inline]
    fn clamp(self, min: Self, max: Self) -> Self {
        self.clamp(min, max)
    }
}

#[cfg(feature = "vqm")]
impl MinMaxClamp for Vector4f32 {
    #[inline]
    fn min(self, other: Self) -> Self {
        self.min(other)
    }
    #[inline]
    fn max(self, other: Self) -> Self {
        self.max(other)
    }
    #[inline]
    fn clamp(self, min: Self, max: Self) -> Self {
        self.clamp(min, max)
    }
}

#[cfg(feature = "vqm")]
impl MinMaxClamp for Vector2f64 {
    #[inline]
    fn min(self, other: Self) -> Self {
        self.min(other)
    }
    #[inline]
    fn max(self, other: Self) -> Self {
        self.max(other)
    }
    #[inline]
    fn clamp(self, min: Self, max: Self) -> Self {
        self.clamp(min, max)
    }
}

#[cfg(feature = "vqm")]
impl MinMaxClamp for Vector3f64 {
    #[inline]
    fn min(self, other: Self) -> Self {
        self.min(other)
    }
    #[inline]
    fn max(self, other: Self) -> Self {
        self.max(other)
    }
    #[inline]
    fn clamp(self, min: Self, max: Self) -> Self {
        self.clamp(min, max)
    }
}

#[cfg(feature = "vqm")]
impl MinMaxClamp for Vector4f64 {
    #[inline]
    fn min(self, other: Self) -> Self {
        self.min(other)
    }
    #[inline]
    fn max(self, other: Self) -> Self {
        self.max(other)
    }
    #[inline]
    fn clamp(self, min: Self, max: Self) -> Self {
        self.clamp(min, max)
    }
}
