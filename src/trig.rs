// Copyright (c) 2020-2024 Via Technology Ltd. All Rights Reserved.
// Consult your license regarding permissions and restrictions.

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

//! The trig module contains types and functions for performing trigonometric
//! calculations.
//!
//! The module uses the [newtype](https://rust-unofficial.github.io/patterns/patterns/behavioural/newtype.html)
//! idiom to represent angles in `Degrees` and `Radians`.  
//! It also uses the `newtype` idiom to represent `UnitNegRange`s; values that
//! lie between -1.0 and +1.0 inclusive, such as the sins and cosines of angles
//! and the coordinates of unit sphere vectors.
//!
//! The `Angle` struct represents an angle as two `UnitNegRange`s: s and c for
//! the sine and cosine of the angle respectively.

#![allow(clippy::float_cmp)]
#![allow(clippy::suboptimal_flops)]

use crate::{clamp, is_small, Validate};
use contracts::{debug_ensures, debug_invariant, debug_requires};
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use std::convert::From;
use std::ops::{Add, Neg, Sub};

/// The Degrees newtype an f64.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct Degrees(pub f64);

/// The Radians newtype an f64.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct Radians(pub f64);

impl Degrees {
    /// Test whether a value is a valid latitude.  
    /// I.e. whether it lies in the range: -90.0 <= value <= 90.0
    #[must_use]
    pub fn is_valid_latitude(value: f64) -> bool {
        (-90.0..=90.0).contains(&value)
    }

    /// Test whether a value is a valid longitude.  
    /// I.e. whether it lies in the range: -180.0 <= value <= 180.0
    #[must_use]
    pub fn is_valid_longitude(value: f64) -> bool {
        (-180.0..=180.0).contains(&value)
    }

    /// Test whether a value is a valid bearing.  
    /// I.e. whether it lies in the range: 0.0 <= value <= 360.0
    #[must_use]
    pub fn is_valid_bearing(value: f64) -> bool {
        (0.0..=360.0).contains(&value)
    }

    /// Normalise a Degrees value into the range: -180.0 < value <= 180.0
    /// # Examples
    /// ```
    /// use via_sphere::trig::Degrees;
    ///
    /// assert_eq!(0.0, Degrees(-360.0).normalise().0);
    /// assert_eq!(180.0, Degrees(-180.0).normalise().0);
    /// assert_eq!(180.0, Degrees(180.0).normalise().0);
    /// assert_eq!(0.0, Degrees(360.0).normalise().0);
    /// ```
    #[must_use]
    pub fn normalise(&self) -> Self {
        if self.0 <= -180.0 {
            Self(self.0 + 360.0)
        } else if self.0 <= 180.0 {
            *self
        } else {
            Self(self.0 - 360.0)
        }
    }
}

impl Validate for Degrees {
    /// Test whether a Degrees is valid.  
    /// I.e. whether it lies in the range: -180.0 <= value <= 180.0
    /// # Examples
    /// ```
    /// use via_sphere::trig::Degrees;
    /// use via_sphere::Validate;
    ///
    /// assert!(!Degrees(-180.0 * (1.0 + std::f64::EPSILON)).is_valid());
    /// assert!(Degrees(-180.0).is_valid());
    /// assert!(Degrees(180.0).is_valid());
    /// assert!(!(Degrees(180.0 * (1.0 + std::f64::EPSILON)).is_valid()));
    /// ```
    fn is_valid(&self) -> bool {
        Self::is_valid_longitude(self.0)
    }
}

impl From<Radians> for Degrees {
    /// Construct an angle in Degrees from an angle in Radians.
    /// # Examples
    /// ```
    /// use via_sphere::trig::Degrees;  
    /// use via_sphere::trig::Radians;
    ///
    /// let arg = Radians(std::f64::consts::FRAC_PI_2);
    /// let answer = Degrees::from(arg);
    /// assert_eq!(90.0, answer.0);
    /// ```
    fn from(a: Radians) -> Self {
        Self(a.0.to_degrees())
    }
}

impl Neg for Degrees {
    type Output = Self;

    /// An implementation of Neg for Degrees, i.e. -angle.
    /// # Examples
    /// ```
    /// use via_sphere::trig::Degrees;
    ///
    /// let angle_45 = Degrees(45.0);
    /// let result_m45 = -angle_45;
    /// assert_eq!(Degrees(-45.0), result_m45);
    /// ```
    fn neg(self) -> Self {
        Self(0.0 - self.0)
    }
}

impl Add for Degrees {
    type Output = Self;

    /// Add two angles in Degrees, automatically wraps around +/- 180 degrees.
    /// # Examples
    /// ```
    /// use via_sphere::trig::Degrees;
    ///
    /// let angle_120 = Degrees(120.0);
    /// let result = angle_120 + angle_120;
    /// assert_eq!(Degrees(-120.0), result);
    /// ```
    fn add(self, other: Self) -> Self {
        Self(self.0 + other.0).normalise()
    }
}

impl Sub for Degrees {
    type Output = Self;

    /// Subtract two angles in Degrees, automatically wraps around.
    /// # Examples
    /// ```
    /// use via_sphere::trig::Degrees;
    ///
    /// let angle_m120 = Degrees(-120.0);
    /// let angle_120 = Degrees(120.0);
    /// let result = angle_m120 - angle_120;
    /// assert_eq!(angle_120, result);
    /// ```
    fn sub(self, other: Self) -> Self {
        Self(self.0 - other.0).normalise()
    }
}

/// The conversion factor from Degrees to Radians.
pub const DEG2RAD: f64 = std::f64::consts::PI / 180.0;

impl Radians {
    /// Normalise a Radians into the range:
    /// -`std::f64::consts::PI` < value <= `std::f64::consts::PI`
    /// # Examples
    /// ```
    /// use via_sphere::trig::Radians;
    ///
    /// assert_eq!(0.0, Radians(-2.0 * std::f64::consts::PI).normalise().0);
    /// assert_eq!(std::f64::consts::PI, Radians(-std::f64::consts::PI).normalise().0);
    /// assert_eq!(std::f64::consts::PI, Radians(std::f64::consts::PI).normalise().0);
    /// assert_eq!(0.0, Radians(2.0 * std::f64::consts::PI).normalise().0);
    /// ```
    #[must_use]
    pub fn normalise(&self) -> Self {
        if self.0 <= -std::f64::consts::PI {
            Self(self.0 + std::f64::consts::TAU)
        } else if self.0 <= std::f64::consts::PI {
            *self
        } else {
            Self(self.0 - std::f64::consts::TAU)
        }
    }
}

impl Validate for Radians {
    /// Test whether a Radians is valid.  
    /// I.e. whether it lies in the range: -PI <= value <= PI
    /// # Examples
    /// ```
    /// use via_sphere::trig::Radians;
    /// use via_sphere::Validate;
    ///
    /// assert!(!Radians(-std::f64::consts::PI * (1.0 + std::f64::EPSILON)).is_valid());
    /// assert!(Radians(-std::f64::consts::PI).is_valid());
    /// assert!(Radians(std::f64::consts::PI).is_valid());
    /// assert!(!(Radians(std::f64::consts::PI * (1.0 + std::f64::EPSILON)).is_valid()));
    /// ```
    fn is_valid(&self) -> bool {
        (-std::f64::consts::PI..=std::f64::consts::PI).contains(&self.0)
    }
}

impl Neg for Radians {
    type Output = Self;

    /// An implementation of Neg for Radians, i.e. -angle.
    /// # Examples
    /// ```
    /// use via_sphere::trig::Radians;
    ///
    /// let angle_45 = Radians(std::f64::consts::FRAC_PI_4);
    /// let result_m45 = -angle_45;
    /// assert_eq!(Radians(-std::f64::consts::FRAC_PI_4), result_m45);
    /// ```
    fn neg(self) -> Self {
        Self(0.0 - self.0)
    }
}

impl Add for Radians {
    type Output = Self;

    /// Add a pair of angles in Radians, wraps around +/-PI.
    /// # Examples
    /// ```
    /// use via_sphere::trig::Radians;
    ///
    /// let angle_120 = Radians(2.0 * std::f64::consts::FRAC_PI_3);
    /// let result = angle_120 + angle_120;
    /// // Note: wrapping is not precise...
    /// // assert_eq!(Radians(-2.0 * std::f64::consts::FRAC_PI_3), result);
    /// let delta_angle = libm::fabs(-2.0 * std::f64::consts::FRAC_PI_3 - result.0);
    /// assert!(delta_angle <= 4.0 * std::f64::EPSILON);
    /// ```
    fn add(self, other: Self) -> Self {
        Self(self.0 + other.0).normalise()
    }
}

impl Sub for Radians {
    type Output = Self;

    /// Subtract a pair of angles in Radians,  wraps around +/-PI.
    /// # Examples
    /// ```
    /// use via_sphere::trig::Radians;
    ///
    /// let angle_m120 = Radians(-2.0 * std::f64::consts::FRAC_PI_3);
    /// let angle_120 = Radians(2.0 * std::f64::consts::FRAC_PI_3);
    /// let result = angle_m120 - angle_120;
    /// // Note: wrapping is not precise...
    /// // assert_eq!(angle_120, result);
    /// let delta_angle = libm::fabs(angle_120.0 - result.0);
    /// assert!(delta_angle <= 4.0 * std::f64::EPSILON);
    /// ```
    fn sub(self, other: Self) -> Self {
        Self(self.0 - other.0).normalise()
    }
}

/// Calculates the length of the adjacent side of a right angled
/// spherical triangle, given the angle and length of the hypotenuse.
#[debug_requires((0.0..std::f64::consts::FRAC_PI_2).contains(&length.0))]
pub fn spherical_cosine_rule(cos_angle: f64, length: Radians) -> Radians {
    Radians(libm::atan(cos_angle * libm::tan(length.0)))
}

/// Convert a Euclidean distance to a Great Circle distance (in radians).
/// e should satisfy: 0 <= e <= 2, if not it is clamped into range.
#[debug_requires(0.0 <= e)] // don't test e <= 2, due to floating point maths issues
#[debug_ensures((0.0..=std::f64::consts::PI).contains(&ret.0))]
pub fn e2gc_distance(e: f64) -> Radians {
    Radians(2.0 * libm::asin(UnitNegRange::clamp(0.5 * e).0))
}

/// Convert a Great Circle distance (in radians) to a Euclidean distance.
#[debug_requires((0.0..=std::f64::consts::PI).contains(&gc.0))]
#[debug_ensures((0.0..=2.0).contains(&ret))]
pub fn gc2e_distance(gc: Radians) -> f64 {
    2.0 * libm::sin(0.5 * gc.0)
}

/// The `UnitNegRange` newtype an f64.  
/// A valid `UnitNegRange` value lies between -1.0 and +1.0 inclusive.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct UnitNegRange(pub f64);

impl Validate for UnitNegRange {
    /// Test whether a `UnitNegRange` is valid.  
    /// I.e. whether it lies in the range: -1.0 <= value <= 1.0
    /// # Examples
    /// ```
    /// use via_sphere::trig::UnitNegRange;
    /// use via_sphere::Validate;
    ///
    /// assert!(!UnitNegRange(-1.0 - std::f64::EPSILON).is_valid());
    /// assert!(UnitNegRange(-1.0).is_valid());
    /// assert!(UnitNegRange(1.0).is_valid());
    /// assert!(!(UnitNegRange(1.0 + std::f64::EPSILON).is_valid()));
    /// ```
    fn is_valid(&self) -> bool {
        (-1.0..=1.0).contains(&self.0)
    }
}

impl Neg for UnitNegRange {
    type Output = Self;

    /// An implementation of Neg for `UnitNegRange`.  
    /// Negates the value.
    fn neg(self) -> Self {
        Self(0.0 - self.0)
    }
}

#[debug_invariant(self.is_valid())]
impl UnitNegRange {
    /// Clamp value into the valid range: -1.0 to +1.0 inclusive.
    /// # Examples
    /// ```
    /// use via_sphere::trig::UnitNegRange;
    ///
    /// assert_eq!(-1.0, UnitNegRange::clamp(-1.0 - std::f64::EPSILON).0);
    /// assert_eq!(-1.0, UnitNegRange::clamp(-1.0).0);
    /// assert_eq!(-0.5, UnitNegRange::clamp(-0.5).0);
    /// assert_eq!(1.0, UnitNegRange::clamp(1.0).0);
    /// assert_eq!(1.0, UnitNegRange::clamp(1.0 + std::f64::EPSILON).0);
    /// ```
    #[must_use]
    pub fn clamp(value: f64) -> Self {
        Self(clamp(value, -1.0, 1.0))
    }
}

/// Swap the sine into the cosine of an Angle and vice versa.  
/// Uses the identity sin<sup>2</sup> + cos<sup>2</sup> = 1
/// # Examples
/// ```
/// use via_sphere::trig::UnitNegRange;
/// use via_sphere::trig::swap_sin_cos;
///
/// assert_eq!(UnitNegRange(0.0), swap_sin_cos(UnitNegRange(-1.0)));
/// assert_eq!(UnitNegRange(1.0), swap_sin_cos(UnitNegRange(0.0)));
/// ```
#[must_use]
pub fn swap_sin_cos(a: UnitNegRange) -> UnitNegRange {
    UnitNegRange::clamp(libm::sqrt((1.0 - a.0) * (1.0 + a.0)))
}

/// Calculate the cosine of an Angle from it's sine and
/// the sign of the cosine.
/// * `a` the sine of the angle.
/// * `sign` the sign of the cosine of the angle.  
///
/// return the cosine of the Angle.
/// # Examples
/// ```
/// use via_sphere::trig::UnitNegRange;
/// use via_sphere::trig::cosine_from_sine;
///
/// assert_eq!(1.0, cosine_from_sine(UnitNegRange(0.0), 1.0).0);
/// assert_eq!(-1.0, cosine_from_sine(UnitNegRange(0.0), -1.0).0);
/// ```
#[must_use]
pub fn cosine_from_sine(a: UnitNegRange, sign: f64) -> UnitNegRange {
    UnitNegRange(libm::copysign(swap_sin_cos(a).0, sign))
}

/// Calculate the sine of the difference of two angles: a - b.
/// * `sin_a`, `cos_a` the sine and cosine of angle a.
/// * `sin_b`, `cos_b` the sine and cosine of angle b.
///
/// return sin(a - b)
#[must_use]
pub fn sine_diff(
    sin_a: UnitNegRange,
    cos_a: UnitNegRange,
    sin_b: UnitNegRange,
    cos_b: UnitNegRange,
) -> UnitNegRange {
    UnitNegRange::clamp(sin_a.0 * cos_b.0 - sin_b.0 * cos_a.0)
}

/// Calculate the sine of the sum of two angles: a + b.
/// * `sin_a`, `cos_a` the sine and cosine of angle a.
/// * `sin_b`, `cos_b` the sine and cosine of angle b.
///
/// return sin(a + b)
#[must_use]
pub fn sine_sum(
    sin_a: UnitNegRange,
    cos_a: UnitNegRange,
    sin_b: UnitNegRange,
    cos_b: UnitNegRange,
) -> UnitNegRange {
    sine_diff(sin_a, cos_a, -sin_b, cos_b)
}

/// Calculate the cosine of the difference of two angles: a - b.
/// * `sin_a`, `cos_a` the sine and cosine of angle a.
/// * `sin_b`, `cos_b` the sine and cosine of angle b.
///
/// return cos(a - b)
#[must_use]
pub fn cosine_diff(
    sin_a: UnitNegRange,
    cos_a: UnitNegRange,
    sin_b: UnitNegRange,
    cos_b: UnitNegRange,
) -> UnitNegRange {
    UnitNegRange::clamp(cos_a.0 * cos_b.0 + sin_a.0 * sin_b.0)
}

/// Calculate the cosine of the sum of two angles: a + b.
/// * `sin_a`, `cos_a` the sine and cosine of angle a.
/// * `sin_b`, `cos_b` the sine and cosine of angle b.
///
/// return cos(a + b)
#[must_use]
pub fn cosine_sum(
    sin_a: UnitNegRange,
    cos_a: UnitNegRange,
    sin_b: UnitNegRange,
    cos_b: UnitNegRange,
) -> UnitNegRange {
    cosine_diff(sin_a, cos_a, -sin_b, cos_b)
}

/// An angle represented by it's sine and cosine as `UnitNegRanges`.
///
/// This representation was chosen to simplify spherical trigonometry
/// calculations such as the `haversine_distance` function which are performed
/// on the sines and cosines of the latitude and longitude differences.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Angle {
    /// The sine of the angle.
    s: UnitNegRange,
    /// The cosine of the angle.
    c: UnitNegRange,
}

/// A default angle: zero degrees or radians.
impl Default for Angle {
    /// Implementation of Default for Angle returns Angle(0.0, 1.0),
    /// i.e. the Angle corresponding to zero degrees or radians.
    /// # Examples
    /// ```
    /// use via_sphere::trig::Angle;
    ///
    /// let angle_zero = Angle::default();
    /// assert_eq!(0.0, angle_zero.sin());
    /// assert_eq!(1.0, angle_zero.cos());
    /// ```
    fn default() -> Self {
        Self::new(UnitNegRange(0.0), UnitNegRange(1.0))
    }
}

impl Neg for Angle {
    type Output = Self;

    /// An implementation of Neg for Angle, i.e. -angle.  
    /// Negates the sine of the Angle, does not affect the cosine.
    /// # Examples
    /// ```
    /// use via_sphere::trig::Degrees;
    /// use via_sphere::trig::Angle;
    ///
    /// let angle_45 = Angle::from(Degrees(45.0));
    /// let result_m45 = -angle_45;
    /// assert_eq!(Degrees(-45.0), Degrees::from(result_m45));
    /// ```
    fn neg(self) -> Self {
        Self::new(-self.s, self.c)
    }
}

impl Sub for Angle {
    type Output = Self;

    /// Subtract two Angles, i.e. a - b
    /// Uses trigonometric identity functions.
    /// # Examples
    /// ```
    /// use via_sphere::trig::Degrees;
    /// use via_sphere::trig::Angle;
    ///
    /// let degrees_30 = Degrees(30.0);
    /// let angle_30 = Angle::from(degrees_30);
    /// let angle_60 = Angle::from(Degrees(60.0));
    /// let result_30 = angle_60 - angle_30;
    ///
    /// // Note: subtraction is not precise...
    /// let delta_angle = libm::fabs(degrees_30.0 - Degrees::from(result_30).0);
    /// assert!(delta_angle <= 32.0 * std::f64::EPSILON);
    /// ```
    fn sub(self, other: Self) -> Self {
        Self::new(
            sine_diff(self.s, self.c, other.s, other.c),
            cosine_diff(self.s, self.c, other.s, other.c),
        )
    }
}

impl Add for Angle {
    type Output = Self;

    /// Add two Angles, i.e. a + b
    /// Uses trigonometric identity functions.
    /// # Examples
    /// ```
    /// use via_sphere::trig::Degrees;
    /// use via_sphere::trig::Angle;
    ///
    /// let angle_30 = Angle::from(Degrees(30.0));
    /// let angle_60 = Angle::from(Degrees(60.0));
    /// let result_90 = angle_30 + angle_60;
    /// assert_eq!(Degrees(90.0), Degrees::from(result_90));
    /// ```
    fn add(self, other: Self) -> Self {
        Self::new(
            sine_sum(self.s, self.c, other.s, other.c),
            cosine_sum(self.s, self.c, other.s, other.c),
        )
    }
}

impl Validate for Angle {
    /// Test whether an Angle is valid, i.e. whether s^2 + c^2 is approx = 1.0.
    fn is_valid(&self) -> bool {
        const MIN_SQ_LENGTH: f64 = 1.0 - 32.0 * std::f64::EPSILON;
        const MAX_SQ_LENGTH: f64 = 1.0 + 32.0 * std::f64::EPSILON;
        let sq_length = self.s.0 * self.s.0 + self.c.0 * self.c.0;
        (MIN_SQ_LENGTH..=MAX_SQ_LENGTH).contains(&sq_length)
    }
}

#[debug_invariant(self.is_valid())]
impl Angle {
    /// Construct an Angle from sin and cos values.
    #[must_use]
    pub const fn new(s: UnitNegRange, c: UnitNegRange) -> Self {
        Self { s, c }
    }

    /// Construct an Angle from y and x values.  
    /// Normalises the values.
    #[must_use]
    pub fn from_y_x(sine: f64, cosine: f64) -> Self {
        let length = libm::hypot(sine, cosine);

        if is_small(length, std::f64::EPSILON) {
            Self::default()
        } else {
            Self::new(
                UnitNegRange::clamp(sine / length),
                UnitNegRange::clamp(cosine / length),
            )
        }
    }

    /// The sine of the Angle.
    pub fn sin(self) -> f64 {
        self.s.0
    }

    /// The cosine of the Angle.
    pub fn cos(self) -> f64 {
        self.c.0
    }

    /// The Angle in Radians.
    #[debug_ensures(ret.is_valid())]
    pub fn to_radians(self) -> Radians {
        Radians(libm::atan2(self.s.0, self.c.0))
    }

    /// Test whether an Angle value is a valid latitude,
    /// i.e. whether: 0.0 <= Angle.cos
    pub fn is_valid_latitude(&self) -> bool {
        0.0 <= self.c.0
    }

    /// The absolute value of the angle, i.e. the angle with a positive sine.
    /// # Examples
    /// ```
    /// use via_sphere::trig::Degrees;
    /// use via_sphere::trig::Angle;
    ///
    /// let angle_m45 = Angle::from(Degrees(-45.0));
    /// let result_45 = angle_m45.abs();
    /// assert_eq!(Degrees(45.0), Degrees::from(result_45));
    /// ```
    pub fn abs(self) -> Self {
        Self::new(UnitNegRange(libm::fabs(self.s.0)), self.c)
    }

    /// Negate the cosine of the Angle.  
    /// I.e. Pi - angle.radians() for positive angles,  
    ///      angle.radians() + Pi for negative angles
    /// # Examples
    /// ```
    /// use via_sphere::trig::Degrees;
    /// use via_sphere::trig::Angle;
    ///
    /// let angle_45 = Angle::from(Degrees(45.0));
    /// let result_45 = angle_45.negate_cos();
    /// assert_eq!(Degrees(135.0), Degrees::from(result_45));
    /// ```
    pub fn negate_cos(self) -> Self {
        Self::new(self.s, -self.c)
    }

    /// The opposite angle on the circle, i.e. +/- 180 degrees.
    /// # Examples
    /// ```
    /// use via_sphere::trig::Degrees;
    /// use via_sphere::trig::Angle;
    ///
    /// let angle_m30 = Angle::from(Degrees(-30.0));
    /// let result_150 = angle_m30.opposite();
    /// assert_eq!(Degrees(150.0), Degrees::from(result_150));
    /// ```
    pub fn opposite(self) -> Self {
        Self::new(-self.s, -self.c)
    }

    /// Two times the Angle.  
    /// See: [Double-angle formulae](https://en.wikipedia.org/wiki/List_of_trigonometric_identities#Double-angle_formulae)
    /// # Examples
    /// ```
    /// use via_sphere::trig::Degrees;
    /// use via_sphere::trig::Angle;
    ///
    /// let angle_30 = Angle::from(Degrees(30.0));
    /// let result_60 = angle_30.x2();
    ///
    /// // Note: multiplication is not precise...
    /// // assert_eq!(Degrees(60.0), Degrees::from(result_60));
    /// let delta_angle = libm::fabs(60.0 - Degrees::from(result_60).0);
    /// assert!(delta_angle <= 32.0 * std::f64::EPSILON);
    /// ```
    pub fn x2(self) -> Self {
        Self::new(
            UnitNegRange::clamp(2.0 * self.s.0 * self.c.0),
            UnitNegRange::clamp((self.c.0 - self.s.0) * (self.c.0 + self.s.0)),
        )
    }

    /// Square of the sine of half the Angle.  
    /// See: [Half-angle formulae](https://en.wikipedia.org/wiki/List_of_trigonometric_identities#Half-angle_formulae)
    /// # Examples
    /// ```
    /// use via_sphere::trig::Degrees;
    /// use via_sphere::trig::Angle;
    ///
    /// let angle_30 = Angle::from(Degrees(30.0));
    /// let angle_60 = Angle::from(Degrees(60.0));
    /// let expected = angle_30.sin() * angle_30.sin();
    /// let result = angle_60.sq_sine_half();
    ///
    /// assert_eq!(expected, result);
    /// ```
    #[debug_ensures((0.0..=1.0).contains(&ret))]
    pub fn sq_sine_half(self) -> f64 {
        (1.0 - self.c.0) * 0.5
    }

    /// Square of the cosine of half the Angle.  
    /// See: [Half-angle formulae](https://en.wikipedia.org/wiki/List_of_trigonometric_identities#Half-angle_formulae)
    /// # Examples
    /// ```
    /// use via_sphere::trig::Degrees;
    /// use via_sphere::trig::Angle;
    ///
    /// let angle_30 = Angle::from(Degrees(30.0));
    /// let angle_60 = Angle::from(Degrees(60.0));
    /// let expected = angle_30.cos() * angle_30.cos();
    /// let result = angle_60.sq_cosine_half();
    ///
    /// // assert_eq!(expected, result);  // Does not work, not accurate enough.
    /// let delta_result = libm::fabs(expected - result);
    /// assert!(delta_result <= std::f64::EPSILON);
    /// ```
    #[debug_ensures((0.0..=1.0).contains(&ret))]
    pub fn sq_cosine_half(self) -> f64 {
        (1.0 + self.c.0) * 0.5
    }
}

impl From<Degrees> for Angle {
    /// Construct an Angle from an angle in Degrees.
    /// In order to minimize round-off errors, this function calculates sines
    /// of angles with sine values <= 1 / sqrt(2): see
    /// <https://stackoverflow.com/questions/31502120/sin-and-cos-give-unexpected-results-for-well-known-angles>
    /// It is based on `GeographicLib::Math::sincosd` function.
    fn from(a: Degrees) -> Self {
        let rq = libm::remquo(a.0, 90.0);

        // Default is zero degrees.
        let mut s = UnitNegRange(0.0);
        let mut c = UnitNegRange(1.0);
        let abs_angle = libm::fabs(rq.0);
        if abs_angle > 0.0 {
            // 45 degrees is a special case
            if abs_angle == 45.0 {
                c = UnitNegRange(std::f64::consts::FRAC_1_SQRT_2);
                s = UnitNegRange(libm::copysign(c.0, rq.0));
            } else {
                // 30 degrees is also a special case
                s = UnitNegRange(if abs_angle == 30.0 {
                    libm::copysign(0.5, rq.0)
                } else {
                    libm::sin(rq.0.to_radians())
                });
                c = swap_sin_cos(s);
            }
        }

        match rq.1 & 3 {
            0 => Self::new(s, c),
            1 => Self::new(c, -s),
            2 => Self::new(-s, -c),
            _ => Self::new(-c, s),
        }
    }
}

impl From<Radians> for Angle {
    /// Construct an Angle from an angle in Radians.
    /// In order to minimize round-off errors, this function calculates sines
    /// of angles with sine values <= 1 / sqrt(2)
    fn from(a: Radians) -> Self {
        const FRAC_3_PI_4: f64 = std::f64::consts::PI - std::f64::consts::FRAC_PI_4;

        let valid_angle = a.normalise();
        let abs_angle = libm::fabs(valid_angle.0);

        let over_45_degrees = std::f64::consts::FRAC_PI_4 < abs_angle;
        let under_135_degrees = abs_angle < FRAC_3_PI_4;
        if over_45_degrees && under_135_degrees {
            let c = UnitNegRange(libm::sin(std::f64::consts::FRAC_PI_2 - abs_angle));
            let s = cosine_from_sine(UnitNegRange(c.0), valid_angle.0);

            Self::new(s, c)
        } else {
            let s = UnitNegRange(libm::sin(valid_angle.0));
            let c = cosine_from_sine(UnitNegRange(s.0), std::f64::consts::FRAC_PI_2 - abs_angle);

            Self::new(s, c)
        }
    }
}

impl From<Angle> for Degrees {
    /// Convert an Angle to Degrees.
    #[debug_ensures(ret.is_valid())]
    fn from(a: Angle) -> Self {
        Self::from(a.to_radians())
    }
}

impl Serialize for Angle {
    /// Serialize an Angle to an value in Degrees.
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        serializer.serialize_newtype_struct("Degrees", &Degrees::from(*self))
    }
}

impl<'de> Deserialize<'de> for Angle {
    /// Deserialize an value in Degrees to an Angle.
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        Ok(Self::from(Degrees::deserialize(deserializer)?))
    }
}

/// Calculate the square of the Euclidean distance (i.e. using Pythagoras)
/// between two points from their Latitudes and their Longitude difference.
/// * `lat_a` - start point Latitude.
/// * `lat_b` - finish point Latitude.
/// * `delta_long` - Longitude difference between start and finish points.
///
/// returns the square of the Euclidean distance between the points.
#[debug_ensures((0.0..=4.0).contains(&ret))]
pub fn sq_euclidean_distance(lat_a: Angle, lat_b: Angle, delta_long: Angle) -> f64 {
    let delta_x = lat_b.cos().mul_add(delta_long.cos(), -lat_a.cos());
    let delta_y = lat_b.cos() * delta_long.sin();
    let delta_z = lat_b.sin() - lat_a.sin();

    let result = delta_x * delta_x + delta_y * delta_y + delta_z * delta_z;
    clamp(result, 0.0, 4.0)
}

/// Calculate the Great Circle distance (angle from centre) between two points
/// from their Latitudes and their Longitude difference.
/// This function is more accurate than `haversine_distance`.
/// * `lat_a` - start point Latitude.
/// * `lat_b` - finish point Latitude.
/// * `delta_long` - Longitude difference between start and finish points.
///
/// returns the Great Circle distance between the points in Radians.
#[must_use]
pub fn calculate_gc_distance(lat_a: Angle, lat_b: Angle, delta_long: Angle) -> Radians {
    e2gc_distance(libm::sqrt(sq_euclidean_distance(lat_a, lat_b, delta_long)))
}

/// Calculate the Great Circle distance (angle from centre) between two points
/// from their Latitudes and their Longitude difference.
/// This function is less accurate than `calculate_gc_distance`.
/// * `lat_a` - start point Latitude.
/// * `lat_b` - finish point Latitude.
/// * `delta_long` - Longitude difference between start and finish points.
///
/// returns the Great Circle distance between the points in Radians.
#[must_use]
pub fn haversine_distance(lat_a: Angle, lat_b: Angle, delta_long: Angle) -> Radians {
    let delta_lat = lat_b - lat_a;
    let haversine_lat = delta_lat.sq_sine_half();
    let haversine_lon = delta_long.sq_sine_half();

    let a = clamp(
        haversine_lat + lat_a.cos() * lat_b.cos() * haversine_lon,
        0.0,
        1.0,
    );

    Radians(2.0 * libm::asin(libm::sqrt(a)))
}

/// Calculate the azimuth (bearing) along the great circle of point b from
/// point a from their Latitudes and their Longitude difference.
/// * `lat_a` - start point Latitude.
/// * `lat_b` - finish point Latitude.
/// * `delta_long` - Longitude difference between start and finish points.
///
/// returns the Great Circle azimuth relative to North of point b from point a
/// as an Angle.
#[must_use]
pub fn calculate_gc_azimuth(lat_a: Angle, lat_b: Angle, delta_long: Angle) -> Angle {
    let sin_azimuth = lat_b.cos() * delta_long.sin();
    let temp = (lat_a.sin() * lat_b.cos() * delta_long.sin() * delta_long.sin())
        / (1.0 + libm::fabs(delta_long.cos()));
    let cos_azimuth = if delta_long.cos() < 0.0 {
        lat_b.sin() * lat_a.cos() + lat_a.sin() * lat_b.cos() - temp
    } else {
        lat_b.sin() * lat_a.cos() - lat_a.sin() * lat_b.cos() + temp
    };

    Angle::from_y_x(sin_azimuth, cos_azimuth)
}

/// Determine whether a slice of values are all valid latitudes in degrees.
#[must_use]
pub fn valid_latitudes(values: &[f64]) -> bool {
    values.iter().all(|&v| Degrees::is_valid_latitude(v))
}

/// Determine whether a slice of values are all valid longitudes in degrees.
#[must_use]
pub fn valid_longitudes(values: &[f64]) -> bool {
    values.iter().all(|&v| Degrees::is_valid_longitude(v))
}

/// A vector of Angles.
pub type Angles = Vec<Angle>;

/// Convert a slice of values into Angles.
#[must_use]
pub fn from_degrees(values: &[f64]) -> Angles {
    values.iter().map(|&v| Angle::from(Degrees(v))).collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::is_within_tolerance;
    use serde_json::to_string;

    #[test]
    fn degrees_traits() {
        let zero = Degrees(0.0);

        let zero_clone = zero.clone();
        assert!(zero_clone == zero);
        assert!(!(zero_clone < zero));

        print!("Degrees: {:?}", zero);
    }

    #[test]
    fn degrees_bearing() {
        assert!(!Degrees::is_valid_bearing(0.0 - std::f64::EPSILON));
        assert!(Degrees::is_valid_bearing(0.0));
        assert!(Degrees::is_valid_bearing(360.0));
        assert!(!Degrees::is_valid_bearing(
            360.0 * (1.0 + std::f64::EPSILON)
        ));
    }

    #[test]
    fn degrees_normalise() {
        assert_eq!(0.0, Degrees(-360.0).normalise().0);
        assert_eq!(180.0, Degrees(-180.0).normalise().0);
        assert_eq!(180.0, Degrees(180.0).normalise().0);
        assert_eq!(0.0, Degrees(360.0).normalise().0);
    }

    #[test]
    fn degrees_maths() {
        // Neg
        let angle_45 = Degrees(45.0);
        let result_m45 = -angle_45;
        assert_eq!(Degrees(-45.0), result_m45);

        // Add
        let angle_120 = Degrees(120.0);
        let result = angle_120 + angle_120;
        assert_eq!(Degrees(-120.0), result);

        // Sub
        let angle_m120 = Degrees(-120.0);
        let result = angle_m120 - angle_120;
        assert_eq!(angle_120, result);
    }

    #[test]
    fn radians_traits() {
        let zero = Radians(0.0);

        let zero_clone = zero.clone();
        assert!(zero_clone == zero);
        assert!(!(zero_clone < zero));

        print!("Radians: {:?}", zero);
    }

    #[test]
    fn radians_normalise() {
        assert_eq!(0.0, Radians(-2.0 * std::f64::consts::PI).normalise().0);
        assert_eq!(
            std::f64::consts::PI,
            Radians(-std::f64::consts::PI).normalise().0
        );
        assert_eq!(
            std::f64::consts::PI,
            Radians(std::f64::consts::PI).normalise().0
        );
        assert_eq!(0.0, Radians(2.0 * std::f64::consts::PI).normalise().0);
    }

    #[test]
    fn radians_maths() {
        // Neg
        let angle_45 = Radians(DEG2RAD * 45.0);
        let result_m45 = -angle_45;
        assert_eq!(Radians(-std::f64::consts::FRAC_PI_4), result_m45);

        // Add
        let angle_120 = Radians(DEG2RAD * 120.0);
        let result = angle_120 + angle_120;
        // Note: wrapping is not precise...
        // assert_eq!(Radians(-2.0 * std::f64::consts::FRAC_PI_3), result);
        assert!(is_within_tolerance(
            -2.0 * std::f64::consts::FRAC_PI_3,
            result.0,
            4.0 * std::f64::EPSILON
        ));

        // Sub
        let angle_m120 = Radians(-2.0 * std::f64::consts::FRAC_PI_3);
        let result = angle_m120 - angle_120;
        // Note: wrapping is not precise...
        // assert_eq!(angle_120, result);
        assert!(is_within_tolerance(
            angle_120.0,
            result.0,
            4.0 * std::f64::EPSILON
        ));
    }

    #[test]
    fn test_spherical_cosine_rule() {
        let a = spherical_cosine_rule(0.5, Radians(0.5 * std::f64::consts::FRAC_PI_2));
        // print!("Radians: {:?}", a);
        assert!(is_within_tolerance(
            0.4636476090008061,
            a.0,
            std::f64::EPSILON
        ));
    }

    #[test]
    #[should_panic]
    fn test_spherical_cosine_rule_requirements_negative_length() {
        let _ = spherical_cosine_rule(0.5, Radians(-std::f64::EPSILON));
    }

    #[test]
    #[should_panic]
    fn test_spherical_cosine_rule_requirements_length_too_long() {
        let _ = spherical_cosine_rule(
            0.5,
            Radians(std::f64::consts::FRAC_PI_2 + std::f64::EPSILON),
        );
    }

    #[test]
    fn test_distance_functions() {
        assert_eq!(std::f64::consts::PI, e2gc_distance(2.1).0);
        assert!(is_within_tolerance(
            std::f64::consts::FRAC_PI_2,
            e2gc_distance(std::f64::consts::SQRT_2).0,
            std::f64::EPSILON
        ));

        assert_eq!(2.0, gc2e_distance(Radians(std::f64::consts::PI)));
        assert!(is_within_tolerance(
            std::f64::consts::SQRT_2,
            gc2e_distance(Radians(std::f64::consts::FRAC_PI_2)),
            std::f64::EPSILON
        ));
    }

    #[test]
    fn unit_neg_range_traits() {
        let zero = UnitNegRange(0.0);

        let zero_clone = zero.clone();
        assert!(zero_clone == zero);
        assert!(!(zero_clone < zero));

        print!("Radians: {:?}", zero);
    }

    #[test]
    fn unit_neg_range_clamp() {
        let value = UnitNegRange::clamp(-1.0 - std::f64::EPSILON);
        assert_eq!(-1.0, value.0);
    }

    #[test]
    fn unit_neg_range_is_valid() {
        assert!(!UnitNegRange(-1.0 - std::f64::EPSILON).is_valid());
        assert!(UnitNegRange(-1.0).is_valid());
        assert!(UnitNegRange(1.0).is_valid());
        assert!(!UnitNegRange(1.0 + std::f64::EPSILON).is_valid());
    }

    #[test]
    fn angle_traits() {
        let zero = Angle::from(Degrees(0.0));

        let zero_clone = zero.clone();
        assert!(zero_clone == zero);

        print!("Radians: {:?}", zero);
    }

    #[test]
    fn angle_from_y_x() {
        // large x, y positions
        let angle = Angle::from_y_x(100.0, 100.0);
        assert_eq!(Degrees(45.0), Degrees::from(angle));

        // small x, y positions
        let angle = Angle::from_y_x(std::f64::EPSILON, std::f64::EPSILON);
        assert_eq!(Degrees(45.0), Degrees::from(angle));

        // zero x, y positions
        let angle = Angle::from_y_x(0.0, 0.0);
        assert_eq!(Degrees(0.0), Degrees::from(angle));
    }

    #[test]
    fn angle_constructor_zero() {
        let angle = Angle::default();
        assert!(angle.is_valid());
        assert_eq!(0.0, angle.sin());
        assert_eq!(1.0, angle.cos());

        assert_eq!(Radians(0.0), angle.to_radians());
        assert_eq!(Degrees(0.0), Degrees::from(angle));

        assert_eq!(Degrees(0.0), Degrees::from(-angle));
        assert_eq!(Radians(std::f64::consts::PI), angle.opposite().to_radians());
        assert_eq!(Degrees(180.0), Degrees::from(angle.opposite()));
    }

    #[test]
    fn angle_constructor_20_degrees() {
        let angle = Angle::from(Degrees(20.0));
        assert!(angle.is_valid());
        let sc = libm::sincos(DEG2RAD * 20.0);
        assert_eq!(sc.0, angle.sin());
        assert_eq!(sc.1, angle.cos());

        assert_eq!(Degrees(20.0), Degrees::from(angle));
        assert_eq!(Degrees(-20.0), Degrees::from(-angle));
        assert_eq!(Degrees(-160.0), Degrees::from(angle.opposite()));
    }

    #[test]
    fn angle_constructor_30_degrees() {
        let angle = Angle::from(Degrees(30.0));
        assert!(angle.is_valid());
        assert_eq!(UnitNegRange(0.5), angle.s);
        assert_eq!(swap_sin_cos(UnitNegRange(0.5)), angle.c);

        // assert_eq!(Degrees(30.0), Degrees::from(angle));
        assert!(is_within_tolerance(
            Degrees(30.0).0,
            Degrees::from(angle).0,
            16.0 * std::f64::EPSILON
        ));

        assert_eq!(Degrees(-150.0), Degrees::from(angle.opposite()));
    }

    #[test]
    fn angle_constructor_45_degrees() {
        let angle = Angle::from(Degrees(45.0));
        assert!(angle.is_valid());
        assert_eq!(std::f64::consts::FRAC_1_SQRT_2, angle.sin());
        assert_eq!(std::f64::consts::FRAC_1_SQRT_2, angle.cos());

        assert_eq!(Degrees(45.0), Degrees::from(angle));
        assert_eq!(Degrees(-45.0), Degrees::from(-angle));
        assert_eq!(Radians(std::f64::consts::PI / 4.0), angle.to_radians());

        assert_eq!(Degrees(-135.0), Degrees::from(angle.opposite()));
    }

    #[test]
    fn angle_constructor_minus_45_degrees() {
        let angle = Angle::from(Degrees(-45.0));
        assert!(angle.is_valid());

        assert_eq!(-std::f64::consts::FRAC_1_SQRT_2, angle.sin());
        assert_eq!(std::f64::consts::FRAC_1_SQRT_2, angle.cos());

        assert_eq!(Degrees(-45.0), Degrees::from(angle));
        assert_eq!(Degrees(45.0), Degrees::from(angle.abs()));

        assert_eq!(Degrees(135.0), Degrees::from(angle.opposite()));
        assert_eq!(Degrees(-135.0), Degrees::from(angle.negate_cos()));
    }

    #[test]
    fn angle_constructor_minus_50_degrees() {
        let tolerance = 1.0e-14;
        let angle = Angle::from(Degrees(-50.0));
        assert!(angle.is_valid());
        let sc = libm::sincos(DEG2RAD * -50.0);
        // assert_eq!(sc.0, angle.s.0);
        assert!(is_within_tolerance(sc.0, angle.s.0, tolerance));
        // assert_eq!(sc.1, angle.c.0);
        assert!(is_within_tolerance(sc.1, angle.c.0, tolerance));
        // assert_eq!(Degrees(-50.0), Degrees::from(angle));
        assert!(is_within_tolerance(
            Degrees(-50.0).0,
            Degrees::from(angle).0,
            32.0 * std::f64::EPSILON
        ));
        assert_eq!(Degrees(130.0), Degrees::from(angle.opposite()));
    }

    #[test]
    fn angle_constructor_plus_50_degrees() {
        let tolerance = 1.0e-14;
        let angle = Angle::from(Degrees(50.0));
        assert!(angle.is_valid());
        let sc = libm::sincos(DEG2RAD * 50.0);
        // assert_eq!(sc.0, angle.s.0);
        assert!(is_within_tolerance(sc.0, angle.s.0, tolerance));
        // assert_eq!(sc.1, angle.c.0);
        assert!(is_within_tolerance(sc.1, angle.c.0, tolerance));
        // assert_eq!(Degrees(50.0), Degrees::from(angle));
        assert!(is_within_tolerance(
            Degrees(50.0).0,
            Degrees::from(angle).0,
            32.0 * std::f64::EPSILON
        ));
        assert_eq!(Degrees(-130.0), Degrees::from(angle.opposite()));
    }

    #[test]
    fn angle_constructor_plus_90_degrees() {
        let angle = Angle::from(Degrees(90.0));
        assert!(angle.is_valid());
        assert_eq!(Degrees(90.0), Degrees::from(angle));

        assert_eq!(Degrees(-90.0), Degrees::from(angle.opposite()));
    }

    #[test]
    fn angle_constructor_minus_120_degrees() {
        let angle = Angle::from(Degrees(-120.0));
        assert!(angle.is_valid());
        assert_eq!(-swap_sin_cos(UnitNegRange(0.5)), angle.s);

        // assert_eq!(Degrees(-120.0), Degrees::from(angle));
        assert!(is_within_tolerance(
            Degrees(-120.0).0,
            Degrees::from(angle).0,
            64.0 * std::f64::EPSILON
        ));
    }

    #[test]
    fn angle_constructor_pi_over_2() {
        let x: f64 = std::f64::consts::FRAC_PI_2;
        let angle = Angle::from(Radians(x));
        assert!(angle.is_valid());
        assert_eq!(1.0, angle.s.0);
        assert_eq!(0.0, angle.c.0);

        assert_eq!(Radians(x), angle.to_radians());
        assert_eq!(Degrees(90.0), Degrees::from(angle));
    }

    #[test]
    fn angle_constructor_minus_pi_over_2() {
        let x = -std::f64::consts::PI / 2.0;
        let angle = Angle::from(Radians(x));
        assert!(angle.is_valid());
        assert_eq!(-1.0, angle.s.0);
        assert_eq!(0.0, angle.c.0);

        assert_eq!(Radians(x), angle.to_radians());
        assert_eq!(Degrees(-90.0), Degrees::from(angle));
    }

    #[test]
    fn angle_constructor_radians_minus_30_degrees() {
        let x = -1.0 * std::f64::consts::FRAC_PI_6;
        let angle = Angle::from(Radians(x));
        assert!(angle.is_valid());
        assert_eq!(-0.5, angle.s.0);
        assert_eq!(swap_sin_cos(UnitNegRange(0.5)).0, angle.c.0);
    }

    #[test]
    fn angle_constructor_radians_minus_120_degrees() {
        let x = -2.0 * std::f64::consts::FRAC_PI_3;
        let angle = Angle::from(Radians(x));
        assert!(angle.is_valid());
        assert!(is_within_tolerance(
            -swap_sin_cos(UnitNegRange(0.5)).0,
            angle.s.0,
            std::f64::EPSILON
        ));
        assert!(is_within_tolerance(-0.5, angle.cos(), std::f64::EPSILON));
    }

    #[test]
    fn angle_constructor_pi() {
        let angle = Angle::from(Degrees(180.0));
        assert!(angle.is_valid());
        assert_eq!(0.0, angle.s.0);
        assert_eq!(-1.0, angle.c.0);

        assert_eq!(Degrees(180.0), Degrees::from(angle));
        assert_eq!(Degrees(0.0), Degrees::from(angle.opposite()));
    }

    #[test]
    fn angle_add() {
        let degrees_30 = Degrees(30.0);
        let angle_30 = Angle::from(degrees_30);

        let degrees_60 = Degrees(60.0);
        let angle_60 = Angle::from(degrees_60);
        let result_90 = angle_30 + angle_60;

        // assert_eq!(1.0, result_90.s.0); // Does not work, not accurate enough.
        assert!(is_within_tolerance(1.0, result_90.s.0, std::f64::EPSILON));
        assert_eq!(0.0, result_90.c.0);

        assert_eq!(Degrees(90.0), Degrees::from(result_90));
    }

    #[test]
    fn angle_sub() {
        let degrees_30 = Degrees(30.0);
        let angle_30 = Angle::from(degrees_30);

        let degrees_60 = Degrees(60.0);
        let angle_60 = Angle::from(degrees_60);
        let result_30 = angle_60 - angle_30;

        // assert_eq!(angle_30.s.0, result_30.s.0); // Does not work, not accurate enough.
        assert!(is_within_tolerance(0.5, result_30.s.0, std::f64::EPSILON));
        assert_eq!(angle_30.c.0, result_30.c.0);

        // assert_eq!(Degrees(30.0), Degrees::from(result_30)); // Does not work, not accurate enough.
        assert!(is_within_tolerance(
            degrees_30.0,
            Degrees::from(result_30).0,
            32.0 * std::f64::EPSILON
        ));
    }

    #[test]
    fn test_x2() {
        for i in 0..90 {
            let value = i as f64;
            let half_angle = Angle::from(Degrees(0.5 * value));
            let result = Degrees::from(half_angle.x2()).0;

            // assert_eq!(value, result); // Does not work, not accurate enough.
            assert!(is_within_tolerance(
                value,
                result,
                100.0 * std::f64::EPSILON
            ));
        }
    }

    #[test]
    fn test_sq_sine_half() {
        for i in 0..180 {
            let value = i as f64;
            let half_value = 0.5 * value;
            let half_angle = Angle::from(Degrees(half_value));
            let expected = half_angle.sin() * half_angle.sin();
            let angle = Angle::from(Degrees(value));
            let result = angle.sq_sine_half();

            // assert_eq!(expected, result);  // Does not work, not accurate enough.
            assert!(is_within_tolerance(expected, result, std::f64::EPSILON));
        }
    }

    #[test]
    fn test_sq_cosine_half() {
        for i in 0..180 {
            let value = i as f64;
            let half_value = 0.5 * value;
            let half_angle = Angle::from(Degrees(half_value));
            let expected = half_angle.cos() * half_angle.cos();
            let angle = Angle::from(Degrees(value));
            let result = angle.sq_cosine_half();

            // assert_eq!(expected, result);  // Does not work, not accurate enough.
            assert!(is_within_tolerance(expected, result, std::f64::EPSILON));
        }
    }

    #[test]
    fn test_calculate_gc_distance_delta_latitude() {
        let start_latitude = -80.0;
        let start_angle = Angle::from(Degrees(start_latitude));
        let delta_lon = Angle::default();

        for i in 1..160 {
            let value = i as f64;
            let latitude = start_latitude + value;
            let angle = Angle::from(Degrees(latitude));
            let expected = DEG2RAD * value;
            let distance = calculate_gc_distance(start_angle, angle, delta_lon);

            // assert_eq!(expected, distance.0); // Does not work, not accurate enough.
            assert!(is_within_tolerance(
                expected,
                distance.0,
                4.0 * std::f64::EPSILON
            ));
        }
    }

    #[test]
    fn test_haversine_distance_delta_latitude() {
        let start_latitude = -80.0;
        let start_angle = Angle::from(Degrees(start_latitude));
        let delta_lon = Angle::default();

        for i in 1..160 {
            let value = i as f64;
            let latitude = start_latitude + value;
            let angle = Angle::from(Degrees(latitude));
            let expected = DEG2RAD * value;
            let distance = haversine_distance(start_angle, angle, delta_lon);

            // assert_eq!(expected, distance.0); // Does not work, not accurate enough.
            assert!(is_within_tolerance(
                expected,
                distance.0,
                32.0 * std::f64::EPSILON
            ));
        }
    }

    #[test]
    fn test_distance_delta_longitude() {
        let end_latitude = 40.0;
        let start_angle = Angle::from(Degrees(-end_latitude));
        let end_angle = Angle::from(Degrees(end_latitude));

        for i in 1..160 {
            let value = i as f64;
            let delta_lon = Angle::from(Degrees(value));
            let expected = calculate_gc_distance(start_angle, end_angle, delta_lon);
            let distance = haversine_distance(start_angle, end_angle, delta_lon);

            // assert_eq!(expected.0, distance.0);  // Does not work, not accurate enough.
            assert!(is_within_tolerance(
                expected.0,
                distance.0,
                8.0 * std::f64::EPSILON
            ));
        }
    }

    #[test]
    fn test_serde_degrees() {
        let degrees_20 = Degrees(20.0);

        let serialized = to_string(&degrees_20).unwrap();
        // println!("serialized = {}", serialized);
        let deserialized: Degrees = serde_json::from_str(&serialized).unwrap();
        assert_eq!(degrees_20, deserialized);
    }

    #[test]
    fn test_serde_angle() {
        let angle_20 = Angle::from(Degrees(20.0));

        let serialized = to_string(&angle_20).unwrap();
        // println!("serialized = {}", serialized);
        let deserialized: Angle = serde_json::from_str(&serialized).unwrap();
        assert_eq!(angle_20, deserialized);
    }

    #[test]
    fn test_valid_latitudes() {
        let valid_lats = vec![90.0, 46.0, 46.0, -90.0];
        assert!(valid_latitudes(&valid_lats));

        let invalid_lats = vec![90.01, 46.0, 46.0, -90.01];
        assert!(!valid_latitudes(&invalid_lats));
    }

    #[test]
    fn test_valid_longitudes() {
        let valid_lons = vec![180.0, 46.0, 46.0, -180.0];
        assert!(valid_longitudes(&valid_lons));

        let invalid_lons = vec![180.01, 46.0, 46.0, -180.01];
        assert!(!valid_longitudes(&invalid_lons));
    }

    #[test]
    fn test_from_degrees() {
        let degrees = vec![44.0, 46.0, 46.0, 44.0];
        let angles = from_degrees(&degrees);

        assert_eq!(degrees.len(), angles.len());
        for i in 0..degrees.len() {
            assert_eq!(degrees[i], Degrees::from(angles[i]).0);
        }
    }
}
