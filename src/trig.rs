// Copyright (c) 2020-2023 Via Technology Ltd. All Rights Reserved.
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

#![allow(clippy::float_cmp)]

use super::{clamp, Validate};
use contracts::*;
use core::cmp::Eq; // , Ordering};
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
    pub fn is_latitude(value: f64) -> bool {
        (-90.0..=90.0).contains(&value)
    }

    /// Test whether a value is a valid longitude.  
    /// I.e. whether it lies in the range: -180.0 <= value <= 180.0
    pub fn is_longitude(value: f64) -> bool {
        (-180.0..=180.0).contains(&value)
    }

    /// Test whether a value is a valid bearing.  
    /// I.e. whether it lies in the range: 0.0 <= value <= 360.0
    pub fn is_bearing(value: f64) -> bool {
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
    pub fn normalise(&self) -> Degrees {
        if self.0 <= -180.0 {
            Degrees(self.0 + 360.0)
        } else if self.0 <= 180.0 {
            *self
        } else {
            Degrees(self.0 - 360.0)
        }
    }

    /// Test whether a Degrees value is a valid latitude.  
    /// I.e. whether it lies in the range: -90.0 <= value <= 90.0
    /// # Examples
    /// ```
    /// use via_sphere::trig::Degrees;
    /// use via_sphere::Validate;
    ///
    /// assert!(!Degrees(-90.0 * (1.0 + std::f64::EPSILON)).is_valid_latitude());
    /// assert!(Degrees(-90.0).is_valid_latitude());
    /// assert!(Degrees(90.0).is_valid_latitude());
    /// assert!(!(Degrees(90.0 * (1.0 + std::f64::EPSILON)).is_valid_latitude()));
    /// ```
    pub fn is_valid_latitude(&self) -> bool {
        Degrees::is_latitude(self.0)
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
        Degrees::is_longitude(self.0)
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
    type Output = Degrees;

    /// An implementation of Neg for Degrees, i.e. -angle.
    /// # Examples
    /// ```
    /// use via_sphere::trig::Degrees;
    ///
    /// let angle_45 = Degrees(45.0);
    /// let result_m45 = -angle_45;
    /// assert_eq!(Degrees(-45.0), result_m45);
    /// ```
    fn neg(self) -> Degrees {
        Degrees(-self.0)
    }
}

impl Add for Degrees {
    type Output = Degrees;

    /// Add two angles in Degrees, automatically wraps around +/- 180 degrees.
    /// # Examples
    /// ```
    /// use via_sphere::trig::Degrees;
    ///
    /// let angle_120 = Degrees(120.0);
    /// let result = angle_120 + angle_120;
    /// assert_eq!(Degrees(-120.0), result);
    /// ```
    fn add(self, other: Degrees) -> Degrees {
        Degrees(self.0 + other.0).normalise()
    }
}

impl Sub for Degrees {
    type Output = Degrees;

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
    fn sub(self, other: Degrees) -> Degrees {
        Degrees(self.0 - other.0).normalise()
    }
}

/// The conversion factor from Degrees to Radians.
pub const DEG2RAD: f64 = std::f64::consts::PI / 180.0;

impl Radians {
    /// Normalise a Radians into the range:
    /// -std::f64::consts::PI < value <= std::f64::consts::PI
    /// # Examples
    /// ```
    /// use via_sphere::trig::Radians;
    ///
    /// assert_eq!(0.0, Radians(-2.0 * std::f64::consts::PI).normalise().0);
    /// assert_eq!(std::f64::consts::PI, Radians(-std::f64::consts::PI).normalise().0);
    /// assert_eq!(std::f64::consts::PI, Radians(std::f64::consts::PI).normalise().0);
    /// assert_eq!(0.0, Radians(2.0 * std::f64::consts::PI).normalise().0);
    /// ```
    pub fn normalise(&self) -> Radians {
        if self.0 <= -std::f64::consts::PI {
            Radians(self.0 + 2.0 * std::f64::consts::PI)
        } else if self.0 <= std::f64::consts::PI {
            *self
        } else {
            Radians(self.0 - 2.0 * std::f64::consts::PI)
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

impl From<Degrees> for Radians {
    /// Construct an angle in Radians from an angle in Degrees.
    /// # Examples
    /// ```
    /// use via_sphere::trig::Degrees;
    /// use via_sphere::trig::Radians;
    ///
    /// let arg = Degrees(-90.0);
    /// let answer = Radians::from(arg);
    /// assert_eq!(-std::f64::consts::FRAC_PI_2, answer.0);
    /// ```
    fn from(a: Degrees) -> Self {
        Self(a.0.to_radians())
    }
}

impl Neg for Radians {
    type Output = Radians;

    /// An implementation of Neg for Radians, i.e. -angle.
    /// # Examples
    /// ```
    /// use via_sphere::trig::Radians;
    ///
    /// let angle_45 = Radians(std::f64::consts::FRAC_PI_4);
    /// let result_m45 = -angle_45;
    /// assert_eq!(Radians(-std::f64::consts::FRAC_PI_4), result_m45);
    /// ```
    fn neg(self) -> Radians {
        Radians(-self.0)
    }
}

impl Add for Radians {
    type Output = Radians;

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
    fn add(self, other: Radians) -> Radians {
        Radians(self.0 + other.0).normalise()
    }
}

impl Sub for Radians {
    type Output = Radians;

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
    fn sub(self, other: Radians) -> Radians {
        Radians(self.0 - other.0).normalise()
    }
}

impl Eq for Radians {}

// impl Ord for Radians {
//     fn cmp(&self, other: &Self) -> Ordering {
//         (self.0).partial_cmp(&(other.0)).unwrap()
//     }
// }

/// The UnitNegRange newtype an f64.  
/// A valid UnitNegRange value lies between -1.0 and +1.0 inclusive.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct UnitNegRange(pub f64);

impl Validate for UnitNegRange {
    /// Test whether a UnitNegRange is valid.  
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
    type Output = UnitNegRange;

    /// An implementation of Neg for UnitNegRange.  
    /// Negates the value.
    fn neg(self) -> UnitNegRange {
        UnitNegRange(-self.0)
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
    pub fn clamp(value: f64) -> UnitNegRange {
        UnitNegRange(clamp(value, -1.0, 1.0))
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
pub fn cosine_from_sine(a: UnitNegRange, sign: f64) -> UnitNegRange {
    UnitNegRange(libm::copysign(swap_sin_cos(a).0, sign))
}

/// Calculate the sine of the difference of two angles: a - b.
/// * `sin_a`, `cos_a` the sine and cosine of angle a.
/// * `sin_b`, `cos_b` the sine and cosine of angle b.
///
/// return sin(a - b)
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
pub fn cosine_sum(
    sin_a: UnitNegRange,
    cos_a: UnitNegRange,
    sin_b: UnitNegRange,
    cos_b: UnitNegRange,
) -> UnitNegRange {
    cosine_diff(sin_a, cos_a, -sin_b, cos_b)
}

/// An angle represented by it's sine and cosine as UnitNegRanges.
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
        Angle {
            s: UnitNegRange(0.0),
            c: UnitNegRange(1.0),
        }
    }
}

impl Neg for Angle {
    type Output = Angle;

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
    fn neg(self) -> Angle {
        Angle {
            s: -self.s,
            c: self.c,
        }
    }
}

impl Sub for Angle {
    type Output = Angle;

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
    fn sub(self, other: Angle) -> Angle {
        Angle {
            s: sine_diff(self.s, self.c, other.s, other.c),
            c: cosine_diff(self.s, self.c, other.s, other.c),
        }
    }
}

impl Add for Angle {
    type Output = Angle;

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
    fn add(self, other: Angle) -> Angle {
        Angle {
            s: sine_sum(self.s, self.c, other.s, other.c),
            c: cosine_sum(self.s, self.c, other.s, other.c),
        }
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
    pub fn new(s: UnitNegRange, c: UnitNegRange) -> Angle {
        Angle { s, c }
    }

    /// Construct an Angle from y and x values.  
    /// Normalises the values.
    pub fn from_y_x(sine: f64, cosine: f64) -> Angle {
        let length = libm::hypot(sine, cosine);

        if length > std::f64::EPSILON {
            Angle {
                s: UnitNegRange::clamp(sine / length),
                c: UnitNegRange::clamp(cosine / length),
            }
        } else {
            Angle {
                s: UnitNegRange(0.0),
                c: UnitNegRange(1.0),
            }
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
    pub fn abs(self) -> Angle {
        Angle {
            s: UnitNegRange(libm::fabs(self.s.0)),
            c: self.c,
        }
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
    pub fn negate_cos(self) -> Angle {
        Angle {
            s: self.s,
            c: -self.c,
        }
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
    pub fn opposite(self) -> Angle {
        Angle {
            s: UnitNegRange(0.0 - self.s.0),
            c: UnitNegRange(0.0 - self.c.0),
        }
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
    pub fn x2(self) -> Angle {
        Angle {
            s: UnitNegRange::clamp(2.0 * self.s.0 * self.c.0),
            c: UnitNegRange::clamp((self.c.0 - self.s.0) * (self.c.0 + self.s.0)),
        }
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
    /// It is based on GeographicLib::Math::sincosd function.
    fn from(a: Degrees) -> Self {
        let rq = libm::remquo(a.0, 90.0);

        // Default is zero degrees.
        let mut sine = UnitNegRange(0.0);
        let mut cosine = UnitNegRange(1.0);
        let abs_angle = libm::fabs(rq.0);
        if abs_angle > 0.0 {
            if abs_angle < 90.0 {
                if abs_angle < 45.0 {
                    sine = UnitNegRange(if abs_angle == 30.0 {
                        0.5
                    } else {
                        libm::sin(abs_angle.to_radians())
                    });
                    cosine = swap_sin_cos(sine);
                } else if abs_angle > 45.0 {
                    cosine = UnitNegRange(if abs_angle == 60.0 {
                        0.5
                    } else {
                        libm::cos(abs_angle.to_radians())
                    });
                    sine = swap_sin_cos(cosine);
                } else {
                    // abs_angle == 45.0
                    sine = UnitNegRange(std::f64::consts::FRAC_1_SQRT_2);
                    cosine = sine;
                }
            } else {
                // 90 degrees
                sine = UnitNegRange(1.0);
                cosine = UnitNegRange(0.0);
            }

            if rq.0 < 0.0 {
                // negative angle
                sine = UnitNegRange(0.0 - sine.0);
            }
        }

        let q = rq.1 as u32;
        match q & 3 {
            0 => Angle { s: sine, c: cosine },
            1 => Angle {
                s: cosine,
                c: UnitNegRange(0.0 - sine.0),
            },
            2 => Angle {
                s: UnitNegRange(0.0 - sine.0),
                c: UnitNegRange(0.0 - cosine.0),
            },
            _ => Angle {
                s: UnitNegRange(0.0 - cosine.0),
                c: sine,
            },
        }
    }
}

impl From<Radians> for Angle {
    /// Construct an Angle from an angle in Radians.
    /// In order to minimize round-off errors, this function calculates sines
    /// of angles with sine values <= 1 / sqrt(2)
    fn from(a: Radians) -> Self {
        const PI_4: f64 = std::f64::consts::FRAC_PI_2 / 2.0;

        let valid_angle = a.normalise();
        let abs_angle = libm::fabs(valid_angle.0);

        let over_45_degrees = PI_4 < abs_angle;
        let under_135_degrees = abs_angle < (std::f64::consts::PI - PI_4);
        if over_45_degrees && under_135_degrees {
            let c = UnitNegRange(libm::sin(std::f64::consts::FRAC_PI_2 - abs_angle));
            let s = cosine_from_sine(UnitNegRange(c.0), valid_angle.0);

            Angle { s, c }
        } else {
            let s = UnitNegRange(libm::sin(valid_angle.0));
            let c = cosine_from_sine(UnitNegRange(s.0), std::f64::consts::FRAC_PI_2 - abs_angle);

            Angle { s, c }
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
    fn deserialize<D>(deserializer: D) -> Result<Angle, D::Error>
    where
        D: Deserializer<'de>,
    {
        Ok(Angle::from(Degrees::deserialize(deserializer)?))
    }
}

/// Determine whether a slice of values are all valid latitudes in degrees.
pub fn valid_latitudes(values: &[f64]) -> bool {
    values.iter().all(|&v| Degrees::is_latitude(v))
}

/// Determine whether a slice of values are all valid longitudes in degrees.
pub fn valid_longitudes(values: &[f64]) -> bool {
    values.iter().all(|&v| Degrees::is_longitude(v))
}

/// A vector of Angles.
pub type Angles = Vec<Angle>;

/// Convert a slice of values into Angles.
pub fn from_degrees(values: &[f64]) -> Angles {
    values.iter().map(|&v| Angle::from(Degrees(v))).collect()
}

#[cfg(test)]
mod tests {
    use super::Validate;
    use crate::trig::*;
    use serde_json::to_string;

    #[test]
    fn angle_constructor_zero() {
        let angle = Angle::default();
        assert!(angle.is_valid());
        assert_eq!(0.0, angle.sin());
        assert_eq!(1.0, angle.cos());

        assert_eq!(Radians(0.0), angle.to_radians());
        assert_eq!(Degrees(0.0), Degrees::from(angle));

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
        assert_eq!(Degrees(-160.0), Degrees::from(angle.opposite()));
    }

    #[test]
    fn angle_constructor_30_degrees() {
        let angle = Angle::from(Degrees(30.0));
        assert!(angle.is_valid());
        assert_eq!(UnitNegRange(0.5), angle.s);
        assert_eq!(swap_sin_cos(UnitNegRange(0.5)), angle.c);

        // assert_eq!(Degrees(30.0), Degrees::from(angle));
        let delta_angle = libm::fabs(Degrees(30.0).0 - Degrees::from(angle).0);
        assert!(delta_angle <= 16.0 * std::f64::EPSILON);

        assert_eq!(Degrees(-150.0), Degrees::from(angle.opposite()));
    }

    #[test]
    fn angle_constructor_45_degrees() {
        let angle = Angle::from(Degrees(45.0));
        assert!(angle.is_valid());
        assert_eq!(std::f64::consts::FRAC_1_SQRT_2, angle.sin());
        assert_eq!(std::f64::consts::FRAC_1_SQRT_2, angle.cos());

        assert_eq!(Degrees(45.0), Degrees::from(angle));
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

        assert_eq!(Degrees(135.0), Degrees::from(angle.opposite()));
    }

    #[test]
    fn angle_constructor_minus_50_degrees() {
        let angle = Angle::from(Degrees(-50.0));
        assert!(angle.is_valid());
        let sc = libm::sincos(DEG2RAD * -50.0);
        // assert_eq!(sc.0, angle.s.0);
        let delta_s = libm::fabs(sc.0 - angle.s.0);
        assert!(delta_s < 1.0e-14);
        // assert_eq!(sc.1, angle.c.0);
        let delta_c = libm::fabs(sc.1 - angle.c.0);
        assert!(delta_c < 1.0e-14);

        // assert_eq!(Degrees(-50.0), Degrees::from(angle));
        let delta_angle = libm::fabs(Degrees(-50.0).0 - Degrees::from(angle).0);
        assert!(delta_angle <= 32.0 * std::f64::EPSILON);

        assert_eq!(Degrees(130.0), Degrees::from(angle.opposite()));
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
        let delta_s = 1.0 - result_90.s.0;
        assert!(delta_s <= 16.0 * std::f64::EPSILON);
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
        let delta_s = 0.5 - result_30.s.0;
        assert!(delta_s <= 16.0 * std::f64::EPSILON);
        assert_eq!(angle_30.c.0, result_30.c.0);

        let delta_angle = libm::fabs(degrees_30.0 - Degrees::from(result_30).0);
        assert!(delta_angle <= 32.0 * std::f64::EPSILON);
    }

    #[test]
    fn test_x2() {
        for i in 0..90 {
            let value = i as f64;
            let half_angle = Angle::from(Degrees(0.5 * value));
            let result = Degrees::from(half_angle.x2()).0;

            // assert_eq!(value, result);  // Does not work, not accurate enough.
            let delta_result = libm::fabs(value - result);
            assert!(delta_result <= 100.0 * std::f64::EPSILON);
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
            let delta_result = libm::fabs(expected - result);
            assert!(delta_result <= std::f64::EPSILON);
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
            let delta_result = libm::fabs(expected - result);
            assert!(delta_result <= std::f64::EPSILON);
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
