// Copyright (c) 2018-2024 Via Technology Ltd.

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

//! [![Rust](https://github.com/kenba/via-sphere-rs/actions/workflows/rust.yml/badge.svg)](https://github.com/kenba/via-sphere-rs/actions)
//! [![License](https://img.shields.io/badge/License-MIT-blue)](https://opensource.org/license/mit/)
//! [![codecov](https://codecov.io/gh/kenba/via-sphere-rs/graph/badge.svg?token=8FBO2N4N69)](https://codecov.io/gh/kenba/via-sphere-rs)
//!
//! This library uses a combination of spherical trigonometry and vectors
//! to calculate angles, distances and positions on the surface of a unit sphere.
//!
//! The `trig` and `latlong` modules perform spherical trigonometric calculations;
//! the `sphere` module uses vectors.
//!
//! The library uses the [contracts](https://crates.io/crates/contracts) crate
//! to implement Design By Contract [(DbC)](https://wiki.c2.com/?DesignByContract).  
//! It also defines a `Validate` trait to define an `is_valid` invariant
//! function to support Design By Contract invariants.

pub mod latlong;
pub mod sphere;
pub mod trig;

use contracts::{debug_ensures, debug_requires};

/// Return the minimum of a or b.
#[inline]
#[must_use]
pub fn min<T>(a: T, b: T) -> T
where
    T: PartialOrd + Copy,
{
    if b < a {
        b
    } else {
        a
    }
}

/// Return the maximum of a or b.
#[inline]
#[must_use]
pub fn max<T>(a: T, b: T) -> T
where
    T: PartialOrd + Copy,
{
    if b < a {
        a
    } else {
        b
    }
}

/// Clamp value into the range: min to max inclusive.
/// * `value` - value to clamp
/// * `min` - minimum value.
/// * `max` - maximum value.
/// Note: there is a f64::max in nightly builds.
#[debug_ensures((min ..= max).contains(&ret))]
#[inline]
#[must_use]
pub fn clamp<T>(value: T, min: T, max: T) -> T
where
    T: PartialOrd + Copy,
{
    if value < min {
        min
    } else if max < value {
        max
    } else {
        value
    }
}

/// The Validate trait.
pub trait Validate {
    /// return true if the type is valid, false otherwise.
    fn is_valid(&self) -> bool;
}

/// Check whether a pair of values are within tolerance of each other
/// * `value` the value to test
/// * `tolerance` the permitted tolerance
/// return true if value is <= tolerance
#[debug_requires(value >= 0.0)]
#[inline]
#[must_use]
pub fn is_small(value: f64, tolerance: f64) -> bool {
    value <= tolerance
}

/// Check whether a value are within tolerance of a reference value.
/// * `reference` the required value
/// * `value` the value to test
/// * `tolerance` the permitted tolerance
/// return true if abs(reference - value) is <= tolerance
#[inline]
#[must_use]
pub fn is_within_tolerance(reference: f64, value: f64, tolerance: f64) -> bool {
    is_small(libm::fabs(reference - value), tolerance)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_min_and_max() {
        // min -ve and +ve
        assert_eq!(min(-1.0 + std::f64::EPSILON, -1.0), -1.0);
        assert_eq!(min(1.0, 1.0 + std::f64::EPSILON), 1.0);
        // max -ve and +ve
        assert_eq!(max(-1.0, -1.0 - std::f64::EPSILON), -1.0);
        assert_eq!(max(1.0 - std::f64::EPSILON, 1.0), 1.0);
    }

    #[test]
    fn test_clamp() {
        // value < min
        assert_eq!(clamp(-1.0 - std::f64::EPSILON, -1.0, 1.0), -1.0);
        // value > max
        assert_eq!(clamp(1.0 + std::f64::EPSILON, -1.0, 1.0), 1.0);
    }

    #[test]
    fn test_is_within_tolerance() {
        // below minimum tolerance
        assert_eq!(
            false,
            is_within_tolerance(1.0 - 2.0 * std::f64::EPSILON, 1.0, std::f64::EPSILON)
        );

        // within minimum tolerance
        assert!(is_within_tolerance(
            1.0 - std::f64::EPSILON,
            1.0,
            std::f64::EPSILON
        ));

        // within maximum tolerance
        assert!(is_within_tolerance(
            1.0 + std::f64::EPSILON,
            1.0,
            std::f64::EPSILON
        ));

        // above maximum tolerance
        assert_eq!(
            false,
            is_within_tolerance(1.0 + 2.0 * std::f64::EPSILON, 1.0, std::f64::EPSILON)
        );
    }
}
