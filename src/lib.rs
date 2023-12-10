// Copyright (c) 2018-2023 Via Technology Ltd.

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

pub mod sphere;
pub mod trig;

use contracts::debug_ensures;

/// Return the minimum of a or b.
#[inline]
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
