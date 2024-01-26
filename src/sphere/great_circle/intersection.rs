// Copyright (c) 2020-2024 Via Technology Ltd. All Rights Reserved.

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

//! The intersection module contains types and functions for calculating
//! intersections between great circles on the surface of a sphere.

use super::sin_atd;
use crate::max;
use crate::sphere::{
    calculate_intersection_point, debug_ensures, debug_requires, gc_distance, is_unit, sq_distance,
    Point, Validate, SQ_EPSILON,
};
use crate::trig;
use crate::trig::Radians;

/// Calculate the great circle distances to an intersection point from the
/// start points of a pair of great circle arcs, on different great circles.
/// * `a1`, `a2` the start points of the great circle arcs
/// * `pole1`, `pole2` the poles of the great circle arcs
/// * `c` the intersection point
///
/// returns a pair of great circle distances along the arcs to the
/// intersection point in Radians.
#[debug_requires(is_unit(c))]
#[debug_ensures(ret.0.is_valid() && ret.1.is_valid())]
#[must_use]
pub fn calculate_intersection_distances(
    a1: &Point,
    pole1: &Point,
    a2: &Point,
    pole2: &Point,
    c: &Point,
) -> (Radians, Radians) {
    let sq_d_a1c = sq_distance(a1, c);
    let gc_d_a1c = if sq_d_a1c < 2.0 * SQ_EPSILON {
        0.0
    } else {
        libm::copysign(
            trig::e2gc_distance(libm::sqrt(sq_d_a1c)).0,
            sin_atd(a1, pole1, c).0,
        )
    };

    let sq_d_a2c = sq_distance(a2, c);
    let gc_d_a2c = if sq_d_a2c < 2.0 * SQ_EPSILON {
        0.0
    } else {
        libm::copysign(
            trig::e2gc_distance(libm::sqrt(sq_d_a2c)).0,
            sin_atd(a2, pole2, c).0,
        )
    };

    (Radians(gc_d_a1c), Radians(gc_d_a2c))
}

/// Whether an intersection point is within an arc
/// * `ref_distance` the distance to the intersection point from the start
/// * `arc_length` the length of the arc.
#[must_use]
pub fn is_within(ref_distance: f64, arc_length: f64) -> bool {
    (-std::f64::EPSILON <= ref_distance)
        && (ref_distance <= arc_length + (std::f64::EPSILON * (1.0 + arc_length)))
}

/// Determine whether the other intersection point is closer to the start
/// of both arcs.
/// * `ref1_distance`, `ref2_distance` the intersection distances.
/// * `arc1_length`, `arc2_length` the arc lengths.
/// return true if the other intersection point is closer to the arc starts,
/// false otherwise.
#[debug_requires(ref1_distance.is_valid() && ref2_distance.is_valid()
              && arc1_length.is_valid() && arc2_length.is_valid())]
#[must_use]
fn use_other_point(
    ref1_distance: Radians,
    ref2_distance: Radians,
    arc1_length: Radians,
    arc2_length: Radians,
) -> bool {
    // is the intersection within both arcs?
    let c_within_arc1 = is_within(ref1_distance.0, arc1_length.0);
    let c_within_arc2 = is_within(ref2_distance.0, arc2_length.0);
    if c_within_arc1 && c_within_arc2 {
        return false;
    }

    // Calculate distances from the other intersection point
    let other_distance1 = ref1_distance + Radians(std::f64::consts::PI);
    let other_distance2 = ref2_distance + Radians(std::f64::consts::PI);

    // is the other intersection within both arcs?
    let d_within_arc1 = is_within(other_distance1.0, arc1_length.0);
    let d_within_arc2 = is_within(other_distance2.0, arc2_length.0);
    if d_within_arc1 && d_within_arc2 {
        return true;
    }

    // if either intersection is within an arc
    let c_within_arc = c_within_arc1 || c_within_arc2;
    let d_within_arc = d_within_arc1 || d_within_arc2;
    if c_within_arc == d_within_arc {
        // calculate the minimum length from a start point to an intersection
        fn min_length(ref_length: Radians, within_arc: bool) -> f64 {
            if within_arc {
                0.0
            } else {
                libm::fabs(ref_length.0)
            }
        }

        // either both intersections are within an arc or neither are
        let min_c1 = min_length(ref1_distance, c_within_arc1);
        let min_c2 = min_length(ref2_distance, c_within_arc2);
        let max_c = max(min_c1, min_c2);
        let min_d1 = min_length(other_distance1, d_within_arc1);
        let min_d2 = min_length(other_distance2, d_within_arc2);
        let max_d = max(min_d1, min_d2);

        // use the intersection that is closest to the start of both arcs
        max_d < max_c
    } else {
        // whichever intersection is within an arc is closest
        d_within_arc
    }
}

/// Calculate the great circle distances to the closest intersection point from the
/// start points of a pair of great circle arcs, on different great circles.
/// * `a1`, `a2` the start points of the great circle arcs
/// * `pole1`, `pole2` the poles of the great circle arcs
/// * `c` an intersection point
///
/// returns a pair of great circle distances along the arcs to the
/// intersection point in Radians and a boolean indicating whether the antipodal
/// intersection point was used instead of the one given.
#[must_use]
pub fn calculate_closest_intersection_distances(
    a1: &Point,
    pole1: &Point,
    length1: Radians,
    a2: &Point,
    pole2: &Point,
    length2: Radians,
    c: &Point,
) -> (Radians, Radians, bool) {
    let (arc1_a_c, arc2_a_c) = calculate_intersection_distances(a1, pole1, a2, pole2, c);

    let use_antipodal_intersection = use_other_point(arc1_a_c, arc2_a_c, length1, length2);
    if use_antipodal_intersection {
        let (arc1_a_c, arc2_a_c) = calculate_intersection_distances(a1, pole1, a2, pole2, &-(*c));
        (arc1_a_c, arc2_a_c, use_antipodal_intersection)
    } else {
        (arc1_a_c, arc2_a_c, use_antipodal_intersection)
    }
}

/// Calculate the lengths along a pair of Arcs on the same (or reciprocal)
/// Great Circles to their closest (reference) points.
/// * `reciprocal` whether the arcs are in reciprocal directions.
/// * `a2_ahead` whether the start of arc2 is ahead of the start of arc1.
/// * `arc1_length`, `arc2_length` the arc lengths.
/// * `gc_d` the great circle distance between the arc start points.
#[debug_ensures(ret.0.is_valid() && ret.1.is_valid())]
#[must_use]
fn calc_same_gc_reference_lengths(
    reciprocal: bool,
    a2_ahead: bool,
    arc1_length: Radians,
    arc2_length: Radians,
    gc_d: Radians,
) -> (Radians, Radians) {
    const TWO_PI: f64 = 2.0 * std::f64::consts::PI;

    if reciprocal {
        let max_length = if arc1_length < arc2_length {
            arc2_length
        } else {
            arc1_length
        };
        if a2_ahead && (gc_d <= max_length) {
            return if gc_d <= arc2_length {
                (Radians(0.0), gc_d)
            } else {
                (gc_d, Radians(0.0))
            };
        }

        // The distance between b ends
        let b_d = gc_d.0 - arc1_length.0 - arc2_length.0;
        let b_gc_d = if a2_ahead { b_d } else { TWO_PI - b_d };

        if b_gc_d < gc_d.0 {
            (Radians(b_gc_d + arc1_length.0), arc2_length)
        } else {
            (-gc_d, Radians(0.0))
        }
    } else {
        // The distance to the start of arc2 from the end of arc1
        let b1a2 = if a2_ahead {
            gc_d.0 - arc1_length.0
        } else {
            TWO_PI - gc_d.0 - arc1_length.0
        };
        // The distance to the start of arc1 from the end of arc2
        let b2a1 = if a2_ahead {
            TWO_PI - gc_d.0 - arc2_length.0
        } else {
            gc_d.0 - arc2_length.0
        };
        if b2a1 < b1a2 {
            (Radians(0.0), Radians(b2a1 + arc2_length.0))
        } else {
            (Radians(b1a2 + arc1_length.0), Radians(0.0))
        }
    }
}

/// Calculate the lengths along a pair of Arcs on the same (or reciprocal)
/// Great Circles to their closest (reference) points.
/// * `a1`, `a2` the arc start points.
/// * `pole1`, `pole1` the arc poles.
/// * `length1`, `length2` the arc lengths.
///
#[debug_ensures(ret.0.is_valid() && ret.1.is_valid())]
#[must_use]
pub fn calculate_same_gc_reference_lengths(
    a1: &Point,
    pole1: &Point,
    length1: Radians,
    a2: &Point,
    pole2: &Point,
    length2: Radians,
    gc_d: Radians,
) -> (Radians, Radians) {
    let reciprocal = pole1.dot(pole2) < 0.0;
    let a2_ahead = 0.0 < sin_atd(a1, pole1, a2).0;
    calc_same_gc_reference_lengths(reciprocal, a2_ahead, length1, length2, gc_d)
}

#[must_use]
pub fn calculate_intersection_point_lengths(
    a1: &Point,
    pole1: &Point,
    length1: Radians,
    a2: &Point,
    pole2: &Point,
    length2: Radians,
) -> (Radians, Radians) {
    // Calculate the great circle distance between the start points.
    let gc_d = gc_distance(a1, a2);
    if gc_d.0 <= 2.0 * std::f64::EPSILON {
        (Radians(0.0), Radians(0.0))
    } else {
        calculate_intersection_point(pole1, pole2).map_or_else(
            || calculate_same_gc_reference_lengths(a1, pole1, length1, a2, pole2, length2, gc_d),
            |c| {
                let (arc1_a_c, arc2_a_c, _) = calculate_closest_intersection_distances(
                    a1, pole1, length1, a2, pole2, length2, &c,
                );
                (arc1_a_c, arc2_a_c)
            },
        )
    }
}

/// Calculate the distance between two arcs on the same great circles.
/// * `reciprocal` whether the arcs are in reciprocal directions.
/// * `a2_ahead` whether the start of arc2 is ahead of the start of arc1.
/// * `arc1_length`, `arc2_length` the arc lengths.
/// * `gc_d` the great circle distance between the arc start points.
#[must_use]
fn calc_same_gc_distance(
    reciprocal: bool,
    a2_ahead: bool,
    arc1_length: Radians,
    arc2_length: Radians,
    gc_d: Radians,
) -> Radians {
    const TWO_PI: f64 = 2.0 * std::f64::consts::PI;

    if reciprocal {
        let max_length = if arc1_length < arc2_length {
            arc2_length
        } else {
            arc1_length
        };
        if a2_ahead && (gc_d <= max_length) {
            return Radians(0.0);
        }

        // The distance between b ends
        let b_d = gc_d.0 - arc1_length.0 - arc2_length.0;
        let b_gc_d = if a2_ahead { b_d } else { TWO_PI - b_d };

        if b_gc_d < gc_d.0 {
            Radians(b_gc_d)
        } else {
            gc_d
        }
    } else {
        // The distance to the start of arc2 from the end of arc1
        let b1a2 = if a2_ahead {
            gc_d.0 - arc1_length.0
        } else {
            TWO_PI - gc_d.0 - arc1_length.0
        };
        // The distance to the start of arc1 from the end of arc2
        let b2a1 = if a2_ahead {
            TWO_PI - gc_d.0 - arc2_length.0
        } else {
            gc_d.0 - arc2_length.0
        };
        // the shorter of the two distances
        if b2a1 < b1a2 {
            Radians(b2a1)
        } else {
            Radians(b1a2)
        }
    }
}

/// Calculate the distance between two arcs on the same great circles.
/// * `a1`, `a2` the arc start points.
/// * `pole1`, `pole1` the arc poles.
/// * `length1`, `length2` the arc lengths in radians.
///
/// return the shortest Great Circle distance between the arcs in Radians.
#[must_use]
pub fn calculate_same_gc_distance(
    a1: &Point,
    pole1: &Point,
    length1: Radians,
    a2: &Point,
    pole2: &Point,
    length2: Radians,
    gc_d: Radians,
) -> Radians {
    let reciprocal = pole1.dot(pole2) < 0.0;
    let a2_ahead = 0.0 < sin_atd(a1, pole1, a2).0;
    calc_same_gc_distance(reciprocal, a2_ahead, length1, length2, gc_d)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_calc_same_gc_reference_lengths_1() {
        let zero = Radians(0.0);
        let length1 = Radians(0.25);
        let length2 = Radians(0.75);

        let result0 = calc_same_gc_reference_lengths(true, true, length2, length1, length2);
        assert_eq!(length2, result0.0);
        assert_eq!(zero, result0.1);

        let result1 = calc_same_gc_reference_lengths(true, true, length1, length2, length2);
        assert_eq!(zero, result1.0);
        assert_eq!(length2, result1.1);

        let result2 = calc_same_gc_reference_lengths(true, true, length1, length2, Radians(1.0));
        assert_eq!(length1, result2.0);
        assert_eq!(length2, result2.1);

        let result3 = calc_same_gc_reference_lengths(true, true, length1, length2, Radians(1.5));
        assert_eq!(length2, result3.0);
        assert_eq!(length2, result3.1);

        let result4 = calc_same_gc_reference_lengths(true, false, length1, length2, Radians(1.5));
        assert_eq!(Radians(-1.5), result4.0);
        assert_eq!(zero, result4.1);

        let result5 = calc_same_gc_reference_lengths(false, false, length1, length2, Radians(1.0));
        assert_eq!(zero, result5.0);
        assert_eq!(Radians(1.0), result5.1);

        let result6 = calc_same_gc_reference_lengths(false, true, length1, length2, Radians(1.0));
        assert_eq!(Radians(1.0), result6.0);
        assert_eq!(zero, result6.1);

        let result7 = calc_same_gc_reference_lengths(false, false, length1, length2, length2);
        assert_eq!(zero, result7.0);
        assert_eq!(length2, result7.1);

        let result8 = calc_same_gc_reference_lengths(false, true, length1, length2, length1);
        assert_eq!(length1, result8.0);
        assert_eq!(zero, result8.1);
    }

    #[test]
    fn test_calc_same_gc_distance_1() {
        let zero = Radians(0.0);
        let length1 = Radians(0.25);
        let length2 = Radians(0.75);

        let result0 = calc_same_gc_distance(true, true, length1, length2, length2);
        assert_eq!(zero, result0);
        let result1 = calc_same_gc_distance(true, true, length1, length2, Radians(1.0));
        assert_eq!(zero, result1);
        let result2 = calc_same_gc_distance(true, true, length1, length2, Radians(1.5));
        assert_eq!(Radians(0.5), result2);
        let result3 = calc_same_gc_distance(true, false, length1, length2, Radians(1.5));
        assert_eq!(Radians(1.5), result3);
        let result4 = calc_same_gc_distance(true, false, length2, length1, Radians(1.5));
        assert_eq!(Radians(1.5), result4);

        let result10 = calc_same_gc_distance(false, false, length1, length2, Radians(1.0));
        assert_eq!(length1, result10);
        let result11 = calc_same_gc_distance(false, true, length1, length2, Radians(1.0));
        assert_eq!(length2, result11);

        let result12 = calc_same_gc_distance(false, false, length1, length2, length2);
        assert_eq!(zero, result12);

        let result13 = calc_same_gc_distance(false, true, length1, length2, length1);
        assert_eq!(zero, result13);
    }
}
