// Copyright (c) 2020-2023 Via Technology Ltd. All Rights Reserved.

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

//! The point3d module contains the Point3d type and its associated functions.
extern crate nalgebra as na;
use super::e2gc_distance;
use crate::trig::{swap_sin_cos, Angle, Radians, UnitNegRange};
use contracts::*;

/// A Point3d is a nalgebra Vector3.
pub type Point3d = na::Vector3<f64>;

/// The square of std::f64::EPSILON.
pub const SQ_EPSILON: f64 = std::f64::EPSILON * std::f64::EPSILON;

/// The minimum length of a vector to normalize.
pub const MIN_LENGTH: f64 = 16384.0 * std::f64::EPSILON;

/// The minimum norm of a vector to normalize.
pub const MIN_NORM: f64 = MIN_LENGTH * MIN_LENGTH;

/// Convert a latitude and longitude to a Point3d.  
/// @pre |lat| <= 90.0 degrees.
/// * `lat` - the latitude.
/// * `lon` - the longitude.
///
/// returns a Point3d on the unit sphere.
#[debug_requires(lat.is_valid_latitude())]
#[debug_ensures(is_unit(&ret))]
pub fn to_sphere(lat: Angle, lon: Angle) -> Point3d {
    Point3d::new(lat.cos() * lon.cos(), lat.cos() * lon.sin(), lat.sin())
}

/// Calculate the latitude of a Point3d.
#[debug_requires(is_unit(a))]
#[debug_ensures(ret.is_valid_latitude())]
pub fn latitude(a: &Point3d) -> Angle {
    let sin_a = UnitNegRange(a.z);
    Angle::new(sin_a, swap_sin_cos(sin_a))
}

/// Calculate the longitude of a Point3d.
#[debug_requires(is_unit(a))]
pub fn longitude(a: &Point3d) -> Angle {
    Angle::from_y_x(a.y, a.x)
}

/// Determine whether point a is West of point b.  
/// It calculates and compares the perp product of the two points.
/// * `a`, `b` - the points.
///
/// returns true if a is West of b, false otherwise.
pub fn is_west_of(a: &Point3d, b: &Point3d) -> bool {
    // Compare with -epsilon to handle floating point errors
    b.xy().perp(&a.xy()) <= -std::f64::EPSILON
}

/// Calculate the relative longitude of point a from point b.
/// * `a`, `b` - the points.
///
/// returns the relative longitude of point a from point b,
/// negative if a is West of b, positive otherwise.
#[debug_requires(is_unit(a) && is_unit(b))]
pub fn delta_longitude(a: &Point3d, b: &Point3d) -> Angle {
    let a_lon = a.xy();
    let b_lon = b.xy();
    Angle::from_y_x(b_lon.perp(&a_lon), b_lon.dot(&a_lon))
}

/// Determine whether a Point3d is a unit vector.
///
/// returns true if Point3d is a unit vector, false otherwise.
pub fn is_unit(a: &Point3d) -> bool {
    const MIN_POINT_SQ_LENGTH: f64 = 1.0 - 12.0 * std::f64::EPSILON;
    const MAX_POINT_SQ_LENGTH: f64 = 1.0 + 12.0 * std::f64::EPSILON;

    (MIN_POINT_SQ_LENGTH..=MAX_POINT_SQ_LENGTH).contains(&(a.norm()))
}

/// Determine whether two Point3ds are orthogonal (perpendicular).
///
/// returns true if a and b are orthogonal, false otherwise.
pub fn are_orthogonal(a: &Point3d, b: &Point3d) -> bool {
    const MAX_LENGTH: f64 = 4.0 * std::f64::EPSILON;

    (-MAX_LENGTH..=MAX_LENGTH).contains(&(a.dot(b)))
}

/// Calculate the winding number of a point against the edge of a polygon.  
/// It determines whether the polygon edge lies between the point and the
/// North or South pole and then calculates the winding number based upon
/// whether the point is North of the edge, see
/// http://geomalgorithms.com/a03-_inclusion.html
/// * `a`, `b` the start and end points of the edge.
/// * `pole` the pole of the great circle of the edge.
/// * `point` the point to compare.
///
/// returns 1 if the point is North of the edge, -1 if the point is South of
/// the edge, 0 otherwise.
#[debug_requires(is_unit(a) && is_unit(pole) && is_unit(b) && is_unit(point)
            && are_orthogonal(a, pole) && are_orthogonal(b, pole))]
pub fn winding_number(a: &Point3d, pole: &Point3d, b: &Point3d, point: &Point3d) -> i32 {
    // if point is East of (or at same longitude as) a
    if !is_west_of(point, a) {
        // and West of b and North of the edge
        if is_west_of(point, b) && (0.0 < pole.dot(point)) {
            return 1;
        }
    } else {
        // point is West of a
        // and East of (or at same longitude as) b and North of the edge
        if !is_west_of(point, b) && (pole.dot(point) < 0.0) {
            return -1;
        }
    }

    0
}

/// Calculate the square of the Euclidean distance between two Point3ds.
/// @post for unit vectors: result <= 4
#[debug_ensures(0.0 <= ret)]
pub fn sq_distance(a: &Point3d, b: &Point3d) -> f64 {
    (b - a).norm_squared()
}

/// Calculate the shortest (Euclidean) distance between two Point3ds.
/// @post for unit vectors: result <= 2
#[debug_ensures(0.0 <= ret)]
pub fn distance(a: &Point3d, b: &Point3d) -> f64 {
    (b - a).norm()
}

/// Calculate the Great Circle distance (in radians) between two points.
#[debug_requires(is_unit(a) && is_unit(b))]
#[debug_ensures(libm::fabs(ret.0) <= std::f64::consts::PI)]
pub fn gc_distance(a: &Point3d, b: &Point3d) -> Radians {
    e2gc_distance(distance(a, b))
}

/// Calculate the Great Circle distance (as an Angle) between two points.
#[debug_requires(is_unit(a) && is_unit(b))]
pub fn gc_distance_angle(a: &Point3d, b: &Point3d) -> Angle {
    let d_2 = UnitNegRange::clamp(0.5 * distance(a, b));
    let gc_d_2 = Angle::new(d_2, swap_sin_cos(d_2));
    gc_d_2.x2()
}

#[cfg(test)]
mod tests {
    use crate::sphere::point3d::*;
    use crate::trig::{Angle, Degrees, Radians};

    #[test]
    fn test_point3d_lat_longs() {
        let zero = Angle::from_y_x(0.0, 1.0);
        let ninety = Angle::from(Degrees(90.0));
        let one_eighty = Angle::from(Degrees(180.0));

        let point_south = to_sphere(-ninety, one_eighty);
        assert!(is_unit(&point_south));
        assert_eq!(-ninety.to_radians(), latitude(&point_south).to_radians());
        assert_eq!(zero.to_radians(), longitude(&point_south).to_radians());

        // Test Greenwich equator
        let point_0 = to_sphere(zero, zero);
        assert!(is_unit(&point_0));
        assert_eq!(zero.to_radians(), latitude(&point_0).to_radians());
        assert_eq!(zero.to_radians(), longitude(&point_0).to_radians());

        // Test IDL equator
        let point_1 = to_sphere(zero, one_eighty);
        assert!(is_unit(&point_1));
        assert_eq!(zero.to_radians(), latitude(&point_1).to_radians());
        assert_eq!(one_eighty.to_radians(), longitude(&point_1).to_radians());

        assert!(!is_west_of(&point_0, &point_1));
        assert_eq!(
            one_eighty.to_radians().0,
            libm::fabs(delta_longitude(&point_0, &point_1).to_radians().0)
        );

        let point_2 = to_sphere(zero, -one_eighty);
        assert!(is_unit(&point_2));
        assert_eq!(zero.to_radians(), latitude(&point_2).to_radians());
        assert_eq!(-one_eighty.to_radians(), longitude(&point_2).to_radians());

        assert!(!is_west_of(&point_0, &point_2));
        assert_eq!(
            one_eighty.to_radians(),
            delta_longitude(&point_0, &point_2).to_radians()
        );

        let three_radians = Angle::from(Radians(3.0));
        let point_3 = to_sphere(zero, three_radians);
        assert!(is_unit(&point_3));
        assert_eq!(zero.to_radians(), latitude(&point_3).to_radians());
        assert_eq!(three_radians.to_radians(), longitude(&point_3).to_radians());

        assert!(is_west_of(&point_0, &point_3));
        assert_eq!(
            -three_radians.to_radians(),
            delta_longitude(&point_0, &point_3).to_radians()
        );

        assert!(!is_west_of(&point_1, &point_3));
        let delta_lon13 =
            std::f64::consts::PI - 3.0 - delta_longitude(&point_1, &point_3).to_radians().0;
        assert!(libm::fabs(delta_lon13) < std::f64::EPSILON);

        let point_4 = to_sphere(zero, -three_radians);
        assert!(is_unit(&point_4));
        assert_eq!(zero.to_radians(), latitude(&point_4).to_radians());
        assert_eq!(
            -three_radians.to_radians(),
            longitude(&point_4).to_radians()
        );

        assert!(!is_west_of(&point_0, &point_4));
        assert_eq!(
            three_radians.to_radians(),
            delta_longitude(&point_0, &point_4).to_radians()
        );

        assert!(is_west_of(&point_1, &point_4));
        let delta_lon14 =
            3.0 - std::f64::consts::PI - delta_longitude(&point_1, &point_4).to_radians().0;
        assert!(libm::fabs(delta_lon14) < std::f64::EPSILON);

        let pi_m16epsilon = Angle::from(Radians(std::f64::consts::PI - 16.0 * std::f64::EPSILON));
        let point_5 = to_sphere(zero, pi_m16epsilon);
        assert!(is_unit(&point_5));

        assert!(is_west_of(&point_0, &point_5));
        assert_eq!(
            -pi_m16epsilon.to_radians(),
            delta_longitude(&point_0, &point_5).to_radians()
        );

        let point_6 = to_sphere(zero, -pi_m16epsilon);
        assert!(is_unit(&point_6));

        assert!(!is_west_of(&point_0, &point_6));
        assert_eq!(
            pi_m16epsilon.to_radians(),
            delta_longitude(&point_0, &point_6).to_radians()
        );
    }

    #[test]
    fn test_point3d_distance() {
        let zero = Angle::from_y_x(0.0, 1.0);
        let ninety = Angle::from(Degrees(90.0));
        let one_eighty = Angle::from(Degrees(180.0));

        let south_pole = to_sphere(-ninety, zero);
        let north_pole = to_sphere(ninety, zero);

        assert_eq!(0.0, distance(&south_pole, &south_pole));
        assert_eq!(0.0, distance(&north_pole, &north_pole));
        assert_eq!(2.0, distance(&south_pole, &north_pole));

        // Greenwich equator
        let g_eq = to_sphere(zero, zero);

        // Test IDL equator
        let idl_eq = to_sphere(zero, one_eighty);

        assert_eq!(0.0, distance(&g_eq, &g_eq));
        assert_eq!(0.0, distance(&idl_eq, &idl_eq));
        assert_eq!(2.0, distance(&g_eq, &idl_eq));
    }

    #[test]
    fn test_point3d_gc_distance() {
        let zero = Angle::from_y_x(0.0, 1.0);
        let ninety = Angle::from(Degrees(90.0));
        let one_eighty = Angle::from(Degrees(180.0));

        let south_pole = to_sphere(-ninety, zero);
        let north_pole = to_sphere(ninety, zero);

        assert_eq!(0.0, gc_distance(&south_pole, &south_pole).0);
        assert_eq!(0.0, gc_distance(&north_pole, &north_pole).0);
        assert_eq!(
            std::f64::consts::PI,
            gc_distance(&south_pole, &north_pole).0
        );

        // Greenwich equator
        let g_eq = to_sphere(zero, zero);

        // Test IDL equator
        let idl_eq = to_sphere(zero, one_eighty);

        assert_eq!(0.0, gc_distance(&g_eq, &g_eq).0);
        assert_eq!(0.0, gc_distance(&idl_eq, &idl_eq).0);
        assert_eq!(std::f64::consts::PI, gc_distance(&g_eq, &idl_eq).0);
    }

    #[test]
    fn test_point3d_gc_distance_angle() {
        let zero = Angle::from_y_x(0.0, 1.0);
        let ninety = Angle::from(Degrees(90.0));
        let one_eighty = Angle::from(Degrees(180.0));

        let south_pole = to_sphere(-ninety, zero);
        let north_pole = to_sphere(ninety, zero);

        assert_eq!(zero, gc_distance_angle(&south_pole, &south_pole));
        assert_eq!(zero, gc_distance_angle(&north_pole, &north_pole));
        assert_eq!(one_eighty, gc_distance_angle(&south_pole, &north_pole));

        // Greenwich equator
        let g_eq = to_sphere(zero, zero);

        // Test IDL equator
        let idl_eq = to_sphere(zero, one_eighty);

        assert_eq!(zero, gc_distance_angle(&g_eq, &g_eq));
        assert_eq!(zero, gc_distance_angle(&idl_eq, &idl_eq));
        assert_eq!(one_eighty, gc_distance_angle(&g_eq, &idl_eq));
    }
}
