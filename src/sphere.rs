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

//! The sphere module contains types and functions for calculating distances
//! and azimuths between points on the surface of a sphere.

pub mod arc;
pub mod arcstring;
pub mod great_circle;

extern crate nalgebra as na;
use crate::is_small;
use crate::latlong::{LatLong, LatLongs};
use crate::trig;
use crate::trig::{Angle, Radians, UnitNegRange};
use crate::Validate;
use contracts::{debug_ensures, debug_requires};

/// A Point is a nalgebra Vector3.
pub type Point = na::Vector3<f64>;

/// The square of `std::f64::EPSILON`.
pub const SQ_EPSILON: f64 = std::f64::EPSILON * std::f64::EPSILON;

/// The minimum length of a vector to normalize.
pub const MIN_LENGTH: f64 = 16384.0 * std::f64::EPSILON;

/// The minimum norm of a vector to normalize.
pub const MIN_NORM: f64 = MIN_LENGTH * MIN_LENGTH;

/// Create a Point from latitude and longitude
/// @pre |lat| <= 90.0 degrees.
/// * `lat` - the latitude.
/// * `lon` - the longitude.
///
/// returns a Point on the unit sphere.
#[debug_requires(lat.is_valid_latitude())]
#[debug_ensures(ret.is_valid())]
pub fn to_sphere(lat: Angle, lon: Angle) -> Point {
    Point::new(lat.cos() * lon.cos(), lat.cos() * lon.sin(), lat.sin())
}

impl From<&LatLong> for Point {
    /// Convert a `LatLong` to a Point on the unit sphere
    fn from(value: &LatLong) -> Self {
        to_sphere(value.lat(), value.lon())
    }
}

/// Calculate the latitude of a Point.
#[debug_requires(a.is_valid())]
#[debug_ensures(ret.is_valid_latitude())]
pub fn latitude(a: &Point) -> Angle {
    let sin_a = UnitNegRange(a.z);
    Angle::new(sin_a, trig::swap_sin_cos(sin_a))
}

/// Calculate the longitude of a Point.
#[debug_requires(a.is_valid())]
pub fn longitude(a: &Point) -> Angle {
    Angle::from_y_x(a.y, a.x)
}

impl From<&Point> for LatLong {
    /// Convert a Point to a`LatLong`  
    fn from(value: &Point) -> Self {
        Self::new(latitude(value), longitude(value))
    }
}

/// Determine whether a Point is a unit vector.
///
/// returns true if Point is a unit vector, false otherwise.
#[must_use]
pub fn is_unit(a: &Point) -> bool {
    const MIN_POINT_SQ_LENGTH: f64 = 1.0 - 12.0 * std::f64::EPSILON;
    const MAX_POINT_SQ_LENGTH: f64 = 1.0 + 12.0 * std::f64::EPSILON;

    (MIN_POINT_SQ_LENGTH..=MAX_POINT_SQ_LENGTH).contains(&(a.norm()))
}

impl Validate for Point {
    /// Test whether a Point is valid.  
    /// I.e. whether the Point is a unit vector.
    fn is_valid(&self) -> bool {
        is_unit(self)
    }
}

/// Determine whether point a is West of point b.  
/// It calculates and compares the perp product of the two points.
/// * `a`, `b` - the points.
///
/// returns true if a is West of b, false otherwise.
#[must_use]
pub fn is_west_of(a: &Point, b: &Point) -> bool {
    // Compare with -epsilon to handle floating point errors
    b.xy().perp(&a.xy()) <= -std::f64::EPSILON
}

/// Calculate the relative longitude of point a from point b.
/// * `a`, `b` - the points.
///
/// returns the relative longitude of point a from point b,
/// negative if a is West of b, positive otherwise.
#[debug_requires(a.is_valid() && b.is_valid())]
#[must_use]
pub fn delta_longitude(a: &Point, b: &Point) -> Angle {
    let a_lon = a.xy();
    let b_lon = b.xy();
    Angle::from_y_x(b_lon.perp(&a_lon), b_lon.dot(&a_lon))
}

/// Determine whether two Points are orthogonal (perpendicular).
///
/// returns true if a and b are orthogonal, false otherwise.
#[debug_requires(a.is_valid() && b.is_valid())]
#[must_use]
pub fn are_orthogonal(a: &Point, b: &Point) -> bool {
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
#[debug_requires(a.is_valid() && pole.is_valid() && b.is_valid() && point.is_valid()
            && are_orthogonal(a, pole) && are_orthogonal(b, pole))]
#[must_use]
pub fn winding_number(a: &Point, pole: &Point, b: &Point, point: &Point) -> i32 {
    // if point is East of (or at same longitude as) a
    if is_west_of(point, a) {
        // point is West of a
        // and East of (or at same longitude as) b and North of the edge
        if !is_west_of(point, b) && (pole.dot(point) < 0.0) {
            return -1;
        }
    } else {
        // and West of b and North of the edge
        if is_west_of(point, b) && (0.0 < pole.dot(point)) {
            return 1;
        }
    }

    0
}

/// Calculate the square of the Euclidean distance between two Points.
/// Note: points do NOT need to be valid Points.
/// @post for unit vectors: result <= 4
#[debug_ensures(0.0 <= ret)]
#[must_use]
pub fn sq_distance(a: &Point, b: &Point) -> f64 {
    (b - a).norm_squared()
}

/// Calculate the shortest (Euclidean) distance between two Points.
/// @post for unit vectors: result <= 2
#[debug_ensures(0.0 <= ret)]
#[must_use]
pub fn distance(a: &Point, b: &Point) -> f64 {
    (b - a).norm()
}

/// Calculate the Great Circle distance (in radians) between two points.
#[debug_requires(is_unit(a) && is_unit(b))]
#[debug_ensures(libm::fabs(ret.0) <= std::f64::consts::PI)]
#[must_use]
pub fn gc_distance(a: &Point, b: &Point) -> Radians {
    trig::e2gc_distance(distance(a, b))
}

/// Calculate the Great Circle distance (as an Angle) between two points.
#[debug_requires(a.is_valid() && b.is_valid())]
#[must_use]
pub fn gc_distance_angle(a: &Point, b: &Point) -> Angle {
    let d_2 = UnitNegRange::clamp(0.5 * distance(a, b));
    let gc_d_2 = Angle::new(d_2, trig::swap_sin_cos(d_2));
    gc_d_2.x2()
}

/// Calculate an intersection point between the poles of two Great Circles
/// * `pole1`, `pole2` the poles.
///
/// return an intersection point or None if the poles are the same (or opposing) Great Circles.
#[must_use]
pub fn calculate_intersection_point(pole1: &Point, pole2: &Point) -> Option<Point> {
    let c = pole1.cross(pole2);
    if is_small(c.norm(), MIN_NORM) {
        None
    } else {
        Some(c.normalize())
    }
}

/// Convert a slice of `LatLongs` to a Vec of Point on the unit sphere
#[must_use]
pub fn from_slice(values: &[LatLong]) -> Vec<Point> {
    values.iter().map(std::convert::Into::into).collect()
}

impl From<&LatLongs> for Vec<Point> {
    /// Convert `LatLongs` to a vector of Points on the unit sphere
    fn from(value: &LatLongs) -> Self {
        from_slice(&value.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::trig::{from_degrees, Angle, Degrees, Radians};

    #[test]
    fn test_point_lat_longs() {
        let zero = Angle::from_y_x(0.0, 1.0);
        let ninety = Angle::from(Degrees(90.0));
        let one_eighty = Angle::from(Degrees(180.0));

        let lat_lon_south = LatLong::new(-ninety, one_eighty);
        let point_south = Point::from(&lat_lon_south);

        let result = LatLong::from(&point_south);
        assert_eq!(Degrees::from(-ninety), Degrees::from(result.lat()));
        // Note: longitude is now zero, since the poles do not have a Longitude
        assert_eq!(Degrees::from(zero), Degrees::from(result.lon()));

        // Test Greenwich equator
        let lat_lon_0_0 = LatLong::new(zero, zero);
        let point_0 = Point::from(&lat_lon_0_0);
        assert!(is_unit(&point_0));
        assert_eq!(lat_lon_0_0, LatLong::from(&point_0));

        // Test IDL equator
        let lat_lon_0_180 = LatLong::new(zero, one_eighty);
        let point_1 = Point::from(&lat_lon_0_180);
        assert!(is_unit(&point_1));
        assert_eq!(lat_lon_0_180, LatLong::from(&point_1));

        assert_eq!(false, is_west_of(&point_0, &point_1));
        assert_eq!(
            one_eighty.to_radians().0,
            libm::fabs(delta_longitude(&point_0, &point_1).to_radians().0)
        );

        let lat_lon_0_m180 = LatLong::new(zero, -one_eighty);
        let point_2 = Point::from(&lat_lon_0_m180);
        assert!(is_unit(&point_2));
        assert_eq!(lat_lon_0_m180, LatLong::from(&point_2));

        assert_eq!(false, is_west_of(&point_0, &point_2));
        assert_eq!(
            -one_eighty.to_radians(),
            delta_longitude(&point_0, &point_2).to_radians()
        );

        let three_radians = Angle::from(Radians(3.0));
        let lat_lon_0_r3 = LatLong::new(zero, three_radians);
        let point_3 = Point::from(&lat_lon_0_r3);
        assert!(is_unit(&point_3));
        assert_eq!(zero.to_radians(), latitude(&point_3).to_radians());
        assert_eq!(three_radians.to_radians(), longitude(&point_3).to_radians());

        assert!(is_west_of(&point_0, &point_3));
        assert_eq!(
            -three_radians.to_radians(),
            delta_longitude(&point_0, &point_3).to_radians()
        );

        assert_eq!(false, is_west_of(&point_1, &point_3));
        let delta_lon13 =
            std::f64::consts::PI - 3.0 - delta_longitude(&point_1, &point_3).to_radians().0;
        assert!(libm::fabs(delta_lon13) < std::f64::EPSILON);

        let lat_lon_0_mr3 = LatLong::new(zero, -three_radians);
        let point_4 = Point::from(&lat_lon_0_mr3);
        assert!(is_unit(&point_4));
        assert_eq!(zero.to_radians(), latitude(&point_4).to_radians());
        assert_eq!(
            -three_radians.to_radians(),
            longitude(&point_4).to_radians()
        );

        assert_eq!(false, is_west_of(&point_0, &point_4));
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

        assert_eq!(false, is_west_of(&point_0, &point_6));
        assert_eq!(
            pi_m16epsilon.to_radians(),
            delta_longitude(&point_0, &point_6).to_radians()
        );
    }

    #[test]
    fn test_point_distance() {
        let zero = Angle::from_y_x(0.0, 1.0);
        let ninety = Angle::from(Degrees(90.0));
        let one_eighty = Angle::from(Degrees(180.0));

        let south_pole = to_sphere(-ninety, zero);
        let north_pole = to_sphere(ninety, zero);

        assert_eq!(0.0, sq_distance(&south_pole, &south_pole));
        assert_eq!(0.0, sq_distance(&north_pole, &north_pole));
        assert_eq!(4.0, sq_distance(&south_pole, &north_pole));

        assert_eq!(0.0, distance(&south_pole, &south_pole));
        assert_eq!(0.0, distance(&north_pole, &north_pole));
        assert_eq!(2.0, distance(&south_pole, &north_pole));

        // Greenwich equator
        let g_eq = to_sphere(zero, zero);

        // Test IDL equator
        let idl_eq = to_sphere(zero, one_eighty);

        assert_eq!(0.0, sq_distance(&g_eq, &g_eq));
        assert_eq!(0.0, sq_distance(&idl_eq, &idl_eq));
        assert_eq!(4.0, sq_distance(&g_eq, &idl_eq));

        assert_eq!(0.0, distance(&g_eq, &g_eq));
        assert_eq!(0.0, distance(&idl_eq, &idl_eq));
        assert_eq!(2.0, distance(&g_eq, &idl_eq));
    }

    #[test]
    fn test_point_gc_distance() {
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
    fn test_point_gc_distance_angle() {
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

    #[test]
    fn test_calculate_intersection_points() {
        let zero = Angle::from_y_x(0.0, 1.0);
        let ninety = Angle::from(Degrees(90.0));
        let one_eighty = Angle::from(Degrees(180.0));

        let south_pole = to_sphere(-ninety, zero);
        let north_pole = to_sphere(ninety, zero);
        let idl = to_sphere(zero, one_eighty);

        let equator_intersection = calculate_intersection_point(&south_pole, &north_pole);
        assert!(equator_intersection.is_none());

        let gc_intersection1 = calculate_intersection_point(&idl, &north_pole).unwrap();
        let gc_intersection2 = calculate_intersection_point(&idl, &south_pole).unwrap();

        assert_eq!(gc_intersection1, -gc_intersection2);
    }

    #[test]
    fn test_point_from_latlongs() {
        let lats = [44.0, 46.0, 46.0, 44.0, 44.0];
        let lons = [1.0, 1.0, -1.0, -1.0, 1.0];

        let lats_angles = from_degrees(&lats);
        let lons_angles = from_degrees(&lons);
        let latlongs = LatLongs::new(lats_angles.as_slice(), lons_angles.as_slice());

        let points = Vec::<Point>::from(&latlongs);
        assert_eq!(5, points.len());
    }
}
