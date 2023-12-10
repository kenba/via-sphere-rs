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

//! The great_circle3d module contains functions for calculating aspects of
//! great circles on a unit sphere.

use super::e2gc_distance;
use super::point3d::*;
use crate::trig::{swap_sin_cos, Angle, Radians, UnitNegRange};
use crate::Validate;
use contracts::*;

/// Calculate the right hand pole vector of a Great Circle from an initial
/// position and an azimuth.  
/// See: http://www.movable-type.co.uk/scripts/latlong-vectors.html#intersection
/// * `lat` - start point Latitude.
/// * `lon` - start point Longitude.
/// * `azi` - start point azimuth.
///
/// returns the right hand pole vector of the great circle.
#[debug_requires(lat.is_valid_latitude())]
#[debug_ensures(is_unit(&ret))]
pub fn calculate_pole(lat: Angle, lon: Angle, azi: Angle) -> Point3d {
    let x = UnitNegRange::clamp(lon.sin() * azi.cos() - lat.sin() * lon.cos() * azi.sin());
    let y = UnitNegRange::clamp(-lon.cos() * azi.cos() - lat.sin() * lon.sin() * azi.sin());
    let z = UnitNegRange(lat.cos() * azi.sin());

    Point3d::new(x.0, y.0, z.0)
}

/// Calculate the azimuth at a point on the Great Circle defined by pole.
/// * `point` - the point.
/// * `pole` - the right hand pole of the Great Circle.
///
/// returns the azimuth at the point on the great circle.
#[debug_requires(is_unit(point) && is_unit(pole) && are_orthogonal(point, pole))]
pub fn calculate_azimuth(point: &Point3d, pole: &Point3d) -> Angle {
    const MAX_LAT: f64 = 1.0 - 2.0 * std::f64::EPSILON;

    let sin_lat = point.z;
    // if a is close to the North or South poles, azimuth is 180 or 0.
    if MAX_LAT <= libm::fabs(sin_lat) {
        return if 0.0 < sin_lat {
            Angle::from_y_x(0.0, -1.0)
        } else {
            Angle::default()
        };
    }

    Angle::from_y_x(pole.z, pole.xy().perp(&point.xy()))
}

/// Calculate the direction vector along a Great Circle from an initial
/// position and an azimuth.  
/// See: Panou and Korakitis equations: 30, 31, & 32a
/// https://arxiv.org/abs/1811.03513
/// * `lat` - start point Latitude.
/// * `lon` - start point Longitude.
/// * `azi` - start point azimuth.
///
/// returns the direction vector at the point on the great circle.
#[debug_requires(lat.is_valid_latitude())]
#[debug_ensures(is_unit(&ret))]
pub fn calculate_direction(lat: Angle, lon: Angle, azi: Angle) -> Point3d {
    let x = UnitNegRange::clamp(-lat.sin() * lon.cos() * azi.cos() - lon.sin() * azi.sin());
    let y = UnitNegRange::clamp(-lat.sin() * lon.sin() * azi.cos() + lon.cos() * azi.sin());
    let z = UnitNegRange(lat.cos() * azi.cos());

    Point3d::new(x.0, y.0, z.0)
}

/// Calculate the direction vector of a Great Circle arc.
/// * `a` - the start point.
/// * `pole` - the pole of a Great Circle.
///
/// returns the direction vector at the point on the great circle.
#[debug_requires(is_unit(a) && is_unit(pole) && are_orthogonal(a, pole))]
#[debug_ensures(is_unit(&ret))]
pub fn direction(a: &Point3d, pole: &Point3d) -> Point3d {
    pole.cross(a)
}

/// Calculate the position of a point along a Great Circle arc.
/// * `a` - the start point.
/// * `dir` - the direction vector of a Great Circle at a.
/// * `distance` - the a Great Circle as an Angle.
///
/// returns the position vector at the point on the great circle.
#[debug_requires(is_unit(a) && is_unit(dir) && are_orthogonal(a, dir))]
#[debug_ensures(is_unit(&ret))]
pub fn position(a: &Point3d, dir: &Point3d, distance: Angle) -> Point3d {
    distance.cos() * a + distance.sin() * dir
}

/// Calculate the direction vector of a Great Circle rotated by angle.
/// * `dir` - the direction vector of a Great Circle arc.
/// * `pole` - the pole of a Great Circle.
/// * `angle` - the angle to rotate the direction vector by.
///
/// returns the direction vector at the point on the great circle
/// rotated by angle.
#[debug_requires(is_unit(dir) && is_unit(pole) && are_orthogonal(dir, pole))]
#[debug_ensures(is_unit(&ret))]
pub fn rotate(dir: &Point3d, pole: &Point3d, angle: Angle) -> Point3d {
    position(dir, pole, angle)
}

/// Calculate the position of a point rotated by angle at radius.
/// * `a` - the start point.
/// * `pole` - the pole of a Great Circle.
/// * `angle` - the angle to rotate the direction vector by.
/// * `radius` - the radius from the start point.
///
/// returns the position vector at angle and radius from the start point.
#[debug_requires(is_unit(a) && is_unit(pole) && are_orthogonal(a, pole))]
#[debug_ensures(is_unit(&ret))]
pub fn rotate_position(a: &Point3d, pole: &Point3d, angle: Angle, radius: Angle) -> Point3d {
    position(a, &rotate(&direction(a, pole), pole, angle), radius)
}

/// The sine of the across track distance of a point relative to a Great Circle pole.  
/// It is simply the dot product of the pole and the point: pole . point
/// * `pole` - the Great Circle pole.
/// * `point` - the point.
///
/// returns the sine of the across track distance of point relative to the pole.
#[debug_ensures(UnitNegRange::is_valid(&ret))] // guaranteed by clamp
fn sin_xtd(pole: &Point3d, point: &Point3d) -> UnitNegRange {
    UnitNegRange::clamp(pole.dot(point))
}

/// The across track distance of a point relative to a Great Circle pole.
/// * `pole` - the Great Circle pole.
/// * `point` - the point.
///
/// returns the across track distance of point relative to pole.
#[debug_requires(is_unit(pole) && is_unit(point))]
#[debug_ensures(libm::fabs(ret.0) <= std::f64::consts::FRAC_PI_2)]
pub fn cross_track_distance(pole: &Point3d, point: &Point3d) -> Radians {
    Radians(libm::asin(sin_xtd(pole, point).0))
}

/// The square of the Euclidean cross track distance of a point relative to a
/// Great Circle pole.
/// * `pole` - the Great Circle pole.
/// * `point` - the point.
///
/// returns the square of the euclidean distance of point relative to pole.
#[debug_requires(is_unit(pole) && is_unit(point))]
#[debug_ensures(ret <= std::f64::consts::FRAC_PI_2 * std::f64::consts::FRAC_PI_2)]
pub fn sq_cross_track_distance(pole: &Point3d, point: &Point3d) -> f64 {
    let sin_d = libm::fabs(sin_xtd(pole, point).0);
    if sin_d < 2.0 * std::f64::EPSILON {
        0.0
    } else {
        2.0 * (1.0 - swap_sin_cos(UnitNegRange(sin_d)).0)
    }
}

/// Calculate the closest point on a plane to the given point.
/// * `pole` - the Great Circle pole (aka normal) of the plane.
/// * `point` - the point.
///
/// returns the closest point on a plane to the given point.
#[debug_requires(is_unit(pole))]
fn calculate_point_on_plane(pole: &Point3d, point: &Point3d) -> Point3d {
    let t = sin_xtd(pole, point);
    point - pole * t.0
}

/// Calculate the square of the Euclidean along track distance of a point
/// from the start of an Arc.
/// It is calculated using the closest point on the plane to the point.
/// * `a` - the start point of the Great Circle arc.
/// * `pole` - the pole of the Great Circle arc.
/// * `point` - the point.
/// returns the square of the Euclidean along track distance
pub fn sq_along_track_distance(a: &Point3d, pole: &Point3d, point: &Point3d) -> f64 {
    let plane_point = calculate_point_on_plane(pole, point);
    if plane_point.norm() < MIN_LENGTH {
        0.0
    } else {
        sq_distance(a, &(plane_point.normalize()))
    }
}

/// The sine of the along track distance of a point along a Great Circle arc.  
/// It is the triple product of the pole, a and the point:
/// (pole X a) . point = pole . (a X point)
/// * `a` - the start point of the Great Circle arc.
/// * `pole` - the pole of the Great Circle arc.
/// * `point` - the point.
///
/// returns the sine of the along track distance of point relative to the start
/// of a great circle arc..
#[debug_requires(are_orthogonal(a, pole) && is_unit(point))]
#[debug_ensures(UnitNegRange::is_valid(&ret))] // guaranteed by clamp
pub fn sin_atd(a: &Point3d, pole: &Point3d, point: &Point3d) -> UnitNegRange {
    UnitNegRange::clamp(pole.cross(a).dot(point))
}

/// The Great Circle distance of a point along the arc relative to a,
/// (+ve) ahead of a, (-ve) behind a.
/// * `a` - the start point of the Great Circle arc.
/// * `pole` - the pole of the Great Circle arc.
/// * `point` - the point.
///
/// returns the along track distance of point relative to the start of a great circle arc.
#[debug_requires(is_unit(a) && is_unit(pole) && are_orthogonal(a, pole))]
#[debug_ensures(libm::fabs(ret.0) <= std::f64::consts::PI)]
pub fn along_track_distance(a: &Point3d, pole: &Point3d, point: &Point3d) -> Radians {
    let sq_atd = sq_along_track_distance(a, pole, point);
    if sq_atd <= SQ_EPSILON {
        Radians(0.0)
    } else {
        Radians(libm::copysign(
            e2gc_distance(libm::sqrt(sq_atd)).0,
            sin_atd(a, pole, point).0,
        ))
    }
}

/// Calculate Great Circle along and across track distances.
/// * `a` - the start point of the Great Circle arc.
/// * `pole` - the pole of the Great Circle arc.
/// * `p` - the point.
///
/// returns the along and across track distances of point relative to the
/// start of a great circle arc.
#[debug_requires(is_unit(a) && is_unit(pole) && are_orthogonal(a, pole) && is_unit(p))]
pub fn calculate_atd_and_xtd(a: &Point3d, pole: &Point3d, p: &Point3d) -> (Radians, Radians) {
    let mut atd = Radians(0.0);
    let mut xtd = Radians(0.0);

    let mut sq_d = sq_distance(a, p);
    // Point is close to start
    if 2.0 * SQ_EPSILON < sq_d {
        let sin_xtd = sin_xtd(pole, p);
        let abs_sin_xtd = libm::fabs(sin_xtd.0);

        if abs_sin_xtd < 1.0 - 2.0 * std::f64::EPSILON {
            // if the across track distance is significant
            if std::f64::EPSILON < abs_sin_xtd {
                // calculate the signed great circle across track distance
                xtd = Radians(libm::asin(sin_xtd.0));

                // the closest point on the plane of the pole to the point
                let plane_point = p - pole * sin_xtd.0;
                sq_d = if MIN_LENGTH < plane_point.norm() {
                    sq_distance(a, &(plane_point.normalize()))
                } else {
                    0.0
                };
            }
            atd = Radians(libm::copysign(
                e2gc_distance(libm::sqrt(sq_d)).0,
                sin_atd(a, pole, p).0,
            ));
        } else {
            xtd = Radians(if 0.0 < sin_xtd.0 {
                std::f64::consts::PI
            } else {
                -std::f64::consts::PI
            });
        }
    }

    (atd, xtd)
}

/// Calculate the shortest Great Circle distance of a point from a great circle arc.
/// * `a` - the start point of the Great Circle arc.
/// * `pole` - the pole of the Great Circle arc.
/// * `length` - the length of the Great Circle arc.
/// * `p` - the point.
///
/// returns the shortest distance of the point from the great circle arc.
pub fn calculate_shortest_distance(
    a: &Point3d,
    pole: &Point3d,
    length: Radians,
    p: &Point3d,
) -> Radians {
    let (atd, xtd) = calculate_atd_and_xtd(a, pole, p);

    // if point abeam arc then cross track distance is closest
    if (Radians(0.0) <= atd) && (atd <= length) {
        return Radians(libm::fabs(xtd.0));
    }

    let half_length_minus_pi = Radians(0.5 * length.0 - std::f64::consts::PI);
    let is_b_closest = (length < atd) || (atd < half_length_minus_pi);
    let sq_d = if is_b_closest {
        let angle = Angle::from(length);
        let b = position(a, &direction(a, pole), angle);
        sq_distance(&b, p)
    } else {
        sq_distance(a, p)
    };

    e2gc_distance(libm::sqrt(sq_d))
}

#[cfg(test)]
mod tests {
    use crate::sphere::great_circle3d::*;
    use crate::sphere::point3d::Point3d;
    use crate::trig::{Angle, Degrees, DEG2RAD};

    #[test]
    fn test_pole_vector() {
        let zero = Angle::default();
        let forty = Angle::from(Degrees(40.0));
        let seventy_five = Angle::from(Degrees(75.0));
        let ninety = Angle::from(Degrees(90.0));
        let one_ten = Angle::from(Degrees(110.0));
        let one_eighty = Angle::from(Degrees(180.0));

        // North pole, 0 longitude
        let pole_0 = calculate_pole(ninety, zero, one_eighty);
        assert_eq!(0.0, pole_0.x);
        assert_eq!(1.0, pole_0.y);
        assert_eq!(0.0, pole_0.z);

        // North pole, 90 longitude
        let pole_1 = calculate_pole(ninety, ninety, one_eighty);
        assert_eq!(-1.0, pole_1.x);
        assert_eq!(0.0, pole_1.y);
        assert_eq!(0.0, pole_1.z);

        // North pole, 180 longitude
        let pole_2 = calculate_pole(ninety, one_eighty, one_eighty);
        assert_eq!(0.0, pole_2.x);
        assert_eq!(-1.0, pole_2.y);
        assert_eq!(0.0, pole_2.z);

        // South pole, 0 longitude
        let pole_3 = calculate_pole(-ninety, zero, zero);
        assert_eq!(0.0, pole_3.x);
        assert_eq!(-1.0, pole_3.y);
        assert_eq!(0.0, pole_3.z);

        let _pole_4 = calculate_pole(-seventy_five, one_ten, forty);
    }

    #[test]
    fn test_calculate_pole_and_azimuth() {
        let zero = Angle::default();
        let ninety = Angle::from(Degrees(90.0));
        let one_eighty = Angle::from(Degrees(180.0));

        // North pole
        let point_np = Point3d::new(0.0, 0.0, 1.0);

        // South pole
        let point_sp = -point_np;

        // Equator Greenwich meridian
        let point_1 = Point3d::new(1.0, 0.0, 0.0);
        let pole_0 = point_np.cross(&point_1);

        // Azimuth at North pole
        let azimuth_np = calculate_azimuth(&point_np, &pole_0);
        assert_eq!(one_eighty, azimuth_np);

        // Azimuth at South pole
        let azimuth_sp = calculate_azimuth(&point_sp, &pole_0);
        assert_eq!(zero, azimuth_sp);

        // North from Equator
        let pole_1 = calculate_pole(zero, zero, zero);
        let azimuth_1 = calculate_azimuth(&point_1, &pole_1);
        assert_eq!(zero, azimuth_1);

        // East from Equator
        let pole_2 = calculate_pole(zero, zero, ninety);
        let azimuth_2 = calculate_azimuth(&point_1, &pole_2);
        assert_eq!(ninety, azimuth_2);

        // West from Equator
        let pole_3 = calculate_pole(zero, zero, -ninety);
        let azimuth_3 = calculate_azimuth(&point_1, &pole_3);
        assert_eq!(-ninety, azimuth_3);

        // South from Equator
        let pole_4 = calculate_pole(zero, zero, one_eighty);
        let azimuth_4 = calculate_azimuth(&point_1, &pole_4);
        assert_eq!(one_eighty, azimuth_4);
    }

    #[test]
    fn test_direction_vector() {
        let zero = Angle::default();
        let ninety = Angle::from(Degrees(90.0));
        let one_eighty = Angle::from(Degrees(180.0));

        // North pole, 0 longitude
        let dir_0 = calculate_direction(ninety, zero, one_eighty);
        assert_eq!(1.0, dir_0.x);
        assert_eq!(0.0, dir_0.y);
        assert_eq!(0.0, dir_0.z);

        // North pole, 90 longitude
        let dir_1 = calculate_direction(ninety, ninety, one_eighty);
        assert_eq!(0.0, dir_1.x);
        assert_eq!(1.0, dir_1.y);
        assert_eq!(0.0, dir_1.z);

        // North pole, 180 longitude
        let dir_2 = calculate_direction(ninety, one_eighty, one_eighty);
        assert_eq!(-1.0, dir_2.x);
        assert_eq!(0.0, dir_2.y);
        assert_eq!(0.0, dir_2.z);

        // South pole, 0 longitude
        let dir_3 = calculate_direction(-ninety, zero, zero);
        assert_eq!(1.0, dir_3.x);
        assert_eq!(0.0, dir_3.y);
        assert_eq!(0.0, dir_3.z);
    }

    #[test]
    fn test_arc_direction_1() {
        let point_0 = Point3d::new(1.0, 0.0, 0.0);
        let point_1 = Point3d::new(0.0, 1.0, 0.0);
        let point_2 = Point3d::new(0.0, 0.0, 1.0);
        let point_3 = Point3d::new(0.0, 0.0, -1.0);

        assert_eq!(point_2, direction(&point_1, &point_0));
        assert_eq!(point_3, direction(&point_0, &point_1));
    }
    #[test]
    fn test_calculate_pole_direction_and_azimuths() {
        let degree_45 = Angle::from(Degrees(45.0));
        let point_1 = to_sphere(degree_45, degree_45);

        // Increase azimuth around compass from due South
        for i in -180..180 {
            let azi = i as f64;
            let azimuth = Angle::from(Degrees(azi));
            let test_pole = calculate_pole(degree_45, degree_45, azimuth);

            let result = calculate_azimuth(&point_1, &test_pole);
            let delta_azi = libm::fabs(azimuth.to_radians().0 - result.to_radians().0);
            assert!(delta_azi <= 2.0 * std::f64::EPSILON);

            // Ensure pole and direction vectors are orthogonal
            let test_dir = calculate_direction(degree_45, degree_45, azimuth);
            assert!(are_orthogonal(&test_pole, &test_dir));

            // Ensure direction vectors are virtually the same
            let dir_pole = direction(&point_1, &test_pole);
            let delta_x = libm::fabs(test_dir.x - dir_pole.x);
            let delta_y = libm::fabs(test_dir.y - dir_pole.y);
            let delta_z = libm::fabs(test_dir.z - dir_pole.z);
            assert!(
                (delta_x <= std::f64::EPSILON)
                    || (delta_y <= std::f64::EPSILON)
                    || (delta_z <= std::f64::EPSILON)
            );
        }
    }

    #[test]
    fn test_arc_perp_positions_1() {
        let zero = Angle::default();
        let one_degree = Angle::from(Degrees(1.0));
        let golden_angle = Degrees(63.434948822922010648427806279547);
        let golden = Angle::from(golden_angle);

        let point_0 = to_sphere(zero, zero);
        let point_1 = to_sphere(zero, golden);
        let point_2 = to_sphere(golden, zero);

        let pole_1 = point_0.cross(&point_1).normalize();
        let pole_2 = point_0.cross(&point_2).normalize();

        let pos_1 = position(&point_0, &pole_1, one_degree);
        assert!(is_unit(&pos_1));
        let xtd_1 = cross_track_distance(&pole_1, &pos_1);
        assert_eq!(one_degree.to_radians(), xtd_1);
        let sq_xtd_1 = sq_cross_track_distance(&pole_1, &pos_1);
        assert!(sq_xtd_1 - xtd_1.0 * xtd_1.0 < 2.0 * std::f64::EPSILON);
        let atd_1 = along_track_distance(&point_0, &pole_1, &pos_1);
        assert_eq!(0.0, atd_1.0);

        let pos_2 = position(&point_0, &pole_2, -one_degree);
        assert!(is_unit(&pos_2));
        let xtd_2 = cross_track_distance(&pole_2, &pos_2);
        assert_eq!(-one_degree.to_radians(), xtd_2);
        let sq_xtd_2 = sq_cross_track_distance(&pole_2, &pos_2);
        assert!(sq_xtd_2 - xtd_2.0 * xtd_2.0 < 2.0 * std::f64::EPSILON);
        let atd_2 = along_track_distance(&point_0, &pole_2, &pos_2);
        assert_eq!(0.0, atd_2.0);
    }

    #[test]
    fn test_calculate_atd_and_xtd_0() {
        let zero = Angle::default();

        // Arc on the Equator
        let point_0 = to_sphere(zero, zero);
        let pole_0 = Point3d::new(0.0, 0.0, 1.0);

        let (atd, xtd) = calculate_atd_and_xtd(&point_0, &pole_0, &point_0);
        assert_eq!(Radians(0.0), atd);
        assert_eq!(Radians(0.0), xtd);

        // Equator
        let latitude = zero;

        // Increase longitude towards the International Date Line
        for i in -179..180 {
            let lon = i as f64;
            let longitude = Angle::from(Degrees(lon));
            let test_point = to_sphere(latitude, longitude);

            let (atd, xtd) = calculate_atd_and_xtd(&point_0, &pole_0, &test_point);

            let delta_atd = libm::fabs(atd.0 - longitude.to_radians().0);
            assert!(delta_atd <= 94.0 * std::f64::EPSILON);
            let delta_xtd = libm::fabs(xtd.0);
            assert!(delta_xtd < std::f64::EPSILON);
        }
    }

    #[test]
    fn test_calculate_atd_and_xtd_1() {
        let zero = Angle::default();
        let one_degree = Angle::from(Degrees(1.0));

        // Arc on the Equator
        let point_0 = to_sphere(zero, zero);
        let pole_0 = Point3d::new(0.0, 0.0, 1.0);

        // North of Equator
        let latitude = one_degree;

        // Increase longitude towards the International Date Line
        for i in -179..180 {
            let lon = i as f64;
            let longitude = Angle::from(Degrees(lon));
            let test_point = to_sphere(latitude, longitude);

            let (atd, xtd) = calculate_atd_and_xtd(&point_0, &pole_0, &test_point);

            let delta_atd = libm::fabs(atd.0 - longitude.to_radians().0);
            assert!(delta_atd <= 78.0 * std::f64::EPSILON);
            assert_eq!(one_degree.to_radians(), xtd);
        }
    }

    #[test]
    fn test_calculate_shortest_distance_1() {
        let zero = Angle::default();
        let m135 = Angle::from(Degrees(-135.0));
        let m180 = Angle::from(Degrees(-180.0));
        let p179 = Angle::from(Degrees(179.0));

        let point_0 = to_sphere(zero, m135);
        let point_1 = to_sphere(zero, p179);

        // Calculate the pole.
        let pole_0 = point_0.cross(&point_1).normalize();
        let gc_length = gc_distance(&point_0, &point_1);

        let longitude = m180;

        // Increase latitude towards the North Pole
        for i in -89..90 {
            let lat = i as f64;
            let latitude = Angle::from(Degrees(lat));
            let test_point = to_sphere(latitude, longitude);

            let distance = libm::fabs(DEG2RAD * lat);
            let result = calculate_shortest_distance(&point_0, &pole_0, gc_length, &test_point);
            // assert_eq!(distance, result.0);
            let delta_dist = libm::fabs(distance - result.0);
            assert!(delta_dist <= 20.0 * std::f64::EPSILON);
        }
    }
}
