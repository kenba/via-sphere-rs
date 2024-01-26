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

//! The arc3d module contains the Arc type and its associated functions.
//!
use super::great_circle::intersection::{
    calculate_closest_intersection_distances, calculate_intersection_point_lengths,
    calculate_same_gc_distance, is_within,
};
use super::great_circle::{
    calculate_atd_and_xtd, calculate_pole, direction, position, sq_cross_track_distance,
};
use super::{
    are_orthogonal, gc_distance, gc_distance_angle, is_unit, sq_distance, winding_number, Point,
    MIN_NORM, SQ_EPSILON,
};
use crate::latlong;
use crate::latlong::LatLong;
use crate::trig;
use crate::trig::{Angle, Radians, UnitNegRange};
use crate::{min, Validate};
use contracts::{debug_ensures, debug_requires};
use std::convert::From;

/// An arc of a Great Circle on a unit sphere in ECEF coordinates.
pub struct Arc {
    /// The start point of the arc.
    a: Point,
    /// The right hand pole of the Great Circle of the arc.
    pole: Point,
    /// The length of the arc.
    length: Radians,
    /// The half width of the arc.
    half_width: Radians,
}

impl Validate for Arc {
    /// Test whether an Arc is valid.  
    /// I.e. both a and pole are on the unit sphere and are orthogonal and
    /// both length and `half_width` are >= 0.0.
    fn is_valid(&self) -> bool {
        is_unit(&self.a)
            && is_unit(&self.pole)
            && are_orthogonal(&self.a, &self.pole)
            && (0.0 <= self.length.0)
            && (0.0 <= self.half_width.0)
    }
}

impl Arc {
    /// Construct an Arc
    /// * `a` - the start point of the arc.
    /// * `pole` - the right hand pole of the Great Circle of the arc.
    /// * `length` - the length of the arc.
    /// * `half_width` - the half width of the arc.
    #[debug_ensures(ret.is_valid())]
    #[must_use]
    pub fn new(a: Point, pole: Point, length: Radians, half_width: Radians) -> Self {
        Self {
            a,
            pole,
            length,
            half_width,
        }
    }

    /// Construct an Arc
    /// * `a` - the start position
    /// * `azi` - the azimuth at the start of the arc.
    /// * `length` - the length of the arc.
    #[must_use]
    pub fn from_lat_lon_azi_length(a: &LatLong, azi: Angle, length: Radians) -> Self {
        Self::new(
            Point::from(a),
            calculate_pole(a.lat(), a.lon(), azi),
            length,
            Radians(0.0),
        )
    }

    /// Construct an Arc from the start and end positions.  
    /// Note: if the points are the same or antipodal, the pole will be invalid.
    /// * `a`, `b` - the start and end positions
    #[must_use]
    pub fn between_positions(a: &LatLong, b: &LatLong) -> Self {
        let (azimuth, length) = latlong::calculate_azimuth_and_distance(a, b);
        Self::from_lat_lon_azi_length(a, azimuth, length)
    }

    /// Construct an Arc from the start and end points.  
    /// Note: if the points are the same or antipodal, the pole will be invalid.
    /// * `a`, `b` - the start and end points of the arc.
    #[debug_requires(is_unit(a) && is_unit(b) && (a != b))]
    #[must_use]
    pub fn between_points(a: &Point, b: &Point) -> Self {
        const MAX_SQ_LENGTH: f64 = 4.0 * (1.0 - std::f64::EPSILON);

        let mut pole = Point::new(0.0, 0.0, 0.0);
        let mut length = Radians(0.0);
        let sq_length = sq_distance(a, b);
        if std::f64::EPSILON <= sq_length {
            if sq_length < MAX_SQ_LENGTH {
                pole = a.cross(b);
                pole.normalize_mut();
                length = gc_distance_angle(a, b).to_radians();
            } else {
                length = Radians(0.0);
            }
        }

        Self::new(*a, pole, length, Radians(0.0))
    }

    /// Set the `half_width` of an Arc
    /// * `half_width` - the half width of the arc.
    #[must_use]
    pub fn set_half_width(&mut self, half_width: Radians) -> &mut Self {
        self.half_width = half_width;
        self
    }

    /// The start point of the arc.
    #[must_use]
    pub const fn a(&self) -> Point {
        self.a
    }

    /// The right hand pole of the Great Circle at the start point of the arc.
    #[must_use]
    pub const fn pole(&self) -> Point {
        self.pole
    }

    /// The length of the arc.
    #[must_use]
    pub const fn length(&self) -> Radians {
        self.length
    }

    /// The half width of the arc.
    #[must_use]
    pub const fn half_width(&self) -> Radians {
        self.half_width
    }

    /// The azimuth at the start point.
    // pub fn azimuth(&self) -> Angle {
    //     calculate_azimuth(&self.a, &self.pole)
    // }

    /// The direction vector of the arc at the start point.
    #[debug_ensures(is_unit(&ret) && are_orthogonal(&ret, &self.a))]
    #[must_use]
    pub fn direction(&self) -> Point {
        direction(&self.a, &self.pole)
    }

    /// The position of a point at a Great Circle distance along the arc.
    #[debug_ensures(is_unit(&ret))]
    #[must_use]
    pub fn position(&self, distance: Radians) -> Point {
        position(&self.a, &self.direction(), Angle::from(distance))
    }

    /// The end point of the arc.
    #[must_use]
    pub fn b(&self) -> Point {
        self.position(self.length)
    }

    /// The position of a perpendicular point at distance from the arc.
    /// * `point` a point on the arc's great circle.
    /// * `distance` the perpendicular distance from the arc's great circle.
    ///
    /// returns the point at perpendicular distance from point.
    #[debug_requires(is_unit(point) && are_orthogonal(point, &self.pole))]
    #[must_use]
    pub fn perp_position(&self, point: &Point, distance: Radians) -> Point {
        position(point, &self.pole, Angle::from(distance))
    }

    /// The position of a point at angle from the arc start, at arc length.
    /// * `angle` the angle from the arc start.
    ///
    /// returns the point at angle from the arc start, at arc length.
    // pub fn angle_position(&self, angle: Angle) -> Point {
    //     rotate_position(&self.a, &self.pole, angle, Angle::from(self.length))
    // }

    /// The Arc at the end of an Arc, just the point if `half_width` is zero.
    /// @param `at_b` if true the arc at b, else the arc at a.
    ///
    /// @return the end arc at a or b.
    #[must_use]
    pub fn end_arc(&self, at_b: bool) -> Self {
        let p = if at_b { self.b() } else { self.a };
        let pole = direction(&p, &self.pole);
        if std::f64::EPSILON < self.half_width.0 {
            let a = self.perp_position(&p, self.half_width);
            Self::new(a, pole, self.half_width + self.half_width, Radians(0.0))
        } else {
            Self::new(p, pole, Radians(0.0), Radians(0.0))
        }
    }

    /// Calculate Great Circle along and across track distances of point from
    /// the Arc.
    /// * `point` - the point.
    ///
    /// returns the along and across track distances of the point in Radians.
    #[debug_requires(is_unit(point))]
    #[must_use]
    pub fn calculate_atd_and_xtd(&self, point: &Point) -> (Radians, Radians) {
        calculate_atd_and_xtd(&self.a, &self.pole(), point)
    }

    /// Whether a point is alongside the arc.
    /// Note: Does NOT include the ends of the arc.
    /// * `point` - the point.
    ///
    /// returns true if the point is alongside the arc, false otherwise.
    #[debug_requires(is_unit(point))]
    #[must_use]
    pub fn is_alongside(&self, point: &Point) -> bool {
        let pole_p = self.pole.cross(point);
        let b = self.b();

        // Note: only testing sign, so pole_p doesn't need to be normalised
        // true if a is behind p and b is ahead of p
        (pole_p.dot(&self.a) < 0.0) && (pole_p.dot(&b) > 0.0)
    }

    /// The square of the shortest Euclidean distance of a point from the arc.
    /// * `point` - the point.
    ///
    /// returns square of the shortest distance of a point from the arc.
    #[debug_requires(is_unit(point))]
    #[must_use]
    pub fn sq_shortest_distance(&self, point: &Point) -> f64 {
        // if point abeam arc then cross track distance is closest
        if self.is_alongside(point) {
            sq_cross_track_distance(&self.pole, point)
        } else {
            // otherwise one of the ends is closest
            let b = &self.b();
            min(sq_distance(b, point), sq_distance(&self.a, point))
        }
    }

    /// The shortest Great Circle distance of a point from the arc.
    /// * `point` - the point.
    ///
    /// returns the shortest distance of a point from the arc, in Radians.
    #[debug_requires(is_unit(point))]
    #[must_use]
    pub fn shortest_distance(&self, point: &Point) -> Radians {
        let (atd, xtd) = calculate_atd_and_xtd(&self.a, &self.pole, point);
        let abs_xtd = libm::fabs(xtd.0);

        // if point abeam arc then cross track distance is closest
        if (Radians(0.0) <= atd) && (atd <= self.length) {
            return Radians(abs_xtd);
        }

        // If the point is closest to the end of the arc
        let half_length_minus_pi = Radians(0.5f64.mul_add(self.length.0, -std::f64::consts::PI));
        let by_end = (self.length < atd) || (atd < half_length_minus_pi);

        // If the point is not on the same Great Circle as the arc
        if 0.0 < abs_xtd {
            if by_end {
                let b = &self.b();
                gc_distance(b, point)
            } else {
                gc_distance(&self.a, point)
            }
        } else {
            // The point is on the same Great Circle
            // Calculate the shortest along track distance from the arc to the point
            if by_end {
                if self.length < atd {
                    atd - self.length
                } else {
                    Radians(std::f64::consts::TAU + atd.0 - self.length.0)
                }
            } else {
                -atd
            }
        }
    }

    /// The angle from the start of the Arc to a Point.
    /// * `point` - the point.
    ///
    /// return the angle from the start of the Arc to the point.
    // #[debug_requires(is_unit(point))]
    // pub fn start_angle(&self, point: &Point) -> Angle {
    //     let d = sq_distance(&self.a, point);
    //     if SQ_EPSILON < d {
    //         let pole_p = self.a.cross(point).normalize();
    //         let cos_angle = UnitNegRange::clamp(self.pole.dot(&pole_p));
    //         let sin_angle = trig::cosine_from_sine(cos_angle, -self.pole.dot(point));
    //         Angle::new(sin_angle, cos_angle)
    //     } else {
    //         Angle::new(UnitNegRange(0.0), UnitNegRange(1.0))
    //     }
    // }

    /// The turn angle from an Arc to the next Arc, +ve clockwise.
    /// * `other` - the next arc.
    ///
    /// return the turn angle from the end of the Arc to the next Arc.
    #[must_use]
    pub fn turn_angle(&self, other: &Self) -> Angle {
        let cos_angle = UnitNegRange::clamp(self.pole.dot(&other.pole));
        let sin_angle = trig::cosine_from_sine(cos_angle, -other.pole.dot(&self.a));
        Angle::new(sin_angle, cos_angle)
    }

    /// Calculate the winding number of a point against the Arc as the edge of a polygon.
    #[debug_requires(is_unit(point))]
    #[must_use]
    pub fn winding_number(&self, point: &Point) -> i32 {
        winding_number(&self.a, &self.pole, &self.b(), point)
    }
}

/// Calculate the lengths along a pair of Arcs on the same (or reciprocal)
/// Great Circles to their closest intersection or reference points.
/// * `arc1`, `arc2` the arcs.
#[debug_ensures(ret.0.is_valid() && ret.1.is_valid())]
#[must_use]
pub fn calculate_reference_lengths(arc1: &Arc, arc2: &Arc) -> (Radians, Radians) {
    calculate_intersection_point_lengths(
        &arc1.a,
        &arc1.pole,
        arc1.length(),
        &arc2.a,
        &arc2.pole,
        arc2.length(),
    )
}

/// Calculate whether a pair of Arcs intersect and (if so) where.
/// * `arc1`, `arc2` the arcs.
///
/// returns the distance along the first arc to the second arc or -1 if they
/// don't intersect.
#[must_use]
pub fn calculate_intersection_point_length(arc1: &Arc, arc2: &Arc) -> Radians {
    let (length1, length2) = calculate_reference_lengths(arc1, arc2);

    // Determine whether both lengths are within the arcs
    if (0.0 <= length2.0)
        && (0.0 <= length1.0)
        && (length2 <= arc2.length())
        && (length1 <= arc1.length())
    {
        length1
    } else {
        Radians(-1.0)
    }
}

/// Calculate the square of the closest Euclidian distance of a pair of
/// arcs on different Great Circles.
/// * `arc1`, `arc2` the arcs
/// * `c` the intersection point.
///
/// returns the square of the Euclidian distance between the arcs in radians.
#[must_use]
fn calculate_different_gc_sq_distance(arc1: &Arc, arc2: &Arc, c: &Point) -> f64 {
    let (ref1_length, ref2_length, _) = calculate_closest_intersection_distances(
        &arc1.a,
        &arc1.pole,
        arc1.length(),
        &arc2.a,
        &arc2.pole,
        arc2.length(),
        c,
    );

    // is the intersection within both arcs?
    let c_within_arc1 = is_within(ref1_length.0, arc1.length.0);
    let c_within_arc2 = is_within(ref2_length.0, arc2.length.0);
    if c_within_arc1 && c_within_arc2 {
        // The arcs intersect
        0.0
    } else {
        // calculate the shortest distances to the geodesic ends
        let min_1 = min(
            arc1.sq_shortest_distance(&arc2.a),
            arc1.sq_shortest_distance(&arc2.b()),
        );
        let min_2 = min(
            arc2.sq_shortest_distance(&arc1.a),
            arc2.sq_shortest_distance(&arc1.b()),
        );
        min(min_1, min_2)
    }
}

/// Calculate the distance between two arcs on different great circles.
/// * `arc1`, `arc2` the arcs
/// * `c` the intersection point.
///
/// returns the distance between the arcs in radians.
#[must_use]
fn calculate_different_gc_distance(arc1: &Arc, arc2: &Arc, c: &Point) -> Radians {
    let (ref1_length, ref2_length, _) = calculate_closest_intersection_distances(
        &arc1.a,
        &arc1.pole,
        arc1.length(),
        &arc2.a,
        &arc2.pole,
        arc2.length(),
        c,
    );

    // is the intersection within both arcs?
    let c_within_arc1 = is_within(ref1_length.0, arc1.length.0);
    let c_within_arc2 = is_within(ref2_length.0, arc2.length.0);
    if c_within_arc1 && c_within_arc2 {
        // The arcs intersect
        Radians(0.0)
    } else {
        // calculate the shortest distances to the geodesic ends
        let min_1 = min(
            arc1.shortest_distance(&arc2.a).0,
            arc1.shortest_distance(&arc2.b()).0,
        );
        let min_2 = min(
            arc2.shortest_distance(&arc1.a).0,
            arc2.shortest_distance(&arc1.b()).0,
        );
        Radians(min(min_1, min_2))
    }
}

/// Calculate the square of the minimum Euclidean distance between two valid Arcs.
/// * `arc1`, `arc2` the arcs.
///
/// return the square of the minimum Euclidean distance between the arcs.
#[must_use]
pub fn calculate_sq_distance(arc1: &Arc, arc2: &Arc) -> f64 {
    // Calculate distance between start points
    let sq_d = sq_distance(&arc1.a, &arc2.a);
    if sq_d < SQ_EPSILON {
        0.0
    } else {
        // Calculate the intersection.
        let c = arc1.pole.cross(&arc2.pole);
        if c.norm() < MIN_NORM {
            let gc_d = trig::e2gc_distance(libm::sqrt(sq_d));
            let min_d = trig::gc2e_distance(calculate_same_gc_distance(
                &arc1.a,
                &arc1.pole,
                arc1.length(),
                &arc2.a,
                &arc2.pole,
                arc2.length(),
                gc_d,
            ));
            min_d * min_d
        } else {
            calculate_different_gc_sq_distance(arc1, arc2, &c.normalize())
        }
    }
}

/// Calculate the shortest Great Circle distance between two Arcs.
/// * `arc1`, `arc2` the arcs.
///
/// return the shortest Great Circle distance between the arcs in Radians.
#[must_use]
pub fn calculate_distance(arc1: &Arc, arc2: &Arc) -> Radians {
    // Calculate the great circle distance between the start points.
    let gc_d = gc_distance(&arc1.a, &arc2.a);
    if 2.0 * std::f64::EPSILON < gc_d.0 {
        // Calculate the intersection.
        let c = arc1.pole.cross(&arc2.pole);
        if c.norm() < MIN_NORM {
            calculate_same_gc_distance(
                &arc1.a,
                &arc1.pole,
                arc1.length(),
                &arc2.a,
                &arc2.pole,
                arc2.length(),
                gc_d,
            )
        } else {
            calculate_different_gc_distance(arc1, arc2, &c.normalize())
        }
    } else {
        Radians(0.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::is_within_tolerance;
    use crate::sphere::*;
    use crate::trig::{gc2e_distance, Angle, Degrees, DEG2RAD};

    #[test]
    fn test_wide_arc3d_1() {
        let lat_lon_amsterdam =
            LatLong::new(Angle::from(Degrees(52.37)), Angle::from(Degrees(4.90)));
        let lat_lon_paris = LatLong::new(Angle::from(Degrees(48.86)), Angle::from(Degrees(2.35)));
        let half_width = Radians(0.01);

        let precision = 100.0 * std::f64::EPSILON;

        // let point3d_1 = Point::from(&lat_lon_paris);
        let point3d_2 = Point::from(&lat_lon_amsterdam);

        let mut binding = Arc::between_positions(&lat_lon_paris, &lat_lon_amsterdam);
        let arc3d1 = binding.set_half_width(half_width);
        assert!(arc3d1.is_valid());

        assert!(is_within_tolerance(
            0.0,
            sq_distance(&point3d_2, &arc3d1.b()),
            precision
        ));
        assert!(is_within_tolerance(
            half_width.0,
            arc3d1.half_width().0,
            precision
        ));

        let end_arc1 = arc3d1.end_arc(false);
        let point3d_3 = end_arc1.a();
        let (_atd, xtd) = arc3d1.calculate_atd_and_xtd(&point3d_3);
        assert!(is_within_tolerance(half_width.0, xtd.0, precision));
        let point3d_4 = end_arc1.b();
        let (_atd, xtd) = arc3d1.calculate_atd_and_xtd(&point3d_4);
        assert!(is_within_tolerance(-half_width.0, xtd.0, precision));

        let end_arc2 = arc3d1.end_arc(true);
        let point3d_5 = end_arc2.a();
        let (_atd, xtd) = arc3d1.calculate_atd_and_xtd(&point3d_5);
        assert!(is_within_tolerance(half_width.0, xtd.0, precision));
        let point3d_6 = end_arc2.b();
        let (_atd, xtd) = arc3d1.calculate_atd_and_xtd(&point3d_6);
        assert!(is_within_tolerance(-half_width.0, xtd.0, precision));
    }

    #[test]
    fn test_arc3d_calculate_atd_and_xtd() {
        let zero = Angle::default();
        let one_degree = Angle::from(Degrees(1.0));
        let degree_179 = Angle::from(Degrees(179.0));

        // Arc on the Equator
        let lat_lon_0_0 = LatLong::new(zero, zero);
        let lat_lon_0_179 = LatLong::new(zero, degree_179);
        let arc = Arc::between_positions(&lat_lon_0_0, &lat_lon_0_179);

        // North of Equator
        let latitude = one_degree;

        // Increase longitude towards the International Date Line
        for i in -179..180 {
            let lon = i as f64;
            let longitude = Angle::from(Degrees(lon));
            let test_point = to_sphere(latitude, longitude);

            let (atd, xtd) = arc.calculate_atd_and_xtd(&test_point);

            let delta_atd = libm::fabs(atd.0 - longitude.to_radians().0);
            assert!(delta_atd <= 78.0 * std::f64::EPSILON);

            assert_eq!(one_degree.to_radians(), xtd);
        }
    }

    #[test]
    fn test_arc3d_shortest_distance_1() {
        let zero = Angle::default();
        let degree_m135 = Angle::from(Degrees(-135.0));
        let degree_179 = Angle::from(Degrees(179.0));

        // Arc on the Equator, around IDL
        let lat_lon_0_m135 = LatLong::new(zero, degree_m135);
        let lat_lon_0_179 = LatLong::new(zero, degree_179);
        let arc = Arc::between_positions(&lat_lon_0_m135, &lat_lon_0_179);

        // International date line
        let longitude = Angle::from(Degrees(-180.0));

        // Increase latitude towards the North Pole
        for i in -89..90 {
            let lat = i as f64;
            let latitude = Angle::from(Degrees(lat));
            let test_point = to_sphere(latitude, longitude);

            let xtd = arc.shortest_distance(&test_point);
            let delta_xtd = libm::fabs(xtd.0 - latitude.abs().to_radians().0);
            assert!(delta_xtd <= 20.0 * std::f64::EPSILON);

            let xtd2 = arc.sq_shortest_distance(&test_point);
            let e_lat = gc2e_distance(latitude.abs().to_radians());
            let delta_xtd2 = libm::fabs(xtd2 - e_lat * e_lat);
            assert!(delta_xtd2 <= 50.0 * std::f64::EPSILON);
        }
    }

    #[test]
    fn test_closest_intersection_lengths_common_start_point() {
        let one = Angle::from(Degrees(1.0));
        let m_one = -one;

        let one_one = to_sphere(one, one);
        let minus_one_one = to_sphere(m_one, one);
        let one_minus_one = to_sphere(one, m_one);

        let line_1 = Arc::between_points(&minus_one_one, &one_one);
        let line_2 = Arc::between_points(&minus_one_one, &one_minus_one);

        let intersection_lengths = calculate_reference_lengths(&line_1, &line_2);
        assert_eq!(Radians(0.0), intersection_lengths.0);
        assert_eq!(Radians(0.0), intersection_lengths.1);

        let intersection_length = calculate_intersection_point_length(&line_1, &line_2);
        assert_eq!(Radians(0.0), intersection_length);
    }

    #[test]
    fn test_closest_intersection_lengths_common_end_point() {
        let zero = Angle::default();
        let one = Angle::from(Degrees(1.0));
        let m_one = -one;

        let zero_zero = to_sphere(zero, zero);
        let minus_one_one = to_sphere(m_one, one);
        let minus_one_minus_one = to_sphere(m_one, m_one);

        let line_1 = Arc::between_points(&minus_one_one, &zero_zero);
        let line_2 = Arc::between_points(&minus_one_minus_one, &zero_zero);

        let intersection_lengths = calculate_reference_lengths(&line_1, &line_2);
        assert_eq!(line_1.length(), intersection_lengths.0);
        assert_eq!(line_2.length(), intersection_lengths.1);

        let intersection_length = calculate_intersection_point_length(&line_1, &line_2);
        assert_eq!(line_1.length(), intersection_length);
    }

    #[test]
    fn test_closest_intersection_lengths_common_start_end_points() {
        let one = Angle::from(Degrees(1.0));
        let m_one = -one;

        let one_one = to_sphere(one, one);
        let minus_one_one = to_sphere(m_one, one);
        let one_minus_one = to_sphere(one, m_one);

        let line_1 = Arc::between_points(&minus_one_one, &one_one);
        let line_2 = Arc::between_points(&one_minus_one, &minus_one_one);

        let intersection_lengths = calculate_reference_lengths(&line_1, &line_2);
        assert_eq!(Radians(0.0), intersection_lengths.0);
        // assert_eq!(line_2.length(), intersection_lengths.1);
        let delta_length = libm::fabs(line_2.length().0 - (intersection_lengths.1).0);
        assert!(delta_length <= std::f64::EPSILON);

        let intersection_length = calculate_intersection_point_length(&line_1, &line_2);
        assert_eq!(Radians(0.0), intersection_length);
    }

    #[test]
    fn test_closest_intersection_lengths_common_end_start_points() {
        let one = Angle::from(Degrees(1.0));
        let m_one = -one;

        let one_one = to_sphere(one, one);
        let minus_one_one = to_sphere(m_one, one);
        let one_minus_one = to_sphere(one, m_one);

        let line_1 = Arc::between_points(&minus_one_one, &one_one);
        let line_2 = Arc::between_points(&one_one, &one_minus_one);

        let intersection_lengths = calculate_reference_lengths(&line_1, &line_2);
        // assert_eq!(sq_length_1, intersection_lengths.0);
        let delta_length = libm::fabs(line_1.length().0 - (intersection_lengths.0).0);
        assert!(delta_length <= std::f64::EPSILON);
        assert_eq!(Radians(0.0), intersection_lengths.1);

        let intersection_length = calculate_intersection_point_length(&line_1, &line_2);
        // assert_eq!(line_1.length(), intersection_length);
        let delta_length = libm::fabs(line_1.length().0 - intersection_length.0);
        assert!(delta_length <= std::f64::EPSILON);
    }

    #[test]
    fn test_closest_intersection_lengths_crossing_lines() {
        let zero = Angle::default();
        let one = Angle::from(Degrees(1.0));
        let m_one = -one;

        let zero_zero = to_sphere(zero, zero);
        let one_one = to_sphere(one, one);
        let minus_one_one = to_sphere(m_one, one);
        let one_minus_one = to_sphere(one, m_one);
        let minus_one_minus_one = to_sphere(m_one, m_one);

        let line_1 = Arc::between_points(&minus_one_minus_one, &one_one);
        let line_2 = Arc::between_points(&minus_one_one, &one_minus_one);

        let gc_result = gc_distance(&minus_one_minus_one, &zero_zero);
        let intersection_lengths = calculate_reference_lengths(&line_1, &line_2);
        assert_eq!(gc_result, intersection_lengths.0);
        assert_eq!(gc_result, intersection_lengths.1);

        let intersection_length = calculate_intersection_point_length(&line_1, &line_2);
        assert_eq!(gc_result, intersection_length);
    }

    #[test]
    fn test_closest_intersection_lengths_opposite_intersections() {
        let one = Angle::from(Degrees(1.0));
        let two = Angle::from(Degrees(2.0));
        let m_two = -two;
        let idl_m_two = Angle::from(Degrees(-178.0));

        let eighty_seven = Angle::from(Degrees(87.0));
        let ninety_two = Angle::from(Degrees(92.0));

        let one_one = to_sphere(one, one);
        let one_ninety_two = to_sphere(one, ninety_two);
        let minus_two_eighty_seven = to_sphere(m_two, eighty_seven);
        let minus_two_idl_minus_two = to_sphere(m_two, idl_m_two);

        let line_1 = Arc::between_points(&one_one, &one_ninety_two);
        let line_2 = Arc::between_points(&minus_two_eighty_seven, &minus_two_idl_minus_two);

        let intersection_lengths = calculate_reference_lengths(&line_1, &line_2);
        assert_eq!(0.3169366019437673, intersection_lengths.0 .0);
        assert_eq!(-1.1851988424812585, intersection_lengths.1 .0);

        let intersection_length = calculate_intersection_point_length(&line_1, &line_2);
        assert_eq!(-1.0, intersection_length.0);
    }

    #[test]
    fn test_closest_intersection_lengths_long_arcs() {
        let one = Angle::from(Degrees(1.0));
        let two = Angle::from(Degrees(2.0));
        let m_two = -two;

        let idl_one = Angle::from(Degrees(179.0));
        let idl_minus_one = -idl_one;

        let one_one = to_sphere(one, one);
        let one_idl_one = to_sphere(one, idl_one);
        let minus_two_minus_two = to_sphere(m_two, m_two);
        let minus_two_idl_minus_one = to_sphere(m_two, idl_minus_one);

        let line_1 = Arc::between_points(&one_one, &one_idl_one);
        let line_2 = Arc::between_points(&minus_two_minus_two, &minus_two_idl_minus_one);

        let intersection_lengths = calculate_reference_lengths(&line_1, &line_2);
        assert_eq!(3.067628989963839, intersection_lengths.0 .0);
        assert_eq!(3.141513106563121, intersection_lengths.1 .0);

        let intersection_length = calculate_intersection_point_length(&line_1, &line_2);
        assert_eq!(-1.0, intersection_length.0);
    }

    #[test]
    fn test_closest_intersection_lengths_same_gc_0() {
        let zero = Angle::default();
        let m87 = Angle::from(Degrees(-87.0));
        let m88 = Angle::from(Degrees(-88.0));
        let m180 = Angle::from(Degrees(-180.0));

        let south_pole_1 = to_sphere(m88, m180);
        let south_pole_2 = to_sphere(m87, zero);

        // A 300 Nm (5 degrees) long arc across the South Pole
        let line_1 = Arc::between_points(&south_pole_1, &south_pole_2);

        let intersection_lengths = calculate_reference_lengths(&line_1, &line_1);
        assert_eq!(Radians(0.0), intersection_lengths.0);
        assert_eq!(Radians(0.0), intersection_lengths.1);

        let gc_length = calculate_intersection_point_length(&line_1, &line_1);
        assert_eq!(Radians(0.0), gc_length);
    }

    #[test]
    fn test_closest_intersection_lengths_same_gc_1() {
        let zero = Angle::default();
        let m85 = Angle::from(Degrees(-85.0));
        let m87 = Angle::from(Degrees(-87.0));
        let m88 = Angle::from(Degrees(-88.0));
        let m180 = Angle::from(Degrees(-180.0));

        let south_pole_1 = to_sphere(m88, m180);
        let south_pole_2 = to_sphere(m87, zero);
        let zero_zero = to_sphere(zero, zero);
        let m85_zero = to_sphere(m85, zero);

        // A 300 Nm (5 degrees) long arc across the South Pole
        let line_1 = Arc::between_points(&south_pole_1, &south_pole_2);

        // A long line to the South Pole from the equator
        let line_2 = Arc::between_points(&zero_zero, &m85_zero);

        let intersection_lengths = calculate_reference_lengths(&line_1, &line_2);
        assert_eq!(Radians(0.12217304763960288), intersection_lengths.0);
        assert_eq!(Radians(1.4835298641951802), intersection_lengths.1);

        let gc_length = calculate_intersection_point_length(&line_1, &line_2);
        assert_eq!(Radians(-1.0), gc_length);
    }

    #[test]
    fn test_closest_intersection_lengths_same_gc_2() {
        let zero = Angle::default();
        let m85 = Angle::from(Degrees(-85.0));
        let m87 = Angle::from(Degrees(-87.0));
        let m88 = Angle::from(Degrees(-88.0));
        let m180 = Angle::from(Degrees(-180.0));

        let south_pole_1 = to_sphere(m88, m180);
        let south_pole_2 = to_sphere(m87, zero);
        let zero_zero = to_sphere(zero, zero);
        let m85_zero = to_sphere(m85, zero);

        // A 300 Nm (5 degrees) long arc across the South Pole
        let line_1 = Arc::between_points(&south_pole_1, &south_pole_2);

        // A long line from the South Pole to the equator
        let line_2 = Arc::between_points(&m85_zero, &zero_zero);

        let intersection_lengths = calculate_reference_lengths(&line_1, &line_2);
        assert_eq!(Radians(0.12217304763960306), intersection_lengths.0);
        assert_eq!(Radians(0.0), intersection_lengths.1);

        let gc_length = calculate_intersection_point_length(&line_1, &line_2);
        assert_eq!(Radians(-1.0), gc_length);
    }

    #[test]
    fn test_winding_number() {
        let zero = Angle::default();
        let one = Angle::from(Degrees(1.0));
        let m_one = -one;

        let zero_zero = to_sphere(zero, zero);
        let zero_minus_one = to_sphere(zero, m_one);
        let zero_one = to_sphere(zero, one);
        let minus_one_zero = to_sphere(m_one, zero);
        let one_zero = to_sphere(one, zero);

        // Arc from West to East
        let arc_1 = Arc::between_points(&zero_minus_one, &zero_one);
        assert_eq!(1, arc_1.winding_number(&one_zero));
        assert_eq!(0, arc_1.winding_number(&zero_zero));
        assert_eq!(0, arc_1.winding_number(&minus_one_zero));

        let end_arc1 = arc_1.end_arc(false);
        assert_eq!(end_arc1.a(), arc_1.a());

        // Arc from East to West
        let arc_m1 = Arc::between_points(&zero_one, &zero_minus_one);
        assert_eq!(-1, arc_m1.winding_number(&one_zero));
        assert_eq!(0, arc_m1.winding_number(&zero_zero));
        assert_eq!(0, arc_m1.winding_number(&minus_one_zero));

        let end_arc_m1 = arc_m1.end_arc(false);
        assert_eq!(end_arc_m1.a(), arc_m1.a());
    }

    #[test]
    fn test_arc_arc_distance_functions_common_start_point() {
        let one = Angle::from(Degrees(1.0));
        let m_one = -one;

        let one_one = to_sphere(one, one);
        let minus_one_one = to_sphere(m_one, one);
        let one_minus_one = to_sphere(one, m_one);

        let line_1 = Arc::between_points(&minus_one_one, &one_one);
        let line_2 = Arc::between_points(&minus_one_one, &one_minus_one);

        // Calculate the intersection.
        let c = line_1.pole.cross(&line_2.pole).normalize();

        let gc_d_1 = calculate_different_gc_distance(&line_1, &line_2, &c);
        assert_eq!(Radians(0.0), gc_d_1);

        let gc_d = calculate_distance(&line_1, &line_2);
        assert_eq!(Radians(0.0), gc_d);

        let sq_d = calculate_sq_distance(&line_1, &line_2);
        assert_eq!(0.0, sq_d);
    }

    #[test]
    fn test_arc_arc_distance_functions_common_end_point() {
        let zero = Angle::default();
        let one = Angle::from(Degrees(1.0));
        let m_one = -one;

        let zero_zero = to_sphere(zero, zero);
        let minus_one_one = to_sphere(m_one, one);
        let minus_one_minus_one = to_sphere(m_one, m_one);

        let line_1 = Arc::between_points(&minus_one_one, &zero_zero);
        let line_2 = Arc::between_points(&minus_one_minus_one, &zero_zero);

        // Calculate the intersection.
        let c = line_1.pole.cross(&line_2.pole).normalize();

        let gc_d_1 = calculate_different_gc_distance(&line_1, &line_2, &c);
        assert!(gc_d_1.0 <= std::f64::EPSILON);

        let gc_d = calculate_distance(&line_1, &line_2);
        assert!(gc_d.0 <= std::f64::EPSILON);

        let sq_d = calculate_sq_distance(&line_1, &line_2);
        assert!(sq_d <= std::f64::EPSILON);
    }

    #[test]
    fn test_arc_arc_distance_functions_common_start_end_points() {
        let one = Angle::from(Degrees(1.0));
        let m_one = -one;

        let one_one = to_sphere(one, one);
        let minus_one_one = to_sphere(m_one, one);
        let one_minus_one = to_sphere(one, m_one);

        let line_1 = Arc::between_points(&minus_one_one, &one_one);
        let line_2 = Arc::between_points(&one_minus_one, &minus_one_one);

        // Calculate the intersection.
        let c = line_1.pole.cross(&line_2.pole).normalize();

        let gc_d_1 = calculate_different_gc_distance(&line_1, &line_2, &c);
        assert_eq!(Radians(0.0), gc_d_1);

        let gc_d = calculate_distance(&line_1, &line_2);
        assert_eq!(Radians(0.0), gc_d);

        let sq_d = calculate_sq_distance(&line_1, &line_2);
        assert_eq!(0.0, sq_d);
    }

    #[test]
    fn test_arc_arc_distance_functions_common_end_start_points() {
        let one = Angle::from(Degrees(1.0));
        let m_one = -one;

        let one_one = to_sphere(one, one);
        let minus_one_one = to_sphere(m_one, one);
        let one_minus_one = to_sphere(one, m_one);

        let line_1 = Arc::between_points(&minus_one_one, &one_one);
        let line_2 = Arc::between_points(&one_one, &one_minus_one);

        // Calculate the intersection.
        let c = line_1.pole.cross(&line_2.pole).normalize();

        let gc_d_1 = calculate_different_gc_distance(&line_1, &line_2, &c);
        assert_eq!(Radians(0.0), gc_d_1);

        let gc_d = calculate_distance(&line_1, &line_2);
        assert_eq!(Radians(0.0), gc_d);

        let sq_d = calculate_sq_distance(&line_1, &line_2);
        assert_eq!(0.0, sq_d);
    }

    #[test]
    fn test_arc_arc_distance_functions_crossing_lines_1() {
        let one = Angle::from(Degrees(1.0));
        let m_one = -one;

        let one_one = to_sphere(one, one);
        let minus_one_one = to_sphere(m_one, one);
        let one_minus_one = to_sphere(one, m_one);
        let minus_one_minus_one = to_sphere(m_one, m_one);

        let line_1 = Arc::between_points(&minus_one_minus_one, &one_one);
        let line_2 = Arc::between_points(&minus_one_one, &one_minus_one);

        // Calculate the intersection.
        let c = line_1.pole.cross(&line_2.pole).normalize();

        let gc_d_1 = calculate_different_gc_distance(&line_1, &line_2, &c);
        assert_eq!(Radians(0.0), gc_d_1);

        let gc_d = calculate_distance(&line_1, &line_2);
        assert_eq!(Radians(0.0), gc_d);

        let sq_d = calculate_sq_distance(&line_1, &line_2);
        assert_eq!(0.0, sq_d);
    }

    #[test]
    fn test_arc_arc_distance_functions_crossing_lines_2() {
        let one = Angle::from(Degrees(1.0));
        let m_one = -one;

        let one_one = to_sphere(one, one);
        let minus_one_one = to_sphere(m_one, one);
        let one_minus_one = to_sphere(one, m_one);
        let minus_one_minus_one = to_sphere(m_one, m_one);

        let line_1 = Arc::between_points(&minus_one_minus_one, &one_one);
        let line_2 = Arc::between_points(&minus_one_one, &one_minus_one);

        // Calculate the antiodal intersection.
        let c = line_2.pole.cross(&line_1.pole).normalize();

        let gc_d_1 = calculate_different_gc_distance(&line_1, &line_2, &c);
        assert_eq!(Radians(0.0), gc_d_1);

        let gc_d = calculate_distance(&line_2, &line_1);
        assert_eq!(Radians(0.0), gc_d);

        let sq_d = calculate_sq_distance(&line_1, &line_2);
        assert_eq!(0.0, sq_d);
    }

    #[test]
    fn test_arc_arc_distance_functions_opposite_intersections() {
        let one = Angle::from(Degrees(1.0));
        let two = Angle::from(Degrees(2.0));
        let m_two = -two;
        let idl_m_two = Angle::from(Degrees(-178.0));

        let eighty_seven = Angle::from(Degrees(87.0));
        let ninety_two = Angle::from(Degrees(92.0));

        let one_one = to_sphere(one, one);
        let one_ninety_two = to_sphere(one, ninety_two);
        let minus_two_eighty_seven = to_sphere(m_two, eighty_seven);
        let minus_two_idl_minus_two = to_sphere(m_two, idl_m_two);

        let line_1 = Arc::between_points(&one_one, &one_ninety_two);
        let line_2 = Arc::between_points(&minus_two_eighty_seven, &minus_two_idl_minus_two);

        // Calculate the intersection.
        let c = line_1.pole.cross(&line_2.pole).normalize();

        let result = Radians(0.053834015693803215);

        let gc_d_1 = calculate_different_gc_distance(&line_1, &line_2, &c);
        assert_eq!(result, gc_d_1);

        let gc_d = calculate_distance(&line_1, &line_2);
        assert_eq!(result, gc_d);

        let sq_result = 0.0028974013974287782;

        let sq_d = calculate_sq_distance(&line_1, &line_2);
        assert_eq!(sq_result, sq_d);
    }

    #[test]
    fn test_arc_arc_distance_functions_long_arcs() {
        let one = Angle::from(Degrees(1.0));
        let two = Angle::from(Degrees(2.0));
        let m_two = -two;

        let idl_one = Angle::from(Degrees(179.0));
        let idl_minus_one = -idl_one;

        let one_one = to_sphere(one, one);
        let one_idl_one = to_sphere(one, idl_one);
        let minus_two_minus_two = to_sphere(m_two, m_two);
        let minus_two_idl_minus_one = to_sphere(m_two, idl_minus_one);

        let line_1 = Arc::between_points(&one_one, &one_idl_one);
        let line_2 = Arc::between_points(&minus_two_minus_two, &minus_two_idl_minus_one);

        // Calculate the intersection.
        let c = line_1.pole.cross(&line_2.pole).normalize();

        let result = Radians(0.06292579139179703);

        let gc_d_1 = calculate_different_gc_distance(&line_1, &line_2, &c);
        assert_eq!(result, gc_d_1);

        let gc_d = calculate_distance(&line_1, &line_2);
        assert_eq!(result, gc_d);

        let e_result = gc2e_distance(result);
        let sq_result = e_result * e_result;

        let sq_d = calculate_sq_distance(&line_1, &line_2);
        assert_eq!(sq_result, sq_d);
    }

    #[test]
    fn test_arc_arc_distance_functions_same_gc_0() {
        let zero = Angle::default();
        let m87 = Angle::from(Degrees(-87.0));
        let m88 = Angle::from(Degrees(-88.0));
        let m180 = Angle::from(Degrees(-180.0));

        let south_pole_1 = to_sphere(m88, m180);
        let south_pole_2 = to_sphere(m87, zero);

        // A 300 Nm (5 degrees) long arc across the South Pole
        let line_1 = Arc::between_points(&south_pole_1, &south_pole_2);

        let gc_d = calculate_distance(&line_1, &line_1);
        assert_eq!(Radians(0.0), gc_d);

        // let e_result = gc2e_distance(result);
        // let sq_result = e_result * e_result;

        let sq_d = calculate_sq_distance(&line_1, &line_1);
        assert_eq!(0.0, sq_d);
    }

    #[test]
    fn test_arc_arc_distance_functions_same_gc_1() {
        let zero = Angle::default();
        let m85 = Angle::from(Degrees(-85.0));
        let m87 = Angle::from(Degrees(-87.0));
        let m88 = Angle::from(Degrees(-88.0));
        let m180 = Angle::from(Degrees(-180.0));

        let south_pole_1 = to_sphere(m88, m180);
        let south_pole_2 = to_sphere(m87, zero);
        let zero_zero = to_sphere(zero, zero);
        let m85_zero = to_sphere(m85, zero);

        // A 300 Nm (5 degrees) long arc across the South Pole
        let line_1 = Arc::between_points(&south_pole_1, &south_pole_2);

        // A long line to the South Pole from the equator
        let line_2 = Arc::between_points(&zero_zero, &m85_zero);

        let result = DEG2RAD * 2.0;

        let gc_d = calculate_distance(&line_1, &line_2);
        // assert_eq!(result, gc_d.0);
        let delta_length = libm::fabs(result - gc_d.0);
        assert!(delta_length <= 1.0 * std::f64::EPSILON);

        let e_result = gc2e_distance(Radians(result));
        let sq_result = e_result * e_result;

        let sq_d = calculate_sq_distance(&line_1, &line_2);
        // assert_eq!(sq_result, sq_d);
        let delta_sq_length = libm::fabs(sq_result - sq_d);
        assert!(delta_sq_length <= 1.0 * std::f64::EPSILON);
    }

    #[test]
    fn test_arc_arc_distance_functions_same_gc_2() {
        let zero = Angle::default();
        let m85 = Angle::from(Degrees(-85.0));
        let m87 = Angle::from(Degrees(-87.0));
        let m88 = Angle::from(Degrees(-88.0));
        let m180 = Angle::from(Degrees(-180.0));

        let south_pole_1 = to_sphere(m88, m180);
        let south_pole_2 = to_sphere(m87, zero);
        let zero_zero = to_sphere(zero, zero);
        let m85_zero = to_sphere(m85, zero);

        // A 300 Nm (5 degrees) long arc across the South Pole
        let line_1 = Arc::between_points(&south_pole_1, &south_pole_2);

        // A long line from the South Pole to the equator
        let line_2 = Arc::between_points(&m85_zero, &zero_zero);

        let result = DEG2RAD * 2.0;

        let gc_d = calculate_distance(&line_1, &line_2);
        // assert_eq!(result, gc_d.0);
        let delta_length = libm::fabs(result - gc_d.0);
        assert!(delta_length <= 1.0 * std::f64::EPSILON);

        let e_result = gc2e_distance(Radians(result));
        let sq_result = e_result * e_result;

        let sq_d = calculate_sq_distance(&line_1, &line_2);
        // assert_eq!(sq_result, sq_d);
        let delta_sq_length = libm::fabs(sq_result - sq_d);
        assert!(delta_sq_length <= 1.0 * std::f64::EPSILON);
    }
}
