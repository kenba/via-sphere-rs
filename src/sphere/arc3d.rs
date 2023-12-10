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

//! The arc3d module contains the Arc3d type and its associated functions.
//!
use super::great_circle3d::*;
use super::point3d::*;
use super::{e2gc_distance, gc2e_distance, LatLon};
use crate::trig::{cosine_from_sine, Angle, Radians, UnitNegRange};
use crate::{max, min, Validate};
use contracts::*;
use std::convert::From;

/// An arc of a Great Circle on a unit sphere in ECEF coordinates.
pub struct Arc3d {
    /// The start point of the arc.
    a: Point3d,
    /// The right hand pole of the Great Circle of the arc.
    pole: Point3d,
    /// The length of the arc.
    length: Radians,
    /// The half width of the arc.
    half_width: Radians,
}

impl Validate for Arc3d {
    /// Test whether an Arc3d is valid.  
    /// I.e. both a and pole are on the unit sphere and are orthogonal and
    /// both length and half_width are >= 0.0.
    fn is_valid(&self) -> bool {
        is_unit(&self.a)
            && is_unit(&self.pole)
            && are_orthogonal(&self.a, &self.pole)
            && (0.0 <= self.length.0)
            && (0.0 <= self.half_width.0)
    }
}

#[debug_invariant(self.is_valid())]
impl Arc3d {
    /// Construct an Arc3d
    /// * `a` - the start point of the arc.
    /// * `pole` - the right hand pole of the Great Circle of the arc.
    /// * `length` - the length of the arc.
    /// * `half_width` - the half width of the arc.
    pub fn new(a: Point3d, pole: Point3d, length: Radians, half_width: Radians) -> Arc3d {
        Arc3d {
            a,
            pole,
            length,
            half_width,
        }
    }

    /// Construct an Arc3d from the start and end points.  
    /// Note: if the points are the same or antipodal, the pole will be invalid.
    /// * `a` - the start point of the arc.
    /// * `b` - the end point of the arc.
    /// * `half_width` - the half width of the arc.
    #[debug_requires(is_unit(a) && is_unit(b) && (a != b) && (0.0 <= half_width.0))]
    pub fn wide_between_points(a: &Point3d, b: &Point3d, half_width: Radians) -> Arc3d {
        const MAX_SQ_LENGTH: f64 = 4.0 * (1.0 - std::f64::EPSILON);

        let mut pole = Point3d::new(0.0, 0.0, 0.0);
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

        Arc3d::new(*a, pole, length, half_width)
    }

    /// Construct an Arc3d from the start and end points.  
    /// Note: if the points are the same or antipodal, the pole will be invalid.
    /// * `a` - the start point of the arc.
    /// * `b` - the end point of the arc.
    #[debug_requires(is_unit(a) && is_unit(b) && (a != b))]
    pub fn between_points(a: &Point3d, b: &Point3d) -> Arc3d {
        Arc3d::wide_between_points(a, b, Radians(0.0))
    }

    /// Construct an Arc3d
    /// * `lat` - the latitude at the start of the arc.
    /// * `lon` - the longitude at the start of the arc.
    /// * `azi` - the azimuth at the start of the arc.
    /// * `length` - the length of the arc.
    /// * `half_width` - the half width of the arc.
    #[debug_requires(lat.is_valid_latitude() && (0.0 <= length.0))]
    pub fn wide_from_lat_lon_azi_length(
        lat: Angle,
        lon: Angle,
        azi: Angle,
        length: Radians,
        half_width: Radians,
    ) -> Arc3d {
        Arc3d::new(
            to_sphere(lat, lon),
            calculate_pole(lat, lon, azi),
            length,
            half_width,
        )
    }

    /// Construct a wide Arc3d from the start and end positions.  
    /// Note: if the points are the same or antipodal, the pole will be invalid.
    /// * `lat1, lon1` - the latitude and longitude at the start of the arc.
    /// * `lat2, lon2` - the latitude and longitude at the end of the arc.
    /// * `half_width` - the half width of the arc.
    pub fn wide_between_positions(
        lat1: Angle,
        lon1: Angle,
        lat2: Angle,
        lon2: Angle,
        half_width: Radians,
    ) -> Arc3d {
        Arc3d::wide_between_points(&to_sphere(lat1, lon1), &to_sphere(lat2, lon2), half_width)
    }

    /// Construct an Arc3d from the start and end positions.  
    /// Note: if the points are the same or antipodal, the pole will be invalid.
    /// * `lat1, lon1` - the latitude and longitude at the start of the arc.
    /// * `lat2, lon2` - the latitude and longitude at the end of the arc.
    pub fn between_positions(lat1: Angle, lon1: Angle, lat2: Angle, lon2: Angle) -> Arc3d {
        Arc3d::wide_between_positions(lat1, lon1, lat2, lon2, Radians(0.0))
    }

    /// The start point of the arc.
    pub fn a(&self) -> Point3d {
        self.a
    }

    /// The right hand pole of the Great Circle at the start point of the arc.
    pub fn pole(&self) -> Point3d {
        self.pole
    }

    /// The length of the arc.
    pub fn length(&self) -> Radians {
        self.length
    }

    /// The half width of the arc.
    pub fn half_width(&self) -> Radians {
        self.half_width
    }

    /// The direction vector of the arc at the start point.
    #[debug_ensures(is_unit(&ret) && are_orthogonal(&ret, &self.a))]
    pub fn to_direction(&self) -> Point3d {
        direction(&self.a, &self.pole)
    }

    /// The position of a point at a Great Circle distance along the arc.
    #[debug_ensures(is_unit(&ret))]
    pub fn position(&self, distance: Radians) -> Point3d {
        position(&self.a, &self.to_direction(), Angle::from(distance))
    }

    /// The end point of the arc.
    pub fn to_b(&self) -> Point3d {
        self.position(self.length)
    }

    /// The position of a perpendicular point at distance from the arc.
    /// * `point` a point on the arc's great circle.
    /// * `distance` the perpendicular distance from the arc's great circle.
    ///
    /// returns the point at perpendicular distance from point.
    #[debug_requires(is_unit(point) && are_orthogonal(point, &self.pole))]
    pub fn perp_position(&self, point: &Point3d, distance: Radians) -> Point3d {
        position(point, &self.pole, Angle::from(distance))
    }

    /// The position of a point at angle from the arc start, at arc length.
    /// * `angle` the angle from the arc start.
    ///
    /// returns the point at angle from the arc start, at arc length.
    pub fn angle_position(&self, angle: Angle) -> Point3d {
        rotate_position(&self.a, &self.pole, angle, Angle::from(self.length))
    }

    /// The Arc at the end of an Arc, just the point if half_width is zero.
    /// @param at_b if true the arc at b, else the arc at a.
    ///
    /// @return the end arc at a or b.
    pub fn end_arc(&self, at_b: bool) -> Arc3d {
        let p = if at_b { self.to_b() } else { self.a };
        let pole = direction(&p, &self.pole);
        if std::f64::EPSILON < self.half_width.0 {
            let a = self.perp_position(&p, self.half_width);
            Arc3d::new(a, pole, self.half_width + self.half_width, Radians(0.0))
        } else {
            Arc3d::new(p, pole, Radians(0.0), Radians(0.0))
        }
    }

    /// The azimuth at the start point.
    pub fn azimuth(&self) -> Angle {
        calculate_azimuth(&self.a, &self.pole)
    }

    /// Calculate Great Circle along and across track distances of point from
    /// the Arc3d.
    /// * `point` - the point.
    ///
    /// returns the along and across track distances of the point in Radians.
    #[debug_requires(is_unit(point))]
    pub fn calculate_atd_and_xtd(&self, point: &Point3d) -> (Radians, Radians) {
        calculate_atd_and_xtd(&self.a, &self.pole(), point)
    }

    /// Whether a point is alongside the arc.
    /// Note: Does NOT include the ends of the arc.
    /// * `point` - the point.
    ///
    /// returns true if the point is alongside the arc, false otherwise.
    #[debug_requires(is_unit(point))]
    pub fn is_alongside(&self, point: &Point3d) -> bool {
        let pole_p = self.pole.cross(point);
        let b = self.to_b();

        // Note: only testing sign, so pole_p doesn't need to be normalised
        // true if a is behind p and b is ahead of p
        (pole_p.dot(&self.a) < 0.0) && (pole_p.dot(&b) > 0.0)
    }

    /// The square of the shortest Euclidean distance of a point from the arc.
    /// * `point` - the point.
    ///
    /// returns square of the shortest distance of a point from the arc.
    #[debug_requires(is_unit(point))]
    pub fn sq_shortest_distance(&self, point: &Point3d) -> f64 {
        // if point abeam arc then cross track distance is closest
        if self.is_alongside(point) {
            sq_cross_track_distance(&self.pole, point)
        } else {
            // otherwise one of the ends is closest
            let b = &self.to_b();
            min(sq_distance(b, point), sq_distance(&self.a, point))
        }
    }

    /// The shortest Great Circle distance of a point from the arc.
    /// * `point` - the point.
    ///
    /// returns the shortest distance of a point from the arc, in Radians.
    #[debug_requires(is_unit(point))]
    pub fn shortest_distance(&self, point: &Point3d) -> Radians {
        let (atd, xtd) = calculate_atd_and_xtd(&self.a, &self.pole, point);
        let abs_xtd = libm::fabs(xtd.0);

        // if point abeam arc then cross track distance is closest
        if (Radians(0.0) <= atd) && (atd <= self.length) {
            return Radians(abs_xtd);
        }

        // If the point is closest to the end of the arc
        let half_length_minus_pi = Radians(0.5 * self.length.0 - std::f64::consts::PI);
        let by_end = (self.length < atd) || (atd < half_length_minus_pi);

        // If the point is not on the same Great Circle as the arc
        if 0.0 < abs_xtd {
            if by_end {
                let b = &self.to_b();
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
                    Radians(2.0 * std::f64::consts::PI + (atd.0 - self.length.0))
                }
            } else {
                -atd
            }
        }
    }

    /// The angle from the start of the Arc3d to a Point3d.
    /// * `point` - the point.
    ///
    /// return the angle from the start of the Arc3d to the point.
    #[debug_requires(is_unit(point))]
    pub fn start_angle(&self, point: &Point3d) -> Angle {
        let d = sq_distance(&self.a, point);
        if SQ_EPSILON < d {
            let pole_p = self.a.cross(point).normalize();
            let cos_angle = UnitNegRange::clamp(self.pole.dot(&pole_p));
            let sin_angle = cosine_from_sine(cos_angle, -self.pole.dot(point));
            Angle::new(sin_angle, cos_angle)
        } else {
            Angle::new(UnitNegRange(0.0), UnitNegRange(1.0))
        }
    }

    /// The turn angle from an Arc3d to the next Arc3d, +ve clockwise.
    /// * `other` - the next arc.
    ///
    /// return the turn angle from the end of the Arc3d to the next Arc3d.
    pub fn turn_angle(&self, other: &Arc3d) -> Angle {
        let cos_angle = UnitNegRange::clamp(self.pole.dot(&other.pole));
        let sin_angle = cosine_from_sine(cos_angle, -other.pole.dot(&self.a));
        Angle::new(sin_angle, cos_angle)
    }

    /// Calculate the winding number of a point against the Arc as the edge of a polygon.
    #[debug_requires(is_unit(point))]
    pub fn winding_number(&self, point: &Point3d) -> i32 {
        winding_number(&self.a, &self.pole, &self.to_b(), point)
    }
}

impl From<&[Point3d]> for Arc3d {
    #[debug_requires(!values.is_empty())]
    fn from(values: &[Point3d]) -> Self {
        let a: &Point3d = values.first().unwrap();
        let b: &Point3d = values.last().unwrap();
        Self::between_points(a, b)
    }
}

impl From<&[LatLon]> for Arc3d {
    #[debug_requires(!values.is_empty())]
    fn from(values: &[LatLon]) -> Self {
        let a: &LatLon = values.first().unwrap();
        let b: &LatLon = values.last().unwrap();
        Self::between_positions(a.lat(), a.lon(), b.lat(), b.lon())
    }
}

/// Calculate the lengths along a pair of Arcs on different Great Circles
/// to an intersection point.
/// * `a1`, `a2` the start points of the great circle arcs
/// * `pole1`, `pole2` the poles of the great circle arcs
/// * `c` the intersection point
///
/// returns distances along the arcs to the intersection point in Radians.
#[debug_requires(is_unit(c))]
#[debug_ensures(ret.0.is_valid() && ret.1.is_valid())]
pub fn calculate_different_gc_intersection_lengths(
    a1: &Point3d,
    pole1: &Point3d,
    a2: &Point3d,
    pole2: &Point3d,
    c: &Point3d,
) -> (Radians, Radians) {
    let sq_d_a1c = sq_distance(a1, c);
    let gc_d_a1c = if sq_d_a1c < 2.0 * SQ_EPSILON {
        0.0
    } else {
        libm::copysign(
            e2gc_distance(libm::sqrt(sq_d_a1c)).0,
            sin_atd(a1, pole1, c).0,
        )
    };

    let sq_d_a2c = sq_distance(a2, c);
    let gc_d_a2c = if sq_d_a2c < 2.0 * SQ_EPSILON {
        0.0
    } else {
        libm::copysign(
            e2gc_distance(libm::sqrt(sq_d_a2c)).0,
            sin_atd(a2, pole2, c).0,
        )
    };

    (Radians(gc_d_a1c), Radians(gc_d_a2c))
}

// Whether an intersection point is within an arc
pub fn is_within(ref_length: f64, arc_length: f64) -> bool {
    (-std::f64::EPSILON <= ref_length)
        && (ref_length <= arc_length + (std::f64::EPSILON * (1.0 + arc_length)))
}

/// Determine whether the other intersection point is closer to the start
/// of both arcs.
/// * `ref1_length`, `ref2_length` the intersection lengths.
/// * `arc1_length`, `arc2_length` the arc lengths.
/// @return true if the other intersection point is closer to the arc starts,
/// false otherwise.
#[debug_requires(ref1_length.is_valid() && ref2_length.is_valid()
              && arc1_length.is_valid() && arc2_length.is_valid())]
pub fn use_other_intersection(
    ref1_length: Radians,
    ref2_length: Radians,
    arc1_length: Radians,
    arc2_length: Radians,
) -> bool {
    // is the intersection within both arcs?
    let c_within_arc1 = is_within(ref1_length.0, arc1_length.0);
    let c_within_arc2 = is_within(ref2_length.0, arc2_length.0);
    if c_within_arc1 && c_within_arc2 {
        return false;
    }

    // Calculate lengths from the other intersection point
    let oth1_length = ref1_length + Radians(std::f64::consts::PI);
    let oth2_length = ref2_length + Radians(std::f64::consts::PI);

    // is the other intersection within both arcs?
    let d_within_arc1 = is_within(oth1_length.0, arc1_length.0);
    let d_within_arc2 = is_within(oth2_length.0, arc2_length.0);
    if d_within_arc1 && d_within_arc2 {
        return true;
    }

    // if either intersection is within an arc
    let c_within_arc = c_within_arc1 || c_within_arc2;
    let d_within_arc = d_within_arc1 || d_within_arc2;
    if c_within_arc != d_within_arc {
        // whichever intersection is within an arc is closest
        d_within_arc
    } else {
        // calculate the minimum length from a start point to an intersection
        fn min_length(ref_length: Radians, within_arc: bool) -> f64 {
            if within_arc {
                0.0
            } else {
                libm::fabs(ref_length.0)
            }
        }

        // either both intersections are within an arc or neither are
        let min_c1 = min_length(ref1_length, c_within_arc1);
        let min_c2 = min_length(ref2_length, c_within_arc2);
        let max_c = max(min_c1, min_c2);
        let min_d1 = min_length(oth1_length, d_within_arc1);
        let min_d2 = min_length(oth2_length, d_within_arc2);
        let max_d = max(min_d1, min_d2);

        // use the intersection that is closest to the start of both arcs
        max_d < max_c
    }
}

/// Calculate the closest lengths along a pair of Arcs on different Great Circles
/// to an intersection point.
/// * `arc1`, `arc2` the great circle arcs
/// * `point` the intersection point
///
/// returns distances along the arcs to the intersection point in Radians.
#[debug_requires(is_unit(point))]
#[debug_ensures(ret.0.is_valid() && ret.1.is_valid())]
fn calculate_different_gc_closest_intersection_lengths(
    arc1: &Arc3d,
    arc2: &Arc3d,
    point: &Point3d,
) -> (Radians, Radians) {
    let (arc1_a_c, arc2_a_c) = calculate_different_gc_intersection_lengths(
        &arc1.a(),
        &arc1.pole(),
        &arc2.a(),
        &arc2.pole(),
        point,
    );

    let use_antipodal_intersection =
        use_other_intersection(arc1_a_c, arc2_a_c, arc1.length(), arc2.length());
    if use_antipodal_intersection {
        calculate_different_gc_intersection_lengths(
            &arc1.a(),
            &arc1.pole(),
            &arc2.a(),
            &arc2.pole(),
            &-(*point),
        )
    } else {
        (arc1_a_c, arc2_a_c)
    }
}

/// Calculate the lengths along a pair of Arcs on the same (or reciprocal)
/// Great Circles to their closest (reference) points.
/// * `reciprocal` whether the arcs are in reciprocal directions.
/// * `a2_ahead` whether the start of arc2 is ahead of the start of arc1.
/// * `arc1_length`, `arc2_length` the arc lengths.
/// * `gc_d` the great circle distance between the arc start points.
#[debug_ensures(ret.0.is_valid() && ret.1.is_valid())]
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
pub fn calculate_same_gc_reference_lengths(
    a1: &Point3d,
    pole1: &Point3d,
    length1: Radians,
    a2: &Point3d,
    pole2: &Point3d,
    length2: Radians,
    gc_d: Radians,
) -> (Radians, Radians) {
    let reciprocal = pole1.dot(pole2) < 0.0;
    let a2_ahead = 0.0 < sin_atd(a1, pole1, a2).0;
    calc_same_gc_reference_lengths(reciprocal, a2_ahead, length1, length2, gc_d)
}

/// Calculate the lengths along a pair of Arcs on the same (or reciprocal)
/// Great Circles to their closest intersection or reference points.
/// * `arc1`, `arc2` the arcs.
#[debug_ensures(ret.0.is_valid() && ret.1.is_valid())]
pub fn calculate_reference_lengths(arc1: &Arc3d, arc2: &Arc3d) -> (Radians, Radians) {
    // Calculate the great circle distance between the start points.
    let gc_d = gc_distance(&arc1.a, &arc2.a);
    if 2.0 * std::f64::EPSILON < gc_d.0 {
        // Calculate the intersection.
        let c = arc1.pole.cross(&arc2.pole);
        let length = c.norm();
        if length < MIN_LENGTH {
            calculate_same_gc_reference_lengths(
                &arc1.a,
                &arc1.pole,
                arc1.length(),
                &arc2.a,
                &arc2.pole,
                arc2.length(),
                gc_d,
            )
        } else {
            calculate_different_gc_closest_intersection_lengths(arc1, arc2, &c.normalize())
        }
    } else {
        (Radians(0.0), Radians(0.0))
    }
}

/// Calculate whether a pair of Arcs intersect and (if so) where.
/// * `arc1`, `arc2` the arcs.
///
/// returns the distance along the first arc to the second arc or -1 if they
/// don't intersect.
pub fn calculate_intersection_length(arc1: &Arc3d, arc2: &Arc3d) -> Radians {
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
fn calculate_different_gc_sq_distance(arc1: &Arc3d, arc2: &Arc3d, c: &Point3d) -> f64 {
    let (ref1_length, ref2_length) =
        calculate_different_gc_closest_intersection_lengths(arc1, arc2, c);

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
            arc1.sq_shortest_distance(&arc2.to_b()),
        );
        let min_2 = min(
            arc2.sq_shortest_distance(&arc1.a),
            arc2.sq_shortest_distance(&arc1.to_b()),
        );
        min(min_1, min_2)
    }
}

/// Calculate the distance between two arcs on different great circles.
/// * `arc1`, `arc2` the arcs
/// * `c` the intersection point.
///
/// returns the distance between the arcs in radians.
fn calculate_different_gc_distance(arc1: &Arc3d, arc2: &Arc3d, c: &Point3d) -> Radians {
    let (ref1_length, ref2_length) =
        calculate_different_gc_closest_intersection_lengths(arc1, arc2, c);

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
            arc1.shortest_distance(&arc2.to_b()).0,
        );
        let min_2 = min(
            arc2.shortest_distance(&arc1.a).0,
            arc2.shortest_distance(&arc1.to_b()).0,
        );
        Radians(min(min_1, min_2))
    }
}

/// Calculate the distance between two arcs on the same great circles.
/// * `reciprocal` whether the arcs are in reciprocal directions.
/// * `a2_ahead` whether the start of arc2 is ahead of the start of arc1.
/// * `arc1_length`, `arc2_length` the arc lengths.
/// * `gc_d` the great circle distance between the arc start points.
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
pub fn calculate_same_gc_distance(
    a1: &Point3d,
    pole1: &Point3d,
    length1: Radians,
    a2: &Point3d,
    pole2: &Point3d,
    length2: Radians,
    gc_d: Radians,
) -> Radians {
    let reciprocal = pole1.dot(pole2) < 0.0;
    let a2_ahead = 0.0 < sin_atd(a1, pole1, a2).0;
    calc_same_gc_distance(reciprocal, a2_ahead, length1, length2, gc_d)
}

/// Calculate the square of the minimum Euclidean distance between two valid Arc3ds.
/// * `arc1`, `arc2` the arcs.
///
/// return the square of the minimum Euclidean distance between the arcs.
pub fn calculate_sq_distance(arc1: &Arc3d, arc2: &Arc3d) -> f64 {
    // Calculate distance between start points
    let sq_d = sq_distance(&arc1.a, &arc2.a);
    if sq_d < SQ_EPSILON {
        0.0
    } else {
        // Calculate the intersection.
        let c = arc1.pole.cross(&arc2.pole);
        if c.norm() < MIN_NORM {
            let gc_d = e2gc_distance(libm::sqrt(sq_d));
            let min_d = gc2e_distance(calculate_same_gc_distance(
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

/// Calculate the shortest Great Circle distance between two Arc3ds.
/// * `arc1`, `arc2` the arcs.
///
/// return the shortest Great Circle distance between the arcs in Radians.
pub fn calculate_distance(arc1: &Arc3d, arc2: &Arc3d) -> Radians {
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
    use crate::sphere::arc3d::*;
    use crate::sphere::gc2e_distance;
    use crate::trig::{Angle, Degrees, DEG2RAD};

    #[test]
    fn test_arc3d_calculate_atd_and_xtd() {
        let zero = Angle::default();
        let one_degree = Angle::from(Degrees(1.0));
        let degree_179 = Angle::from(Degrees(179.0));

        // Arc on the Equator
        let arc = Arc3d::between_positions(zero, zero, zero, degree_179);

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
        let arc = Arc3d::between_positions(zero, degree_m135, zero, degree_179);

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

        let line_1 = Arc3d::between_points(&minus_one_one, &one_one);
        let line_2 = Arc3d::between_points(&minus_one_one, &one_minus_one);

        let intersection_lengths = calculate_reference_lengths(&line_1, &line_2);
        assert_eq!(Radians(0.0), intersection_lengths.0);
        assert_eq!(Radians(0.0), intersection_lengths.1);

        let intersection_length = calculate_intersection_length(&line_1, &line_2);
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

        let line_1 = Arc3d::between_points(&minus_one_one, &zero_zero);
        let line_2 = Arc3d::between_points(&minus_one_minus_one, &zero_zero);

        let intersection_lengths = calculate_reference_lengths(&line_1, &line_2);
        assert_eq!(line_1.length(), intersection_lengths.0);
        assert_eq!(line_2.length(), intersection_lengths.1);

        let intersection_length = calculate_intersection_length(&line_1, &line_2);
        assert_eq!(line_1.length(), intersection_length);
    }

    #[test]
    fn test_closest_intersection_lengths_common_start_end_points() {
        let one = Angle::from(Degrees(1.0));
        let m_one = -one;

        let one_one = to_sphere(one, one);
        let minus_one_one = to_sphere(m_one, one);
        let one_minus_one = to_sphere(one, m_one);

        let line_1 = Arc3d::between_points(&minus_one_one, &one_one);
        let line_2 = Arc3d::between_points(&one_minus_one, &minus_one_one);

        let intersection_lengths = calculate_reference_lengths(&line_1, &line_2);
        assert_eq!(Radians(0.0), intersection_lengths.0);
        // assert_eq!(line_2.length(), intersection_lengths.1);
        let delta_length = libm::fabs(line_2.length().0 - (intersection_lengths.1).0);
        assert!(delta_length <= std::f64::EPSILON);

        let intersection_length = calculate_intersection_length(&line_1, &line_2);
        assert_eq!(Radians(0.0), intersection_length);
    }

    #[test]
    fn test_closest_intersection_lengths_common_end_start_points() {
        let one = Angle::from(Degrees(1.0));
        let m_one = -one;

        let one_one = to_sphere(one, one);
        let minus_one_one = to_sphere(m_one, one);
        let one_minus_one = to_sphere(one, m_one);

        let line_1 = Arc3d::between_points(&minus_one_one, &one_one);
        let line_2 = Arc3d::between_points(&one_one, &one_minus_one);

        let intersection_lengths = calculate_reference_lengths(&line_1, &line_2);
        // assert_eq!(sq_length_1, intersection_lengths.0);
        let delta_length = libm::fabs(line_1.length().0 - (intersection_lengths.0).0);
        assert!(delta_length <= std::f64::EPSILON);
        assert_eq!(Radians(0.0), intersection_lengths.1);

        let intersection_length = calculate_intersection_length(&line_1, &line_2);
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

        let line_1 = Arc3d::between_points(&minus_one_minus_one, &one_one);
        let line_2 = Arc3d::between_points(&minus_one_one, &one_minus_one);

        let gc_result = gc_distance(&minus_one_minus_one, &zero_zero);
        let intersection_lengths = calculate_reference_lengths(&line_1, &line_2);
        assert_eq!(gc_result, intersection_lengths.0);
        assert_eq!(gc_result, intersection_lengths.1);

        let intersection_length = calculate_intersection_length(&line_1, &line_2);
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

        let line_1 = Arc3d::between_points(&one_one, &one_ninety_two);
        let line_2 = Arc3d::between_points(&minus_two_eighty_seven, &minus_two_idl_minus_two);

        let intersection_lengths = calculate_reference_lengths(&line_1, &line_2);
        assert_eq!(0.3169366019437673, intersection_lengths.0 .0);
        assert_eq!(-1.1851988424812585, intersection_lengths.1 .0);

        let intersection_length = calculate_intersection_length(&line_1, &line_2);
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

        let line_1 = Arc3d::between_points(&one_one, &one_idl_one);
        let line_2 = Arc3d::between_points(&minus_two_minus_two, &minus_two_idl_minus_one);

        let intersection_lengths = calculate_reference_lengths(&line_1, &line_2);
        assert_eq!(3.067628989963839, intersection_lengths.0 .0);
        assert_eq!(3.141513106563121, intersection_lengths.1 .0);

        let intersection_length = calculate_intersection_length(&line_1, &line_2);
        assert_eq!(-1.0, intersection_length.0);
    }

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
    fn test_closest_intersection_lengths_same_gc_0() {
        let zero = Angle::default();
        let m87 = Angle::from(Degrees(-87.0));
        let m88 = Angle::from(Degrees(-88.0));
        let m180 = Angle::from(Degrees(-180.0));

        let south_pole_1 = to_sphere(m88, m180);
        let south_pole_2 = to_sphere(m87, zero);

        // A 300 Nm (5 degrees) long arc across the South Pole
        let line_1 = Arc3d::between_points(&south_pole_1, &south_pole_2);

        let intersection_lengths = calculate_reference_lengths(&line_1, &line_1);
        assert_eq!(Radians(0.0), intersection_lengths.0);
        assert_eq!(Radians(0.0), intersection_lengths.1);

        let gc_length = calculate_intersection_length(&line_1, &line_1);
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
        let line_1 = Arc3d::between_points(&south_pole_1, &south_pole_2);

        // A long line to the South Pole from the equator
        let line_2 = Arc3d::between_points(&zero_zero, &m85_zero);

        let intersection_lengths = calculate_reference_lengths(&line_1, &line_2);
        assert_eq!(Radians(0.12217304763960288), intersection_lengths.0);
        assert_eq!(Radians(1.4835298641951802), intersection_lengths.1);

        let gc_length = calculate_intersection_length(&line_1, &line_2);
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
        let line_1 = Arc3d::between_points(&south_pole_1, &south_pole_2);

        // A long line from the South Pole to the equator
        let line_2 = Arc3d::between_points(&m85_zero, &zero_zero);

        let intersection_lengths = calculate_reference_lengths(&line_1, &line_2);
        assert_eq!(Radians(0.12217304763960306), intersection_lengths.0);
        assert_eq!(Radians(0.0), intersection_lengths.1);

        let gc_length = calculate_intersection_length(&line_1, &line_2);
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
        let arc_1 = Arc3d::between_points(&zero_minus_one, &zero_one);
        assert_eq!(1, arc_1.winding_number(&one_zero));
        assert_eq!(0, arc_1.winding_number(&zero_zero));
        assert_eq!(0, arc_1.winding_number(&minus_one_zero));

        // Arc from East to West
        let arc_m1 = Arc3d::between_points(&zero_one, &zero_minus_one);
        assert_eq!(-1, arc_m1.winding_number(&one_zero));
        assert_eq!(0, arc_m1.winding_number(&zero_zero));
        assert_eq!(0, arc_m1.winding_number(&minus_one_zero));
    }

    #[test]
    fn test_arc_arc_distance_functions_common_start_point() {
        let one = Angle::from(Degrees(1.0));
        let m_one = -one;

        let one_one = to_sphere(one, one);
        let minus_one_one = to_sphere(m_one, one);
        let one_minus_one = to_sphere(one, m_one);

        let line_1 = Arc3d::between_points(&minus_one_one, &one_one);
        let line_2 = Arc3d::between_points(&minus_one_one, &one_minus_one);

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

        let line_1 = Arc3d::between_points(&minus_one_one, &zero_zero);
        let line_2 = Arc3d::between_points(&minus_one_minus_one, &zero_zero);

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

        let line_1 = Arc3d::between_points(&minus_one_one, &one_one);
        let line_2 = Arc3d::between_points(&one_minus_one, &minus_one_one);

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

        let line_1 = Arc3d::between_points(&minus_one_one, &one_one);
        let line_2 = Arc3d::between_points(&one_one, &one_minus_one);

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

        let line_1 = Arc3d::between_points(&minus_one_minus_one, &one_one);
        let line_2 = Arc3d::between_points(&minus_one_one, &one_minus_one);

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

        let line_1 = Arc3d::between_points(&minus_one_minus_one, &one_one);
        let line_2 = Arc3d::between_points(&minus_one_one, &one_minus_one);

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

        let line_1 = Arc3d::between_points(&one_one, &one_ninety_two);
        let line_2 = Arc3d::between_points(&minus_two_eighty_seven, &minus_two_idl_minus_two);

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

        let line_1 = Arc3d::between_points(&one_one, &one_idl_one);
        let line_2 = Arc3d::between_points(&minus_two_minus_two, &minus_two_idl_minus_one);

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

        let result4 = calc_same_gc_distance(false, false, length1, length2, Radians(1.0));
        assert_eq!(length1, result4);
        let result5 = calc_same_gc_distance(false, true, length1, length2, Radians(1.0));
        assert_eq!(length2, result5);

        let result6 = calc_same_gc_distance(false, false, length1, length2, length2);
        assert_eq!(zero, result6);

        let result7 = calc_same_gc_distance(false, true, length1, length2, length1);
        assert_eq!(zero, result7);
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
        let line_1 = Arc3d::between_points(&south_pole_1, &south_pole_2);

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
        let line_1 = Arc3d::between_points(&south_pole_1, &south_pole_2);

        // A long line to the South Pole from the equator
        let line_2 = Arc3d::between_points(&zero_zero, &m85_zero);

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
        let line_1 = Arc3d::between_points(&south_pole_1, &south_pole_2);

        // A long line from the South Pole to the equator
        let line_2 = Arc3d::between_points(&m85_zero, &zero_zero);

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
