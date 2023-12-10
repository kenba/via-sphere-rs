// Copyright (c) 2020-2022 Via Technology Ltd. All Rights Reserved.

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

//! The arc3dstring module contains the Arc3dString type and its associated functions.

use super::arc3d::*;
use super::great_circle3d::*;
use super::point3d::*;
use super::*;
use crate::trig::{Angle, Radians};
use contracts::*;
use std::convert::From;

/// An ordered collection of points and the poles (great circle arcs)
/// between them. It corresponds to an OGC/GeoJSON LineString.  
/// If there are enough points (more than 3) and the last point is the
/// same as the first point, then it corresponds to an OGC/GeoJSON LineLoop,
/// i.e. a mathematical polygon. In which case the is_within function can
/// be called to determine whether a point is within the polygon.
#[derive(Clone, Debug, PartialEq)]
pub struct Arc3dString {
    /// The points.
    points: Vec<Point3d>,
    /// The poles of the great circle arcs.
    poles: Vec<Point3d>,
    /// The lengths of the arcs, in radians.
    arc_lengths: Vec<Radians>,
}

impl Validate for Arc3dString {
    /// Test whether an Arc3dString is valid, it must have more than one point.
    fn is_valid(&self) -> bool {
        let count = self.points.len();
        1 < count
            && self.poles.len() == (count - 1)
            && self.arc_lengths.len() == count
            && self.arc_lengths[0] == Radians(0.0)
    }
}

#[debug_invariant(self.is_valid())]
impl Arc3dString {
    /// Construct an Arc3dString
    /// * `points` - the points.
    /// * `poles` - the poles of the great circle arcs.
    /// * `arc_lengths` - the lengths of the arcs, in radians.
    pub fn new(points: &[Point3d], poles: &[Point3d], arc_lengths: &[Radians]) -> Self {
        Self {
            points: points.to_vec(),
            poles: poles.to_vec(),
            arc_lengths: arc_lengths.to_vec(),
        }
    }

    /// The number of points.
    pub fn count(&self) -> usize {
        self.points.len()
    }

    /// Accessor for the points.
    pub fn points<'a>(&'a self) -> &[Point3d] {
        &self.points
    }

    /// Accessor for the great circle poles.
    pub fn poles<'a>(&'a self) -> &[Point3d] {
        &self.poles
    }

    /// Accessor for the arc lengths, in radians.
    pub fn lengths<'a>(&'a self) -> &[Radians] {
        &self.arc_lengths
    }
    /// The points in reverse order.
    pub fn reverse(&self) -> Vec<Point3d> {
        let mut points: Vec<Point3d> = self.points.to_vec();
        points.reverse();

        points
    }

    /// Get the arc at the given index.
    #[debug_requires(index < self.poles.len())]
    pub fn arc(&self, index: usize) -> Arc3d {
        Arc3d::new(
            self.points[index],
            self.poles[index],
            self.arc_lengths[index + 1],
            Radians(0.0),
        )
    }

    /// Determine whether the Arc3dString forms a closed loop,
    /// i.e. is a polygon.
    ///
    /// returns true if it is a closed loop, false otherwise.
    pub fn is_closed_loop(&self) -> bool {
        (3 < self.count()) && (self.points.first() == self.points.last())
    }

    /// Calculate the shortest distances between a point and the Arc3dString.
    /// * `point` the point
    ///
    /// returns the shortest distances between the point and each line of the
    /// Arc3dString.
    pub fn shortest_distances(&self, point: &Point3d) -> Vec<Radians> {
        let size = self.count() - 1;

        let mut distances: Vec<Radians> = vec![Radians(0.0); size];
        for (index, distance_itr) in distances.iter_mut().enumerate().take(size) {
            *distance_itr = calculate_shortest_distance(
                &self.points[index],
                &self.poles[index],
                self.arc_lengths[index + 1],
                point,
            );
        }

        distances
    }

    /// Find the index of the closest line on the Arc3dString to a point.
    /// * `point` the point
    ///
    /// returns the index of the closest line.
    #[debug_ensures(ret < self.count() - 1)]
    pub fn closest_line_index(&self, point: &Point3d) -> usize {
        let distances = self.shortest_distances(point);

        // f64 doesn't support the Ord trait, so this won't work
        // let min_index = distances.iter().enumerate().min();
        let mut min_index = 0;
        for (i, &value) in distances.iter().enumerate() {
            if value < distances[min_index] {
                min_index = i;
            }
        }

        min_index
    }

    /// Whether a point is abeam an arc.
    /// * `index` the arc index
    /// * `point` the point
    ///
    /// returns true if the point is abeam the arc, false otherwise.
    #[debug_requires(index < self.poles.len())]
    pub fn is_abeam(&self, index: usize, point: &Point3d) -> bool {
        let atd = along_track_distance(&self.points[index], &self.poles[index], point);

        (0.0 <= atd.0) && (atd <= self.lengths()[index + 1])
    }

    /// The along and across track distances of a point relative to the arc at index.
    /// * `index` the arc index
    /// * `point` the point
    ///
    /// returns the along and across track distances of the point relative to
    /// the arc at index.
    #[debug_requires(index < self.poles.len())]
    pub fn line_atd_xtd(&self, index: usize, point: &Point3d) -> (Radians, Radians) {
        calculate_atd_and_xtd(&self.points[index], &self.poles[index], point)
    }

    /// Calculate the index and ratio of a point relative to the Arc string.
    /// * `point` the point
    ///
    /// returns the index of the closest arc in the Arc string and the ratio of
    /// the point relative to the arc.
    pub fn find_index_and_ratio(&self, point: &Point3d) -> (usize, f64) {
        let mut index = self.closest_line_index(point);

        let (atd, _xtd) = self.line_atd_xtd(index, point);
        let mut ratio = atd.0 / self.lengths()[index + 1].0;

        if 1.0 <= ratio {
            ratio = 0.0;
            index += 1;
        }

        (index, ratio)
    }

    /// Calculate the distances where an arc intersects the Arc3dString.
    /// * `arc` the arc
    ///
    /// returns the distances where the arc intersects the edges of the
    /// Arc3dString. An empty vector if there are no intersections.
    pub fn intersection_lengths(&self, arc: &Arc3d) -> Vec<Radians> {
        let size = self.count() - 1;

        let mut lengths: Vec<Radians> = vec![Radians(0.0); size];
        for (index, length_itr) in lengths.iter_mut().enumerate().take(size) {
            *length_itr = calculate_intersection_length(arc, &self.arc(index));
        }

        // Remove all values less than zero
        lengths.retain(|&x| Radians(0.0) <= x);

        lengths
    }

    /// Determine whether point is within a closed loop Arc3dString.  
    /// It determines whether point is within a polygon by summing winding
    /// numbers, see http://geomalgorithms.com/a03-_inclusion.html
    /// * `point` the point
    ///
    /// returns true if point is within the closed loop Arc3dString, false otherwise.
    #[debug_requires(self.is_closed_loop() && is_unit(point))]
    pub fn is_within(&self, point: &Point3d) -> bool {
        let size = self.count() - 1;

        let mut values: Vec<i32> = vec![0; size];
        for (index, value_itr) in values.iter_mut().enumerate().take(size) {
            *value_itr = self.arc(index).winding_number(point);
        }

        0 != values.iter().sum::<i32>()
    }

    /// Calculate the position of the point at index and ratio.
    /// * `index` the index of the arc along the ArcString.
    /// * `ratio` the ratio of the point along the arc.
    ///
    /// returns the point at index and ratio along the ArcString.
    #[debug_requires((index < self.poles.len()) && (0.0..1.0).contains(&ratio))]
    pub fn point_at(&self, index: usize, ratio: f64) -> Point3d {
        if 0.0 < ratio {
            let distance = Angle::from(Radians(ratio * self.arc_lengths[index + 1].0));
            position(
                &self.points[index],
                &direction(&self.points[index], &self.poles[index]),
                distance,
            )
        } else {
            self.points[index]
        }
    }

    /// Get the points between start index and ratio and finish index and ratio.
    /// * `start_index`, `finish_index` the indicies of the poles.
    /// * `start_ratio`, `finish_ratio` the ratios along the poles.
    ///
    /// returns the points in the subsection between start_index - start_ratio
    /// and finish_index - finish_ratio.
    #[debug_requires((finish_index < self.poles.len()) && (start_index <= finish_index) &&
    ((start_index as f64) + start_ratio) < ((finish_index as f64) + finish_ratio))]
    pub fn subsection_points(
        &self,
        start_index: usize,
        start_ratio: f64,
        finish_index: usize,
        finish_ratio: f64,
    ) -> Vec<Point3d> {
        let mut points: Vec<Point3d> = [self.point_at(start_index, start_ratio)].to_vec();

        for index in start_index..finish_index {
            points.push(self.points[index + 1]);
        }

        if 0.0 < finish_ratio {
            points.push(self.point_at(finish_index, finish_ratio));
        }

        points
    }
}

impl From<Vec<Point3d>> for Arc3dString {
    #[debug_requires(!points.is_empty())]
    fn from(points: Vec<Point3d>) -> Self {
        let mut poles: Vec<Point3d> = Vec::with_capacity(points.len() - 1);
        let mut arc_lengths: Vec<Radians> = vec![Radians(0.0); points.len()];

        let mut prev = points[0];
        for index in 1..points.len() {
            let next = points[index];

            let mut pole = prev.cross(&next);
            // If the pole is not long enough to normalize,
            // break out to invalidate the Arc3dString
            if pole.norm() < MIN_LENGTH {
                break;
            }
            pole.normalize_mut();
            poles.push(pole);

            arc_lengths[index] = gc_distance(&prev, &next);

            prev = next;
        }

        Self {
            points,
            poles,
            arc_lengths,
        }
    }
}

impl From<&[LatLon]> for Arc3dString {
    #[debug_requires(!values.is_empty())]
    fn from(values: &[LatLon]) -> Self {
        Self::from(from_slice(values))
    }
}

impl From<&LatLons> for Arc3dString {
    #[debug_requires(!values.0.is_empty())]
    fn from(values: &LatLons) -> Self {
        Self::from(Vec::<Point3d>::from(values))
    }
}

#[cfg(test)]
mod tests {
    use crate::sphere::arc3dstring::*;

    #[test]
    fn test_short_arc_strings() {
        let lats_1 = vec![45.0, 46.0];
        let lons_1 = vec![1.0, 1.0];
        let lat_lons = LatLons::try_from((lats_1.as_slice(), lons_1.as_slice())).unwrap();

        let arc_string = Arc3dString::from(&lat_lons);
        assert!(arc_string.is_valid());
        assert!(!arc_string.is_closed_loop());
        assert_eq!(arc_string.poles().len(), arc_string.count() - 1);

        assert_eq!(arc_string.lengths().len(), arc_string.count());
        assert_eq!(Radians(0.0), arc_string.lengths()[0]);
        assert!(Radians(0.0) < arc_string.lengths()[1]);
    }

    #[test]
    fn test_short_arc_string_1() {
        let lats = vec![44.0, 46.0, 46.0, 44.0];
        let lons = vec![1.0, 1.0, -1.0, -1.0];
        let lat_lons = LatLons::try_from((lats.as_slice(), lons.as_slice())).unwrap();

        let arc_string = Arc3dString::from(&lat_lons);
        assert!(arc_string.is_valid());
        assert!(!arc_string.is_closed_loop());
        assert_eq!(arc_string.poles().len(), arc_string.count() - 1);

        assert_eq!(arc_string.lengths().len(), arc_string.count());
        assert_eq!(Radians(0.0), arc_string.lengths()[0]);
        assert!(Radians(0.0) < arc_string.lengths()[lat_lons.0.len() - 1]);

        let points = arc_string.points();
        let reverse_points = arc_string.reverse();
        assert_eq!(points.first(), reverse_points.last());
        assert_eq!(points.last(), reverse_points.first());

        // Arc East of arc_string
        let pos_43_2 = to_sphere(Angle::from(Degrees(43.0)), Angle::from(Degrees(2.0)));
        let pos_47_2 = to_sphere(Angle::from(Degrees(47.0)), Angle::from(Degrees(2.0)));
        let arc_1 = Arc3d::between_points(&pos_43_2, &pos_47_2);
        assert!(arc_1.is_valid());

        let values_01 = arc_string.shortest_distances(&pos_43_2);
        assert_eq!(arc_string.poles().len(), values_01.len());

        let min_index_01 = arc_string.closest_line_index(&pos_43_2);
        assert_eq!(0, min_index_01);

        let values_1 = arc_string.intersection_lengths(&arc_1);
        assert_eq!(0, values_1.len());

        // Arc through centre of arc_string, note NOT a closed loop
        let pos_43_0 = to_sphere(Angle::from(Degrees(43.0)), Angle::default());
        let pos_47_0 = to_sphere(Angle::from(Degrees(47.0)), Angle::default());
        let arc_2 = Arc3d::between_points(&pos_43_0, &pos_47_0);
        assert!(arc_2.is_valid());

        let values_02 = arc_string.shortest_distances(&pos_43_0);
        assert_eq!(values_02.len(), 3);

        assert_eq!(0, arc_string.closest_line_index(&pos_43_0));
        assert!(!arc_string.is_abeam(0, &pos_43_0));
        assert!(arc_string.is_abeam(1, &pos_43_0));
        assert!(!arc_string.is_abeam(2, &pos_43_0));

        let (atd0, xtd0) = arc_string.line_atd_xtd(0, &pos_43_0);
        assert_eq!(-0.017377319412642603, atd0.0);
        assert_eq!(0.01276422865037213, xtd0.0);

        let values_03 = arc_string.shortest_distances(&pos_47_0);
        assert_eq!(3, values_03.len());

        assert_eq!(1, arc_string.closest_line_index(&pos_47_0));
        assert!(!arc_string.is_abeam(0, &pos_47_0));
        assert!(arc_string.is_abeam(1, &pos_47_0));
        assert!(!arc_string.is_abeam(2, &pos_47_0));

        let (atd1, xtd1) = arc_string.line_atd_xtd(1, &pos_47_0);
        assert_eq!(0.012123757216859347, atd1.0);
        assert_eq!(-0.017377180894479542, xtd1.0);

        let (index1, ratio1) = arc_string.find_index_and_ratio(&pos_47_0);
        assert_eq!(1, index1);
        assert_eq!(0.5000000000000001, ratio1);

        let values_2 = arc_string.intersection_lengths(&arc_2);
        assert_eq!(1, values_2.len());

        // Arc South of arc_string
        let pos_33_0 = to_sphere(Angle::from(Degrees(33.0)), Angle::default());
        let pos_37_0 = to_sphere(Angle::from(Degrees(37.0)), Angle::default());
        let arc_3 = Arc3d::between_points(&pos_33_0, &pos_37_0);
        assert!(arc_3.is_valid());

        let (index2, ratio2) = arc_string.find_index_and_ratio(&pos_33_0);
        assert_eq!(0, index2);
        assert_eq!(-5.498006790057533, ratio2);

        let values_3 = arc_string.intersection_lengths(&arc_3);
        assert_eq!(0, values_3.len());

        let points_1 = arc_string.subsection_points(0, 0.5, 2, 0.5);
        assert_eq!(arc_string.count(), points_1.len());
    }

    #[test]
    fn test_polygon1() {
        // A closed rectangle in Lat Long coords
        let rect_lats = vec![44.0, 46.0, 46.0, 44.0, 44.0];
        let rect_lons = vec![1.0, 1.0, -1.0, -1.0, 1.0];
        let lat_lons = LatLons::try_from((rect_lats.as_slice(), rect_lons.as_slice())).unwrap();

        let polygon = Arc3dString::from(&lat_lons);
        assert!(polygon.is_valid());
        assert!(polygon.is_closed_loop());
        assert_eq!(polygon.poles().len(), polygon.count() - 1);

        // Points on the rhumb line mid points
        let points_lats = vec![44.0, 46.0, 45.0, 45.0];
        let points_lons = vec![0.0, 0.0, 1.0, -1.0];
        let points_lat_lons =
            LatLons::try_from((points_lats.as_slice(), points_lons.as_slice())).unwrap();
        let points = Vec::<Point3d>::from(&points_lat_lons);

        // lower rhumb line is outside
        assert!(!polygon.is_within(&points[0]));
        // upper rhumb line is inside
        assert!(polygon.is_within(&points[1]));

        // right longitude is outside
        assert!(!polygon.is_within(&points[2]));
        // left longitude is inside
        assert!(polygon.is_within(&points[3]));

        // Arc East of Polygon
        let points1_lats = vec![43.0, 47.0];
        let points1_lons = vec![2.0, 2.0];
        let points1_lat_lons =
            LatLons::try_from((points1_lats.as_slice(), points1_lons.as_slice())).unwrap();
        let arc_1 = Arc3d::from(points1_lat_lons.0.as_slice());
        assert!(arc_1.is_valid());

        let values_1 = polygon.intersection_lengths(&arc_1);
        assert_eq!(0, values_1.len());

        // Arc through centre of Polygon
        let points2_lats = vec![43.0, 47.0];
        let points2_lons = vec![0.0, 0.0];
        let points2_lat_lons =
            LatLons::try_from((points2_lats.as_slice(), points2_lons.as_slice())).unwrap();
        let arc_2 = Arc3d::from(points2_lat_lons.0.as_slice());
        assert!(arc_2.is_valid());

        let values_2 = polygon.intersection_lengths(&arc_2);
        assert_eq!(2, values_2.len());

        // Arc South of Polygon
        let points3_lats = vec![33.0, 37.0];
        let points3_lons = vec![0.0, 0.0];
        let points3_lat_lons =
            LatLons::try_from((points3_lats.as_slice(), points3_lons.as_slice())).unwrap();
        let arc_3 = Arc3d::from(points3_lat_lons.0.as_slice());
        assert!(arc_3.is_valid());

        let values_3 = polygon.intersection_lengths(&arc_3);
        assert_eq!(0, values_3.len());
    }

    #[test]
    fn test_polygon2() {
        // A closed rectangle in Lat Long coords around the International Date Line
        let rect_lats = vec![-44.0, -46.0, -46.0, -44.0, -44.0];
        let rect_lons = vec![-179.0, -179.0, 179.0, 179.0, -179.0];
        let lat_lons = LatLons::try_from((rect_lats.as_slice(), rect_lons.as_slice())).unwrap();

        let polygon = Arc3dString::from(&lat_lons);
        assert!(polygon.is_valid());
        assert!(polygon.is_closed_loop());
        assert_eq!(polygon.poles().len(), polygon.count() - 1);

        // Points on the lower and upper rhumb lines
        let points_lats = vec![-44.0, -46.0, -45.0, -45.0];
        let points_lons = vec![180.0, 180.0, -179.0, 179.0];
        let points_lat_lons =
            LatLons::try_from((points_lats.as_slice(), points_lons.as_slice())).unwrap();
        let points = Vec::<Point3d>::from(&points_lat_lons);

        // lower rhumb line is outside
        assert!(!polygon.is_within(&points[0]));
        // upper rhumb line is inside
        assert!(polygon.is_within(&points[1]));

        // right longitude is outside
        assert!(!polygon.is_within(&points[2]));
        // left longitude is inside
        assert!(polygon.is_within(&points[3]));
    }

    #[test]
    fn test_polygon3() {
        // A closed rectangle in Lat Long coords West of the Greenwich meridian
        let rect_lats = vec![-44.0, -46.0, -46.0, -44.0, -44.0];
        let rect_lons = vec![-160.0, -160.0, -150.0, -150.0, -160.0];
        let lat_lons = LatLons::try_from((rect_lats.as_slice(), rect_lons.as_slice())).unwrap();

        let polygon = Arc3dString::from(&lat_lons);
        assert!(polygon.is_valid());
        assert!(polygon.is_closed_loop());
        assert_eq!(polygon.poles().len(), polygon.count() - 1);

        // Points on the lower and upper rhumb lines
        let points_lats = vec![-44.0, -46.0, -44.0, -46.0];
        let points_lons = vec![-155.0, -155.0, -160.0, -150.0];
        let points_lat_lons =
            LatLons::try_from((points_lats.as_slice(), points_lons.as_slice())).unwrap();
        let points = Vec::<Point3d>::from(&points_lat_lons);

        // lower rhumb line is outside
        assert!(!polygon.is_within(&points[0]));
        // upper rhumb line is inside
        assert!(polygon.is_within(&points[1]));
    }

    #[test]
    fn test_route_1() {
        let route_lats = vec![1.0, 1.0, 1.0, -1.0, -1.0, 1.0, 1.0, 0.0, 0.0, 1.0, 2.0, 3.0];
        let route_lons = vec![
            -3.0, -2.0, -1.0, -1.0, 1.0, 0.0, 3.0, 2.0, 5.0, 4.0, 4.0, 4.0,
        ];
        let lat_lons = LatLons::try_from((route_lats.as_slice(), route_lons.as_slice())).unwrap();

        let arc_string = Arc3dString::from(&lat_lons);
        assert!(arc_string.is_valid());
        assert!(!arc_string.is_closed_loop());
        assert_eq!(arc_string.poles().len(), arc_string.count() - 1);

        let arc1 = arc_string.arc(1);
        let arc2 = arc_string.arc(2);
        let turn_angle1 = arc1.turn_angle(&arc2);
        assert_eq!(89.9912735753293, Degrees::from(turn_angle1).0);
    }
}
