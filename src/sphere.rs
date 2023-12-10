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

//! The sphere module contains types and functions for calculating distances
//! and azimuths between points on the surface of a sphere.

pub mod arc3d;
pub mod arc3dstring;
pub mod great_circle3d;
pub mod point3d;

use super::trig::{
    from_degrees, valid_latitudes, valid_longitudes, Angle, Degrees, Radians, UnitNegRange,
};
use super::{clamp, Validate};
use contracts::*;
use serde::{Deserialize, Serialize};
use std::convert::{From, TryFrom};

/// Calculates the length of the adjacent side of a right angled
/// spherical triangle, given the angle and length of the hypotenuse.
#[debug_requires((0.0..std::f64::consts::FRAC_PI_2).contains(&length.0))]
pub fn spherical_cosine_rule(cos_angle: f64, length: Radians) -> Radians {
    Radians(libm::atan(cos_angle * libm::tan(length.0)))
}

/// Convert a Euclidean distance to a Great Circle distance (in radians).
/// e should satisfy: 0 <= e <= 2, if not it is clamped into range.
#[debug_requires(0.0 <= e)] // don't test e <= 2, due to floating point maths issues
#[debug_ensures((0.0..=std::f64::consts::PI).contains(&ret.0))]
pub fn e2gc_distance(e: f64) -> Radians {
    Radians(2.0 * libm::asin(UnitNegRange::clamp(0.5 * e).0))
}

/// Convert a Great Circle distance (in radians) to a Euclidean distance.
#[debug_requires((0.0..=std::f64::consts::PI).contains(&gc.0))]
#[debug_ensures((0.0..=2.0).contains(&ret))]
pub fn gc2e_distance(gc: Radians) -> f64 {
    2.0 * libm::sin(0.5 * gc.0)
}

/// Calculate the square of the Euclidean distance (i.e. using Pythagoras)
/// between two points from their Latitudes and their Longitude difference.
/// * `lat_a` - start point Latitude.
/// * `lat_b` - finish point Latitude.
/// * `delta_long` - Longitude difference between start and finish points.
///
/// returns the square of the Euclidean distance between the points.
#[debug_ensures((0.0..=4.0).contains(&ret))]
pub fn sq_euclidean_distance(lat_a: Angle, lat_b: Angle, delta_long: Angle) -> f64 {
    let delta_x = lat_b.cos() * delta_long.cos() - lat_a.cos();
    let delta_y = lat_b.cos() * delta_long.sin();
    let delta_z = lat_b.sin() - lat_a.sin();

    let result = delta_x * delta_x + delta_y * delta_y + delta_z * delta_z;
    clamp(result, 0.0, 4.0)
}

/// Calculate the Great Circle distance (angle from centre) between two points
/// from their Latitudes and their Longitude difference.
/// This function is more accurate than haversine_distance.
/// * `lat_a` - start point Latitude.
/// * `lat_b` - finish point Latitude.
/// * `delta_long` - Longitude difference between start and finish points.
///
/// returns the Great Circle distance between the points in Radians.
pub fn calculate_gc_distance(lat_a: Angle, lat_b: Angle, delta_long: Angle) -> Radians {
    e2gc_distance(libm::sqrt(sq_euclidean_distance(lat_a, lat_b, delta_long)))
}

/// Calculate the Great Circle distance (angle from centre) between two points
/// from their Latitudes and their Longitude difference.
/// This function is less accurate than calculate_gc_distance.
/// * `lat_a` - start point Latitude.
/// * `lat_b` - finish point Latitude.
/// * `delta_long` - Longitude difference between start and finish points.
///
/// returns the Great Circle distance between the points in Radians.
pub fn haversine_distance(lat_a: Angle, lat_b: Angle, delta_long: Angle) -> Radians {
    let delta_lat = lat_b - lat_a;
    let haversine_lat = delta_lat.sq_sine_half();
    let haversine_lon = delta_long.sq_sine_half();

    let a = clamp(
        haversine_lat + lat_a.cos() * lat_b.cos() * haversine_lon,
        0.0,
        1.0,
    );

    Radians(2.0 * libm::asin(libm::sqrt(a)))
}

/// Calculate the azimuth (bearing) along the great circle of point b from
/// point a from their Latitudes and their Longitude difference.
/// * `lat_a` - start point Latitude.
/// * `lat_b` - finish point Latitude.
/// * `delta_long` - Longitude difference between start and finish points.
///
/// returns the Great Circle azimuth relative to North of point b from point a
/// as an Angle.
pub fn calculate_gc_azimuth(lat_a: Angle, lat_b: Angle, delta_long: Angle) -> Angle {
    let sin_azimuth = lat_b.cos() * delta_long.sin();
    let temp = (lat_a.sin() * lat_b.cos() * delta_long.sin() * delta_long.sin())
        / (1.0 + libm::fabs(delta_long.cos()));
    let cos_azimuth = if delta_long.cos() < 0.0 {
        lat_b.sin() * lat_a.cos() + lat_a.sin() * lat_b.cos() - temp
    } else {
        lat_b.sin() * lat_a.cos() - lat_a.sin() * lat_b.cos() + temp
    };

    Angle::from_y_x(sin_azimuth, cos_azimuth)
}

/// Calculate the azimuth and distance along the great circle of point b from
/// point a from their Latitudes and their Longitude difference.
/// * `lat_a` - start point Latitude.
/// * `lat_b` - finish point Latitude.
/// * `delta_long` - Longitude difference between start and finish points.
///
/// returns the Great Circle azimuth relative to North and distance of point b
/// from point a.
pub fn calculate_gc_azimuth_distance(
    lat_a: Angle,
    lat_b: Angle,
    delta_long: Angle,
) -> (Angle, Radians) {
    let azimuth = calculate_gc_azimuth(lat_a, lat_b, delta_long);
    let distance = calculate_gc_distance(lat_a, lat_b, delta_long);
    (azimuth, distance)
}

/// A position as a latitude and longitude pair.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct LatLon {
    lat: Angle,
    lon: Angle,
}

impl Validate for LatLon {
    /// Test whether a LatLon is valid.  
    /// I.e. whether the latitude lies in the range: -90.0 <= value <= 90.0
    fn is_valid(&self) -> bool {
        self.lat.is_valid_latitude() && self.lon.is_valid()
    }
}

#[debug_invariant(self.is_valid())]
impl LatLon {
    pub fn new(lat: Angle, lon: Angle) -> Self {
        Self { lat, lon }
    }

    pub fn lat(&self) -> Angle {
        self.lat
    }

    pub fn lon(&self) -> Angle {
        self.lon
    }
}

impl From<&LatLon> for point3d::Point3d {
    /// Convert a LatLon to a Point3d on the unit sphere
    fn from(value: &LatLon) -> point3d::Point3d {
        point3d::Point3d::new(
            value.lat.cos() * value.lon.cos(),
            value.lat.cos() * value.lon.sin(),
            value.lat.sin(),
        )
    }
}

/// Convert a slice of LatLon to a Vec of Point3d on the unit sphere
pub fn from_slice(values: &[LatLon]) -> Vec<point3d::Point3d> {
    values.iter().map(|v| v.into()).collect()
}

impl From<&LatLon> for (f64, f64) {
    /// Convert a LatLon to a Latitude, Longitude pair in degrees
    fn from(value: &LatLon) -> (f64, f64) {
        (Degrees::from(value.lat).0, Degrees::from(value.lon).0)
    }
}

impl From<&LatLon> for Vec<f64> {
    /// Convert a LatLon to a geojson PointType or Position
    fn from(value: &LatLon) -> Vec<f64> {
        vec![Degrees::from(value.lon).0, Degrees::from(value.lat).0]
    }
}

impl From<&point3d::Point3d> for LatLon {
    /// Create a LatLon from a Point3d.  
    fn from(value: &point3d::Point3d) -> Self {
        Self::new(point3d::latitude(value), point3d::longitude(value))
    }
}

impl TryFrom<(f64, f64)> for LatLon {
    type Error = &'static str;

    /// Attempt to convert a pair of f64 values in Latitude, Longitude order.
    fn try_from(item: (f64, f64)) -> Result<Self, Self::Error> {
        let lat = Degrees(item.0);
        let lon = Degrees(item.1);
        if lat.is_valid_latitude() {
            Ok(LatLon::new(Angle::from(lat), Angle::from(lon)))
        } else {
            Err("invalid latitude")
        }
    }
}

impl TryFrom<&[f64]> for LatLon {
    type Error = &'static str;

    /// Attempt to convert a slice of f64 values in Longitude, Latitude order.
    /// To convert a geojson PointType or Position to a LatLon.
    fn try_from(item: &[f64]) -> Result<Self, Self::Error> {
        if item.len() == 2 {
            // Note: order reversed for geojson
            Self::try_from((item[1], item[0]))
        } else {
            Err("not 2 values")
        }
    }
}

/// Calculate the azimuth and distance along the great circle of point b from
/// point a.
/// * `a` - start point.
/// * `b` - finish point.
///
/// returns the Great Circle azimuth relative to North and distance of point b
/// from point a.
pub fn calc_gc_azimuth_distance(a: LatLon, b: LatLon) -> (Angle, Radians) {
    let delta_long = b.lon() - a.lon();
    calculate_gc_azimuth_distance(a.lat(), b.lat(), delta_long)
}

/// A collection of positions as a latitude and longitude pairs.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct LatLons(pub Vec<LatLon>);

impl LatLons {
    pub fn new(lats: &[Angle], lons: &[Angle]) -> Self {
        assert!((1 < lats.len()) && (lats.len() == lons.len()));
        LatLons(
            lats.iter()
                .zip(lons.iter())
                .map(|(&lat, &lon)| LatLon::new(lat, lon))
                .collect(),
        )
    }

    /// Determine whether the LatLons form a closed loop,
    /// i.e. is a polygon.
    ///
    /// returns true if it is a closed loop, false otherwise.
    pub fn is_closed_loop(&self) -> bool {
        (3 < self.0.len()) && (self.0.first() == self.0.last())
    }
}

impl TryFrom<(&[f64], &[f64])> for LatLons {
    type Error = &'static str;
    /// Attempt to convert a pair of f64 slices of values in Latitude, Longitude order.
    fn try_from(values: (&[f64], &[f64])) -> Result<LatLons, Self::Error> {
        let lats = values.0;
        let lons = values.1;

        if !lats.is_empty()
            && lats.len() == lons.len()
            && valid_latitudes(lats)
            && valid_longitudes(lons)
        {
            let lats_angles = from_degrees(lats);
            let lons_angles = from_degrees(lons);
            Ok(LatLons::new(lats_angles.as_slice(), lons_angles.as_slice()))
        } else {
            Err("lats and/or lons not valid to combine")
        }
    }
}

fn try_from_geojson_linestring(values: &[Vec<f64>]) -> Result<Vec<LatLon>, &'static str> {
    values
        .iter()
        .map(|v| LatLon::try_from(v.as_slice()))
        .collect()
}

impl TryFrom<&[Vec<f64>]> for LatLons {
    type Error = &'static str;

    fn try_from(values: &[Vec<f64>]) -> Result<LatLons, Self::Error> {
        Ok(LatLons(try_from_geojson_linestring(values)?))
    }
}

impl From<&LatLons> for Vec<point3d::Point3d> {
    /// Convert LatLons to a vector of Point3ds on the unit sphere
    fn from(value: &LatLons) -> Vec<point3d::Point3d> {
        from_slice(&value.0)
    }
}

impl From<&LatLons> for Vec<Vec<f64>> {
    /// Convert a LatLons to a geojson LineString
    fn from(value: &LatLons) -> Vec<Vec<f64>> {
        value.0.iter().map(|v| v.into()).collect::<Vec<_>>()
    }
}

/// A collection of LatLons.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct MultiLatLons(pub Vec<LatLons>);

fn try_from_geojson_multi_line_string(
    values: &[Vec<Vec<f64>>],
) -> Result<Vec<LatLons>, &'static str> {
    values
        .iter()
        .map(|v| LatLons::try_from(v.as_slice()))
        .collect()
}

impl TryFrom<&[Vec<Vec<f64>>]> for MultiLatLons {
    type Error = &'static str;

    fn try_from(values: &[Vec<Vec<f64>>]) -> Result<MultiLatLons, Self::Error> {
        Ok(MultiLatLons(try_from_geojson_multi_line_string(values)?))
    }
}

impl From<&MultiLatLons> for Vec<Vec<Vec<f64>>> {
    /// Convert a MultiLatLons to a geojson MultiLineString / Polygon
    fn from(value: &MultiLatLons) -> Vec<Vec<Vec<f64>>> {
        value.0.iter().map(|v| v.into()).collect::<Vec<_>>()
    }
}

#[cfg(test)]
mod tests {
    use crate::sphere::*;
    use crate::trig::DEG2RAD;

    use serde_json::to_string;

    #[test]
    fn test_distance_functions() {
        assert_eq!(std::f64::consts::PI, e2gc_distance(2.1).0);
        let delta_angle = std::f64::consts::FRAC_PI_2 - e2gc_distance(std::f64::consts::SQRT_2).0;
        assert!(delta_angle < std::f64::EPSILON);

        assert_eq!(2.0, gc2e_distance(Radians(std::f64::consts::PI)));
        let delta_dist =
            std::f64::consts::SQRT_2 - gc2e_distance(Radians(std::f64::consts::FRAC_PI_2));
        assert!(delta_dist <= std::f64::EPSILON);
    }

    #[test]
    fn test_calculate_gc_distance_delta_latitude() {
        let start_latitude = -80.0;
        let start_angle = Angle::from(Degrees(start_latitude));
        let delta_lon = Angle::default();

        for i in 1..160 {
            let value = i as f64;
            let latitude = start_latitude + value;
            let angle = Angle::from(Degrees(latitude));
            let expected = DEG2RAD * value;
            let distance = calculate_gc_distance(start_angle, angle, delta_lon);

            // assert_eq!(expected, distance.0);  // Does not work, not accurate enough.
            let delta_result = libm::fabs(expected - distance.0);
            assert!(delta_result <= 4.0 * std::f64::EPSILON);
        }
    }

    #[test]
    fn test_haversine_distance_delta_latitude() {
        let start_latitude = -80.0;
        let start_angle = Angle::from(Degrees(start_latitude));
        let delta_lon = Angle::default();

        for i in 1..160 {
            let value = i as f64;
            let latitude = start_latitude + value;
            let angle = Angle::from(Degrees(latitude));
            let expected = DEG2RAD * value;
            let distance = haversine_distance(start_angle, angle, delta_lon);

            // assert_eq!(expected, distance.0);  // Does not work, not accurate enough.
            let delta_result = libm::fabs(expected - distance.0);
            assert!(delta_result <= 32.0 * std::f64::EPSILON);
        }
    }

    #[test]
    fn test_distance_delta_longitude() {
        let end_latitude = 40.0;
        let start_angle = Angle::from(Degrees(-end_latitude));
        let end_angle = Angle::from(Degrees(end_latitude));

        for i in 1..160 {
            let value = i as f64;
            let delta_lon = Angle::from(Degrees(value));
            let expected = calculate_gc_distance(start_angle, end_angle, delta_lon);
            let distance = haversine_distance(start_angle, end_angle, delta_lon);

            // assert_eq!(expected.0, distance.0);  // Does not work, not accurate enough.
            let delta_result = libm::fabs(expected.0 - distance.0);
            assert!(delta_result <= 8.0 * std::f64::EPSILON);
        }
    }

    #[test]
    fn test_great_circle_90n_0n_0e() {
        let angle_90 = Angle::from_y_x(1.0, 0.0);
        let angle_0 = Angle::default();

        let a = LatLon::new(angle_90, angle_0);
        let b = LatLon::new(angle_0, angle_0);
        let (azimuth, distance) = calc_gc_azimuth_distance(a, b);

        // Note: multiplication is not precise...
        // assert_eq!(DEG2RAD * 30.0, distance.0);
        let delta_distance = libm::fabs(DEG2RAD * 90.0 - distance.0);
        assert!(delta_distance <= 48.0 * std::f64::EPSILON);
        assert_eq!(180.0, Degrees::from(azimuth).0);
    }

    #[test]
    fn test_great_circle_90s_0n_0e() {
        let angle_m90 = Angle::from_y_x(-1.0, 0.0);
        let angle_0 = Angle::default();

        let a = LatLon::new(angle_m90, angle_0);
        let b = LatLon::new(angle_0, angle_0);
        let (azimuth, distance) = calc_gc_azimuth_distance(a, b);

        // Note: multiplication is not precise...
        // assert_eq!(DEG2RAD * 30.0, distance.0);
        let delta_distance = libm::fabs(DEG2RAD * 90.0 - distance.0);
        assert!(delta_distance <= 48.0 * std::f64::EPSILON);
        assert_eq!(0.0, Degrees::from(azimuth).0);
    }

    #[test]
    fn test_great_circle_30n_60n_0e() {
        let angle_30 = Angle::from(Degrees(30.0));
        let angle_60 = Angle::from(Degrees(60.0));
        let angle_0 = Angle::default();

        let a = LatLon::new(angle_30, angle_0);
        let b = LatLon::new(angle_60, angle_0);
        let (azimuth, distance) = calc_gc_azimuth_distance(a, b);

        // Note: multiplication is not precise...
        // assert_eq!(DEG2RAD * 30.0, distance.0);
        let delta_distance = libm::fabs(DEG2RAD * 30.0 - distance.0);
        assert!(delta_distance <= 48.0 * std::f64::EPSILON);
        assert_eq!(0.0, Degrees::from(azimuth).0);
    }

    #[test]
    fn test_great_circle_60n_30n_0e() {
        let angle_30 = Angle::from(Degrees(30.0));
        let angle_60 = Angle::from(Degrees(60.0));
        let angle_0 = Angle::default();

        let a = LatLon::new(angle_30, angle_0);
        let b = LatLon::new(angle_60, angle_0);
        let (azimuth, distance) = calc_gc_azimuth_distance(b, a);

        // let distance = calculate_gc_distance(angle_60, angle_30, angle_0);
        // Note: multiplication is not precise...
        // assert_eq!(DEG2RAD * 30.0, distance.0);
        let delta_distance = libm::fabs(DEG2RAD * 30.0 - distance.0);
        assert!(delta_distance <= 48.0 * std::f64::EPSILON);
        assert_eq!(180.0, Degrees::from(azimuth).0);
    }

    #[test]
    fn test_great_circle_60n_60n_30w() {
        let angle_m30 = Angle::from(Degrees(-30.0));
        let angle_60 = Angle::from(Degrees(60.0));

        let a = LatLon::new(angle_60, Angle::default());
        let b = LatLon::new(angle_60, angle_m30);
        let (azimuth, distance) = calc_gc_azimuth_distance(a, b);

        assert_eq!(DEG2RAD * 14.87094445226370419, distance.0);
        assert_eq!(-76.93568657049171, Degrees::from(azimuth).0);
    }

    #[test]
    fn test_great_circle_60s_60s_30e() {
        let angle_30 = Angle::from(Degrees(30.0));
        let angle_m60 = Angle::from(Degrees(-60.0));

        let a = LatLon::new(angle_m60, Angle::default());
        let b = LatLon::new(angle_m60, angle_30);
        let (azimuth, distance) = calc_gc_azimuth_distance(a, b);

        assert_eq!(DEG2RAD * 14.87094445226370419, distance.0);
        assert_eq!(180.0 - 76.93568657049171, Degrees::from(azimuth).0);
    }

    #[test]
    fn test_latlons_new() {
        let lats = [44.0, 46.0, 46.0, 44.0];
        let lons = [1.0, 1.0, -1.0, -1.0];

        let lats_angles = from_degrees(&lats);
        let lons_angles = from_degrees(&lons);
        let latlons = LatLons::new(lats_angles.as_slice(), lons_angles.as_slice());

        assert_eq!(latlons.0.len(), lats.len());
        for i in 0..lats.len() {
            assert_eq!(lats[i], Degrees::from(latlons.0[i].lat()).0);
            assert_eq!(lons[i], Degrees::from(latlons.0[i].lon()).0);
        }
    }

    #[test]
    fn test_latlons_try_from_vec_pair() {
        let lats = vec![44.0, 46.0, 46.0, 44.0];
        let lons = vec![1.0, 1.0, -1.0, -1.0];

        let latlons = LatLons::try_from((lats.as_slice(), lons.as_slice())).unwrap();

        assert!(!latlons.is_closed_loop());
        assert_eq!(latlons.0.len(), lats.len());
        for i in 0..lats.len() {
            assert_eq!(lats[i], Degrees::from(latlons.0[i].lat()).0);
            assert_eq!(lons[i], Degrees::from(latlons.0[i].lon()).0);
        }

        let empty_vals = Vec::<f64>::new();
        let result = LatLons::try_from((empty_vals.as_slice(), lons.as_slice()));
        assert_eq!(Err("lats and/or lons not valid to combine"), result);

        let short_lons = vec![1.0, 1.0, -1.0];
        let result = LatLons::try_from((lats.as_slice(), short_lons.as_slice()));
        assert_eq!(Err("lats and/or lons not valid to combine"), result);

        let invalid_lats = vec![44.0, 90.01, 46.0, 44.0];
        let result = LatLons::try_from((invalid_lats.as_slice(), lons.as_slice()));
        assert_eq!(Err("lats and/or lons not valid to combine"), result);

        let invalid_lons = vec![1.0, 180.01, -1.0, -1.0];
        let result = LatLons::try_from((lats.as_slice(), invalid_lons.as_slice()));
        assert_eq!(Err("lats and/or lons not valid to combine"), result);
    }

    #[test]
    fn test_serde_latlon() {
        let angle_20 = Angle::from(Degrees(20.0));
        let latlon = LatLon::new(angle_20, angle_20);

        let serialized = to_string(&latlon).unwrap();
        // println!("serialized = {}", serialized);
        let deserialized: LatLon = serde_json::from_str(&serialized).unwrap();
        assert_eq!(latlon, deserialized);
    }

    #[test]
    fn test_serde_latlons() {
        let angle_20 = Angle::from(Degrees(20.0));
        let latlon0 = LatLon::new(angle_20, angle_20);

        let angle_05 = Angle::from(Degrees(0.5));
        let angle_102 = Angle::from(Degrees(102.0));
        let latlon1 = LatLon::new(angle_05, angle_102);

        let latlons: Vec<LatLon> = vec![latlon0, latlon1];

        let serialized = to_string(&latlons).unwrap();
        // println!("serialized = {}", serialized);
        let deserialized: Vec<LatLon> = serde_json::from_str(&serialized).unwrap();
        assert_eq!(latlons, deserialized);
    }

    #[test]
    fn test_serde_multi_line_string() {
        let angle_20 = Angle::from(Degrees(20.0));
        let latlon0 = LatLon::new(angle_20, angle_20);

        let angle_05 = Angle::from(Degrees(0.5));
        let angle_102 = Angle::from(Degrees(102.0));
        let latlon1 = LatLon::new(angle_05, angle_102);

        let latlons0: Vec<LatLon> = vec![latlon0, latlon1];
        let latlons1: Vec<LatLon> = vec![latlon0, latlon1];

        let latlonsvec: Vec<Vec<LatLon>> = vec![latlons0, latlons1];

        let serialized = to_string(&latlonsvec).unwrap();
        // println!("serialized = {}", serialized);
        let deserialized: Vec<Vec<LatLon>> = serde_json::from_str(&serialized).unwrap();
        assert_eq!(latlonsvec, deserialized);
    }

    #[test]
    fn test_serde_geojson_latlon() {
        let angle_05 = Angle::from(Degrees(0.5));
        let angle_102 = Angle::from(Degrees(102.0));
        let latlon = LatLon::new(angle_05, angle_102);

        let position = geojson::PointType::from(&latlon);
        let serialized = to_string(&position).unwrap();
        println!("serialized = {}", serialized);
        let deserialized: geojson::PointType = serde_json::from_str(&serialized).unwrap();
        assert_eq!(position, deserialized);

        let value = LatLon::try_from(deserialized.as_slice()).unwrap();
        assert_eq!(latlon, value);
    }

    #[test]
    fn test_serde_geojson_latlons() {
        let angle_20 = Angle::from(Degrees(20.0));
        let latlon0 = LatLon::new(angle_20, angle_20);

        let angle_05 = Angle::from(Degrees(0.5));
        let angle_102 = Angle::from(Degrees(102.0));
        let latlon1 = LatLon::new(angle_05, angle_102);

        let latlons = LatLons(vec![latlon0, latlon1]);

        let linestring = geojson::LineStringType::from(&latlons);
        let serialized = to_string(&linestring).unwrap();
        // println!("serialized = {}", serialized);
        let deserialized: geojson::LineStringType = serde_json::from_str(&serialized).unwrap();
        assert_eq!(linestring, deserialized);

        let value = LatLons::try_from(deserialized.as_slice()).unwrap();
        assert_eq!(latlons, value);
    }

    #[test]
    fn test_serde_geojson_polygon() {
        let angle_20 = Angle::from(Degrees(20.0));
        let latlon0 = LatLon::new(angle_20, angle_20);

        let angle_05 = Angle::from(Degrees(0.5));
        let angle_102 = Angle::from(Degrees(102.0));
        let latlon1 = LatLon::new(angle_05, angle_102);

        let latlons0 = LatLons(vec![latlon0, latlon1]);
        let latlons1 = LatLons(vec![latlon0, latlon1]);

        let latlonsvec = MultiLatLons(vec![latlons0, latlons1]);

        let polygon = geojson::PolygonType::from(&latlonsvec);
        let serialized = to_string(&polygon).unwrap();
        // println!("serialized = {}", serialized);
        let deserialized: geojson::PolygonType = serde_json::from_str(&serialized).unwrap();
        assert_eq!(polygon, deserialized);

        let value = MultiLatLons::try_from(deserialized.as_slice()).unwrap();
        assert_eq!(latlonsvec, value);
    }
}
