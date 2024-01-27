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

//! The latlong module contains types and functions for representing positions
//! on the surface of a sphere.
//!
//! All distances on the surface of the unit sphere are measured in radians.  
//! Physical distances can be calculated by multiplying by the radius of the sphere.
//!
//! The `latlong` module includes the `calculate_azimuth_and_distance` function to
//! calculate the initial azimuth (a.k.a bearing and distance) between two positions.
//! The azimuth is returned as an `Angle` and the distance is returned in `Radians`.
//!
//! The module also contains types and functions for serializing and
//! deserializing `LatLong` and slices/vectors of `LatLongs` using
//! [serde](https://crates.io/crates/serde).

pub mod geojson;

use crate::trig::{
    calculate_gc_azimuth, calculate_gc_distance, from_degrees, valid_latitudes, valid_longitudes,
    Angle, Degrees, Radians,
};
use crate::Validate;
use contracts::{debug_invariant, debug_requires};

/// A position as a latitude and longitude pair of `Angles`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LatLong {
    lat: Angle,
    lon: Angle,
}

impl Validate for LatLong {
    /// Test whether a `LatLong` is valid.  
    /// I.e. whether the latitude lies in the range: -90.0 <= value <= 90.0
    fn is_valid(&self) -> bool {
        self.lat.is_valid_latitude() && self.lon.is_valid()
    }
}

#[debug_invariant(self.is_valid())]
impl LatLong {
    #[debug_requires(lat.is_valid_latitude())]
    #[must_use]
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

/// Calculate the azimuth and distance along the great circle of point b from
/// point a.
/// * `a`, `b` - the start and end positions
///
/// returns the Great Circle azimuth relative to North and distance of point b
/// from point a.
#[must_use]
pub fn calculate_azimuth_and_distance(a: &LatLong, b: &LatLong) -> (Angle, Radians) {
    let delta_long = b.lon() - a.lon();
    let azimuth = calculate_gc_azimuth(a.lat(), b.lat(), delta_long);
    let distance = calculate_gc_distance(a.lat(), b.lat(), delta_long);
    (azimuth, distance)
}

/// A collection of positions as a latitude and longitude pairs.  
/// Corresponds to a `GeoJSON` `LineString`.
#[derive(Clone, Debug, PartialEq)]
pub struct LatLongs(pub Vec<LatLong>);

impl LatLongs {
    #[debug_requires((1 < lats.len()) && (lats.len() == lons.len()))]
    #[must_use]
    pub fn new(lats: &[Angle], lons: &[Angle]) -> Self {
        Self(
            lats.iter()
                .zip(lons.iter())
                .map(|(&lat, &lon)| LatLong::new(lat, lon))
                .collect(),
        )
    }

    /// Determine whether the `LatLongs` form a closed loop, i.e. is a
    /// [Polygon](https://docs.rs/geo-types/0.7.12/geo_types/geometry/struct.Polygon.html)
    /// `LineString`.
    ///
    /// returns true if a closed loop, false otherwise.
    #[must_use]
    pub fn is_closed_loop(&self) -> bool {
        (2 < self.0.len()) && (self.0.first() == self.0.last())
    }
}

/////////////////////////////////////////////////////////////////////////////
// Pairs of vectors: latitude vector and longitude vector

impl TryFrom<(&[f64], &[f64])> for LatLongs {
    type Error = &'static str;

    /// Attempt to convert a pair of f64 slices of values in Latitude, Longitude order.
    /// * `lats` latitudes
    /// * `lons` longitudes
    ///
    /// return a vector of valid `LatLongs`.
    fn try_from(values: (&[f64], &[f64])) -> Result<Self, Self::Error> {
        let lats = values.0;
        let lons = values.1;

        if !lats.is_empty()
            && lats.len() == lons.len()
            && valid_latitudes(lats)
            && valid_longitudes(lons)
        {
            let lats_angles = from_degrees(lats);
            let lons_angles = from_degrees(lons);
            Ok(Self::new(lats_angles.as_slice(), lons_angles.as_slice()))
        } else {
            Err("lats and/or lons not valid to combine")
        }
    }
}

impl From<LatLongs> for (Vec<Degrees>, Vec<Degrees>) {
    /// Convert a slice of LatLongs to a pair of latitude and longitude vectors
    /// of `Degrees`.
    ///
    /// * `values` LatLongs
    ///
    /// return a pair of latitude and longitude vectors of `Degrees`.
    fn from(values: LatLongs) -> Self {
        {
            (
                values.0.iter().map(|a| Degrees::from(a.lat())).collect(),
                values.0.iter().map(|a| Degrees::from(a.lon())).collect(),
            )
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::is_within_tolerance;
    use crate::trig::{from_degrees, Degrees, DEG2RAD};

    #[test]
    fn test_latlong_traits() {
        let angle_0 = Angle::default();
        let angle_90 = Angle::from_y_x(1.0, 0.0);
        let a = LatLong::new(angle_0, angle_90);

        let a_clone = a.clone();
        assert!(a_clone == a);

        print!("LatLong: {:?}", a);
    }

    #[test]
    fn test_great_circle_90n_0n_0e() {
        let angle_90 = Angle::from_y_x(1.0, 0.0);
        let angle_0 = Angle::default();

        let a = LatLong::new(angle_90, angle_0);
        let b = LatLong::new(angle_0, angle_0);
        let (azimuth, distance) = calculate_azimuth_and_distance(&a, &b);

        // Note: multiplication is not precise...
        // assert_eq!(std::f64::consts::FRAC_PI_2, distance.0);
        assert!(is_within_tolerance(
            std::f64::consts::FRAC_PI_2,
            distance.0,
            48.0 * std::f64::EPSILON
        ));
        assert_eq!(180.0, Degrees::from(azimuth).0);
    }

    #[test]
    fn test_great_circle_0n_60e_0n_60w() {
        let angle_60 = Angle::from(Degrees(60.0));
        let angle_0 = Angle::default();

        let a = LatLong::new(angle_0, angle_60);
        let b = LatLong::new(angle_0, -angle_60);
        let (azimuth, distance) = calculate_azimuth_and_distance(&a, &b);

        assert_eq!(DEG2RAD * 120.0, distance.0);
        assert_eq!(-90.0, Degrees::from(azimuth).0);
    }

    #[test]
    fn test_great_circle_30n_60n_0e() {
        let angle_30 = Angle::from(Degrees(30.0));
        let angle_60 = Angle::from(Degrees(60.0));
        let angle_0 = Angle::default();

        let a = LatLong::new(angle_30, angle_0);
        let b = LatLong::new(angle_60, angle_0);
        let (azimuth, distance) = calculate_azimuth_and_distance(&a, &b);

        // Note: multiplication is not precise...
        // assert_eq!(DEG2RAD * 30.0, distance.0);
        assert!(is_within_tolerance(
            std::f64::consts::FRAC_PI_6,
            distance.0,
            48.0 * std::f64::EPSILON
        ));
        assert_eq!(0.0, Degrees::from(azimuth).0);
    }

    #[test]
    fn test_great_circle_60n_30n_0e() {
        let angle_30 = Angle::from(Degrees(30.0));
        let angle_60 = Angle::from(Degrees(60.0));
        let angle_0 = Angle::default();

        let a = LatLong::new(angle_30, angle_0);
        let b = LatLong::new(angle_60, angle_0);
        let (azimuth, distance) = calculate_azimuth_and_distance(&b, &a);

        // Note: multiplication is not precise...
        // assert_eq!(DEG2RAD * 30.0, distance.0);
        assert!(is_within_tolerance(
            std::f64::consts::FRAC_PI_6,
            distance.0,
            48.0 * std::f64::EPSILON
        ));
        assert_eq!(180.0, Degrees::from(azimuth).0);
    }

    #[test]
    fn test_great_circle_60n_60n_30w() {
        let angle_m30 = Angle::from(Degrees(-30.0));
        let angle_60 = Angle::from(Degrees(60.0));

        let a = LatLong::new(angle_60, Angle::default());
        let b = LatLong::new(angle_60, angle_m30);
        let (azimuth, distance) = calculate_azimuth_and_distance(&a, &b);

        assert_eq!(DEG2RAD * 14.87094445226370419, distance.0);
        assert_eq!(-76.93568657049171, Degrees::from(azimuth).0);
    }

    #[test]
    fn test_great_circle_60s_60s_30e() {
        let angle_30 = Angle::from(Degrees(30.0));
        let angle_m60 = Angle::from(Degrees(-60.0));

        let a = LatLong::new(angle_m60, Angle::default());
        let b = LatLong::new(angle_m60, angle_30);
        let (azimuth, distance) = calculate_azimuth_and_distance(&a, &b);

        assert_eq!(DEG2RAD * 14.87094445226370419, distance.0);
        assert_eq!(180.0 - 76.93568657049171, Degrees::from(azimuth).0);
    }

    #[test]
    fn test_latlongs_new() {
        let lats = [44.0, 46.0, 46.0, 44.0, 44.0];
        let lons = [1.0, 1.0, -1.0, -1.0, 1.0];

        let lats_angles = from_degrees(&lats);
        let lons_angles = from_degrees(&lons);
        let latlongs = LatLongs::new(lats_angles.as_slice(), lons_angles.as_slice());

        assert!(latlongs.is_closed_loop());

        assert_eq!(latlongs.0.len(), lats.len());
        for i in 0..lats.len() {
            assert_eq!(lats[i], Degrees::from(latlongs.0[i].lat()).0);
            assert_eq!(lons[i], Degrees::from(latlongs.0[i].lon()).0);
        }

        let latlongs_clone = latlongs.clone();
        assert!(latlongs_clone == latlongs);

        print!("LatLongs: {:?}", latlongs);
    }

    #[test]
    fn test_latlongs_try_from_vec_pair() {
        let lats = vec![10.0, 45.0, 40.0, 20.0, 10.0];
        let lons = vec![35.0, 45.0, 15.0, 10.0, 35.0];

        let latlongs = LatLongs::try_from((lats.as_slice(), lons.as_slice())).unwrap();

        assert!(latlongs.is_closed_loop());
        assert_eq!(latlongs.0.len(), lats.len());

        for i in 0..lats.len() {
            assert!(is_within_tolerance(
                lats[i],
                Degrees::from(latlongs.0[i].lat()).0,
                64.0 * std::f64::EPSILON
            ));
            assert!(is_within_tolerance(
                lons[i],
                Degrees::from(latlongs.0[i].lon()).0,
                64.0 * std::f64::EPSILON
            ));
        }

        let result: (Vec<Degrees>, Vec<Degrees>) = LatLongs::into(latlongs);
        for i in 0..lats.len() {
            assert!(is_within_tolerance(
                lats[i],
                result.0[i].0,
                64.0 * std::f64::EPSILON
            ));
            assert!(is_within_tolerance(
                lons[i],
                result.1[i].0,
                64.0 * std::f64::EPSILON
            ));
        }

        let empty_vals = Vec::<f64>::new();
        let result = LatLongs::try_from((empty_vals.as_slice(), lons.as_slice()));
        assert_eq!(Err("lats and/or lons not valid to combine"), result);

        let short_lons = vec![1.0, 1.0, -1.0];
        let result = LatLongs::try_from((lats.as_slice(), short_lons.as_slice()));
        assert_eq!(Err("lats and/or lons not valid to combine"), result);

        let invalid_lats = vec![44.0, 90.01, 46.0, 44.0];
        let result = LatLongs::try_from((invalid_lats.as_slice(), lons.as_slice()));
        assert_eq!(Err("lats and/or lons not valid to combine"), result);

        let invalid_lons = vec![1.0, 180.01, -1.0, -1.0];
        let result = LatLongs::try_from((lats.as_slice(), invalid_lons.as_slice()));
        assert_eq!(Err("lats and/or lons not valid to combine"), result);
    }
}
