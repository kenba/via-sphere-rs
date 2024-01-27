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

//! The `gojson` module includes types and functions for handling
//! [GeoJSON](https://geojson.org/) types,
//! see: [RFC7946](https://datatracker.ietf.org/doc/html/rfc7946).  
//! Note: [Antimeridian Cutting](https://datatracker.ietf.org/doc/html/rfc7946#section-3.1.9) is **not** required, since this library **can** handle
//! features that cross the antimeridian.

use crate::latlong::{LatLong, LatLongs};
use crate::trig::{Angle, Degrees};
use geo_types;

impl TryFrom<&geo_types::Coord> for LatLong {
    type Error = &'static str;

    /// Attempt to convert a `GeoJSON Coord` to a `LatLong`.
    /// Note: `GeoJSON Coord` order is **lon, lat.**
    fn try_from(item: &geo_types::Coord) -> Result<Self, Self::Error> {
        if Degrees::is_valid_longitude(item.x) && Degrees::is_valid_latitude(item.y) {
            Ok(Self::new(
                Angle::from(Degrees(item.y)),
                Angle::from(Degrees(item.x)),
            ))
        } else if !Degrees::is_valid_latitude(item.y) {
            Err("latitude invalid")
        } else {
            Err("longitude invalid")
        }
    }
}

impl From<&LatLong> for geo_types::Coord {
    fn from(a: &LatLong) -> Self {
        Self {
            x: Degrees::from(a.lon()).0,
            y: Degrees::from(a.lat()).0,
        }
    }
}

impl TryFrom<&geo_types::Point> for LatLong {
    type Error = &'static str;

    /// Attempt to convert a `GeoJSON Point` to a `LatLong`.
    fn try_from(item: &geo_types::Point) -> Result<Self, Self::Error> {
        Self::try_from(&item.0)
    }
}

impl From<&LatLong> for geo_types::Point {
    fn from(a: &LatLong) -> Self {
        Self::new(Degrees::from(a.lon()).0, Degrees::from(a.lat()).0)
    }
}

// LatLongs

fn try_from_geojson_linestring(values: &[geo_types::Coord]) -> Result<Vec<LatLong>, &'static str> {
    values.iter().map(LatLong::try_from).collect()
}

impl TryFrom<&geo_types::LineString> for LatLongs {
    type Error = &'static str;

    /// Attempt to convert a `GeoJSON LineString` to `LatLongs`.
    fn try_from(values: &geo_types::LineString) -> Result<Self, Self::Error> {
        Ok(Self(try_from_geojson_linestring(&values.0)?))
    }
}

impl From<&LatLongs> for geo_types::LineString {
    fn from(values: &LatLongs) -> Self {
        Self::new(values.0.iter().map(geo_types::Coord::from).collect())
    }
}

// MultiLatLongs

/// A collection of `LatLongs`.
/// Corresponds to a `GeoJSON` `MultiLineString` / `Polygon`.
#[derive(Clone, Debug, PartialEq)]
pub struct MultiLatLongs(pub Vec<LatLongs>);

fn try_from_geojson_multi_linestring(
    values: &[geo_types::LineString],
) -> Result<Vec<LatLongs>, &'static str> {
    values.iter().map(LatLongs::try_from).collect()
}

impl TryFrom<&geo_types::MultiLineString> for MultiLatLongs {
    type Error = &'static str;

    /// Attempt to convert a `GeoJSON MultiLineString` to `MultiLatLongs`.
    fn try_from(values: &geo_types::MultiLineString) -> Result<Self, Self::Error> {
        Ok(Self(try_from_geojson_multi_linestring(&values.0)?))
    }
}

impl From<&MultiLatLongs> for geo_types::MultiLineString {
    fn from(values: &MultiLatLongs) -> Self {
        Self::new(values.0.iter().map(geo_types::LineString::from).collect())
    }
}

// Polygon

/// A `GeoJSON` `Polygon`: a `LatLongs` and a Vec of `LatLongs`.
/// Corresponds to a [geo-types](https://crates.io/crates/geo-types)
/// [Polygon](https://docs.rs/geo-types/0.7.12/geo_types/geometry/struct.Polygon.html).
#[derive(Clone, Debug, PartialEq)]
pub struct Polygon {
    exterior: LatLongs,
    interiors: Vec<LatLongs>,
}

impl TryFrom<&geo_types::Polygon> for Polygon {
    type Error = &'static str;

    /// Attempt to convert a `geo_types::PolygonType` to `MultiLatLongs`.
    fn try_from(value: &geo_types::Polygon) -> Result<Self, Self::Error> {
        let exterior = LatLongs(try_from_geojson_linestring(&value.exterior().0)?);
        let interiors: Vec<LatLongs> = try_from_geojson_multi_linestring(value.interiors())?;
        Ok(Self {
            exterior,
            interiors,
        })
    }
}

impl From<&Polygon> for geo_types::Polygon {
    fn from(value: &Polygon) -> Self {
        Self::new(
            value
                .exterior
                .0
                .iter()
                .map(geo_types::Coord::from)
                .collect(),
            value
                .interiors
                .iter()
                .map(geo_types::LineString::from)
                .collect(),
        )
    }
}

// MultiPolygons

/// A collection of Polygons.
/// Corresponds to a [geo-types](https://crates.io/crates/geo-types)
/// [MultiPolygons](https://docs.rs/geo-types/0.7.12/geo_types/geometry/struct.MultiPolygons.html).
#[derive(Clone, Debug, PartialEq)]
pub struct MultiPolygons(pub Vec<Polygon>);

fn try_from_geojson_multi_polygon(
    values: &[geo_types::Polygon],
) -> Result<Vec<Polygon>, &'static str> {
    values.iter().map(Polygon::try_from).collect()
}

impl TryFrom<&geo_types::MultiPolygon> for MultiPolygons {
    type Error = &'static str;

    /// Attempt to convert a `geo_types::MultiPolygon` to `MultiPolygons`.
    fn try_from(values: &geo_types::MultiPolygon) -> Result<Self, Self::Error> {
        Ok(Self(try_from_geojson_multi_polygon(&values.0)?))
    }
}

impl From<&MultiPolygons> for geo_types::MultiPolygon {
    fn from(values: &MultiPolygons) -> Self {
        Self::new(values.0.iter().map(geo_types::Polygon::from).collect())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::is_within_tolerance;
    use crate::trig::{from_degrees, Degrees};
    use geo_types;
    use geo_types::{line_string, polygon};

    #[test]
    fn test_geo_types_coord_invalid() {
        // Bad latitude
        let bad_latitude = geo_types::Coord::from((0.0, 90.0001));
        let latlong = LatLong::try_from(&bad_latitude);
        assert!(latlong.is_err());

        let bad_latitude = geo_types::Coord::from((0.0, 90.0001));
        let latlong = LatLong::try_from(&bad_latitude);
        assert!(latlong.is_err());

        let bad_longitude = geo_types::Coord::from((180.0001, 0.0));
        let latlong = LatLong::try_from(&bad_longitude);
        assert!(latlong.is_err());
    }

    #[test]
    fn test_geo_types_coord() {
        let angle_80 = Angle::from(Degrees(80.0));
        let angle_160 = Angle::from(Degrees(160.0));
        let latlong = LatLong::new(angle_80, angle_160);

        let coord = geo_types::Coord::from((160.0, 80.0));
        let result = LatLong::try_from(&coord);
        assert_eq!(latlong, result.unwrap());

        let geo_result = geo_types::Coord::from(&latlong);
        assert_eq!(coord.x, geo_result.x);
        assert!(is_within_tolerance(
            coord.y,
            geo_result.y,
            64.0 * std::f64::EPSILON
        ));
    }

    #[test]
    fn test_geo_types_point() {
        let angle_80 = Angle::from(Degrees(80.0));
        let angle_160 = Angle::from(Degrees(160.0));
        let latlong = LatLong::new(angle_80, angle_160);

        let point = geo_types::Point::new(160.0, 80.0);
        let result = LatLong::try_from(&point);
        assert_eq!(latlong, result.unwrap());

        let geo_result = geo_types::Point::from(&latlong);
        assert_eq!(point.x(), geo_result.x());
        assert!(is_within_tolerance(
            point.y(),
            geo_result.y(),
            64.0 * std::f64::EPSILON
        ));
    }

    fn create_test_latlongs() -> LatLongs {
        let lats = vec![10.0, 45.0, 40.0, 20.0, 10.0];
        let lons = vec![35.0, 45.0, 15.0, 10.0, 35.0];

        let lats_angles = from_degrees(&lats);
        let lons_angles = from_degrees(&lons);

        LatLongs::new(&lats_angles, &lons_angles)
    }

    #[test]
    fn test_geo_types_linestring() {
        let latlongs = create_test_latlongs();

        let line_string = geo_types::line_string![
            (x: 35.0, y: 10.0),
            (x: 45.0, y: 45.0),
            (x: 15.0, y: 40.0),
            (x: 10.0, y: 20.0),
            (x: 35.0, y: 10.0)
        ];
        let result = LatLongs::try_from(&line_string).unwrap();
        assert_eq!(latlongs, result);

        let geo_result = geo_types::LineString::from(&latlongs);
        assert_eq!(line_string.0.len(), geo_result.0.len());
    }

    #[test]
    fn test_multilatlongs_new() {
        let lats1 = [10.0, 20.0, 40.0];
        let lons1 = [10.0, 20.0, 10.0];

        let lats_angles1 = from_degrees(&lats1);
        let lons_angles1 = from_degrees(&lons1);
        let latlongs1 = LatLongs::new(lats_angles1.as_slice(), lons_angles1.as_slice());

        assert_eq!(false, latlongs1.is_closed_loop());

        let lats2 = [40.0, 30.0, 20.0, 10.0];
        let lons2 = [40.0, 30.0, 40.0, 30.0];

        let lats_angles2 = from_degrees(&lats2);
        let lons_angles2 = from_degrees(&lons2);
        let latlongs2 = LatLongs::new(lats_angles2.as_slice(), lons_angles2.as_slice());

        assert_eq!(false, latlongs2.is_closed_loop());

        let multilatlongs = MultiLatLongs(vec![latlongs1, latlongs2]);

        let multilatlongs_clone = multilatlongs.clone();
        assert!(multilatlongs_clone == multilatlongs);

        print!("MultiLatLongs: {:?}", multilatlongs);
    }

    #[test]
    fn test_geo_types_multilinestring() {
        let latlongs = create_test_latlongs();
        let latlongsvec = MultiLatLongs(vec![latlongs.clone(), latlongs]);

        let line_string = geo_types::line_string![
            (x: 35.0, y: 10.0),
            (x: 45.0, y: 45.0),
            (x: 15.0, y: 40.0),
            (x: 10.0, y: 20.0),
            (x: 35.0, y: 10.0)
        ];
        let multi_line_string =
            geo_types::MultiLineString::new(vec![line_string.clone(), line_string]);
        let result = MultiLatLongs::try_from(&multi_line_string).unwrap();
        assert_eq!(latlongsvec, result);

        let geo_result = geo_types::MultiLineString::from(&latlongsvec);
        assert_eq!(multi_line_string.0.len(), geo_result.0.len());
    }

    #[test]
    fn test_polygon_new() {
        let lats1 = [10.0, 45.0, 40.0, 20.0, 10.0];
        let lons1 = [35.0, 45.0, 15.0, 10.0, 35.0];

        let lats_angles1 = from_degrees(&lats1);
        let lons_angles1 = from_degrees(&lons1);
        let latlongs1 = LatLongs::new(lats_angles1.as_slice(), lons_angles1.as_slice());

        let lats2 = [30.0, 35.0, 20.0, 30.0];
        let lons2 = [20.0, 35.0, 30.0, 20.0];

        let lats_angles2 = from_degrees(&lats2);
        let lons_angles2 = from_degrees(&lons2);
        let latlongs2 = LatLongs::new(lats_angles2.as_slice(), lons_angles2.as_slice());

        let polygon = Polygon {
            exterior: latlongs1,
            interiors: vec![latlongs2],
        };

        assert!(polygon.exterior.is_closed_loop());
        assert!(polygon.interiors[0].is_closed_loop());

        let polygon_clone = polygon.clone();
        assert!(polygon_clone == polygon);

        print!("Polygon: {:?}", polygon);
    }

    fn create_test_polygon() -> Polygon {
        let exterior = create_test_latlongs();

        let lats = vec![30.0, 35.0, 20.0, 30.0];
        let lons = vec![20.0, 35.0, 30.0, 20.0];

        let lats_angles = from_degrees(&lats);
        let lons_angles = from_degrees(&lons);
        let interior1 = LatLongs::new(&lats_angles, &lons_angles);

        let interiors = vec![interior1];

        Polygon {
            exterior,
            interiors,
        }
    }

    #[test]
    fn test_geo_types_polygon() {
        let polygon = create_test_polygon();

        let geo_polygon = polygon!(
            exterior:[
                (x: 35.0, y: 10.0),
                (x: 45.0, y: 45.0),
                (x: 15.0, y: 40.0),
                (x: 10.0, y: 20.0),
                (x: 35.0, y: 10.0)
            ],
            interiors: [
                [
                    (x: 20.0, y: 30.0),
                    (x: 35.0, y: 35.0),
                    (x: 30.0, y: 20.0),
                    (x: 20.0, y: 30.0)
                ],
            ],
        );
        let result = Polygon::try_from(&geo_polygon).unwrap();
        assert_eq!(polygon, result);

        let geo_result = geo_types::Polygon::from(&polygon);
        assert_eq!(
            geo_polygon.exterior().0.len(),
            geo_result.exterior().0.len()
        );
        assert_eq!(geo_polygon.interiors().len(), geo_result.interiors().len());
    }

    #[test]
    fn test_multipolygon_new() {
        let lats1 = [10.0, 45.0, 40.0, 20.0, 10.0];
        let lons1 = [35.0, 45.0, 15.0, 10.0, 35.0];

        let lats_angles1 = from_degrees(&lats1);
        let lons_angles1 = from_degrees(&lons1);
        let latlongs1 = LatLongs::new(lats_angles1.as_slice(), lons_angles1.as_slice());

        let lats2 = [30.0, 35.0, 20.0, 30.0];
        let lons2 = [20.0, 35.0, 30.0, 20.0];

        let lats_angles2 = from_degrees(&lats2);
        let lons_angles2 = from_degrees(&lons2);
        let latlongs2 = LatLongs::new(lats_angles2.as_slice(), lons_angles2.as_slice());

        let polygon1 = Polygon {
            exterior: latlongs1,
            interiors: vec![latlongs2],
        };

        let lats3 = [40.0, 45.0, 30.0, 40.0];
        let lons3 = [40.0, 20.0, 45.0, 40.0];

        let lats_angles3 = from_degrees(&lats3);
        let lons_angles3 = from_degrees(&lons3);
        let latlongs3 = LatLongs::new(lats_angles3.as_slice(), lons_angles3.as_slice());

        let polygon2 = Polygon {
            exterior: latlongs3,
            interiors: vec![],
        };

        let multipolygon = MultiPolygons(vec![polygon1, polygon2]);

        assert!(multipolygon.0[0].exterior.is_closed_loop());
        assert!(multipolygon.0[0].interiors[0].is_closed_loop());
        assert!(multipolygon.0[1].exterior.is_closed_loop());

        let multipolygon_clone = multipolygon.clone();
        assert!(multipolygon_clone == multipolygon);

        print!("MultiPolygons: {:?}", multipolygon);
    }

    #[test]
    fn test_serde_geojson_multi_polygon() {
        let polygon = create_test_polygon();
        let polygons = MultiPolygons(vec![polygon.clone(), polygon]);

        let geo_polygon = polygon!(
            exterior:[
                (x: 35.0, y: 10.0),
                (x: 45.0, y: 45.0),
                (x: 15.0, y: 40.0),
                (x: 10.0, y: 20.0),
                (x: 35.0, y: 10.0)
            ],
            interiors: [
                [
                    (x: 20.0, y: 30.0),
                    (x: 35.0, y: 35.0),
                    (x: 30.0, y: 20.0),
                    (x: 20.0, y: 30.0)
                ],
            ],
        );
        let geo_polygons = geo_types::MultiPolygon(vec![geo_polygon.clone(), geo_polygon]);

        let result = MultiPolygons::try_from(&geo_polygons).unwrap();
        assert_eq!(polygons, result);

        let geo_result = geo_types::MultiPolygon::from(&polygons);
        assert_eq!(geo_polygons.0.len(), geo_result.0.len());
    }
}
