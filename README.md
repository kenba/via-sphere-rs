# via-sphere

[![License](https://img.shields.io/badge/License-MIT-blue)](https://opensource.org/license/mit/)
[![Rust](https://github.com/kenba/via-sphere-rs/workflows/Rust/badge.svg)](https://github.com/kenba/via-sphere-rs/actions)

A library for performing geometric calculations on the surface of a sphere.

## Description

[Spherical vector geometry](docs/Spherical_Vector_Geometry.md) uses a combination
of spherical trigonometry and vector geometry to calculate distances, angles and
positions on the surface of a sphere.

Spherical trigonometry is geometry performed on the surface of a sphere.  
Menelaus of Alexandria established a basis for spherical trigonometry nearly two thousand years ago.
Abu al-Bīrūnī and John Napier (among others) provided equations and tables that have been depended
upon for [global navigation](docs/Global_Navigation.md) across deserts and oceans for
many hundreds of years.

Vector geometry is geometry performed using vectors.  
Vector geometry is relatively new compared to spherical geometry,
being only a few hundred years old.
It is widely used in computer systems and it can also be used to
perform measurements on the surface of a sphere, see:
[n-vector](http://www.navlab.net/nvector/).

## Design

Points on the surface of a sphere are represented by 3D vectors with x, y and z
coordinates see Figure 1.

![Spherical Vector Coordinates](docs/images/ECEF_coordinates.png)  
*Figure 1 Spherical Vector Coordinates*

All distances and angles on the surface of a sphere are measured in radians.  
Physical distances can be calculated by multiplying by the radius of the sphere.

Functions are provided to convert latitude/longitude coordinates into
spherical vectors and back again.  
Latitude and longitude are measured in degrees.  
Note: when representing points on the Earth's surface, the 3D vector coordinates
are in the standard WGS84 [ECEF](https://en.wikipedia.org/wiki/ECEF)
orientation with the z axis between the North and South poles, see Figure 1.

## License

`via-sphere` is provided under a MIT license, see [LICENSE](LICENSE.txt).

Contact <sphere@via-technology.aero> for more information.
