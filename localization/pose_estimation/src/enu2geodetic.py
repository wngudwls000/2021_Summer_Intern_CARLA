import numpy as np
from numpy import radians, sin, cos, tan, arctan as atan, hypot, degrees, arctan2 as atan2, sqrt, pi, vectorize
from numpy import hypot, cos, sin, arctan2 as atan2, radians, pi, asarray

from math import radians, cos, sin, pi

class Ellipsoid:
    def __init__(self, model: str = "wgs84"):
        if model == "wgs84":
            """https://en.wikipedia.org/wiki/World_Geodetic_System#WGS84"""
            self.semimajor_axis = 6378137.0
            self.semiminor_axis = 6378137.0
        else:
            raise NotImplementedError(
                "{} model not implemented, let us know and we will add it (or make a pull request)".format(model)
            )

def enu2uvw(east: float, north: float, up: float, lat0: float, lon0: float, deg: bool = True):
    if deg:
        lat0 = radians(lat0)
        lon0 = radians(lon0)

    t = cos(lat0) * up - sin(lat0) * north
    w = sin(lat0) * up + cos(lat0) * north

    u = cos(lon0) * t - sin(lon0) * east
    v = sin(lon0) * t + cos(lon0) * east

    return u, v, w

def enu2ecef(
    e1: float, n1: float, u1: float, lat0: float, lon0: float, h0: float, ell: Ellipsoid = None, deg: bool = True
):

    x0, y0, z0 = geodetic2ecef(lat0, lon0, h0, ell, deg=deg)
    dx, dy, dz = enu2uvw(e1, n1, u1, lat0, lon0, deg=deg)

    return x0 + dx, y0 + dy, z0 + dz

def ecef2geodetic(x: float, y: float, z: float, ell: Ellipsoid = None, deg: bool = True):
    if vectorize is not None:
        fun = vectorize(ecef2geodetic_point)
        return fun(x, y, z, ell, deg)
    else:
        return ecef2geodetic_point(x, y, z, ell, deg)

def enu2geodetic(e: float, n: float, u: float, lat0: float, lon0: float, h0: float, ell: Ellipsoid = None, deg: bool = True):

    x, y, z = enu2ecef(e, n, u, lat0, lon0, h0, ell, deg=deg)

    return ecef2geodetic(x, y, z, ell, deg=deg)

def geodetic2ecef(lat: float, lon: float, alt: float, ell: Ellipsoid = None, deg: bool = True):
    lat, ell = sanitize(lat, ell, deg)
    if deg:
        lon = radians(lon)

    # radius of curvature of the prime vertical section
    N = ell.semimajor_axis ** 2 / sqrt(ell.semimajor_axis ** 2 * cos(lat) ** 2 + ell.semiminor_axis ** 2 * sin(lat) ** 2)
    # Compute cartesian (geocentric) coordinates given  (curvilinear) geodetic
    # coordinates.
    x = (N + alt) * cos(lat) * cos(lon)
    y = (N + alt) * cos(lat) * sin(lon)
    z = (N * (ell.semiminor_axis / ell.semimajor_axis) ** 2 + alt) * sin(lat)

    return x, y, z

def sanitize(lat: float, ell: Ellipsoid, deg: bool):
    if ell is None:
        ell = Ellipsoid()
    if asarray is not None:
        lat = asarray(lat)
    if deg:
        lat = radians(lat)

    try:
        if (abs(lat) > pi / 2).any():
            raise ValueError("-pi <= latitude <= pi")
    except (AttributeError, TypeError):
        if abs(lat) > pi / 2:
            raise ValueError("-pi <= latitude <= pi")

    return lat, ell

def ecef2geodetic_point(x: float, y: float, z: float, ell: Ellipsoid = None, deg: bool = True):
    if ell is None:
        ell = Ellipsoid()

    r = sqrt(x ** 2 + y ** 2 + z ** 2)

    E = sqrt(ell.semimajor_axis ** 2 - ell.semiminor_axis ** 2)

    # eqn. 4a
    u = sqrt(0.5 * (r ** 2 - E ** 2) + 0.5 * sqrt((r ** 2 - E ** 2) ** 2 + 4 * E ** 2 * z ** 2))

    Q = hypot(x, y)

    huE = hypot(u, E)

    # eqn. 4b
    try:
        Beta = atan(huE / u * z / hypot(x, y))
    except ZeroDivisionError:
        if z >= 0:
            Beta = pi / 2
        else:
            Beta = -pi / 2

    # eqn. 13
    eps = ((ell.semiminor_axis * u - ell.semimajor_axis * huE + E ** 2) * sin(Beta)) / (
        ell.semimajor_axis * huE * 1 / cos(Beta) - E ** 2 * cos(Beta)
    )

    Beta += eps
    # %% final output
    lat = atan(ell.semimajor_axis / ell.semiminor_axis * tan(Beta))

    lon = atan2(y, x)

    # eqn. 7
    alt = hypot(z - ell.semiminor_axis * sin(Beta), Q - ell.semimajor_axis * cos(Beta))

    # inside ellipsoid?
    inside = x ** 2 / ell.semimajor_axis ** 2 + y ** 2 / ell.semimajor_axis ** 2 + z ** 2 / ell.semiminor_axis ** 2 < 1
    if inside:
        alt = -alt

    if deg:
        lat = degrees(lat)
        lon = degrees(lon)

    return lat, lon, alt

if __name__ == '__main__':
    xyz = enu2geodetic(0.5, 0.5, 0.0, 0.0, 0.0, 0.0)
    print(xyz)

    #0.000003593