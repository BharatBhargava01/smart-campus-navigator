"""Geodesy helpers: great-circle distance and a local metric projection.

The campus spans < 1 km, so an equirectangular projection centred on the
campus is accurate to well under 0.1 % — good enough for snapping and
plotting. Edge lengths themselves use the haversine formula.
"""
from __future__ import annotations

import math

EARTH_RADIUS_M = 6_371_008.8

# Approximate campus centre (South Asian University, Maidan Garhi, New Delhi)
REF_LAT = 28.4850
REF_LON = 77.2000
_M_PER_DEG_LAT = math.pi * EARTH_RADIUS_M / 180.0
_M_PER_DEG_LON = _M_PER_DEG_LAT * math.cos(math.radians(REF_LAT))


def haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance in metres."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(a))


def to_xy(lat: float, lon: float) -> tuple[float, float]:
    """Project (lat, lon) to local (x east, y north) metres."""
    return (lon - REF_LON) * _M_PER_DEG_LON, (lat - REF_LAT) * _M_PER_DEG_LAT


def to_latlon(x: float, y: float) -> tuple[float, float]:
    return REF_LAT + y / _M_PER_DEG_LAT, REF_LON + x / _M_PER_DEG_LON


def project_point_on_segment(p, a, b):
    """Closest point on segment ab to p (all (x, y)). Returns (point, t, dist)."""
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    qx, qy = ax + t * dx, ay + t * dy
    return (qx, qy), t, math.hypot(px - qx, py - qy)
