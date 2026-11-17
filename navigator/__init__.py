"""SAU Smart Campus Navigator.

A multi-modal route-planning agent for South Asian University (SAU) campus graph.
Supports walking, 2-wheelers, 4-wheelers, parking rules, accessibility,
uncertainty modeling, dynamic closures with online replanning, and multi-stop
tours under deadlines.
"""
from __future__ import annotations

__version__ = "1.0.0"

from .agent import NavigatorAgent, Route
from .graph import CampusGraph, Edge, Node, build_graph, campus
from .multistop import Stop, TourPlanner, TourResult, follow_with_replanning
from .problem import Closures, Profile, RouteProblem, PARK_TIME, PARKING_RULES, SPEEDS
from .uncertainty import TravelModel

__all__ = [
    "__version__",
    "NavigatorAgent",
    "Route",
    "CampusGraph",
    "Node",
    "Edge",
    "campus",
    "build_graph",
    "Profile",
    "RouteProblem",
    "Closures",
    "SPEEDS",
    "PARK_TIME",
    "PARKING_RULES",
    "Stop",
    "TourPlanner",
    "TourResult",
    "follow_with_replanning",
    "TravelModel",
]
