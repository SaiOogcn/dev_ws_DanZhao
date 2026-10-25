"""Metric course constraints only; no rendering or simulator dependency.

Bounds were traced from the supplied 5 x 5 m drawing. They are a course
configuration, not a surveyed map; align and verify them before driving.
"""
import math

OUTER = (.50, 2.92, 4.50, 4.56, .43)
INNER = (1.00, 3.42, 4.00, 4.06, .16)


def rounded_distance(x, y, bounds):
    x0, y0, x1, y1, radius = bounds
    dx = abs(x - (x0 + x1) / 2) - (x1 - x0) / 2 + radius
    dy = abs(y - (y0 + y1) / 2) - (y1 - y0) / 2 + radius
    return math.hypot(max(dx, 0), max(dy, 0)) + min(max(dx, dy), 0) - radius


class OfficialLayout:
    """Independent scalar field check used for course validation."""

    def allowed(self, x, y, margin=0.):
        if margin < 0:
            raise ValueError('margin must be nonnegative')
        if not (margin <= x <= 5 - margin and margin <= y <= 5 - margin):
            return False
        if y < 2:
            return True
        if y < 2.5:
            return 2 + margin <= x <= 3 - margin
        ring = (rounded_distance(x, y, OUTER) <= -margin and
                rounded_distance(x, y, INNER) >= margin)
        stem = 2 + margin <= x <= 3 - margin and y <= OUTER[1] - margin
        return ring or stem
