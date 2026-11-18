# Vendored GCOPTER mathematical headers

Upstream: https://github.com/ZJU-FAST-Lab/GCOPTER

Pinned revision: `e0444f6d47b84f972ced91746b05feb36ce1fd4f`.

The three files under `include/gcopter/` are unmodified copies from
`gcopter/include/gcopter/` at that revision: `minco.hpp`, `trajectory.hpp`, and
`root_finder.hpp`. Copyright Zhepei Wang; see the upstream MIT `LICENSE` and the
copyright notices in each header. No online retrieval occurs during flight.

Only `minco::MINCO_S4NU` and its polynomial root solver are used by
`native/fixed_min_snap.cpp`. XY is the seventh-degree minimum-snap solution;
the wrapper substitutes seventh-degree monotone Z transitions and globally
dilates time according to the full three-dimensional derivative extrema.
Coefficients exported to Python use normalized segment time; physical
derivatives are scaled analytically by each segment duration.

Initial travel times are `(segment_distance + mean_segment_distance) / max_velocity`.
The shared mean baseline avoids the enormous excursions and poor conditioning
caused by forcing a near-coincident waypoint into an almost-zero time interval
next to a normal interval. It does not merge distinct points, add a stop or dwell,
or impose a runtime admission rule. All times are subsequently dilated together
to meet the requested full three-dimensional speed and acceleration limits.
