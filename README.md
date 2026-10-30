# XCTrack task → Gaia KML

Convert a CLASSIC version 1 `.xctsk` JSON task into a single KML containing:

- A polygon for each nonzero-radius cylinder (180 vertices by default).
- Named center waypoints with original descriptions and radii.
- A numerically optimized path between cylinder boundaries, in task order.

## Setup and use

Python 3.9 or later:

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python xctsk_to_kml.py Route.xctsk -o Route.kml
```

Import `Route.kml` at https://www.gaiagps.com/map using **Import Data**, select
the areas, waypoints, and path, then save and sync with the mobile app.
Gaia may change colors or styles during import.

The default starts on the first cylinder's boundary. To anchor the path at
its center instead:

```sh
.venv/bin/python xctsk_to_kml.py Route.xctsk -o Route-center.kml --start center
```

Optional flags: `--segments 360` for finer circles, `--restarts 32` for more
optimizer starting points, and `--force` to replace an existing output.
The input file is never overwritten. No account or network access is needed
after dependency installation.

## Google Earth: 3D cylinders

Use the same environment and dependencies, keeping both Python converter files
in the same directory:

```sh
.venv/bin/python xctsk_to_google_earth.py Route.xctsk -o Route-earth.kml
```

Open `Route-earth.kml` in Google Earth. It contains translucent cylinder
polygons extruded from the terrain to **5,000 m above ground** by default,
plus center markers and the same ground-clamped optimized path. Tilt the view
to see their height. The original Gaia file is a separate output.

Use `--height-agl 3000` for a different height in metres. `--start center`,
`--segments`, `--restarts`, and `--force` work as in the Gaia converter.

The cylinder polygons use `extrude=1` and `relativeToGround`, with the requested
altitude on every boundary vertex. Each vertex is raised relative to terrain
beneath it, so the roof is not necessarily level. This is a visualization,
not a terrain-sampled volume guaranteeing exact AGL everywhere across its roof.
The optimized path remains on the ground because the task has no flight
altitude profile. Google Earth rendering has not been visually verified.

Reference: [Google KML altitude modes](https://developers.google.com/kml/documentation/altitudemode).

## Geometry and limitations

GeographicLib constructs the cylinder boundaries and measures geodesic legs
using the task's earth model: WGS84 by default, or FAI_SPHERE (6,371,000 m
radius). SciPy minimizes total distance over one angular position on each
cylinder boundary, using deterministic multiple starts and a derivative-free
polishing pass. This is an approximate numerical solution, not a proof of the
global shortest path. The KML path is sampled at intervals no greater than
500 m along each geodesic leg.

The optimization is purely geometric. It does not reproduce XCTrack's live
navigation, timing, crossing-direction checks, competition tolerances, or
wind/terrain/altitude constraints. Overlapping or nested cylinders can require
additional task-rule interpretation; the converter only visits their
boundaries in order. TAKEOFF markers and their cylinders are displayed but
excluded from optimization. The `--start` choice applies to the first
non-TAKEOFF turnpoint. The Gaia export is clamped to the ground rather than
assigning waypoint elevations to the flight path; the Earth export elevates
only the cylinder polygons.

Line goals, QR/version 2 tasks, non-CLASSIC tasks, and invalid coordinates or
radii are rejected explicitly. This version accepts radii up to 1,000 km.
Gaia's documented web import limit is 1,000 features per file; a cylinder
task with N positive-radius turnpoints produces 2N + 1 features.

## Verify

```sh
.venv/bin/python -m unittest -v
```

Tests check analytic shortest distances, fixed-center starts, zero radii,
TAKEOFF handling, a bent route, rejected input, and exported KML geometry.
The KML is verified locally; importing it into a signed-in Gaia account is
a separate step.

## Sources

- [XCTrack task format](https://xctrack.org/Competition_Interfaces.html)
- [Gaia supported imports](https://help.gaiagps.com/hc/en-us/articles/360052763513-Import-GPX-KML-KMZ-GeoJSON-or-FIT-Files-on-gaiagps-com)
