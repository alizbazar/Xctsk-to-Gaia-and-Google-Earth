#!/usr/bin/env python3
"""Convert XCTrack v1 cylinder tasks to Gaia-compatible KML."""
import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import random
import sys
import xml.etree.ElementTree as ET

from geographiclib.geodesic import Geodesic
from scipy.optimize import minimize


NS = "http://www.opengis.net/kml/2.2"
ET.register_namespace("", NS)


@dataclass(frozen=True)
class Turnpoint:
    name: str
    lat: float
    lon: float
    radius: float
    description: str = ""
    kind: str = ""

    @property
    def center(self):
        return self.lat, self.lon


def number(value, label, lower, upper):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be a number")
    if not math.isfinite(value) or not lower <= value <= upper:
        raise ValueError(f"{label} must be between {lower} and {upper}")
    return float(value)


def read_task(path):
    task = json.loads(Path(path).read_text(encoding="utf-8-sig"))
    if not isinstance(task, dict):
        raise ValueError("Task must be a JSON object")
    if task.get("version") != 1 or task.get("taskType") != "CLASSIC":
        raise ValueError("Only CLASSIC version 1 .xctsk JSON files are supported")
    goal = task.get("goal", {})
    if not isinstance(goal, dict) or goal.get("type", "CYLINDER") != "CYLINDER":
        raise ValueError("Line goals are not supported; this converter requires cylinder goals")
    model = task.get("earthModel", "WGS84")
    if model not in ("WGS84", "FAI_SPHERE"):
        raise ValueError(f"Unsupported earthModel: {model}")
    geod = Geodesic.WGS84 if model == "WGS84" else Geodesic(6371000, 0)
    rows = task.get("turnpoints")
    if not isinstance(rows, list) or len(rows) < 2:
        raise ValueError("Task needs at least two turnpoints")
    points = []
    for i, row in enumerate(rows):
        if not isinstance(row, dict) or not isinstance(row.get("waypoint"), dict):
            raise ValueError(f"Turnpoint {i + 1}: missing waypoint object")
        wp = row["waypoint"]
        name = wp.get("name")
        if not isinstance(name, str) or not name.strip():
            raise ValueError(f"Turnpoint {i + 1}: missing name")
        kind = row.get("type", "")
        if kind not in ("", "TAKEOFF", "SSS", "ESS") or (kind == "TAKEOFF" and i != 0):
            raise ValueError(f"Turnpoint {i + 1}: invalid type")
        points.append(Turnpoint(
            name,
            number(wp.get("lat"), f"{name} latitude", -90, 90),
            number(wp.get("lon"), f"{name} longitude", -180, 180),
            number(row.get("radius"), f"{name} radius", 0, 1000000),
            str(wp.get("description", "")), kind,
        ))
    return points, geod, model


def destination(geod, point, bearing, distance):
    result = geod.Direct(*point, bearing, distance)
    return result["lat2"], result["lon2"]


def distance(geod, a, b):
    return geod.Inverse(*a, *b)["s12"]


def path_length(geod, points):
    return sum(distance(geod, a, b) for a, b in zip(points, points[1:]))


def optimize_path(points, geod, start="boundary", restarts=16):
    """Minimize geodesic length over one boundary point per ordered cylinder.

    Angles are unconstrained and periodic. Multiple deterministic starts reduce
    local-minimum risk; this is not a certificate of a global optimum. TAKEOFF
    is a display marker only and is excluded from the task distance.
    """
    active = [p for p in points if p.kind != "TAKEOFF"]
    if len(active) < 2:
        raise ValueError("At least two non-TAKEOFF turnpoints are required")
    variable = [i for i, p in enumerate(active)
                if p.radius > 0 and not (i == 0 and start == "center")]

    def locations(angles):
        result = [p.center for p in active]
        for i, angle in zip(variable, angles):
            p = active[i]
            result[i] = destination(geod, p.center, math.degrees(angle), p.radius)
        return result

    if not variable:
        route = locations([])
        return route, path_length(geod, route)

    def objective(angles):
        return path_length(geod, locations(angles)) / 1000

    seed = []
    for i in variable:
        other = active[i + 1] if i + 1 < len(active) else active[i - 1]
        seed.append(math.radians(geod.Inverse(*active[i].center, *other.center)["azi1"]))
    rng = random.Random(20260929)
    starts = [seed] + [[rng.uniform(-math.pi, math.pi) for _ in variable]
                       for _ in range(restarts - 1)]
    candidates = []
    for initial in starts:
        fit = minimize(objective, initial, method="BFGS",
                       options={"gtol": 1e-6, "maxiter": 600})
        # Finite differences can report precision loss near convergence. Polish
        # with a derivative-free method and require a successful result.
        polished = minimize(objective, fit.x, method="Powell",
                             options={"xtol": 1e-9, "ftol": 1e-11, "maxiter": 1000})
        if polished.success and math.isfinite(polished.fun):
            candidates.append(polished)
    if not candidates:
        raise ValueError("Path optimization did not converge; no output written")
    best = min(candidates, key=lambda candidate: candidate.fun)
    route = locations(best.x)
    return route, path_length(geod, route)


def element(parent, tag, text=None, **attributes):
    node = ET.SubElement(parent, f"{{{NS}}}{tag}", attributes)
    if text is not None:
        node.text = str(text)
    return node


def coordinates(points, altitude=0):
    return " ".join(f"{lon:.9f},{lat:.9f},{altitude:g}" for lat, lon in points)


def densify(geod, route, step=500):
    result = [route[0]]
    for a, b in zip(route, route[1:]):
        leg = geod.Inverse(*a, *b)
        count = max(1, math.ceil(leg["s12"] / step))
        result.extend(destination(geod, a, leg["azi1"], leg["s12"] * j / count)
                      for j in range(1, count))
        result.append(b)
    return result


def write_kml(path, name, points, geod, model, route, length, start, segments=180,
              height_agl=0):
    number(height_agl, "Cylinder height AGL", 0, 1000000)
    root = ET.Element(f"{{{NS}}}kml")
    doc = element(root, "Document")
    element(doc, "name", name)
    explanation = (f"Earth model: {model}. Start: {start} of first non-TAKEOFF cylinder. "
                   f"Numerically optimized boundary path: {length / 1000:.3f} km. "
                   "Static geometric approximation; no timing, wind, terrain, altitude, "
                   "crossing-direction or competition-rule validation. TAKEOFF is excluded "
                   "from path distance.")
    if height_agl:
        explanation += (f" Cylinders extend from terrain to {height_agl:g} m AGL "
                        "at each boundary vertex. Path and centers are ground-clamped.")
    element(doc, "description", explanation)
    for identifier, color, width in (("cylinder", "ffff8800", 2), ("route", "ff0044ff", 4)):
        style = element(doc, "Style", id=identifier)
        line = element(style, "LineStyle")
        element(line, "color", color)
        element(line, "width", width)
        if identifier == "cylinder":
            poly = element(style, "PolyStyle")
            element(poly, "color", "22ff8800")
            element(poly, "fill", 1)
            element(poly, "outline", 1)
    centers = element(doc, "Folder")
    element(centers, "name", "Turnpoint centers")
    cylinders = element(doc, "Folder")
    element(cylinders, "name", "Cylinders")
    for i, p in enumerate(points, 1):
        label = f"{i:02d} {p.name} ({p.radius:g} m)"
        desc = f"{p.description}\nType: {p.kind or 'turnpoint'}\nRadius: {p.radius:g} m"
        marker = element(centers, "Placemark")
        element(marker, "name", label)
        element(marker, "description", desc)
        geometry = element(marker, "Point")
        element(geometry, "altitudeMode", "clampToGround")
        element(geometry, "coordinates", coordinates([p.center]))
        if p.radius == 0:
            continue
        area = element(cylinders, "Placemark")
        element(area, "name", label + " cylinder")
        element(area, "description", desc + (f"\nHeight: {height_agl:g} m AGL" if height_agl else ""))
        element(area, "styleUrl", "#cylinder")
        polygon = element(area, "Polygon")
        if height_agl:
            element(polygon, "extrude", 1)
            element(polygon, "altitudeMode", "relativeToGround")
        else:
            element(polygon, "tessellate", 1)
            element(polygon, "altitudeMode", "clampToGround")
        ring = element(element(polygon, "outerBoundaryIs"), "LinearRing")
        # Negative bearings give the exterior ring counterclockwise winding.
        circle = [destination(geod, p.center, -360 * j / segments, p.radius)
                  for j in range(segments)]
        circle.append(circle[0])
        element(ring, "coordinates", coordinates(circle, height_agl))
    line = element(doc, "Placemark")
    element(line, "name", f"Optimized path ({length / 1000:.3f} km)")
    element(line, "description", explanation)
    element(line, "styleUrl", "#route")
    geometry = element(line, "LineString")
    element(geometry, "tessellate", 1)
    element(geometry, "altitudeMode", "clampToGround")
    element(geometry, "coordinates", coordinates(densify(geod, route)))
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def main(google_earth=False):
    parser = argparse.ArgumentParser(description=(
        "Convert XCTrack v1 tasks to Google Earth KML with extruded AGL cylinders."
        if google_earth else __doc__))
    parser.add_argument("input", type=Path, help="CLASSIC v1 .xctsk file")
    parser.add_argument("-o", "--output", type=Path, help=(
        "Default: <input>-earth.kml" if google_earth else "Default: <input>.kml"))
    if google_earth:
        parser.add_argument("--height-agl", type=float, default=5000,
                            help="Cylinder height above local terrain in metres (default: 5000)")
    parser.add_argument("--start", choices=("boundary", "center"), default="boundary")
    parser.add_argument("--segments", type=int, default=180, help="Vertices per cylinder (default: 180)")
    parser.add_argument("--restarts", type=int, default=16, help="Optimizer starts (default: 16)")
    parser.add_argument("--force", action="store_true", help="Replace an existing output file")
    args = parser.parse_args()
    output = args.output or (args.input.with_name(args.input.stem + "-earth.kml")
                             if google_earth else args.input.with_suffix(".kml"))
    height_agl = args.height_agl if google_earth else 0
    try:
        if args.segments < 36 or args.restarts < 1:
            raise ValueError("Use at least 36 segments and one optimizer start")
        if google_earth:
            number(height_agl, "Cylinder height AGL", 0.001, 1000000)
        if output.resolve() == args.input.resolve():
            raise ValueError("Output must not overwrite the input task")
        if output.exists() and not args.force:
            raise ValueError(f"Output exists: {output}; use --force to replace it")
        points, geod, model = read_task(args.input)
        route, length = optimize_path(points, geod, args.start, args.restarts)
        write_kml(output, args.input.stem, points, geod, model, route, length,
                  args.start, args.segments, height_agl)
    except (OSError, ValueError) as error:
        parser.exit(2, f"Error: {error}\n")
    print(f"Created {output}: {sum(p.radius > 0 for p in points)} cylinders, "
          f"{len(points)} centers, optimized path {length / 1000:.3f} km "
          f"({model}; start={args.start})"
          + (f"; cylinders extend to {height_agl:g} m AGL." if height_agl else "."))
    return 0


if __name__ == "__main__":
    sys.exit(main())
