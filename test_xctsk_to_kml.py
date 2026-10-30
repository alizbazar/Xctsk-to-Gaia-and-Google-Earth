import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from geographiclib.geodesic import Geodesic
from xctsk_to_kml import Turnpoint, distance, optimize_path, read_task, write_kml


class ConverterTests(unittest.TestCase):
    def test_two_circles_have_known_shortest_distance(self):
        for geod in (Geodesic.WGS84, Geodesic(6371000, 0)):
            points = [Turnpoint("A", 39, -111, 100), Turnpoint("B", 39.1, -111, 400)]
            route, length = optimize_path(points, geod, restarts=3)
            expected = distance(geod, points[0].center, points[1].center) - 500
            self.assertAlmostEqual(length, expected, delta=0.01)
            for p, crossing in zip(points, route):
                self.assertAlmostEqual(distance(geod, p.center, crossing), p.radius, delta=0.001)

    def test_center_start(self):
        points = [Turnpoint("A", 39, -111, 100), Turnpoint("B", 39.1, -111, 400)]
        route, length = optimize_path(points, Geodesic.WGS84, start="center", restarts=2)
        self.assertEqual(route[0], points[0].center)
        self.assertAlmostEqual(length, distance(Geodesic.WGS84, *[p.center for p in points]) - 400, delta=0.01)

    def test_zero_radius_and_takeoff(self):
        points = [Turnpoint("Launch", 38, -111, 100, kind="TAKEOFF"),
                  Turnpoint("A", 39, -111, 0), Turnpoint("B", 39.1, -111, 0)]
        route, length = optimize_path(points, Geodesic.WGS84)
        self.assertEqual(route, [p.center for p in points[1:]])
        self.assertAlmostEqual(length, distance(Geodesic.WGS84, *route))

    def test_bent_path_visits_middle_boundary(self):
        points = [Turnpoint("A", 0, 0, 0), Turnpoint("B", 0.1, 0.1, 1000),
                  Turnpoint("C", 0, 0.2, 0)]
        route, length = optimize_path(points, Geodesic.WGS84, restarts=3)
        self.assertAlmostEqual(route[1][1], 0.1, delta=0.00001)
        self.assertLess(route[1][0], 0.1)
        self.assertAlmostEqual(distance(Geodesic.WGS84, points[1].center, route[1]), 1000, delta=0.001)
        self.assertLess(length, sum(distance(Geodesic.WGS84, a.center, b.center)
                                    for a, b in zip(points, points[1:])))

    def test_input_validation(self):
        task = {"version": 1, "taskType": "CLASSIC", "turnpoints": [
            {"waypoint": {"name": "A", "lat": 0, "lon": 0}, "radius": 100},
            {"waypoint": {"name": "B", "lat": 1, "lon": 1}, "radius": 400}]}
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "task.xctsk"
            for change in ({"version": 2}, {"goal": {"type": "LINE"}},
                           {"earthModel": "unknown"}, {"turnpoints": []}):
                path.write_text(json.dumps(task | change))
                with self.assertRaises(ValueError):
                    read_task(path)
            task["turnpoints"][0]["radius"] = -1
            path.write_text(json.dumps(task))
            with self.assertRaises(ValueError):
                read_task(path)

    def test_kml_geometry_and_escaping(self):
        points = [Turnpoint("A & <B>", 39, -111, 100), Turnpoint("C", 39.1, -111, 400)]
        route, length = optimize_path(points, Geodesic.WGS84, restarts=2)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "task.kml"
            write_kml(path, "Test", points, Geodesic.WGS84, "WGS84", route, length, "boundary")
            root = ET.parse(path).getroot()
            ns = {"k": "http://www.opengis.net/kml/2.2"}
            self.assertEqual(len(root.findall(".//k:Point", ns)), 2)
            self.assertEqual(len(root.findall(".//k:Polygon", ns)), 2)
            self.assertEqual(len(root.findall(".//k:LineString", ns)), 1)
            for ring in root.findall(".//k:LinearRing/k:coordinates", ns):
                values = ring.text.split()
                self.assertEqual(values[0], values[-1])
                self.assertEqual(len(values), 181)
            for p, ring in zip(points, root.findall(".//k:LinearRing/k:coordinates", ns)):
                for value in ring.text.split():
                    lon, lat, _ = map(float, value.split(","))
                    self.assertAlmostEqual(distance(Geodesic.WGS84, p.center, (lat, lon)), p.radius, delta=0.001)
            self.assertIn("A & <B>", root.find(".//k:Folder/k:Placemark/k:name", ns).text)


if __name__ == "__main__":
    unittest.main()
