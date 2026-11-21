import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import unittest


detector_path = str(Path(sys.argv.pop(1)).resolve())
colors_path = str(Path(sys.argv.pop(1)).resolve())


class ImageDetectorIntegration(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.image_path = self.root / "input.ppm"
        self.output_path = self.root / "output.png"
        self.calibration_path = self.root / "plane.yaml"
        self.calibration_path.write_text(
            "%YAML:1.0\n"
            "frame_id: base_link\n"
            "image_coordinates: undistorted_pixels\n"
            "image_width: 640\n"
            "image_height: 480\n"
            "surface_z_metres: 0.025\n"
            "maximum_fit_error_metres: 0.000001\n"
            "image_points: [[100.,80.],[500.,80.],[500.,320.],[100.,320.]]\n"
            "robot_points_metres: [[0.1,-0.1],[0.3,-0.1],[0.3,0.02],[0.1,0.02]]\n"
        )
        self.write_image([(300, 200, (255, 0, 0))])

    def write_image(self, regions, width=640, height=480):
        pixels = bytearray(width * height * 3)
        for center_x, center_y, color in regions:
            for y in range(center_y - 15, center_y + 16):
                start = (y * width + center_x - 15) * 3
                pixels[start:start + 31 * 3] = bytes(color) * 31
        self.image_path.write_bytes(f"P6\n{width} {height}\n255\n".encode() + pixels)

    def run_detector(self, conversion=False, **overrides):
        parameters = {
            "image_path": str(self.image_path),
            "output_path": str(self.output_path),
        }
        if conversion:
            parameters.update({
                "plane_calibration_path": str(self.calibration_path),
                "image_coordinates": "undistorted_pixels",
            })
        parameters.update(overrides)
        arguments = [detector_path, "--ros-args", "--params-file", colors_path]
        for name, value in parameters.items():
            arguments.extend(["-p", f"{name}:={value}"])
        environment = dict(os.environ)
        environment.update({
            "ROS_DOMAIN_ID": "229",
            "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
            "RCUTILS_COLORIZED_OUTPUT": "0",
        })
        return subprocess.run(arguments, env=environment, text=True,
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=5)

    def metric_points(self, output):
        return [tuple(map(float, match)) for match in re.findall(
            r"surface_point_m=\[([^,]+), ([^,]+), ([^\]]+)\]", output)]

    def assert_failed_without_output(self, result, reason):
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn(reason, result.stdout)
        self.assertNotIn("surface_point_m=", result.stdout)
        self.assertFalse(self.output_path.exists())

    def test_pixel_only_operation_preserves_input(self):
        original = self.image_path.read_bytes()
        result = self.run_detector()
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn("center_px=[300.00, 200.00]", result.stdout)
        self.assertNotIn("surface_point_m=", result.stdout)
        self.assertTrue(self.output_path.exists())
        self.assertEqual(self.image_path.read_bytes(), original)

    def test_reports_independent_known_points_for_multiple_regions(self):
        self.write_image([(300, 200, (255, 0, 0)), (200, 140, (0, 255, 0))])
        result = self.run_detector(conversion=True)
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn("frame_id=base_link", result.stdout)
        self.assertEqual(self.metric_points(result.stdout),
                         [(0.15, -0.07, 0.025), (0.2, -0.04, 0.025)])
        self.assertTrue(self.output_path.exists())

    def test_rejects_outside_region_without_losing_valid_detection(self):
        self.write_image([(30, 30, (255, 0, 0)), (300, 200, (0, 255, 0))])
        result = self.run_detector(conversion=True)
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn("Detection 1: color=red, coordinate conversion rejected", result.stdout)
        self.assertIn("outside the calibrated region", result.stdout)
        self.assertEqual(self.metric_points(result.stdout), [(0.2, -0.04, 0.025)])
        self.assertTrue(self.output_path.exists())

    def test_all_outside_regions_produce_no_metric_points(self):
        self.write_image([(30, 30, (255, 0, 0))])
        result = self.run_detector(conversion=True)
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn("coordinate conversion rejected", result.stdout)
        self.assertEqual(self.metric_points(result.stdout), [])
        self.assertTrue(self.output_path.exists())

    def test_requires_explicit_undistorted_input(self):
        result = self.run_detector(plane_calibration_path=str(self.calibration_path))
        self.assert_failed_without_output(result, "requires image_coordinates=undistorted_pixels")
        for value in ["raw_pixels", "normalized_coordinates", "typo"]:
            with self.subTest(image_coordinates=value):
                result = self.run_detector(conversion=True, image_coordinates=value)
                self.assert_failed_without_output(result, "image_coordinates")

    def test_rejects_unknown_coordinates_without_calibration(self):
        result = self.run_detector(image_coordinates="typo")
        self.assert_failed_without_output(result, "must be raw_pixels or undistorted_pixels")

    def test_invalid_calibration_fails_before_annotation(self):
        valid = self.calibration_path.read_text()
        cases = [
            (valid.replace("frame_id: base_link", "frame_id: camera"), "requires frame_id=base_link"),
            (valid.replace("image_coordinates: undistorted_pixels", "image_coordinates: raw_pixels"),
             "requires frame_id=base_link"),
            (valid.replace("0.000001", "-1.0"), "positive fit-error limit"),
            (valid.replace("[[100.,80.],[500.,80.],[500.,320.],[100.,320.]]",
                           "[[100.,80.],[200.,80.],[300.,80.],[400.,80.]]"), "geometry is degenerate"),
            ("not a calibration file", "Invalid plane calibration file"),
        ]
        for contents, reason in cases:
            with self.subTest(reason=reason):
                self.calibration_path.write_text(contents)
                result = self.run_detector(conversion=True)
                self.assert_failed_without_output(result, reason)
        self.calibration_path.unlink()
        result = self.run_detector(conversion=True)
        self.assert_failed_without_output(result, "Could not open plane calibration")

    def test_dimension_mismatch_fails_even_without_detections(self):
        self.write_image([], width=320, height=240)
        result = self.run_detector(conversion=True)
        self.assert_failed_without_output(result, "dimensions must match the plane calibration")

    def test_empty_image_succeeds_without_metric_points(self):
        self.write_image([])
        result = self.run_detector(conversion=True)
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn("detections=0", result.stdout)
        self.assertEqual(self.metric_points(result.stdout), [])
        self.assertTrue(self.output_path.exists())

    def test_output_cannot_replace_input_or_calibration_through_aliases(self):
        for source in [self.image_path, self.calibration_path]:
            original = source.read_bytes()
            for alias_type in ["direct", "symbolic", "hard"]:
                with self.subTest(source=source.name, alias_type=alias_type):
                    alias = self.root / "alias.png"
                    if alias_type == "symbolic":
                        alias.symlink_to(source)
                    elif alias_type == "hard":
                        os.link(source, alias)
                    target = source if alias_type == "direct" else alias
                    result = self.run_detector(conversion=True, output_path=str(target))
                    self.assertNotEqual(result.returncode, 0, result.stdout)
                    self.assertIn("output_path must differ", result.stdout)
                    self.assertNotIn("surface_point_m=", result.stdout)
                    self.assertEqual(source.read_bytes(), original)
                    if alias.exists():
                        alias.unlink()

    def configure_height_conversion(self):
        self.calibration_path.write_text(
            "%YAML:1.0\nframe_id: base_link\nimage_coordinates: undistorted_pixels\n"
            "image_width: 640\nimage_height: 480\nsurface_z_metres: -0.003\n"
            "maximum_fit_error_metres: 0.000001\n"
            "image_points: [[100.,80.],[500.,80.],[500.,320.],[100.,320.]]\n"
            "robot_points_metres: [[-0.22,0.16],[0.18,0.16],[0.18,-0.08],[-0.22,-0.08]]\n"
        )
        camera_path = self.root / "camera.yaml"
        camera_path.write_text(
            "%YAML:1.0\ndistortion_model: plumb_bob\nimage_width: 640\nimage_height: 480\n"
            "camera_matrix: !!opencv-matrix\n  rows: 3\n  cols: 3\n  dt: d\n"
            "  data: [600.,0.,320.,0.,600.,240.,0.,0.,1.]\n"
            "distortion_coefficients: [0.,0.,0.,0.,0.]\n"
        )
        return camera_path

    def test_height_conversion_changes_xy_and_adds_table_height(self):
        camera_path = self.configure_height_conversion()
        result = self.run_detector(conversion=True,
                                   camera_calibration_path=str(camera_path),
                                   object_height_metres=0.0297)
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn("mode=known_height", result.stdout)
        points = self.metric_points(result.stdout)
        self.assertEqual(len(points), 1, result.stdout)
        self.assertAlmostEqual(points[0][0], -0.01901, places=6)
        self.assertAlmostEqual(points[0][1], 0.03802, places=6)
        self.assertAlmostEqual(points[0][2], 0.0267, places=6)

    def test_height_conversion_requires_both_calibration_files(self):
        result = self.run_detector(conversion=True, object_height_metres=0.0297)
        self.assert_failed_without_output(result, "requires camera_calibration_path")
        camera_path = self.configure_height_conversion()
        result = self.run_detector(camera_calibration_path=str(camera_path), object_height_metres=0.0297)
        self.assert_failed_without_output(result, "requires camera_calibration_path")

    def test_height_conversion_rejects_raw_pixels_and_invalid_camera_before_annotation(self):
        camera_path = self.configure_height_conversion()
        result = self.run_detector(conversion=True, camera_calibration_path=str(camera_path),
                                   image_coordinates="raw_pixels", object_height_metres=0.0297)
        self.assert_failed_without_output(result, "requires image_coordinates=undistorted_pixels")
        camera_path.write_text("%YAML:1.0\ndistortion_model: wrong\n")
        result = self.run_detector(conversion=True, camera_calibration_path=str(camera_path),
                                   object_height_metres=0.0297)
        self.assert_failed_without_output(result, "requires distortion_model=plumb_bob")

    def test_height_conversion_preserves_camera_file_through_output_aliases(self):
        camera_path = self.configure_height_conversion()
        original = camera_path.read_bytes()
        alias = self.root / "camera_alias.png"
        os.link(camera_path, alias)
        result = self.run_detector(conversion=True, camera_calibration_path=str(camera_path),
                                   object_height_metres=0.0297, output_path=str(alias))
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn("output_path must differ from camera_calibration_path", result.stdout)
        self.assertEqual(camera_path.read_bytes(), original)

    def test_height_conversion_rejects_outside_workspace_without_metric_output(self):
        camera_path = self.configure_height_conversion()
        self.write_image([(30, 30, (255, 0, 0))])
        result = self.run_detector(conversion=True, camera_calibration_path=str(camera_path),
                                   object_height_metres=0.0297)
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn("outside the calibrated workspace", result.stdout)
        self.assertEqual(self.metric_points(result.stdout), [])

    def test_height_conversion_rejects_negative_height(self):
        result = self.run_detector(object_height_metres=-0.01)
        self.assert_failed_without_output(result, "must be finite and nonnegative")


if __name__ == "__main__":
    unittest.main(verbosity=2)
