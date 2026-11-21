import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


detector_path = str(Path(sys.argv.pop(1)).resolve())
colors_path = str(Path(sys.argv.pop(1)).resolve())


class CameraDetectorIntegration(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.calibration_path = self.root / "camera.yaml"
        self.calibration_path.write_text(
            "%YAML:1.0\n"
            "distortion_model: plumb_bob\n"
            "image_width: 128\n"
            "image_height: 96\n"
            "camera_matrix: !!opencv-matrix\n"
            "  rows: 3\n"
            "  cols: 3\n"
            "  dt: d\n"
            "  data: [80.,0.,64.,0.,80.,48.,0.,0.,1.]\n"
            "distortion_coefficients: [0.,0.,0.,0.,0.]\n"
        )
        for index in range(3):
            self.write_frame(index, 40 + index * 10)

    def write_frame(self, index, center_x=60, width=128, height=96):
        pixels = bytearray(width * height * 3)
        for y in range(33, 64):
            start = (y * width + center_x - 15) * 3
            pixels[start:start + 31 * 3] = bytes((255, 0, 0)) * 31
        (self.root / f"frame{index:02d}.ppm").write_bytes(
            f"P6\n{width} {height}\n255\n".encode() + pixels)

    def run_detector(self, correction=False, **overrides):
        parameters = {
            "video_path": str(self.root / "frame%02d.ppm"),
            "show_window": "false",
        }
        if correction:
            parameters["camera_calibration_path"] = str(self.calibration_path)
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

    def test_raw_video_preserves_previous_detection_behavior(self):
        result = self.run_detector()
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn("lens_correction=disabled", result.stdout)
        self.assertIn("processed_frames=3, last_detections=1", result.stdout)
        self.assertIn("center_px=[60.00, 48.00]", result.stdout)
        self.assertNotIn("surface_point_m=", result.stdout)

    def test_zero_distortion_processes_all_frames_with_unchanged_centres(self):
        inputs = {p: p.read_bytes() for p in self.root.iterdir()}
        result = self.run_detector(correction=True)
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn("lens_correction=enabled", result.stdout)
        self.assertIn("reason=video input ended, processed_frames=3, last_detections=1", result.stdout)
        self.assertIn("center_px=[60.00, 48.00]", result.stdout)
        self.assertNotIn("surface_point_m=", result.stdout)
        for path, contents in inputs.items():
            self.assertEqual(path.read_bytes(), contents)

    def test_nonzero_correction_changes_detected_centres(self):
        text = self.calibration_path.read_text().replace("[0.,0.,0.,0.,0.]", "[0.5,0.,0.,0.,0.]")
        self.calibration_path.write_text(text)
        self.write_frame(2, center_x=100)
        raw = self.run_detector()
        corrected = self.run_detector(correction=True)
        self.assertEqual(raw.returncode, 0, raw.stdout)
        self.assertEqual(corrected.returncode, 0, corrected.stdout)
        self.assertIn("center_px=[100.00, 48.00]", raw.stdout)
        self.assertNotIn("center_px=[100.00, 48.00]", corrected.stdout)
        self.assertIn("processed_frames=3, last_detections=1", corrected.stdout)

    def test_dimension_mismatch_stops_processing(self):
        self.write_frame(0, width=160)
        result = self.run_detector(correction=True)
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn("matching the camera calibration dimensions", result.stdout)
        self.assertNotIn("Frame processed", result.stdout)

    def test_measurement_mode_requires_a_window(self):
        result = self.run_detector(measurement_mode="true")
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn("measurement_mode requires show_window=true", result.stdout)
        self.assertNotIn("Video input opened", result.stdout)

    def test_measurement_mode_rejects_an_empty_snapshot_directory(self):
        result = self.run_detector(measurement_mode="true", show_window="true", snapshot_directory="''")
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn("nonempty snapshot_directory", result.stdout)
        self.assertNotIn("Video input opened", result.stdout)

    def test_invalid_calibration_fails_before_opening_video(self):
        valid = self.calibration_path.read_text()
        for contents in [
            valid.replace("plumb_bob", "equidistant"),
            valid.replace("image_width: 128", "image_width: 0"),
            valid.replace("[80.,0.,64.,0.,80.,48.,0.,0.,1.]", "[0.,0.,64.,0.,80.,48.,0.,0.,1.]"),
            valid.replace("[0.,0.,0.,0.,0.]", "[0.,0.,.Nan,0.,0.]"),
            "not a calibration file",
        ]:
            with self.subTest(contents=contents):
                self.calibration_path.write_text(contents)
                result = self.run_detector(correction=True)
                self.assertNotEqual(result.returncode, 0, result.stdout)
                self.assertNotIn("Video input opened", result.stdout)
        self.calibration_path.unlink()
        result = self.run_detector(correction=True)
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn("Could not open camera calibration", result.stdout)
        self.assertNotIn("Video input opened", result.stdout)


if __name__ == "__main__":
    unittest.main(verbosity=2)
