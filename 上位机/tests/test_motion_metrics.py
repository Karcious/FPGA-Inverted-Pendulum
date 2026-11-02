import math
import contextlib
import io
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest

from dbg_host import cmd_capture
from motion_metrics import compute_motion_metrics, format_motion_metrics

class MotionMetricsTests(unittest.TestCase):
    def test_full_window_rate_slew_reversal_saturation_and_delta(self):
        result = compute_motion_metrics(
            adc=[100] * 10,
            delta=[0, 1, -1, 2, -2, 0, 1, -1, 3, -3],
            pos=list(range(10)),
            duty=[0, 1, -1, 0, 2, -2, -2, 0, 10, 10],
            sample_hz=10,
            center=100,
            duty_limit=10,
            steady_start_s=0.5,
        )
        full = result["full"]
        self.assertAlmostEqual(full["position_drift_rate"], 10.0)
        self.assertAlmostEqual(full["duty_slew_rms"], math.sqrt(130 / 9))
        self.assertEqual(full["duty_reversal_count"], 4)
        self.assertAlmostEqual(full["duty_reversals_per_s"], 4.0)
        self.assertAlmostEqual(full["duty_sat_longest_ms"], 200.0)
        self.assertAlmostEqual(full["delta_rms"], math.sqrt(3))
        self.assertEqual(full["delta_p95_abs"], 3.0)
        self.assertEqual(full["delta_max_abs"], 3.0)

    def test_steady_window_boundary_and_format(self):
        result = compute_motion_metrics(
            adc=[110] * 5 + [100] * 5,
            delta=[1] * 10,
            pos=list(range(10)),
            duty=[2] * 10,
            sample_hz=10,
            center=100,
            duty_limit=10,
            steady_start_s=0.5,
        )
        steady = result["steady"]
        self.assertEqual(steady["samples"], 5)
        self.assertEqual(steady["angle_rms"], 0.0)
        self.assertAlmostEqual(steady["position_drift_rate"], 10.0)
        self.assertEqual(steady["duty_slew_rms"], 0.0)
        text = "\n".join(format_motion_metrics(result))
        self.assertIn("稳态窗口", text)
        self.assertIn("漂移速率", text)
        self.assertIn("delta RMS", text)

    def test_short_capture_has_no_steady_window(self):
        result = compute_motion_metrics([1], [0], [0], [0], 10, 1, 10, 1.5)
        self.assertIsNone(result["steady"])
        self.assertIn("数据不足", "\n".join(format_motion_metrics(result)))

    def test_serial_capture_prints_and_saves_metrics(self):
        class FakeLink:
            def ping(self):
                return 0x0121

            def get_param(self, address):
                return {11: 1024, 18: 100, 21: 0}.get(address, 0)

            def arm_capture(self, *args):
                return 0

            def wait_done(self, timeout):
                return True

            def read_capture(self):
                return [[100, index % 2, index, (-1) ** index * 10]
                        for index in range(1600)]

            def cap_status(self):
                return 2, 0

        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "capture.csv")
            args = SimpleNamespace(timeout=2, ch=0, edge=2, level=0, pre=0,
                                   trigger_cmd=False, save=path, no_plot=True)
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                cmd_capture(FakeLink(), args)
            self.assertIn("串口采集指标", output.getvalue())
            self.assertIn("漂移速率", output.getvalue())
            metadata = json.loads(Path(path + ".json").read_text(encoding="utf-8"))
            self.assertIn("motion_metrics", metadata)
            self.assertAlmostEqual(
                metadata["motion_metrics"]["full"]["position_drift_rate"], 1000)

if __name__ == "__main__":
    unittest.main()
