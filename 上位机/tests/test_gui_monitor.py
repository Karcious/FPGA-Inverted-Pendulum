import unittest
from gui_monitor import StabilityStats, balanced, error_series, q16, window_error

class TestErrors(unittest.TestCase):
    def test_angle_and_position_scaling(self):
        params = [0] * 51
        params[18] = 2048
        params[13] = 0
        angle, position = error_series([[3072, 0, 1040, 0]], params)
        self.assertAlmostEqual(angle[0], 90.0)
        self.assertAlmostEqual(position[0], 360.0)

    def test_position_target_is_q16_signed(self):
        params = [0] * 51
        params[18] = 2048
        params[13] = (-520 * 65536) & 0xFFFFFFFF
        angle, position = error_series([[2048, 0, 0, 0]], params)
        self.assertAlmostEqual(angle[0], 0.0)
        self.assertAlmostEqual(position[0], 180.0)
        self.assertAlmostEqual(q16(params[13]), -520)

    def test_window_uses_board_time(self):
        times = [0.0, 0.5, 1.0, 1.5, 2.0]
        result = window_error([1.0, 1.0, 1.0, 1.0, 1.0], 2, times, 1.0)
        self.assertEqual(result['count'], 3)
        self.assertAlmostEqual(result['duration'], 1.0)
        self.assertAlmostEqual(result['rms'], 1.0)

    def test_window_uses_tail_for_complete_capture(self):
        values = [1.0] * 20
        result = window_error(values, 10, None, 1.5)
        self.assertEqual(result['count'], 15)
        self.assertAlmostEqual(result['duration'], 1.4)

    def test_rms_and_mean_abs_differ_on_signs(self):
        result = window_error([1.0, -1.0], 1, None, 5)
        self.assertAlmostEqual(result['rms'], 1.0)
        self.assertAlmostEqual(result['mean_abs'], 1.0)

    def test_window_needs_two_samples(self):
        self.assertIsNone(window_error([1.0], 50, [0.0], 5))
        self.assertIsNone(window_error([], 50, None, 5))
        self.assertIsNone(window_error([1.0, 2.0], 0, None, 5))

    def test_mismatched_time_axis(self):
        with self.assertRaises(ValueError):
            window_error([1.0, 2.0], 50, [0.0], 5)

    def test_balance_gate(self):
        params = [0] * 51
        params[16] = 3
        self.assertTrue(balanced(params, 4))
        self.assertFalse(balanced(params, 4 | 8))
        self.assertFalse(balanced(params, 4 | 16))
        self.assertFalse(balanced(params, 3))
        params[16] = 2
        self.assertFalse(balanced(params, 4))

class TestStability(unittest.TestCase):

    def steps(self, stats, pairs, start=0.0):
        for i, (error, ok) in enumerate(pairs):
            stats.observe(start + i * 0.1, error, ok)

    def test_delay_is_measured_to_run_start(self):
        stats = StabilityStats()
        self.steps(stats, [(9.0, False), (9.0, True)] + [(0.4, True)] * 15)

        self.assertEqual(stats.attempts, 1)
        self.assertEqual(stats.successes, 1)
        self.assertAlmostEqual(stats.total_delay, 0.2)
        self.assertAlmostEqual(stats.mean_delay, 0.2)
        self.assertAlmostEqual(stats.longest, 1.3)

    def test_longest_keeps_running(self):
        stats = StabilityStats()
        self.steps(stats, [(0.2, True)] * 21)
        self.assertAlmostEqual(stats.longest, 2.0)
        self.assertAlmostEqual(stats.current, 2.0)
        self.assertEqual(stats.attempts, 0)

    def test_constant_lean_counts_as_steady(self):

        stats = StabilityStats()
        self.steps(stats, [(2.0, True)] * 21)
        self.assertAlmostEqual(stats.center, 2.0)
        self.assertAlmostEqual(stats.longest, 2.0)
        self.assertAlmostEqual(stats.current, 2.0)

    def test_center_follows_the_mean_not_the_upright(self):
        stats = StabilityStats()
        self.steps(stats, [(-1.8, True)] * 11)
        self.assertAlmostEqual(stats.center, -1.8)
        self.assertAlmostEqual(stats.longest, 1.0)

    def test_real_wobble_leaves_the_band(self):

        stats = StabilityStats()
        self.steps(stats, [(1.5, True)] * 21)
        self.assertAlmostEqual(stats.longest, 2.0)
        stats.observe(2.1, 3.5, True)
        self.assertAlmostEqual(stats.current, 0.0)
        self.assertAlmostEqual(stats.longest, 2.0)
        self.assertAlmostEqual(stats.center, 1.5)

    def test_center_restarts_when_balance_is_lost(self):
        stats = StabilityStats()
        self.steps(stats, [(2.0, True)] * 5)
        stats.observe(0.5, 2.0, False)
        self.assertIsNone(stats.center)
        stats.observe(0.6, -2.0, True)
        self.assertAlmostEqual(stats.center, -2.0)
        self.assertAlmostEqual(stats.current, 0.0)

    def test_leaving_band_resets_current_not_longest(self):
        stats = StabilityStats()
        self.steps(stats, [(0.2, True), (0.2, True), (0.2, True), (9.0, True)])
        self.assertAlmostEqual(stats.longest, 0.2)
        self.assertAlmostEqual(stats.current, 0.0)

    def test_gap_interrupts_run(self):
        stats = StabilityStats()
        stats.observe(0.0, 0.2, True)
        stats.observe(0.1, 0.2, True)
        stats.observe(0.5, 0.2, True)
        self.assertAlmostEqual(stats.last_time, 0.5)
        self.assertAlmostEqual(stats.current, 0.0)
        self.assertAlmostEqual(stats.longest, 0.1)

    def test_already_balanced_has_no_delay(self):
        stats = StabilityStats()
        self.steps(stats, [(0.2, True)] * 11)
        self.assertEqual(stats.attempts, 0)
        self.assertEqual(stats.successes, 0)
        self.assertIsNone(stats.mean_delay)
        self.assertAlmostEqual(stats.longest, 1.0)

    def test_one_delay_per_entry(self):
        stats = StabilityStats()
        self.steps(stats, [(9.0, False), (9.0, True)] + [(0.2, True)] * 20)
        self.assertEqual(stats.attempts, 1)
        self.assertEqual(stats.successes, 1)

        self.assertAlmostEqual(stats.mean_delay, 0.2)

    def test_duplicate_timestamp_ignored(self):
        stats = StabilityStats()
        stats.observe(0.0, 0.2, True)
        stats.observe(0.0, 9.0, True)
        self.assertAlmostEqual(stats.current, 0.0)

if __name__ == '__main__':
    unittest.main()
