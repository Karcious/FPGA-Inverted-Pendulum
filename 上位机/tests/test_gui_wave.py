import math
import time
import unittest

from gui_wave import (BRAKE_SECONDS, BRAKE_SECONDS_MAX, DEFAULT_EXPRESSION, FINISH_GONE_COUNTS,
                      MAX_ACCEL, MAX_AMPLITUDE, SETTLE_ROD_DEG, SETTLE_TIMEOUT, TILT_ABORT_DEG,
                      WaveSession, brake_seconds, build_cycle, counts_of, cycle_formula,
                      degrees_of, q16, shape_value, unq16)

class Link:

    def __init__(self):
        self.reg = {0: 0x0207_0E, 16: 3, 31: 4 | (1 << 21), 64: 0, 18: 3076}
        self.writes = []

        self.row = [3076, 0, None, 0]
        self.pos = 0.0
        self.zero_seq = 0
        self.zero_gate = True

    def get_param(self, a):
        return self.reg.get(a, 0)

    def set_param(self, a, v):
        self.writes.append((a, v))
        if a == 79:
            self.zero_seq = (self.zero_seq + 1) & 0xffffff
            accept = 0x80 if self.zero_gate else 0
            self.reg[79] = (self.zero_seq << 8) | accept | (v & 7)
            return
        self.reg[a] = v & 0xFFFFFFFF
        if a == 13 and self.reg.get(16) == 2:

            self.pos = unq16(v)

    def read_live(self):
        row = list(self.row)
        if row[2] is None:
            row[2] = int(round(self.pos))
        return {'row': row}

    def addr(self, a):
        return [v for (aa, v) in self.writes if aa == a]

CFG = dict(shape='sine', amplitude_deg=90, period_s=8.0)

class TestShapes(unittest.TestCase):
    def test_sine_starts_at_zero_and_hits_the_amplitude(self):
        c = build_cycle('sine', 90, 6.0)
        self.assertEqual(c['period_ms'], 6000)
        self.assertAlmostEqual(c['counts'][0], 0.0, places=6)
        self.assertAlmostEqual(max(c['counts']), counts_of(90), places=6)
        self.assertAlmostEqual(min(c['counts']), -counts_of(90), places=6)
        self.assertTrue(c['ok'])

    def test_triangle_quota_is_rounded_but_still_sharper_than_sine(self):
        tri = build_cycle('triangle', 90, 4.0)
        sin = build_cycle('sine', 90, 4.0)

        self.assertTrue(1.0 < tri['accel'] / sin['accel'] < 4.0,
                        '三角/正弦加速度比 = %.2f' % (tri['accel'] / sin['accel']))
        self.assertAlmostEqual(sum(tri['counts']), 0.0, places=3)
        self.assertAlmostEqual(max(tri['counts']), counts_of(90), places=6)

    def test_shape_value_is_one_full_period(self):
        for name in ('sine', 'triangle'):
            with self.subTest(name=name):
                self.assertAlmostEqual(shape_value(name, 0.0), 0.0, places=6)
                self.assertAlmostEqual(shape_value(name, 1.0), 0.0, places=6)
                self.assertAlmostEqual(shape_value(name, 0.25), 1.0, places=6)
                self.assertAlmostEqual(shape_value(name, 0.75), -1.0, places=6)

    def test_accel_scales_with_period_squared(self):
        a = build_cycle('sine', 90, 4.0)['accel']
        b = build_cycle('sine', 90, 8.0)['accel']
        self.assertAlmostEqual(a / b, 4.0, places=2)

    def test_min_period_is_the_shortest_that_still_fits(self):
        bad = build_cycle('sine', 180, 6.0)
        self.assertFalse(bad['ok'])
        fixed = build_cycle('sine', 180, bad['min_period'])
        self.assertTrue(fixed['ok'])
        self.assertTrue(MAX_ACCEL * 0.9 < fixed['accel'] <= MAX_ACCEL, fixed['accel'])

    def test_rejects_input_outside_the_envelope(self):
        for args in [('sine', 0, 4.0), ('sine', MAX_AMPLITUDE + 1, 40.0),
                     ('sine', 90, 1.0), ('sine', 90, 41.0), ('saw', 90, 4.0)]:
            with self.subTest(args=args), self.assertRaises(ValueError):
                build_cycle(*args)

    def test_cosine_is_the_negated_sine_and_still_starts_at_zero(self):
        cos = build_cycle('cosine', 90, 6.0)
        sin = build_cycle('sine', 90, 6.0)

        self.assertAlmostEqual(cos['counts'][0], 0.0, places=6)
        for a, b in zip(cos['counts'], sin['counts']):
            self.assertAlmostEqual(a, -b, places=6)
        self.assertAlmostEqual(degrees_of(cos['counts'][1500]), -90.0, places=3)
        self.assertTrue(cos['ok'])

    def test_custom_sine_is_point_for_point_the_builtin_sine(self):
        builtin = build_cycle('sine', 90, 6.0)
        custom = build_cycle('custom', 90, 6.0, DEFAULT_EXPRESSION)
        self.assertEqual(len(custom['counts']), len(builtin['counts']))
        for a, b in zip(custom['counts'], builtin['counts']):
            self.assertAlmostEqual(a, b, places=6)

    def test_expression_coefficients_only_change_the_shape(self):

        plain = build_cycle('custom', 90, 6.0, 'sin(2*pi*t/T)')
        scaled = build_cycle('custom', 90, 6.0, '90*sin(2*pi*t/T)')
        for a, b in zip(plain['counts'], scaled['counts']):
            self.assertAlmostEqual(a, b, places=6)

    def test_custom_rejects_constant_and_non_closing_functions(self):
        with self.assertRaises(ValueError) as ctx:
            build_cycle('custom', 90, 6.0, '2+3')
        self.assertIn('没有来回', str(ctx.exception))

        with self.assertRaises(ValueError) as ctx:
            build_cycle('custom', 90, 6.0, 't/T')
        self.assertIn('首尾接不上', str(ctx.exception))

    def test_custom_dc_is_removed_and_reported(self):

        offset = build_cycle('custom', 90, 6.0, '2 + sin(2*pi*t/T)')
        plain = build_cycle('custom', 90, 6.0, 'sin(2*pi*t/T)')
        self.assertAlmostEqual(offset['dc'], 2.0, places=6)
        self.assertAlmostEqual(sum(offset['counts']) / len(offset['counts']), 0.0, places=6)
        for a, b in zip(offset['counts'], plain['counts']):
            self.assertAlmostEqual(a, b, places=6)
        self.assertTrue(offset['ok'])

    def test_half_sine_is_refused_because_the_seam_has_a_velocity_jump(self):

        c = build_cycle('custom', 90, 6.0, 'sin(pi*t/T)')
        self.assertFalse(c['ok'])
        self.assertGreater(c['seam'], 50.0, c['seam'])

        self.assertGreater(c['min_period'], 40.0)

    def test_custom_expression_is_sandboxed(self):
        for bad in ("__import__('os').system('dir')", 'open("x")', 'lambda t: t',
                    't.real', '[t][0]'):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                build_cycle('custom', 90, 6.0, bad)
        with self.assertRaises(ValueError):
            build_cycle('custom', 90, 6.0, 'x' * 300)

    def test_custom_goes_through_the_same_accel_gate(self):
        bad = build_cycle('custom', 180, 3.0, 'sin(2*pi*t/T)**3')
        self.assertFalse(bad['ok'])
        fixed = build_cycle('custom', 180, bad['min_period'], 'sin(2*pi*t/T)**3')
        self.assertTrue(fixed['ok'], fixed['accel'])
        self.assertTrue(MAX_ACCEL * 0.9 < fixed['accel'] <= MAX_ACCEL, fixed['accel'])

    def test_cycle_formula_spells_out_the_running_curve(self):
        self.assertIn('sin', cycle_formula('sine', 90, 8))
        self.assertIn('cos', cycle_formula('cosine', 90, 8))
        self.assertIn('tri', cycle_formula('triangle', 90, 8))
        self.assertIn('sin(2*pi*t/T)', cycle_formula('custom', 90, 8, 'sin(2*pi*t/T)'))

    def test_q16_and_degree_helpers(self):
        self.assertEqual(q16(1.0), 65536)
        self.assertEqual(q16(-1.0), 0xFFFF0000)
        self.assertEqual(q16(-1 / 65536.0), 0xFFFFFFFF)
        self.assertEqual(q16(0), 0)
        self.assertAlmostEqual(degrees_of(260), 90.0, places=9)
        self.assertAlmostEqual(counts_of(90), 260.0, places=9)
        self.assertAlmostEqual(unq16(q16(-149.5)), -149.5, places=6)
        self.assertAlmostEqual(unq16(0xFFFFFFFF), -1 / 65536.0, places=12)

class TestBegin(unittest.TestCase):
    def test_happy_path_clears_target_then_switches_mode(self):
        l, s = Link(), WaveSession()
        s.begin(l, CFG, now=0.0)
        self.assertEqual(l.addr(13), [0], '先把 mem[13] 清零，再换模式')
        self.assertEqual(l.addr(16), [2])
        self.assertTrue(s.active)
        self.assertEqual(s.state, s.RUN)
        self.assertEqual((s.center, s.phase), (0.0, 0.0))
        self.assertIn('模式 2', s.note)

    def test_preconditions_never_touch_the_mode(self):
        cases = [(0, 0x0207_0D), (64, 1), (16, 2), (31, 6), (31, 12),
                 (31, 4), (31, 4 | (1 << 21) | (1 << 24))]
        for a, v in cases:
            with self.subTest(a=a, v=v):
                l = Link()
                l.reg[a] = v
                s = WaveSession()
                with self.assertRaises(ValueError):
                    s.begin(l, CFG, now=0.0)
                self.assertEqual(l.addr(16), [], '前置条件不满足时不许动模式')
                self.assertEqual(l.addr(13), [], '也不许动目标')
                self.assertFalse(s.active)

    def test_over_accel_refused_before_any_write(self):
        l, s = Link(), WaveSession()
        with self.assertRaises(ValueError) as ctx:
            s.begin(l, dict(shape='sine', amplitude_deg=180, period_s=6.0), now=0.0)
        self.assertIn('加速度', str(ctx.exception))
        self.assertEqual(l.writes, [])

class TestRun(unittest.TestCase):
    def _start(self, **kw):
        cfg = dict(CFG)
        cfg.update(kw)
        l, s = Link(), WaveSession()
        s.begin(l, cfg, now=0.0)
        l.writes.clear()
        return l, s

    def test_writes_at_ten_ms_and_target_follows_the_phase(self):
        l, s = self._start()
        samples = []
        for i in range(301):
            out = s.poll(l, i * 0.01)
            if out:
                samples.append(out)
        self.assertTrue(295 <= len(l.addr(13)) <= 302, len(l.addr(13)))

        self.assertAlmostEqual(samples[-1]['target'], counts_of(90) * 0.7071, delta=2.0)
        self.assertIn('state', samples[-1])

    def test_actual_position_comes_from_live_telemetry(self):
        l, s = self._start()
        l.row = [3076, 0, 123, 456]
        for i in range(3):
            s.poll(l, i * 0.01)
        self.assertEqual(s.actual, 123.0)
        self.assertAlmostEqual(s.rod_deg, 0.0, places=6)

    def test_stop_brakes_then_marks_the_stop_point_as_the_new_zero(self):
        l, s = self._start()
        now = 0.0
        for _ in range(30):
            now += 0.01
            s.poll(l, now)
        s.stop(l, now=now)
        self.assertEqual(s.state, s.BRAKE)
        deadline = now + 20.0
        while s.active and now < deadline:
            now += 0.01
            s.poll(l, now)
        self.assertFalse(s.active, '收尾没结束')
        self.assertEqual(l.addr(16), [3], '切回模式 3 才能搬零点')
        self.assertIn((79, 3), l.writes, '要发地址 79 命令 3')

        self.assertLess(l.writes.index((16, 3)), l.writes.index((79, 3)))
        self.assertEqual(l.addr(13)[-1], 0)
        self.assertIn('新的 0 点', s.note)

        self.assertAlmostEqual(s.hold, s.target, delta=counts_of(1.0))

    def test_zero_command_is_retried_while_the_board_has_not_opened_the_gate(self):
        l, s = self._start()
        s.hold = 0.0
        s._begin_finish(l, now=0.0)
        l.zero_gate = False
        now = 0.0
        for _ in range(20):
            now += 0.01
            s.poll(l, now)
        self.assertGreaterEqual(len(l.addr(79)), 5, '被拒之后要继续发')

    def test_stop_reports_honestly_when_the_board_never_accepts_the_zero(self):
        l, s = self._start()
        s.hold = 0.0
        s._begin_finish(l, now=0.0)
        l.zero_gate = False
        now = 0.0
        while s.active and now < 20.0:
            now += 0.01
            s.poll(l, now)
        self.assertFalse(s.active)
        self.assertIn('没接受', s.note)
        self.assertIn('原来的', s.note)

    def test_a_rod_falling_during_finish_lets_go_instead_of_waiting_five_seconds(self):

        l, s = self._start()
        s.hold = 0.0
        s._begin_finish(l, now=0.0)
        l.zero_gate = False
        l.row[0] = 3076 + int(round((TILT_ABORT_DEG + 5) / 360.0 * 4096))
        now = 0.0
        while s.active and now < 20.0:
            now += 0.01
            s.poll(l, now)
        self.assertFalse(s.active)
        self.assertLess(now, 1.0, '杆倒了要立刻收手，不能等满 5 秒')
        self.assertEqual(s.state, s.IDLE)
        self.assertIn('没有搬成', s.note)
        self.assertIn('原来的', s.note)

        self.assertEqual(l.addr(16), [3])

    def test_an_arm_moved_away_during_finish_stops_the_zero_move(self):
        l, s = self._start()
        s.hold = 0.0
        s._begin_finish(l, now=0.0)
        l.zero_gate = False
        l.row[2] = int(FINISH_GONE_COUNTS) + 50
        now = 0.0
        while s.active and now < 20.0:
            now += 0.01
            s.poll(l, now)
        self.assertFalse(s.active)
        self.assertLess(now, 1.0)
        self.assertIn('摆臂被挪走了', s.note)
        self.assertIn('原来的', s.note)

    def test_pressing_stop_during_finish_is_an_escape_hatch(self):

        l, s = self._start()
        s.hold = 0.0
        s._begin_finish(l, now=0.0)
        l.zero_gate = False
        s.poll(l, 0.01)
        s.stop(l, now=0.02)
        self.assertFalse(s.active)
        self.assertEqual(s.state, s.IDLE)
        self.assertIn('原来的', s.note)
        self.assertIsNone(s.poll(l, 0.03), '收了手就不该再往外吐样本')

    def test_external_mode_change_deactivates_without_forcing_it_back(self):
        l, s = self._start()
        l.reg[16] = 4
        s.poll(l, 0.1)
        self.assertFalse(s.active)
        self.assertIn('模式', s.note)
        self.assertEqual(l.addr(16), [], '不抢回模式 3')

    def test_tilt_aborts_and_switches_back_to_mode_three(self):
        l, s = self._start()
        for i in range(11):
            s.poll(l, i * 0.01)
        l.row[0] = 3076 + int(round(30 / 360 * 4096))
        for i in range(11, 31):
            s.poll(l, i * 0.01)
        self.assertFalse(s.active)
        self.assertEqual(l.addr(16), [3])
        self.assertIn('起摆', s.note)

    def test_brake_seconds_is_short_enough_to_feel_immediate(self):
        self.assertTrue(0.3 <= BRAKE_SECONDS <= 1.5)

    def test_brake_time_grows_with_amplitude_so_big_waves_stop_gently(self):

        small = brake_seconds(build_cycle('sine', 90, 8.0))
        big = brake_seconds(build_cycle('sine', 180, 8.0))
        self.assertAlmostEqual(small, 70.7 * math.pi / 240, delta=0.05)
        self.assertGreater(big, small)
        self.assertAlmostEqual(big, 141.4 * math.pi / 240, delta=0.05)
        self.assertLessEqual(big, BRAKE_SECONDS_MAX)

        v0 = build_cycle('sine', 180, 8.0)['speed']
        self.assertLessEqual(v0 * math.pi / (2 * big), 120.0 + 1e-6)

    def test_stop_waits_for_the_rod_to_settle_before_switching_modes(self):

        l, s = self._start()
        now = 0.0
        for _ in range(30):
            now += 0.01
            s.poll(l, now)
        s.stop(l, now=now)

        l.row[0] = 3076 + int(round(17.6 / 360 * 4096))
        deadline = now + s.brake_seconds + SETTLE_TIMEOUT - 0.5
        held = 0
        while now < deadline:
            now += 0.01
            before = len(l.addr(13))
            s.poll(l, now)
            held += len(l.addr(13)) - before
        self.assertTrue(s.active, '杆没站住就不该收尾')
        self.assertEqual(l.addr(16), [], '门没开时不许切模式 3')
        self.assertGreater(held, 20, '等的时候必须继续按原地写目标（摆臂不能撒手）')

        l.row[0] = 3076
        while s.active and now < deadline + 5.0:
            now += 0.01
            s.poll(l, now)
        self.assertFalse(s.active, '杆站住以后要收尾')
        self.assertEqual(l.addr(16), [3])
        self.assertIn((79, 3), l.writes)
        self.assertIn('新的 0 点', s.note)

    def test_settle_gate_matches_the_board_threshold(self):

        self.assertLess(SETTLE_ROD_DEG, 60 * 360.0 / 4096)
        self.assertGreater(SETTLE_ROD_DEG, 2.0)

class TestDemoLink(unittest.TestCase):

    def test_demo_arm_follows_the_streamed_target(self):
        from gui_model import DemoLink
        l = DemoLink()
        l.set_param(16, 2)
        l.set_param(13, q16(200))
        l.read_live()
        time.sleep(.15)
        pos = l.read_live()['row'][2]
        self.assertGreater(pos, 100)
        self.assertLess(pos, 200)

    def test_demo_arm_ignores_address_13_outside_mode_two(self):
        from gui_model import DemoLink
        l = DemoLink()
        l.set_param(13, q16(200))
        self.assertLess(abs(l.read_live()['row'][2]), 20)

    def test_demo_starts_from_the_target_it_is_already_at(self):
        from gui_model import DemoLink
        l = DemoLink()
        l.set_param(13, q16(50))
        l.set_param(16, 2)
        self.assertEqual(l.read_live()['row'][2], 50)

if __name__ == '__main__':
    unittest.main()
