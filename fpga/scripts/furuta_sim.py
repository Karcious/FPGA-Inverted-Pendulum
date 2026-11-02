#!/usr/bin/env python3

import argparse
import csv
import math
import os
import sys

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

CENTER      = 3095.0
HANG        = 1017.0
SPAN_180    = CENTER - HANG
RAD_PER_CODE = math.pi / SPAN_180
CODE_PER_RAD = 1.0 / RAD_PER_CODE

ENC_CPR     = 1040.0
RAD_PER_CNT = 2.0 * math.pi / ENC_CPR

ADC_LSB_CODES = 1.0

ADC_NOISE_CODES = 0.59

R_ARM    = 0.156
L_ROD    = 0.135
L_EFF    = 0.106

M_ROD    = 0.050

L_C      = 0.070
K2       = L_EFF * L_C

TAU_STALL = 0.647
OMEGA_NL  = 61.0
TAU_C_ARM = 0.030
TAU_S_ARM = 0.075

J_ARM    = 2.9e-3

B_THETA  = 0.3

TAU_C_ROD = 0.0

CTRL_HZ    = 1000

INNER_DIV  = 1
OUTER_DIV  = 25
DUTY_FULL  = 1024
ARM_CLAMP  = 750

RAMP_STEP  = 8

class Furuta:

    def __init__(self, r=R_ARM, m=M_ROD, l_c=L_C, k2=K2,
                 J_a=J_ARM, tau_stall=TAU_STALL, omega_nl=OMEGA_NL,
                 tau_c_arm=TAU_C_ARM, tau_s_arm=TAU_S_ARM,
                 b_theta=B_THETA, tau_c_rod=TAU_C_ROD):
        self.r, self.m, self.l_c, self.k2 = r, m, l_c, k2
        self.J_a, self.tau_stall, self.omega_nl = J_a, tau_stall, omega_nl
        self.tau_c_arm, self.tau_s_arm = tau_c_arm, tau_s_arm
        self.b_theta, self.tau_c_rod = b_theta, tau_c_rod
        self.b_emf = tau_stall / omega_nl
        self.th, self.thd, self.ph, self.phd = 0.0, 0.0, 0.0, 0.0
        self.omega_eps = 0.02

    def _coulomb(self, tau_c, vel):
        return tau_c * math.tanh(vel / 0.5)

    def _stuck(self, tau_drive):
        return abs(self.phd) < self.omega_eps and abs(tau_drive) <= self.tau_s_arm

    def derivs(self, th, thd, ph, phd, duty):
        m, r, l_c, k2 = self.m, self.r, self.l_c, self.k2
        s, c = math.sin(th), math.cos(th)

        A11 = m * k2
        b1 = (m * 9.81 * l_c * s
              + m * k2 * s * c * phd * phd
              - A11 * self.b_theta * thd
              - self._coulomb(self.tau_c_rod, thd))

        tau_drive = self.tau_stall * duty / DUTY_FULL
        tau_arm = (tau_drive
                   - self.b_emf * phd
                   - self._coulomb(self.tau_c_arm, phd))

        tau_cpl = (- 2.0 * m * k2 * s * c * thd * phd
                   - m * r * l_c * s * thd * thd)

        A22 = self.J_a + m * r * r + m * k2 * s * s
        a12 = -m * r * l_c * c
        det = A11 * A22 - a12 * a12

        if self._stuck(tau_drive + tau_cpl):
            return thd, b1 / A11, phd, 0.0

        if abs(det) < 1e-18:
            det = 1e-18
        thdd = (b1 * A22 - a12 * (tau_arm + tau_cpl)) / det
        phdd = (A11 * (tau_arm + tau_cpl) - b1 * a12) / det
        return thd, thdd, phd, phdd

    def step(self, duty, dt):

        k1 = self.derivs(self.th, self.thd, self.ph, self.phd, duty)
        k2_ = self.derivs(self.th + dt / 2 * k1[0], self.thd + dt / 2 * k1[1],
                          self.ph + dt / 2 * k1[2], self.phd + dt / 2 * k1[3], duty)
        k3_ = self.derivs(self.th + dt / 2 * k2_[0], self.thd + dt / 2 * k2_[1],
                          self.ph + dt / 2 * k2_[2], self.phd + dt / 2 * k2_[3], duty)
        k4 = self.derivs(self.th + dt * k3_[0], self.thd + dt * k3_[1],
                         self.ph + dt * k3_[2], self.phd + dt * k3_[3], duty)
        self.th  += dt / 6 * (k1[0] + 2 * k2_[0] + 2 * k3_[0] + k4[0])
        self.thd += dt / 6 * (k1[1] + 2 * k2_[1] + 2 * k3_[1] + k4[1])
        self.ph  += dt / 6 * (k1[2] + 2 * k2_[2] + 2 * k3_[2] + k4[2])
        self.phd += dt / 6 * (k1[3] + 2 * k2_[3] + 2 * k3_[3] + k4[3])

    def theta_code(self, noise=0.0, rng=None):
        v = CENTER + self.th * CODE_PER_RAD
        v = round(v / ADC_LSB_CODES) * ADC_LSB_CODES
        if noise and rng:
            v += rng.gauss(0.0, noise)
        return v

    def pos_count(self):
        return -self.ph / RAD_PER_CNT

class PidQ16:

    INT32_MAX = 2147483647
    INT32_MIN = -2147483648

    def __init__(self, kp, ki, kd, out_max, out_min, d_lpf=False, d_span=1):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_max, self.out_min = out_max, out_min
        self.err_d = 0
        self.err_d1 = 0
        self.err_d2 = 0
        self.integ = 0
        self.out = 0
        self.d_lpf = d_lpf

        self.d_span = d_span
        self.d_state = 0

        self.err_d3 = 0
        self.err_d4 = 0

    @staticmethod
    def _sat32(v):
        return max(PidQ16.INT32_MIN, min(PidQ16.INT32_MAX, v))

    def step(self, target, actual, clear):
        err = target - actual
        prev_err = self.err_d

        err_d1_new = err if clear else prev_err
        if self.d_span == 2:
            err_d2_new = err if clear else self.err_d1

            err_d3_new, err_d4_new = self.err_d2, self.err_d3
            d_err = (err - err_d2_new) >> 1
        elif self.d_span == 4:

            err_d2_new = err if clear else self.err_d1
            err_d3_new = err if clear else self.err_d2
            err_d4_new = err if clear else self.err_d3
            d_err = (err - err_d4_new) >> 2
        else:
            err_d2_new = self.err_d2
            err_d3_new = self.err_d3
            err_d4_new = self.err_d4
            d_err = err - err_d1_new

        pp = self._sat32((self.kp * err) >> 16)
        ii = self._sat32((self.ki * err) >> 16)
        dd = self._sat32((self.kd * d_err) >> 16)
        if self.d_lpf:
            self.d_state = 0 if clear else (self.d_state + dd) >> 1
            dd = self.d_state

        if clear:
            integ = 0
        else:
            sat_hi = (self.out >= self.out_max) and (ii > 0)
            sat_lo = (self.out <= self.out_min) and (ii < 0)
            raw = self.integ + ii
            lim = max(self.out_min, min(self.out_max, raw))
            integ = self.integ if (sat_hi or sat_lo) else lim

        s = pp + dd + (0 if clear else integ)
        out = max(self.out_min, min(self.out_max, s))

        self.err_d, self.err_d1, self.err_d2 = err, err_d1_new, err_d2_new
        self.err_d3, self.err_d4 = err_d3_new, err_d4_new
        self.integ, self.out = integ, out
        return out

class FuzzyGainScheduler:

    NB, NS, ZE, PS, PB = range(5)
    KP_SCALE_Q8 = (256, 267, 277, 288)
    KD_SCALE_Q8 = (256, 277, 299, 320)

    def __init__(self, kp, ki, kd, e_shift, ec_shift, tune_enable=True,
                 e_deadband_q16=2*65536, ec_deadband_q16=32768,
                 ec_filter_shift=2, kp_slew_q8=1, kd_slew_q8=2):
        self.base = (kp, ki, kd)
        self.e_shift, self.ec_shift = e_shift, ec_shift
        self.tune_enable = tune_enable
        self.e_deadband_q16, self.ec_deadband_q16 = e_deadband_q16, ec_deadband_q16
        self.ec_filter_shift = ec_filter_shift
        self.kp_slew_q8, self.kd_slew_q8 = kp_slew_q8, kd_slew_q8
        self.err_prev = 0
        self.ec_filtered = 0
        self.kp_scale = self.kd_scale = 256
        self.initialized = False

    @staticmethod
    def _deadzone(value, width):
        magnitude = max(abs(value)-width, 0)
        return -magnitude if value < 0 else magnitude

    @staticmethod
    def _sat32(value):
        return max(-(1 << 31), min((1 << 31)-1, value))

    @staticmethod
    def _slew(current, target, limit):
        return current + max(-limit, min(limit, target-current))

    @classmethod
    def _sets(cls, value, shift):
        step = 65536 << shift
        mag = abs(value)
        if mag >= 2*step:
            edge = cls.NB if value < 0 else cls.PB
            return edge, edge, 0
        if mag < step:
            return cls.ZE, cls.NS if value < 0 else cls.PS, (mag << 8) // step
        lo = cls.NS if value < 0 else cls.PS
        hi = cls.NB if value < 0 else cls.PB
        return lo, hi, ((mag-step) << 8) // step

    @classmethod
    def _kp_rule(cls, e, ec):
        table = (
            (3, 2, 0, 0, 0),
            (2, 1, 0, 0, 0),
            (0, 0, 0, 0, 0),
            (0, 0, 0, 1, 2),
            (0, 0, 0, 2, 3),
        )
        return table[e][ec]

    @classmethod
    def _kd_rule(cls, e, ec):
        table = (
            (2,1, 0,1,2),
            (3,2, 0,2,3),
            (3,2, 0,2,3),
            (3,2, 0,2,3),
            (2,1, 0,1,2),
        )
        return table[e][ec]

    @classmethod
    def _blend(cls, e_sets, ec_sets, rule, scale_values):
        e_lo, e_hi, e_hi_mu = e_sets
        c_lo, c_hi, c_hi_mu = ec_sets
        e_lo_mu, c_lo_mu = 256-e_hi_mu, 256-c_hi_mu
        terms = (
            (e_lo_mu*c_lo_mu, e_lo, c_lo),
            (e_lo_mu*c_hi_mu, e_lo, c_hi),
            (e_hi_mu*c_lo_mu, e_hi, c_lo),
            (e_hi_mu*c_hi_mu, e_hi, c_hi),
        )
        total = sum(weight*scale_values[rule(es, cs)]
                    for weight, es, cs in terms)
        return (total + 32768) >> 16

    def step(self, error, clear=False, enabled=True):
        if not self.tune_enable or not enabled:
            self.err_prev = error
            self.ec_filtered = 0
            self.kp_scale = self.kd_scale = 256
            self.initialized = False
            return self.base
        if clear or not self.initialized:
            self.err_prev = error
            self.ec_filtered = 0
            self.kp_scale = self.kd_scale = 256
            self.initialized = True
            return self.base

        ec_raw = error-self.err_prev
        self.err_prev = error
        self.ec_filtered += (ec_raw-self.ec_filtered) >> self.ec_filter_shift
        e_sets = self._sets(self._deadzone(error, self.e_deadband_q16), self.e_shift)
        ec_sets = self._sets(self._deadzone(self.ec_filtered, self.ec_deadband_q16), self.ec_shift)
        kp_target = max(256, min(288, self._blend(
            e_sets, ec_sets, self._kp_rule, self.KP_SCALE_Q8)))
        kd_target = max(256, min(320, self._blend(
            e_sets, ec_sets, self._kd_rule, self.KD_SCALE_Q8)))
        self.kp_scale = self._slew(self.kp_scale, kp_target, self.kp_slew_q8)
        self.kd_scale = self._slew(self.kd_scale, kd_target, self.kd_slew_q8)
        kp, ki, kd = self.base
        return (self._sat32((kp*self.kp_scale) >> 8), ki,
                self._sat32((kd*self.kd_scale) >> 8))

class Controller:

    def __init__(self, kp_bal, kd_bal, kp_arm, ki_arm, kd_arm,
                 duty_max=DUTY_FULL, duty_min=130, center=CENTER, target_pos=0.0,
                 arm_clamp=ARM_CLAMP, closed=True, friction_mode=0, moving_comp=50,
                 fuzzy=False, inner_d_span=2, outer_d_span=2):
        self.kp_bal, self.kd_bal = kp_bal, kd_bal
        self.duty_max, self.duty_min, self.center = duty_max, duty_min, center
        self.target_pos, self.closed = target_pos, closed

        q = lambda x: int(round(x * 65536.0))

        dm = min(duty_max, 8191)
        self.pid_max = dm << 16

        self.inner = PidQ16(q(kp_bal), 0, q(kd_bal), self.pid_max, -self.pid_max,
                            d_lpf=True, d_span=inner_d_span)

        am = min(arm_clamp, duty_max)
        self.arm_max = am << 16
        self.outer = PidQ16(q(kp_arm), q(ki_arm), q(kd_arm),
                            self.arm_max, -self.arm_max, d_lpf=True, d_span=outer_d_span)
        self.fuzzy = fuzzy
        self.inner_fuzzy = FuzzyGainScheduler(q(kp_bal), 0, q(kd_bal), 2, 0)
        self.outer_fuzzy = FuzzyGainScheduler(q(kp_arm), q(ki_arm), q(kd_arm), 4, 1,
                                             tune_enable=False)

        self.inner_cnt = 0
        self.outer_cnt = 0
        self.outer_out = 0
        self.duty_cmd = 0
        self.ang_target = int(self.center) << 16
        self.origin = None
        self.prev_pos = 0
        self.prev_theta = None
        self.safe = False
        self.ok_count = 0
        self.position_fault = False
        self.friction_mode = friction_mode
        self.moving_comp = moving_comp
        self.motion_count = self.travel = self.quiet = 0
        self.moving = False
        self.comp_level = duty_min

    def tick(self, theta, pos):

        pos = int(round(pos))
        theta = int(round(theta))
        if self.origin is None or not self.closed:
            self.origin = pos
            self.prev_pos = pos
            self.inner.err_d = (int(self.center) - theta) << 16
        relative = pos - self.origin
        delta = pos - self.prev_pos
        self.prev_pos = pos
        bad = (abs(theta-self.center)>800 or abs(delta)>3 or
               (self.prev_theta is not None and abs(theta-self.prev_theta)>1024))
        self.prev_theta = theta
        if not self.closed:
            self.safe = self.position_fault = False
            self.ok_count = 0
        elif bad:
            self.safe = True
            self.ok_count = 0
        elif self.safe:
            if abs(theta-self.center)<400 and abs(delta)<=1:
                self.ok_count += 1
                if self.ok_count>=8: self.safe=False
            else: self.ok_count=0
        if self.closed and (abs(relative)>8191 or abs(self.target_pos)>8191):
            self.position_fault = True
        clear = not self.closed or self.safe or self.position_fault
        if clear:
            self.inner.integ=self.outer.integ=0
            self.inner.d_state=self.outer.d_state=0

        desired = min(self.moving_comp,self.duty_min) if self.moving else self.duty_min
        if clear or not self.friction_mode:
            self.motion_count=self.travel=self.quiet=0
            self.moving=False; self.comp_level=self.duty_min
        else:
            self.comp_level += max(-RAMP_STEP,min(RAMP_STEP,desired-self.comp_level))
            self.travel += abs(delta); self.motion_count += 1
            if self.motion_count==20:
                if self.travel>=3: self.moving=True; self.quiet=0
                elif self.travel<=1:
                    self.quiet += 1
                    if self.quiet==3: self.moving=False; self.quiet=0
                else: self.quiet=0
                self.motion_count=self.travel=0

        if self.inner_cnt == INNER_DIV - 1:
            self.inner_cnt = 0
            inner_tick = True
            if self.outer_cnt == OUTER_DIV - 1:
                self.outer_cnt = 0
                outer_tick = True
            else:
                self.outer_cnt += 1
                outer_tick = False
        else:
            self.inner_cnt += 1
            inner_tick = outer_tick = False

        if outer_tick:
            error = max(-(8191<<16), min(8191<<16,
                        int(round(self.target_pos*65536)) - (relative<<16)))
            self.outer_out = self.outer.step(
                0, -error, clear)

        self.ang_target = int(round(self.center)) << 16

        if inner_tick:
            inner_error = self.ang_target - (int(round(theta)) << 16)
            self.inner.kp, self.inner.ki, self.inner.kd = self.inner_fuzzy.step(
                inner_error, clear, enabled=self.fuzzy)
            out = self.inner.step(self.ang_target,
                                  int(round(theta)) << 16,
                                  clear)
        self.duty_cmd = max(-32768, min(32767, (self.inner.out-self.outer_out)>>16))

        if clear:
            self.duty_cmd = 0
        return self.apply_duty(self.duty_cmd)

    def apply_duty(self, duty):
        d = int(round(duty))
        mag = abs(d)
        lim = self.duty_max if self.duty_max else DUTY_FULL
        mag = min(mag, lim)
        dmin = min(self.comp_level if self.friction_mode else self.duty_min, lim)
        if mag == 0:
            return 0
        comp = min(mag << (1 if self.friction_mode else 0), dmin)
        mag = min(lim, mag + comp)
        return -mag if d < 0 else mag

def cmd_freefall(args):

    path = args.csv if args.csv else "sim/fall1.csv"
    rows = _load_csv(path)

    n_ok = len(rows)
    for i, r in enumerate(rows):
        if r["adc"] <= HANG:
            n_ok = i + 1
            break
    rows = rows[:n_ok]

    th0 = (rows[0]["adc"] - CENTER) * RAD_PER_CODE
    print(f"== 自由落体（对照 {path}）==")
    print(f"  起点 {math.degrees(th0):.2f}°（adc={rows[0]['adc']}），"
          f"有效落体段 {n_ok} 拍 @1kHz")
    print(f"  l_eff = {L_EFF*1000:.1f} mm  ->  sqrt(g/l_eff) = {math.sqrt(9.81/L_EFF):.3f} rad/s")
    print()
    print("  角度    实测(ms)   仿真(ms)    差       角速度(实测/仿真)")
    t, dt = 0.0, 1e-4
    plant = Furuta()
    plant.th, plant.thd = th0, 0.0

    targets = [30, 60, 90, 120, 150, 180, 220, 256]
    real_ms, sim_ms = {}, {}
    for i, r in enumerate(rows):
        d = abs(math.degrees((r["adc"] - CENTER) * RAD_PER_CODE))
        for tg in targets:
            if tg not in real_ms and d >= tg:
                real_ms[tg] = float(i)
    while t < 3.0:
        plant.step(0.0, dt)
        t += dt
        d = abs(math.degrees(plant.th))
        for tg in targets:
            if tg not in sim_ms and d >= tg:
                sim_ms[tg] = t * 1000
        if abs(math.degrees(plant.th)) > max(targets):
            break

    for tg in targets:
        r, s = real_ms.get(tg), sim_ms.get(tg)
        if r is None or s is None:
            continue
        print(f"   {tg:3d}°   {r:7.0f}   {s:8.1f}   {s-r:+7.1f}")
    print()
    print("  判据：仿真**应当**比实测快一点点（模型里没有摆杆摩擦）。")
    print("        差出 20% 以上 → l_eff 不对；差得很小 → 摩擦也小，可以忽略。")

def _load_csv(path):
    rows = []
    with open(path, newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            rows.append({k: int(v) for k, v in r.items()})
    return rows

def _const_windows(duty, min_len=300):

    out, i = [], 0
    while i < len(duty):
        j = i
        while j + 1 < len(duty) and duty[j + 1] == duty[i]:
            j += 1
        if j - i >= min_len and duty[i] != 0:
            out.append((i, j + 1, duty[i]))
        i = j + 1
    return out

def cmd_replay(args):

    rows = _load_csv(args.csv)
    duty = [r["duty"] for r in rows]
    pos = [r["pos"] for r in rows]
    wins = _const_windows(duty, args.min_len)

    print(f"== 开环标定 {args.csv} ==")
    if not wins:
        print(f"  没有找到长度 ≥{args.min_len}ms 的恒定 duty 窗口 —— 这个采集"
              f"测不出工作点（duty 一直在变，说明是闭环跑出来的）。")
        print(f"  换一个：spin1.csv（duty 恒 150）/ sign3.csv（恒 500）/ out1.csv（恒 150）")
        return

    print(f"  参数：J_arm={J_ARM:.1e}  τ_stall={TAU_STALL}  ω_nl={OMEGA_NL:.0f} rad/s")
    print(f"        τ_动={TAU_C_ARM:.3f}  τ_静={TAU_S_ARM:.3f} N·m")
    print()
    print("   窗口          duty   时长   实测稳态     模型稳态    差")
    print("   (ms)                 (ms)   (rad/s)      (rad/s)")
    print("   " + "-" * 62)
    for (a, b, d) in wins:

        h = (b - a) // 2
        w_real = -(pos[b - 1] - pos[a + h]) / (b - 1 - a - h) * RAD_PER_CNT * 1000
        w_sim = _steady_speed(d)
        print(f"   {a:5d}~{b:5d} {d:+6d}  {b-a:5d}  {w_real:9.3f}   {w_sim:9.3f}  "
              f"{w_sim-w_real:+8.3f}")
    print()
    print("  判据：")
    print("   · 模型**读高**是正常的 —— 模型的稳态转速是按'摆杆垂着不动'算的，")
    print("     而实测里摆杆在甩，`φ̇²` 那一项会把能量吸走，摆臂自然转得慢。")
    print("     转得越快差得越多（φ̇² 是平方项），正是这个原因。")
    print("   · 真正该盯的是 **duty 150 那一档**（`spin1.csv`，摆杆垂着不动、")
    print("     唯一没有杆负载的采集）：那里应该差在 0.1 rad/s 以内。")
    print("     对上了 = 摆臂模型可信，闭环仿真才有意义。")
    print("   · 如果连 spin1 都对不上，调文件头『摆臂/电机』那三个：")
    print("     ω_nl 管斜率、τ_动 管截距、τ_静 管'多少 duty 才推得动'")

def _steady_speed(duty, plant=None, t_max=8.0):

    p = plant if plant is not None else Furuta()
    p.th = math.pi
    p.thd = 0.0
    p.ph = 0.0
    p.phd = 0.0
    dt = 1e-4
    for _ in range(int(t_max / dt)):
        p.step(duty, dt)
        if abs(p.phd) > 1e-6 and abs(p.th - math.pi) < 1e-6:
            pass
    return p.phd

def cmd_loop(args):

    ctrl = Controller(kp_bal=args.kp, kd_bal=args.kd,
                      kp_arm=args.kparm, ki_arm=args.kiarm, kd_arm=args.kdarm,
                      duty_max=args.duty_max, duty_min=args.duty_min,
                      closed=args.closed, friction_mode=args.friction_mode,
                      fuzzy=args.fuzzy)
    plant = Furuta()
    plant.th = math.radians(args.deg)

    import random
    rng = random.Random(20260920)

    dt, n_sub = 1e-4, 10
    T = args.ms / 1000.0
    n = int(T * CTRL_HZ)
    hist, t = [], 0.0
    for i in range(n):
        theta = plant.theta_code(ADC_NOISE_CODES, rng)
        pos = plant.pos_count()
        duty = ctrl.tick(theta, pos)
        for _ in range(n_sub):
            plant.step(duty, dt)
            t += dt
        hist.append((i, theta, pos, duty, math.degrees(plant.th)))

    fell_at = next((h[0] for h in hist if abs(h[4]) > 90.0), None)

    print(f"== 闭环仿真 ==")
    print(f"  {'模糊PID' if args.fuzzy else '固定PID'}：内环 Kp={args.kp} Kd={args.kd}（每 {INNER_DIV}ms）"
          f"   外环 Kp={args.kparm} Ki={args.kiarm} Kd={args.kdarm}（每 {INNER_DIV*OUTER_DIV}ms）")
    print(f"  duty_max={args.duty_max}  duty_min={args.duty_min}  "
          f"{'闭环' if args.closed else '开环'}  起点 {args.deg:+.1f}°  跑 {args.ms:.0f}ms")
    print()
    step = max(1, n // 12)
    print("   拍    杆偏角(deg)   adc    pos     duty")
    for i in range(0, n, step):
        _, th, ps, du, dg = hist[i]
        print(f"  {i:5d}   {dg:+9.2f}  {th:5.0f}  {ps:8.0f}  {du:+6d}")
    print()
    print("  ---- 判据 ----")
    if fell_at is None:
        tail = hist[-n // 5:]
        dmax = max(abs(h[4]) for h in tail)
        span_pos = max(h[2] for h in tail) - min(h[2] for h in tail)
        print(f"  ✅ 全程没倒（|杆偏角| 始终 < 90°）")
        print(f"     最后 20% 时间里：杆偏角最大 {dmax:.2f}°，摆臂摆幅 {span_pos:.0f} 计数 "
              f"({span_pos/ENC_CPR:.2f} 圈)")
        if span_pos / ENC_CPR > 0.5:
            print(f"     ⚠️ 摆臂还在大幅来回摆 —— 杆虽然没倒，但**位置环没收敛**")
        else:
            print(f"     ✅ 摆臂基本停在原地 —— 外环收敛了")
    else:
        print(f"  ❌ 第 {fell_at} ms（{fell_at/1000:.2f} 秒）倒了")
        print("     模型未通过该工况；机械参数存在不确定性，不能据此断定实机根因。")
        print(f"        先看内环符号/大小（把外环关掉：--kparm 0 --kdarm 0）")

def main():
    ap = argparse.ArgumentParser(
        description="旋转倒立摆 Python 仿真（不上板）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("被控对象")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("freefall", help="复现 fall1.csv，验证 l_eff")
    p.add_argument("--csv", default=None, help="默认 sim/fall1.csv")
    p.set_defaults(func=cmd_freefall)

    p = sub.add_parser("replay", help="从 CSV 里提取恒定 duty 工作点，标定摆臂模型")
    p.add_argument("csv")
    p.add_argument("--min-len", dest="min_len", type=int, default=300,
                   help="恒定 duty 窗口的最短长度(ms)，默认 300")
    p.set_defaults(func=cmd_replay)

    p = sub.add_parser("loop", help="闭环仿真")
    p.add_argument("--kp",     type=float, default=3.25)
    p.add_argument("--kd",     type=float, default=100.0)
    p.add_argument("--kparm",  type=float, default=-1.1)
    p.add_argument("--kiarm",  type=float, default=-0.025)

    p.add_argument("--kdarm",  type=float, default=-25.0)
    p.add_argument("--duty-max", dest="duty_max", type=int, default=DUTY_FULL)
    p.add_argument("--duty-min", dest="duty_min", type=int, default=130)
    p.add_argument("--deg",    type=float, default=2.0, help="初始偏角(度)")
    p.add_argument("--ms",     type=float, default=5000)
    p.add_argument("--friction-mode",type=int,choices=(0,1),default=0)
    p.add_argument("--fixed", dest="fuzzy", action="store_false",
                   help="使用固定增益 PID；默认模拟内环保守模糊 P/D 调度")
    p.add_argument("--open",   dest="closed", action="store_false",
                   help="不进闭环（只开电机看摆臂）")
    p.set_defaults(func=cmd_loop, closed=True, fuzzy=True)

    args = ap.parse_args()
    args.func(args)

if __name__ == "__main__":
    main()
