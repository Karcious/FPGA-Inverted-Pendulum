import math
import time

from gui_motion import request_zero_calibration
from gui_trajectory import build_function

COUNTS_PER_REV = 1040.0
DEG_PER_COUNT = 360.0 / COUNTS_PER_REV

MAX_DISPLACEMENT = 500.0

MAX_ACCEL = 120.0
WRITE_INTERVAL = 0.010
BRAKE_SECONDS = 0.80
BRAKE_SECONDS_MAX = 3.0
BRAKE_DECEL = 120.0

SETTLE_ROD_DEG = 4.0
SETTLE_SAMPLES = 2
SETTLE_TIMEOUT = 3.0

FINISH_WATCH = 0.10

FINISH_GONE_COUNTS = 160.0
FINISH_RETRY = 0.03
MIN_PERIOD, MAX_PERIOD = 2.0, 40.0
MAX_AMPLITUDE = 180.0
CORNER_FRACTION = 0.05
CORNER_PASSES = 3
SHAPES = {'sine': '正弦', 'cosine': '余弦', 'triangle': '三角', 'custom': '自定义'}

WAVE_SEEDS = {'sine': 'sin(2*pi*t/T)', 'cosine': 'cos(2*pi*t/T + pi/2)'}

DEFAULT_EXPRESSION = WAVE_SEEDS['sine']

BIT_SAFE = 3
BIT_POSITION_FAULT = 4
BIT_SENSOR_OK = 7
BIT_MOTION_READY = 21
BIT_HOME_FAULT = 24
TILT_ABORT_DEG = 25.0

def counts_of(degrees):
    return float(degrees) / DEG_PER_COUNT

def degrees_of(counts):
    return float(counts) * DEG_PER_COUNT

def q16(counts):
    return int(round(float(counts) * 65536.0)) & 0xFFFFFFFF

def unq16(raw):

    raw = int(raw) & 0xFFFFFFFF
    if raw & 0x80000000:
        raw -= 1 << 32
    return raw / 65536.0

def shape_value(name, phase):

    p = phase % 1.0
    if name == 'sine':
        return math.sin(2.0 * math.pi * p)
    if name == 'cosine':

        return -math.sin(2.0 * math.pi * p)
    if p < 0.25:
        return 4.0 * p
    if p < 0.75:
        return 2.0 - 4.0 * p
    return 4.0 * p - 4.0

def _boxcar(values, half):

    n = len(values)
    if half < 1 or n < 2 * half + 2:
        return list(values)
    width = 2 * half + 1
    out = list(values)
    for _ in range(CORNER_PASSES):
        ext = out[-half:] + out + out[:half + 1]
        acc = sum(ext[:width])
        nxt = []
        for i in range(n):
            nxt.append(acc / width)
            if i + width < len(ext):
                acc += ext[i + width] - ext[i]
        out = nxt
    return out

def custom_shape(expression, period_s, n):

    f = build_function(expression, period_s)
    samples = [f(i * period_s / n) for i in range(n + 1)]
    peak = max(abs(v) for v in samples)
    jump = abs(samples[-1] - samples[0])
    if jump > peak * 0.02:
        raise ValueError('首尾接不上：t=T 时是 %.4g，t=0 是 %.4g，每圈开头会跳一下。'
                         '请用 sin/cos 这类一圈走完回到原处的函数（周期用 T）。'
                         % (samples[-1], samples[0]))
    body = samples[:-1]
    dc = sum(body) / len(body)
    if abs(dc) > peak * 0.01:
        body = [v - dc for v in body]

    if max(abs(v) for v in body) <= abs(peak) * 1e-9:
        raise ValueError('这个函数一圈里一直是同一个值，没有来回的位移。')
    return body, dc

def build_cycle(shape, amplitude_deg, period_s, expression=''):

    period_s = float(period_s)
    amplitude_deg = float(amplitude_deg)
    if shape not in SHAPES:
        raise ValueError('不认识的波形：%r' % (shape,))
    if not (MIN_PERIOD <= period_s <= MAX_PERIOD) or not math.isfinite(period_s):
        raise ValueError('周期需在 %g～%g 秒之间。' % (MIN_PERIOD, MAX_PERIOD))
    if not (0.0 < amplitude_deg <= MAX_AMPLITUDE) or not math.isfinite(amplitude_deg):
        raise ValueError('幅值需在 0～%g° 之间（0 就是不动）。' % MAX_AMPLITUDE)
    n = int(round(period_s * 1000.0))
    amplitude = counts_of(amplitude_deg)
    dc = 0.0
    if shape == 'custom':
        raw, dc = custom_shape(expression, period_s, n)
    else:
        raw = [shape_value(shape, i / n) for i in range(n)]

        if shape not in ('sine', 'cosine'):
            raw = _boxcar(raw, int(round(n * CORNER_FRACTION)))
    peak = max(abs(v) for v in raw) or 1.0
    counts = [v / peak * amplitude for v in raw]
    dt = period_s / n

    seam = 0.0
    if shape == 'custom':
        v_in = degrees_of((counts[0] - counts[-1]) / dt)
        v_out = degrees_of((counts[1] - counts[0]) / dt)
        seam = abs(v_out - v_in)
    accel_peak = 0.0
    for i in range(n):
        a = (counts[(i + 1) % n] - 2.0 * counts[i] + counts[(i - 1) % n]) / (dt * dt)
        if abs(a) > accel_peak:
            accel_peak = abs(a)
    accel_deg = degrees_of(accel_peak)
    speed_peak = max(abs(counts[(i + 1) % n] - counts[i]) for i in range(n))
    speed_deg = degrees_of(speed_peak / dt)

    min_period = period_s * math.sqrt(accel_deg / MAX_ACCEL) * 1.02 if accel_deg > 0 else MIN_PERIOD
    return dict(shape=shape, amplitude_deg=amplitude_deg, period_s=period_s,
                period_ms=n, counts=counts, accel=accel_deg, speed=speed_deg,
                dc=dc, expression=expression, seam=seam,
                ok=accel_deg <= MAX_ACCEL, min_period=max(MIN_PERIOD, min_period))

def brake_seconds(cycle):

    v0 = abs(float(cycle.get('speed', 0.0))) if cycle else 0.0
    t = v0 * math.pi / (2.0 * BRAKE_DECEL)
    return min(BRAKE_SECONDS_MAX, max(BRAKE_SECONDS, t))

def cycle_formula(shape, amplitude_deg, period_s, expression=''):

    amp = '%g' % float(amplitude_deg)
    T = '%g' % float(period_s)
    if shape == 'sine':
        return '%s·sin(2π·t/%s) °' % (amp, T)
    if shape == 'cosine':
        return '%s·cos(2π·t/%s + π/2) °' % (amp, T)
    if shape == 'triangle':
        return '%s·tri(2π·t/%s) °（尖角磨圆）' % (amp, T)
    return '%s·f(t)/max|f|，f(t) = %s' % (amp, expression or DEFAULT_EXPRESSION)

class WaveSession:

    IDLE, RUN, BRAKE, FINISH = 0, 1, 2, 3
    NAMES = {IDLE: '空闲', RUN: '运行中', BRAKE: '减速停车中', FINISH: '搬零点中'}

    def __init__(self):
        self.active = False
        self.state = self.IDLE
        self.note = ''
        self.cycle = None
        self.center = 0.0
        self.phase = 0.0
        self.hold = 0.0
        self.target = 0.0
        self.actual = None
        self.rod_deg = None
        self.center_code = 0
        self.revision = 0
        self.t_last = 0.0
        self.brake_t0 = 0.0
        self.brake_seconds = BRAKE_SECONDS
        self.settle_count = 0
        self.settle_timed_out = False
        self.next_write = 0.0
        self.next_status = 0.0
        self.ticks = 0
        self.finish_deadline = 0.0
        self.next_zero = 0.0
        self.next_watch = 0.0
        self.zero_done = False
        self.last_reject = ''
        self.failures = 0
        self.cycles_done = 0

    def supported(self, link):
        magic = link.get_param(0)
        self.revision = magic & 255
        if magic >> 8 != 0x0207 or self.revision < 0x0E:
            raise ValueError('往复轨迹需要 0x0002070E（REV14）或更新的固件；当前 0x%08X' % magic)
        return magic

    def begin(self, link, cfg, now=None):
        now = time.monotonic() if now is None else now
        cycle = build_cycle(cfg['shape'], cfg['amplitude_deg'], cfg['period_s'],
                            cfg.get('expression', ''))
        if not cycle['ok']:
            if cycle['min_period'] > MAX_PERIOD:

                raise ValueError('这个形状在 ±%.0f° 下峰加速度 %.0f°/s²，超过 %g°/s²；'
                                 '周期拉到最长（%g 秒）也放不下%s。'
                                 % (cycle['amplitude_deg'], cycle['accel'], MAX_ACCEL,
                                    MAX_PERIOD,
                                    '，因为一圈首尾接不上（接缝处速度要跳 %.0f°/s）'
                                    % cycle['seam'] if cycle['seam'] > 5 else ''))
            raise ValueError('这个幅值/周期下峰加速度 %.0f°/s²，超过 %g°/s²；'
                             '把周期调到 %.1f 秒以上再来。'
                             % (cycle['accel'], MAX_ACCEL, cycle['min_period']))
        self.supported(link)
        if self.active:
            raise ValueError('往复已经在跑了。')
        if link.get_param(64) & 1:
            raise ValueError('速度轨迹仍在运行，请先停止并等它停稳。')
        mode = link.get_param(16)
        if mode != 3:
            raise ValueError('请先自动起摆（模式 3）并确认倒立平衡；当前模式 %d。' % mode)
        status = link.get_param(31)
        if status & (1 << BIT_SAFE) or status & (1 << BIT_POSITION_FAULT):
            raise ValueError('保护或位置故障生效中，先排除故障再开始。')
        if status & 7 != 4:
            raise ValueError('还没确认倒立平衡（摆杆状态 %d），等起摆完成再开始。' % (status & 7))
        if status & (1 << BIT_HOME_FAULT):
            raise ValueError('回零故障：稳定后长按 SW3 约 2 秒重新置零，再来开始。')
        if not status & (1 << BIT_MOTION_READY):
            raise ValueError('摆臂还没有回零停稳（按键未就绪）；等它回到 0 点再开始，'
                             '否则第一拍目标会把摆臂拽回去。')
        link.set_param(13, 0)
        if link.get_param(13) != 0:
            raise ValueError('地址 13 清零回读不一致，没有开始。')
        link.set_param(16, 2)
        got = link.get_param(16)
        if got != 2:
            raise ValueError('切到模式 2 失败（回读模式 %d），没有开始。' % got)
        self.cycle = cycle
        self.brake_seconds = brake_seconds(cycle)
        self.settle_count = 0
        self.settle_timed_out = False
        self.center = 0.0
        self.phase = 0.0
        self.hold = 0.0
        self.target = 0.0
        self.actual = None
        self.rod_deg = None
        self.cycles_done = 0
        self.center_code = link.get_param(18)
        self.state = self.RUN
        self.active = True
        self.zero_done = False
        self.t_last = now
        self.next_write = now
        self.next_status = now + 0.1
        self.ticks = 0
        self.failures = 0
        self.last_reject = ''
        self.note = ('已切到模式 2（上位机给位置点）。%s · 幅值 %.0f° · 周期 %.1f s'
                     % (SHAPES[cycle['shape']], cycle['amplitude_deg'], cycle['period_s']))

    def poll(self, link, now):

        if not self.active:
            return None
        if self.state == self.FINISH:
            return self._poll_finish(link, now)
        self._advance(now)
        if self.state == self.BRAKE and now - self.brake_t0 >= self.brake_seconds:
            if not self._rod_settled(now):

                if not self.note.startswith('减速走完'):
                    self.note = ('减速走完，杆还在晃 —— 等它晃回来站住（±%.0f° 以内）'
                                 '再搬零点…' % SETTLE_ROD_DEG)
                return None

            self.hold = self.center + self._offset()
            self._begin_finish(link, now)
            return None
        if now >= self.next_status:
            self.next_status = now + 0.1
            self._check_health(link, now)
            if not self.active:
                return None
        if now < self.next_write:
            return None

        self.next_write += WRITE_INTERVAL
        if self.next_write <= now:
            self.next_write = now + WRITE_INTERVAL
        target = self.center + self._offset()
        link.set_param(13, q16(target))
        self.target = target
        self.ticks += 1
        if self.ticks % 2 == 0:
            self._read_actual(link)
        return {'t': now, 'target': target, 'actual': self.actual,
                'state': self.state, 'center': self.center,
                'phase': self.phase % 1.0, 'note': self.note}

    def _rod_settled(self, now):

        if self.settle_count >= SETTLE_SAMPLES:
            return True
        if now - self.brake_t0 - self.brake_seconds >= SETTLE_TIMEOUT:
            self.settle_timed_out = True
            return True
        return False

    def _offset(self):
        counts = self.cycle['counts']
        return counts[int((self.phase % 1.0) * len(counts)) % len(counts)]

    def _advance(self, now):
        dt = now - self.t_last
        self.t_last = now
        if dt <= 0:
            return
        period = self.cycle['period_s']
        if self.state == self.RUN:
            before = self.phase
            self.phase += dt / period
            self.cycles_done += int(self.phase) - int(before)
        elif self.state == self.BRAKE:
            tau = now - self.brake_t0
            weight = 0.5 + 0.5 * math.cos(math.pi * min(1.0, tau / self.brake_seconds))
            self.phase += dt / period * weight

    def _read_actual(self, link):
        try:
            row = link.read_live()['row']
        except Exception:
            self.actual = None
            return
        self.actual = float(row[2])
        self.rod_deg = (float(row[0]) - float(self.center_code)) * 360.0 / 4096.0

    def _check_health(self, link, now):
        mode = link.get_param(16)
        status = link.get_param(31)
        if mode != 2:
            self.active = False
            self.state = self.IDLE
            self.note = ('模式被改成了 %d（不是 2），往复已停；板上按现在的模式继续。' % mode)
            return
        if status & (1 << BIT_SAFE) or status & (1 << BIT_POSITION_FAULT):
            self._abort(link, '保护/位置故障生效（状态字第 3/4 位）')
            return
        if self.rod_deg is not None and abs(self.rod_deg) > TILT_ABORT_DEG:
            self._abort(link, '摆杆偏了 %.0f°，模式 2 不会自己起摆' % self.rod_deg)
            return

        if (self.state == self.BRAKE and self.rod_deg is not None
                and now - self.brake_t0 >= self.brake_seconds
                and abs(self.rod_deg) <= SETTLE_ROD_DEG):
            self.settle_count += 1

    def _abort(self, link, why):

        self.active = False
        self.state = self.IDLE
        try:
            if link.get_param(16) == 2:
                link.set_param(16, 3)

                link.set_param(13, 0)
                self.note = why + '；已切回模式 3（自动平衡/起摆）。'
                return
        except Exception:
            pass
        self.note = why + '；没能切回模式 3，请自己按 SW1 或断开重连。'

    def stop(self, link, now=None):

        now = time.monotonic() if now is None else now
        if self.state == self.FINISH:
            if not self.active:
                return
            self._finish_give_up(
                '你按了停止：不搬零点了。摆臂停在原地不动；杆稳下来后'
                '板子会按**原来的** 0 点回零。')
            return
        if not self.active:
            return
        if self.state != self.BRAKE:
            self._advance(now)
            self.state = self.BRAKE
            self.brake_t0 = now
            self.settle_count = 0
            self.settle_timed_out = False
        self.note = ('减速停车中（%.1f 秒）；停稳后把这里设成新的 0 点。'
                     % self.brake_seconds)

    def _begin_finish(self, link, now):
        self.state = self.FINISH
        self.finish_deadline = now + 5.0
        self.zero_done = False
        self.next_zero = now
        self.next_watch = now
        self.last_reject = ''
        try:

            link.set_param(16, 3)
            link.set_param(13, 0)
        except Exception as exc:
            self.note = ('停车后切模式 3 失败（%s）；请自己按 SW1 或断开重连。' % exc)
            self.active = False
            self.state = self.IDLE
            return
        self.note = ('%s已停车（%+.1f°），正在把这里设成新的 0 点…'
                     % ('杆还没完全站住（等了 %.0f 秒）；' % SETTLE_TIMEOUT
                        if self.settle_timed_out else '',
                        degrees_of(self.hold)))

    def _finish_give_up(self, why):

        self.active = False
        self.state = self.IDLE
        self.zero_done = False
        self.note = why
        return {'done': True, 'note': why, 'aborted': True}

    def _poll_finish(self, link, now):

        if now >= self.next_watch:
            self.next_watch = now + FINISH_WATCH
            self._read_actual(link)
            if self.rod_deg is not None and abs(self.rod_deg) > TILT_ABORT_DEG:
                return self._finish_give_up(
                    '搬零点的时候杆倒了（偏 %.0f°），停止点已经不成立 —— 没有搬成。'
                    '板子在模式 3，会自己把这根杆重新起摆；摆臂接下来会按'
                    '**原来的** 0 点回零，不是刚才停下的地方。'
                    % self.rod_deg)
            if self.actual is not None and abs(self.actual - self.hold) > FINISH_GONE_COUNTS:
                return self._finish_give_up(
                    '搬零点的时候摆臂被挪走了（离停止点 %.0f 码），停止点已经不成立 —— '
                    '没有搬成。板子在模式 3，摆臂接下来会按**原来的** 0 点回零。'
                    % (self.actual - self.hold))
        if not self.zero_done and now >= self.next_zero:
            self.next_zero = now + FINISH_RETRY
            try:
                request_zero_calibration(link)
            except ValueError as exc:

                self.last_reject = str(exc)
            else:
                self.zero_done = True
                self.active = False
                self.state = self.IDLE
                self.note = ('已停车：这里（%+.1f°）就是新的 0 点，零点已经搬过来了。'
                             % degrees_of(self.hold))
                return {'done': True, 'note': self.note}
        if now >= self.finish_deadline:
            self.active = False
            self.state = self.IDLE
            self.note = ('已停车，但板子没接受「标定 0 点」（%s）。摆臂现在停在原地，'
                         '杆稳下来后会按**原来的** 0 点回去。'
                         % (self.last_reject or '没有回执'))
            return {'done': True, 'note': self.note}
        return None
