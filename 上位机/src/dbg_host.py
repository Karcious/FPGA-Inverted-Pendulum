#!/usr/bin/env python3

import argparse
import json
import os
import struct
import sys
import time

from motion_metrics import compute_motion_metrics, format_motion_metrics

try:
    import serial
    HAVE_SERIAL = True
except ImportError:
    serial = None
    HAVE_SERIAL = False

MAGIC0 = 0xAA
MAGIC1 = 0x55
RESP_FLAG = 0x80

CMD_PING         = 0x01
CMD_READ_FUZZY   = 0x0A
CMD_SET_PARAM    = 0x10
CMD_GET_PARAM    = 0x11
CMD_DUMP_PARAMS  = 0x12
CMD_ARM_CAPTURE  = 0x20
CMD_READ_CAPTURE = 0x21
CMD_CAP_STATUS   = 0x22
CMD_DISARM       = 0x23
CMD_READ_LIVE    = 0x24

DEPTH   = 1024
NCH     = 4
DW      = 16

N_PARAM = 52

P_MAGIC        = 0
P_CTRL_HZ      = 1
P_PWM_HZ       = 2
P_KP_BAL       = 3
P_KD_BAL       = 4
P_KP_ARM       = 5
P_KI_ARM       = 6
P_THETA_SWITCH = 7
P_OMEGA_SWITCH = 8

P_CAPTURE_MS   = 30

P_FILT_SEL     = 9
P_DUTY_MIN     = 10
P_DUTY_MAX     = 11
P_SWING_GAIN   = 12
P_TARGET_POS   = 13
P_POS_SLEW     = 14
P_TRIM_THETA   = 15

P_OPENLOOP     = 16
P_DUTY_TEST    = 17

P_CENTER_ANGLE = 18

P_MOTOR_EN = 19

P_KD_ARM = 20

P_WAITUP_BRAKE = 32

P_GAIN_A_KP   = 33
P_GAIN_A_KD   = 34
P_GAIN_B_KP   = 35
P_GAIN_B_KD   = 36
P_PHASE_A_MS  = 37
P_PHASE_B_MS  = 38

P_SETTLE_DAMP = 39

P_MOVE_SLEW = 40
P_PROF_SLEW = 41
P_MOVE_ACC  = 42
P_SPIN_W    = 43
P_SPIN_KP   = 44
P_SPIN_KI   = 45
P_SPIN_ACC  = 46

P_HOME_RAMP = 47

P_ARM_MAX = 48

P_MOVE_DEC = 49

P_COMP_SHIFT = 50

P_TRAVEL_FOLLOW = 51

PARAM_TABLE = {
    P_MAGIC:        ("MAGIC",        "",      False),
    P_CTRL_HZ:      ("CTRL_HZ",      "Hz",    False),
    P_PWM_HZ:       ("PWM_HZ",       "Hz",    False),
    P_KP_BAL:       ("KP_BAL",       "",      True),
    P_KD_BAL:       ("KD_BAL",       "",      True),
    P_KP_ARM:       ("KP_ARM",       "",      True),
    P_KI_ARM:       ("KI_ARM",       "",      True),
    P_THETA_SWITCH: ("THETA_SWITCH", "deg",   False),
    P_OMEGA_SWITCH: ("OMEGA_SWITCH", "deg/s", False),

    P_FILT_SEL:     ("FILT_SEL",     "",      False),
    P_DUTY_MIN:     ("DUTY_MIN",     "cnt",   False),
    P_DUTY_MAX:     ("DUTY_MAX",     "cnt",   False),
    P_SWING_GAIN:   ("SWING_KP",     "",      True),
    P_TARGET_POS:   ("TARGET_POS",   "",      True),
    P_POS_SLEW:     ("SWING_KD",     "",      True),

    P_TRIM_THETA:   ("TRIM_THETA",   "code",  False),

    P_OPENLOOP:     ("OPENLOOP",     "",      False),
    P_DUTY_TEST:    ("DUTY_TEST",    "cnt",   False),

    P_CENTER_ANGLE: ("CENTER_ANGLE", "code",  False),

    P_MOTOR_EN:     ("MOTOR_EN",     "mask",  False),

    P_KD_ARM:       ("KD_ARM",       "",      True),
    21: ("CAPTURE_LOG2", "log2", False),
    22: ("FRICTION_MODE", "", False),
    23: ("MOVING_COMP", "cnt", False),
    24: ("SWING_DUTY", "signed cnt", False),
    25: ("SWING_MS", "ms", False),
    26: ("SWING_TIMEOUT", "ms", False),
    27: ("SWING_TRAVEL", "cnt", False),
    28: ("HANG_ANGLE", "code", False),
    29: ("SWING_TARGET", "signed encoder cnt", False),
    30: ("CAPTURE_MS", "ms", False),
    31: ("STATUS", "bits", False),

    P_WAITUP_BRAKE: ("WAITUP_BRAKE", "",      True),

    P_GAIN_A_KP:   ("GAIN_A_KP",   "",   True),
    P_GAIN_A_KD:   ("GAIN_A_KD",   "",   True),
    P_GAIN_B_KP:   ("GAIN_B_KP",   "",   True),
    P_GAIN_B_KD:   ("GAIN_B_KD",   "",   True),
    P_PHASE_A_MS:  ("PHASE_A_MS",  "ms", False),
    P_PHASE_B_MS:  ("PHASE_B_MS",  "ms", False),

    P_SETTLE_DAMP: ("SETTLE_DAMP", "",    True),

    P_MOVE_SLEW:   ("MOVE_SLEW",   "cnt/ms", True),
    P_PROF_SLEW:   ("PROF_SLEW",   "cnt/ms", True),

    P_MOVE_ACC:    ("MOVE_ACC",    "cnt/ms2", True),

    P_SPIN_W:      ("SPIN_W",      "cnt/25ms", False),

    P_SPIN_KP:     ("SPIN_KP",     "",       True),
    P_SPIN_KI:     ("SPIN_KI",     "",       True),

    P_SPIN_ACC:    ("SPIN_ACC",    "cnt/25ms", False),

    P_HOME_RAMP:   ("HOME_RAMP",   "duty/25ms", False),

    P_ARM_MAX:     ("ARM_MAX",     "duty",      False),

    P_MOVE_DEC:    ("MOVE_DEC",    "cnt/ms2",   True),

    P_COMP_SHIFT:  ("COMP_SHIFT",  "shape",     False),

    P_TRAVEL_FOLLOW: ("TRAVEL_FOLLOW", "cnt",   False),
}

BANK_STEADY = "steady"
BANK_A      = "a"
BANK_B      = "b"

BANK_LABEL = {BANK_STEADY: "稳态", BANK_A: "A段(接管后头一段)", BANK_B: "B段(第二段)"}

BANK_CHOICES = {
    "0": BANK_A,   "a": BANK_A,
    "1": BANK_B,   "b": BANK_B,
    "2": BANK_STEADY, "steady": BANK_STEADY, "s": BANK_STEADY,
}

GAIN_SHADOW = {
    P_KP_ARM: {BANK_A: P_GAIN_A_KP, BANK_B: P_GAIN_B_KP},
    P_KD_ARM: {BANK_A: P_GAIN_A_KD, BANK_B: P_GAIN_B_KD},
}

BANK_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".bank")

def read_bank() -> str:

    try:
        with open(BANK_FILE, "r", encoding="utf-8") as f:
            v = f.read().strip().lower()
        return v if v in (BANK_STEADY, BANK_A, BANK_B) else BANK_STEADY
    except OSError:
        return BANK_STEADY

def write_bank(bank: str) -> None:
    try:
        with open(BANK_FILE, "w", encoding="utf-8") as f:
            f.write(bank + "\n")
    except OSError as e:

        print(f"  !! 选择器没能存盘（{e}），这条命令之后会退回稳态")

def pick_bank(args) -> str:

    one_shot = getattr(args, "bank", None)
    if one_shot:
        return BANK_CHOICES[one_shot]
    return read_bank()

def bank_tag(bank: str) -> str:
    return "" if bank == BANK_STEADY else f"[{BANK_LABEL[bank]}] "

def resolve_addr(addr: int, bank: str):

    if bank == BANK_STEADY:
        return addr, ""
    m = GAIN_SHADOW.get(addr)
    if m is None:

        return addr, f"（地址 {addr} 不分段，写的就是它本身）"
    real = m[bank]
    name_src = PARAM_TABLE.get(addr, (f"P{addr}",))[0]
    name_dst = PARAM_TABLE.get(real, (f"P{real}",))[0]
    return real, f"（{name_src} → {BANK_LABEL[bank]}的 {name_dst} = 地址 {real}）"

CH_NAMES = ["adc", "delta", "pos", "duty"]
CH_UNITS = ["code", "cnt/tick", "cnt", "duty"]

def crc8(data: bytes) -> int:
    c = 0
    for d in data:
        c ^= d
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if (c & 0x80) else ((c << 1) & 0xFF)
    return c

def q16_to_float(v: int) -> float:

    if v & 0x8000_0000:
        v -= 0x1_0000_0000
    return v / 65536.0

def float_to_q16(f: float) -> int:
    return int(round(f * 65536.0)) & 0xFFFF_FFFF

def q16_str(raw: int) -> str:
    return f"{q16_to_float(raw):.5f}"

class DbgError(Exception):
    pass

class DbgLink:

    def __init__(self, port=None, baud=460800, timeout=2.0, transport=None,
                 nch=NCH):

        self.nch = nch
        if transport is not None:
            self.ser = transport
            return
        if port is None:
            raise DbgError("必须给 port 或 transport")
        if not HAVE_SERIAL:
            raise DbgError("缺少 pyserial：pip install pyserial")
        self.ser = serial.Serial(port, baud, timeout=timeout,
                                 bytesize=8, parity='N', stopbits=1)

        time.sleep(0.05)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def close(self):
        self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.close()

    CAP_RETRIES = 5
    CAP_TIMEOUT = 3.0

    def _read_exact(self, n: int) -> bytes:
        buf = bytearray()
        while len(buf) < n:
            chunk = self.ser.read(n - len(buf))
            if not chunk:
                raise DbgError(
                    f"串口读超时：想要 {n} 字节，只收到 {len(buf)} 字节。"
                    "检查波特率、接线，以及 FPGA 是否已加载比特流。")
            buf += chunk
        return bytes(buf)

    SYNC_TIMEOUT = 1.5

    def _sync_magic(self):

        limit = time.monotonic() + self.SYNC_TIMEOUT
        while True:
            b = self._read_exact(1)[0]
            if b != MAGIC0:
                if time.monotonic() > limit:
                    raise DbgError(
                        "串口在持续吐数据，但找不到帧头 0xAA55 —— "
                        "多半是选错了串口（这个口上不是我们的板子）。")
                continue
            if self._read_exact(1)[0] == MAGIC1:
                return

    def request(self, cmd: int, payload: bytes = b"", timeout=None) -> tuple:

        if len(payload) > 16:
            raise DbgError("请求 payload 超过 FPGA 侧缓冲区（16 字节）")
        frame = bytes([MAGIC0, MAGIC1, cmd,
                       len(payload) & 0xFF, (len(payload) >> 8) & 0xFF])
        body = frame[2:] + payload
        frame += payload + bytes([crc8(body)])

        if timeout is not None:
            self.ser.timeout = timeout
        self.ser.reset_input_buffer()
        self.ser.write(frame)
        self.ser.flush()

        self._sync_magic()
        head = self._read_exact(3)
        rcmd, lo, hi = head[0], head[1], head[2]
        rlen = lo | (hi << 8)
        data = self._read_exact(rlen)
        got = self._read_exact(1)[0]
        want = crc8(bytes([rcmd, lo, hi]) + data)
        if got != want:
            raise DbgError(f"响应 CRC 错：收到 {got:#04x}，应为 {want:#04x}")

        if rcmd != (cmd | RESP_FLAG):
            raise DbgError(f"响应命令码不匹配：请求 {cmd:#04x}，"
                           f"收到 {rcmd:#04x}")
        return rcmd, data

    def ping(self) -> tuple:
        _, p = self.request(CMD_PING)
        return (p[0] << 8) | p[1]

    def read_live(self):

        _, payload = self.request(CMD_READ_LIVE)
        if len(payload) != 16:
            raise DbgError('固件不支持实时遥测或帧长度错误（需要 0x011F）')
        values = struct.unpack('<4hII', payload)
        return {'row': list(values[:4]), 'sequence': values[4], 'status': values[5]}

    def read_fuzzy(self):

        _, payload = self.request(CMD_READ_FUZZY)
        if len(payload) != 16:
            if len(payload) == 1:
                raise DbgError(f'模糊调度状态读取失败：固件错误码 {payload[0]:#04x}')
            raise DbgError('固件不支持模糊调度状态或帧长度错误'
                           f'（需要 16 字节，收到 {len(payload)} 字节）')
        kp, kd, ec, flags = struct.unpack('<iiiI', payload)
        p_scale = flags & 0x3FF
        d_scale = (flags >> 10) & 0x3FF
        return {'kp': kp / 65536.0, 'kd': kd / 65536.0, 'ec': ec / 65536.0,
                'kp_q16': kp, 'kd_q16': kd, 'ec_q16': ec,
                'p_scale': p_scale / 256.0, 'd_scale': d_scale / 256.0,
                'p_scale_q8': p_scale, 'd_scale_q8': d_scale,
                'enabled': bool(flags & (1 << 20))}

    def get_param(self, addr: int) -> int:
        _, p = self.request(CMD_GET_PARAM, bytes([addr]))
        if p[0] != addr:
            raise DbgError("回读的地址和请求对不上")
        return struct.unpack("<I", p[1:5])[0]

    def set_param(self, addr: int, raw: int) -> None:
        payload = bytes([addr]) + struct.pack("<I", raw & 0xFFFF_FFFF)
        _, p = self.request(CMD_SET_PARAM, payload)
        if p[0] != addr:
            raise DbgError("SET_PARAM 回显地址不匹配")

    DUMP_WIDTHS = (32, 33, 39, 40, 47, 48, 49, 50)

    def dump_params(self) -> list:
        _, p = self.request(CMD_DUMP_PARAMS)
        if len(p) % 4 or len(p) // 4 not in self.DUMP_WIDTHS + (N_PARAM,):
            raise DbgError(f"参数表长度不对：{len(p)}")
        return list(struct.unpack("<%dI" % (len(p)//4), p))

    def arm_capture(self, ch: int, edge: int, level: int, pre_len: int) -> int:

        payload = bytes([ch & 0xFF, edge & 0xFF]) \
                  + struct.pack("<h", level) \
                  + struct.pack("<H", pre_len)
        _, p = self.request(CMD_ARM_CAPTURE, payload)
        return p[0]

    def read_capture(self, retries=None) -> list:

        tries = self.CAP_RETRIES if retries is None else retries
        p = None
        for i in range(tries + 1):
            try:
                _, p = self.request(CMD_READ_CAPTURE, timeout=self.CAP_TIMEOUT)
                break
            except DbgError as e:
                if i >= tries:
                    raise DbgError(
                        f"采集读回连续 {tries + 1} 次失败，最后一次：{e}\n"
                        "  这是 USB 串口丢字节，不是 FPGA 的问题 —— 收到的数据\n"
                        "  本身是对的，只是少了一截。\n"
                        "  ⚠️ 别去设备管理器改 CH340 —— Windows 自带驱动那一页\n"
                        "     没有「延迟计时器」，FIFO 滑块也救不了（字节在进那个\n"
                        "     缓冲区之前就丢了）。根治是降波特率，见 CLAUDE.md 3.5 ④\n"
                        "     —— **2026-09-18 起已经是 460800 了**，所以再遇到连续失败，\n"
                        "     多半只是这一次恰好赶上，或者线接触不良。\n"
                        "  现在：直接再敲一次同样的命令即可。缓冲区在 done 之后\n"
                        "  是冻结的，重读拿到的还是同一份数据，不会掺进新采样点。"
                    ) from None
                print(f"  [读回第 {i + 1} 次短读，重试] {e}", file=sys.stderr)
                time.sleep(0.05)

        frame = self.nch * 2
        if len(p) == 0 or len(p) % frame != 0:
            raise DbgError(
                f"采集数据长度 {len(p)} 不是 {frame}（nch={self.nch} × 2）的整数倍。"
                "检查上位机的 nch 是否和 dbg_top 的 NCH 参数一致。")
        n = len(p) // frame
        vals = struct.unpack("<%dh" % (n * self.nch), p)
        return [list(vals[i * self.nch:(i + 1) * self.nch]) for i in range(n)]

    def cap_status(self) -> tuple:
        _, p = self.request(CMD_CAP_STATUS)
        return p[0], p[1] | (p[2] << 8)

    def disarm(self) -> int:
        _, p = self.request(CMD_DISARM)
        return p[0]

    def wait_done(self, timeout=5.0) -> bool:

        t0 = time.time()
        while time.time() - t0 < timeout:
            try:
                st, _ = self.cap_status()
            except DbgError as e:
                print(f"  [轮询丢了一次，继续等] {e}", file=sys.stderr)
                time.sleep(0.05)
                continue
            if st & 0x02:
                return True
            time.sleep(0.02)
        return False

def fmt_param(addr: int, raw: int) -> str:
    name, unit, is_q = PARAM_TABLE.get(addr, (f"P{addr}", "", False))
    if addr == P_MAGIC:

        return f"{addr:2d}  {name:<13} {raw:#010x}  (v{raw >> 16}.{(raw >> 8) & 0xFF:02x})"
    if is_q:
        return f"{addr:2d}  {name:<13} {raw:#010x}  = {q16_str(raw)} {unit}"
    return f"{addr:2d}  {name:<13} {raw:#010x}  = {raw} {unit}"

def cmd_ping(link, args):
    v = link.ping()
    print(f"PING 正常，固件版本 v{v >> 8}.{v & 0xFF:02x} (0x{v:04x})")

def cmd_dump(link, args):
    params = link.dump_params()
    print("参数表：")
    for a in sorted(PARAM_TABLE):
        print("   " + fmt_param(a, params[a]))

def cmd_fuzzy_status(link, args):
    status = link.read_fuzzy()
    print(f"内环模糊调度：{'启用' if status['enabled'] else '关闭'}；"
          f"有效 Kp={status['kp']:.6f}，Kd={status['kd']:.6f}")
    print(f"P倍率={status['p_scale']:.6f}，D倍率={status['d_scale']:.6f}；"
          f"滤波后误差变化率={status['ec']:.6f} ADC码/内环采样")

def decode_status(raw: int) -> str:
    states = {0:'停止', 1:'静止准备', 2:'位置起摆', 3:'等待捕获', 4:'确认平衡', 5:'故障', 6:'PID捕获'}
    reasons = {0:'无', 1:'传感器', 2:'配置', 3:'编码器速度', 4:'角度跳变', 5:'行程',
               6:'准备超时', 7:'起摆超时', 8:'闭环保护/捕获偏离', 9:'端点过渡超时', 10:'捕获确认超时'}
    return (f"状态={states.get(raw & 7, '未知')}，故障={reasons.get((raw >> 10) & 15, '未知')}；"
            f"曾接管={(raw >> 14) & 1}，曾确认平衡={(raw >> 15) & 1}，"
            f"曾经过端点={(raw >> 16) & 1}，端点过渡中={(raw >> 17) & 1}；"
            f"按键就绪={(raw >> 21) & 1}，已重标零={(raw >> 22) & 1}，"
            f"自动回零={(raw >> 23) & 1}，回零故障={(raw >> 24) & 1}，"
            f"轨迹运行={(raw >> 25) & 1}，轨迹完成={(raw >> 26) & 1}，"
            f"轨迹故障={(raw >> 27) & 1}，曲线={'B' if (raw >> 28) & 1 else 'A'}，"
            f"第{((raw >> 29) & 7) + 1}段；"
            "（零位：0x0202+，轨迹：0x0203+；0x0200 修订03也支持两者）")

def cmd_bank(link, args):

    cur = read_bank()
    if args.which is None:
        print(f"当前段：{BANK_LABEL[cur]}")
        if cur == BANK_STEADY:
            print("  `set 5` / `set 20` 写的就是稳态那两格（mem[5]/mem[20]），和以前一样。")
        else:
            for a, m in GAIN_SHADOW.items():
                n_src = PARAM_TABLE[a][0]
                n_dst = PARAM_TABLE[m[cur]][0]
                print(f"  `set {a}` ({n_src}) → 实际写 mem[{m[cur]}] ({n_dst})")
            print("  想回到稳态：`bank 2`（或 `bank steady`）")
        return
    new = BANK_CHOICES[args.which]
    write_bank(new)
    print(f"当前段：{BANK_LABEL[new]}  （已存盘，下一条命令仍是这一段）")
    if new == BANK_STEADY:
        print("  `set 5` / `set 20` 写的就是稳态那两格。")
    else:
        print(f"  接下来 `set 5` / `set 20` 会写进 mem["
              f"{GAIN_SHADOW[P_KP_ARM][new]}] / mem[{GAIN_SHADOW[P_KD_ARM][new]}]。")
        print("  ⚠️ 改完记得 `bank 2` 回稳态 —— 不然下次想调稳态时会改错格子。")

def cmd_get(link, args):
    bank = pick_bank(args)
    addr, note = resolve_addr(args.addr, bank)
    raw = link.get_param(addr)
    print(bank_tag(bank) + fmt_param(addr, raw) + ("  " + note if note else ""))
    if addr == 31:
        print(decode_status(raw))

def cmd_set(link, args):
    bank = pick_bank(args)
    addr, note = resolve_addr(args.addr, bank)
    name, unit, is_q = PARAM_TABLE.get(addr, (f"P{addr}", "", False))
    raw = float_to_q16(args.value) if is_q else int(args.value) & 0xFFFF_FFFF
    link.set_param(addr, raw)
    back = link.get_param(addr)
    if back != raw:
        print(f"  !! 回读不一致：写入 {raw:#010x}，读回 {back:#010x}")
    print(bank_tag(bank) + f"已写入 {name} = {args.value}"
          + ("  " + note if note else "")
          + f"  ({fmt_param(addr, back)})")

def cmd_watch(link, args):
    addrs = [int(x) for x in args.addrs.split(",")] if args.addrs else \
            [P_KP_BAL, P_KD_BAL, P_TARGET_POS]
    print("Ctrl-C 退出。")
    try:
        while True:
            row = "  ".join(f"{PARAM_TABLE.get(a, ('P%d' % a,))[0]}="
                            f"{q16_str(link.get_param(a))}" for a in addrs)
            print(f"\r{row}    ", end="", flush=True)
            time.sleep(args.period)
    except KeyboardInterrupt:
        print()

def cmd_capture(link, args):
    version=link.ping()

    snapshot={str(a):link.get_param(a) for a in range(N_PARAM)}
    log2=min(4,snapshot['21']) if version>=0x0112 else 0
    sample_hz=1000/(1<<log2)
    timeout=args.timeout if args.timeout is not None else 4096/sample_hz+8
    print("采样率 %g Hz，4096点窗口 %.3f秒"%(sample_hz,4096/sample_hz))
    print(f"ARM：通道 {args.ch}，边沿 {args.edge}，电平 {args.level}，"
          f"触发前 {args.pre} 点")
    link.arm_capture(args.ch, args.edge, args.level, args.pre)

    if args.trigger_cmd:
        input(">>> 现在让系统动起来，然后按回车开始等触发...")
    print("等待触发...", end="", flush=True)
    if not link.wait_done(timeout=timeout):
        print(" 超时，没有触发。")
        print("  排查：触发电平是否在被观测量能到达的范围内？"
              "边沿方向是否搞反？sample_en 有没有真的在跳？")
        link.disarm()
        return
    print(" 已触发，读回数据...")

    rows = link.read_capture()
    _, trig_idx = link.cap_status()
    print(f"触发点位于缓冲区索引 {trig_idx}，"
          f"即逻辑第 {args.pre} 号点（触发点之前 {args.pre} 个采样）")

    for c in range(NCH):
        col = [r[c] for r in rows]
        print(f"  ch{c} {CH_NAMES[c]:<6} "
              f"min={min(col):7d} max={max(col):7d} "
              f"首={col[0]:7d} 触发点={col[args.pre]:7d} 末={col[-1]:7d}")

    motion = compute_motion_metrics(
        [row[0] for row in rows], [row[1] for row in rows],
        [row[2] for row in rows], [row[3] for row in rows],
        sample_hz, snapshot["18"], snapshot["11"], steady_start_s=1.5)
    print("串口采集指标（由本次读回的四路样本计算）：")
    for line in format_motion_metrics(motion):
        print(line)

    if args.save:
        with open(args.save, "w", encoding="utf-8") as f:
            f.write("idx," + ",".join(CH_NAMES[:NCH]) + "\n")
            for i, r in enumerate(rows):
                f.write(f"{i}," + ",".join(str(x) for x in r) + "\n")
        print(f"已保存到 {args.save}")
        with open(args.save+'.json','w',encoding='utf-8') as f:
            json.dump({'firmware':version,'sample_hz':sample_hz,'capture_log2':log2,
                       'params_raw':snapshot,'filter':'block_mean' if log2 else 'none',
                       'complete':True,'samples':len(rows),
                       'motion_metrics':motion},f,ensure_ascii=False,indent=2)

    if not args.no_plot:
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            print("没装 matplotlib，跳过画图。")
            return
        fig, axes = plt.subplots(NCH, 1, sharex=True, figsize=(11, 8))
        t = [(i-args.pre)/sample_hz for i in range(len(rows))]
        for c in range(NCH):
            axes[c].plot(t, [r[c] for r in rows], lw=0.9)
            axes[c].axvline(0, color='r', ls='--', lw=0.8,
                            label='trigger')
            axes[c].set_ylabel(f"{CH_NAMES[c]}\n[{CH_UNITS[c]}]", fontsize=9)
            axes[c].grid(alpha=0.3)
        axes[0].legend(fontsize=8)
        axes[-1].set_xlabel("seconds relative to trigger")
        fig.suptitle(f"capture: trig ch{args.ch} level={args.level} "
                     f"edge={args.edge} pre={args.pre}")
        fig.tight_layout()
        plt.show()

def main():
    ap = argparse.ArgumentParser(description="倒立摆 FPGA 调试上位机")

    ap.add_argument("--port", default="", help="串口号，例如 COM3")
    ap.add_argument("--baud", type=int, default=460800,
                    help="须与 pend_top.v 的 BAUD 一致（2026-09-18 起 460800）")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("ping").set_defaults(func=cmd_ping)
    sub.add_parser("dump").set_defaults(func=cmd_dump)
    sub.add_parser("fuzzy-status", help="只读内环有效增益、调度倍率和滤波变化率").set_defaults(
        func=cmd_fuzzy_status)

    p = sub.add_parser("bank", help="选/看当前改的是哪一段的增益（0=A段 1=B段 2=稳态）")
    p.add_argument("which", nargs="?", default=None, choices=sorted(BANK_CHOICES),
                   help="0=A段 1=B段 2=稳态；不填则只显示当前")
    p.set_defaults(func=cmd_bank)

    p = sub.add_parser("get")
    p.add_argument("addr", type=int)
    p.add_argument("--bank", choices=sorted(BANK_CHOICES), default=None,
                   help="只看这一次的段，不改存盘的选择器")
    p.set_defaults(func=cmd_get)

    p = sub.add_parser("set")
    p.add_argument("addr", type=int)
    p.add_argument("value", type=float,
                   help="Q16.16 参数直接写小数，例如 1.5")
    p.add_argument("--bank", choices=sorted(BANK_CHOICES), default=None,
                   help="只写这一次的段，不改存盘的选择器")
    p.set_defaults(func=cmd_set)

    p = sub.add_parser("watch")
    p.add_argument("--addrs", default="", help="逗号分隔的地址，例如 3,4,13")
    p.add_argument("--period", type=float, default=0.2)
    p.set_defaults(func=cmd_watch)

    p = sub.add_parser("capture")
    p.add_argument("--ch", type=int, default=0)
    p.add_argument("--edge", type=int, default=0, choices=[0, 1, 2, 3],
                   help="0=上升 1=下降 2=变化 3=禁用")
    p.add_argument("--level", type=int, default=0)
    p.add_argument("--pre", type=int, default=256)
    p.add_argument("--timeout", type=float, default=None, help="默认采集窗口+8秒")
    p.add_argument("--save", default="")
    p.add_argument("--no-plot", action="store_true")
    p.add_argument("--trigger-cmd", action="store_true",
                   help="arm 之后先等回车，方便你手动释放摆杆")
    p.set_defaults(func=cmd_capture)

    args = ap.parse_args()

    if args.func is cmd_bank:
        args.func(None, args)
        return

    if not args.port:

        sys.exit("错误：这条命令要连板子，请加 `--port COM3`。"
                 "（只有 `bank` 不用端口 —— 它改的是电脑这边的段选择器。）")

    try:
        with DbgLink(args.port, args.baud) as link:
            args.func(link, args)
    except DbgError as e:
        sys.exit(f"错误：{e}")
    except Exception as e:
        if HAVE_SERIAL and isinstance(e, serial.SerialException):
            sys.exit(f"打不开串口 {args.port}：{e}")
        raise

if __name__ == "__main__":
    main()
