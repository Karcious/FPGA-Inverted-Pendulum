#!/usr/bin/env python3

import os
import re
import struct
import sys
from contextlib import redirect_stdout
from io import StringIO

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from dbg_host import (
    DbgLink, DbgError, crc8, q16_to_float, float_to_q16,
    CMD_READ_FUZZY, cmd_fuzzy_status,
    CMD_PING, CMD_GET_PARAM, CMD_SET_PARAM, CMD_DUMP_PARAMS,
    CMD_ARM_CAPTURE, CMD_READ_CAPTURE, CMD_CAP_STATUS, CMD_DISARM,
    RESP_FLAG, DEPTH, NCH, P_MAGIC, P_CTRL_HZ, P_KP_BAL, P_KD_BAL,
    P_OPENLOOP, P_DUTY_TEST, P_CENTER_ANGLE, P_TRIM_THETA, P_POS_SLEW,
    P_KD_ARM, P_KI_ARM, P_FILT_SEL, P_THETA_SWITCH, P_OMEGA_SWITCH,
    P_CAPTURE_MS, P_WAITUP_BRAKE, N_PARAM,
    P_GAIN_A_KP, P_GAIN_A_KD, P_GAIN_B_KP, P_GAIN_B_KD,
    P_PHASE_A_MS, P_PHASE_B_MS, P_SETTLE_DAMP,
    P_MOVE_ACC, P_MOVE_DEC,
    PARAM_TABLE,
)

FAIL = []
NCHK = [0]

def check(cond, name):
    NCHK[0] += 1
    if not cond:
        FAIL.append(name)
        print(f"  [FAIL] {name}")

def test_basics():
    print("== 1. 基础向量 ==")

    check(crc8(b"123456789") == 0xF4, "CRC-8 标准向量 123456789 -> 0xF4")
    check(crc8(b"") == 0x00, "CRC-8 空输入 -> 0x00")

    check(crc8(bytes([0x81, 0x02, 0x00, 0x01, 0x00])) == 0xCC,
          "CRC-8 PING 响应体 -> 0xCC")

    for f in (0.0, 1.0, -1.0, 0.5, -0.125, 123.456, -0.00002):
        raw = float_to_q16(f)
        back = q16_to_float(raw)
        check(abs(back - f) < 1e-4, f"Q16.16 往返 {f} -> {back}")
    check(float_to_q16(-1.0) == 0xFFFF0000, "Q16.16 -1.0 -> 0xFFFF0000")
    check(q16_to_float(0x80000000) == -32768.0, "Q16.16 最小负值")

    check(PARAM_TABLE[P_CENTER_ANGLE][2] is False,
          "CENTER_ANGLE 必须是整数码值（is_q16 = False）")

    check(PARAM_TABLE[P_TRIM_THETA][2] is False,
          "TRIM_THETA 必须是整数码值（is_q16 = False）")

class ReplaySerial:

    def __init__(self, data: bytes):
        self.buf = data
        self.pos = 0
        self.written = bytearray()
        self.timeout = 2.0

    def read(self, n):
        chunk = self.buf[self.pos:self.pos + n]
        self.pos += len(chunk)
        return chunk

    def write(self, b):
        self.written += b
        return len(b)

    def flush(self):
        pass

    def reset_input_buffer(self):

        pass

    def reset_output_buffer(self):
        pass

    def close(self):
        pass

def test_fuzzy_status():

    print("== 模糊调度只读遥测 ==")

    def frame(payload, cmd=CMD_READ_FUZZY | RESP_FLAG):
        body = bytes([cmd]) + struct.pack('<H', len(payload)) + payload
        return b'\xAA\x55' + body + bytes([crc8(body)])

    request_body = bytes([CMD_READ_FUZZY, 0, 0])
    expected_request = b'\xAA\x55' + request_body + bytes([crc8(request_body)])
    flags = 288 | (320 << 10) | (1 << 20)
    payload = struct.pack('<iiiI', 258048, 8192000, -40960, flags)
    transport = ReplaySerial(frame(payload))
    status = DbgLink(transport=transport).read_fuzzy()
    check(status['kp'] == 3.9375 and status['kd'] == 125.0,
          'FUZZY effective gain Q16 conversion')
    check(status['ec'] == -0.625 and status['ec_q16'] == -40960,
          'FUZZY signed filtered ec conversion')
    check(status['kp_q16'] == 258048 and status['kd_q16'] == 8192000,
          'FUZZY exact raw effective gains')
    check(status['p_scale'] == 1.125 and status['d_scale'] == 1.25,
          'FUZZY Q8 multiplier conversion')
    check(status['p_scale_q8'] == 288 and status['d_scale_q8'] == 320,
          'FUZZY bit-field boundaries')
    check(status['enabled'] is True, 'FUZZY active bit decoded')
    check(bytes(transport.written) == expected_request,
          'FUZZY only sends zero-payload 0x0A request')
    check(transport.pos == len(transport.buf), 'FUZZY frame completely consumed')

    limits = struct.pack('<iiiI', -2147483648, -65536, 2147483647,
                         256 | (256 << 10))
    status = DbgLink(transport=ReplaySerial(frame(limits))).read_fuzzy()
    check(status['kp'] == -32768.0 and status['kd'] == -1.0,
          'FUZZY negative gains preserve sign')
    check(status['ec_q16'] == 2147483647 and
          status['ec'] == 2147483647 / 65536.0,
          'FUZZY saturated positive ec preserves precision')
    check(status['enabled'] is False and
          status['p_scale'] == status['d_scale'] == 1.0,
          'FUZZY disabled identity multiplier snapshot')
    negative_limit = struct.pack('<iiiI', 0, 0, -2147483648, 0)
    status = DbgLink(transport=ReplaySerial(frame(negative_limit))).read_fuzzy()
    check(status['ec'] == -32768.0, 'FUZZY saturated negative ec preserves sign')

    rejected = [
        ('unsupported old firmware', frame(b''), '收到 0 字节'),
        ('firmware error code', frame(b'\x02'), '错误码 0x02'),
        ('short payload', frame(payload[:-1]), '收到 15 字节'),
        ('long payload', frame(payload + b'\x00'), '收到 17 字节'),
        ('bad CRC', frame(payload)[:-1] + bytes([frame(payload)[-1] ^ 1]), 'CRC'),
        ('wrong response command', frame(payload, CMD_PING | RESP_FLAG), '命令码'),
        ('truncated frame', frame(payload)[:-3], '超时'),
    ]
    for name, stream, message in rejected:
        try:
            DbgLink(transport=ReplaySerial(stream)).read_fuzzy()
        except DbgError as exc:
            check(message in str(exc), f'FUZZY rejects {name} with useful error')
        else:
            check(False, f'FUZZY rejects {name}')

    transport = ReplaySerial(frame(payload))
    output = StringIO()
    with redirect_stdout(output):
        cmd_fuzzy_status(DbgLink(transport=transport), None)
    check(bytes(transport.written) == expected_request,
          'fuzzy-status CLI performs exactly one read-only request')
    check('Kp=3.937500' in output.getvalue() and
          'D倍率=1.250000' in output.getvalue() and
          '-0.625000 ADC码/内环采样' in output.getvalue(),
          'fuzzy-status CLI labels effective gain, multiplier and ec units')

def expected_ver():

    src = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "..", "..", "fpga", "src", "debug", "dbg_top.v")
    try:
        with open(os.path.normpath(src), encoding="utf-8") as f:
            m = re.search(r"parameter\s*\[15:0\]\s*VER\s*=\s*16'h([0-9A-Fa-f]+)", f.read())
        return int(m.group(1), 16) if m else None
    except OSError:
        return None

EXPECTED_VER = None
EXPECTED_MAGIC = None

def expected_magic():
    src = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "..", "..", "fpga", "src", "debug", "param_regs.v")
    with open(os.path.normpath(src), encoding="utf-8") as f:
        m = re.search(r"parameter\s*\[31:0\]\s*MAGIC\s*=\s*32'h([0-9A-Fa-f_]+)", f.read())
    return int(m.group(1).replace('_', ''), 16)

def load_hex(path):
    with open(path, "r") as f:
        vals = []
        for line in f:
            line = line.strip()
            if line:
                vals.append(int(line, 16))
    return bytes(vals)

def test_replay(hex_path):

    print(f"== 2. 回放 {hex_path} ==")
    data = load_hex(hex_path)
    print(f"   DUT 共发出 {len(data)} 字节")
    link = DbgLink(transport=ReplaySerial(data))

    v = link.ping()

    rev = (EXPECTED_MAGIC & 0xff) if EXPECTED_MAGIC is not None else 0
    screenshot_defaults = (EXPECTED_MAGIC is not None and EXPECTED_MAGIC >> 8 == v and
                           (rev >= 4 if v == 0x0200 else v > 0x0200))

    kp_arm_default = 0xFFFECCCD if screenshot_defaults else 0xFFFECCCD
    kd_arm_default = (0xFFD80000 if screenshot_defaults else
                      0xFF880000 if v == 0x011B else 0xFFE80000 if v >= 0x0114 else 0)
    check(v in (0x0109,0x0113,0x0114,0x0115,0x0116,0x0117,0x0118,0x0119,
                0x011A,0x011B,0x011C,0x011D,0x011E,0x011F,0x0120,0x0121,
                0x0122,0x0123,0x0124,0x0125,0x0126,0x0200,0x0201,0x0202,0x0203,
                0x0206,0x0207),
          f"PING version 0x{v:04x}")

    if EXPECTED_VER is None:
        check(True, "读不到 dbg_top.v 的 VER，跳过新旧比对")
    else:
        check(v == EXPECTED_VER,
              f"回放的字节流不是当前固件：流里是 0x{v:04x}，"
              f"dbg_top.v 现在是 0x{EXPECTED_VER:04x} —— "
              f"改了 VER 就要重新生成 sim/regression/dut_tx.hex")

    check(link.get_param(P_CTRL_HZ) == 1000, "CTRL_HZ 默认 1000")
    check(link.get_param(P_KP_BAL) == (0x00048000 if screenshot_defaults else
          0x0003_0000 if v>=0x0114 else 0x0001_0000), "KP_BAL default")

    kd_default = (0x00780000 if screenshot_defaults else 0x00C8_0000 if v>=0x011A else
                  0x0096_0000 if v>=0x0119 else
                  0x001e_0000 if v>=0x0114 else 0x0000_8000)
    check(link.get_param(P_KD_BAL) == kd_default, "KD_BAL default")

    check(link.get_param(P_KI_ARM) ==
          (0xFFFF_FC29 if v>=0x0207 else 0xFFFF_FE77 if v==0x011B else 0xFFFF_F852 if v>=0x011A else 0),
          "KI_ARM default")

    current_defaults = EXPECTED_MAGIC>>8 == 0x0207 and (EXPECTED_MAGIC&255)>=12
    filt_default = (271 if current_defaults else 15 if v>=0x011D
                    else 7 if v>=0x011A else 0x0000_4000)
    check(link.get_param(P_FILT_SEL) == filt_default, "FILT_SEL default")
    check(link.get_param(P_MAGIC) == EXPECTED_MAGIC and EXPECTED_MAGIC>>8 == v,
          "MAGIC agrees with PING and current feature revision")

    check(link.get_param(P_KD_ARM) == kd_arm_default, "KD_ARM default")

    check(link.get_param(P_WAITUP_BRAKE) == 0,
          "WAITUP_BRAKE 默认 0（越界地址会回 0xDEADBEEF，能读到 0 就证明 mem[32] 在表里）")

    check(link.get_param(P_GAIN_A_KP) == kp_arm_default, "GAIN_A_KP matches steady default")
    check(link.get_param(P_GAIN_A_KD) == kd_arm_default, "GAIN_A_KD matches steady default")
    check(link.get_param(P_GAIN_B_KP) == kp_arm_default, "GAIN_B_KP matches steady default")
    check(link.get_param(P_GAIN_B_KD) == kd_arm_default, "GAIN_B_KD matches steady default")
    check(link.get_param(P_PHASE_A_MS) == 0,
          "PHASE_A_MS 默认 0（**调度关**：冷启动逐位等于 v1.26）")
    check(link.get_param(P_PHASE_B_MS) == 0,
          "PHASE_B_MS 默认 0（无 B 段 ⟹ 就算开了调度也是二段式）")

    check(link.get_param(P_SETTLE_DAMP) == 0x0002_0000,
          "SETTLE_DAMP 默认 +2.0（非 0 ⟹ 这一格不是「默认关」）")

    link.set_param(P_KP_BAL, 0x0002_5800)
    check(link.get_param(P_KP_BAL) == 0x0002_5800, "KP_BAL 写入后回读一致")

    link.set_param(P_POS_SLEW, 0xFFFF_FF00)
    check(link.get_param(P_POS_SLEW) == 0xFFFF_FF00, "负值参数写入后回读一致")

    params = link.dump_params()

    check(len(params) == N_PARAM, f"DUMP 返回 {N_PARAM} 个参数")
    check(params[P_KP_BAL] == 0x0002_5800, "DUMP 里 KP_BAL 是新写入的值")
    check(params[P_POS_SLEW] == 0xFFFF_FF00, "DUMP 里负值参数正确")
    check(params[P_MAGIC] == EXPECTED_MAGIC, "DUMP 里 MAGIC 未被写坏（地址 0 只读）")
    check(params[P_KD_ARM] == kd_arm_default, "DUMP KD_ARM default")
    check(params[5] == kp_arm_default, "DUMP KP_ARM default")
    check(params[P_KD_BAL] == kd_default, "DUMP KD_BAL default")
    check(params[P_KI_ARM] ==
          (0xFFFF_FC29 if v>=0x0207 else 0xFFFF_FE77 if v==0x011B else 0xFFFF_F852 if v>=0x011A else 0),
          "DUMP KI_ARM default")

    check(params[P_OPENLOOP] == 0, "OPENLOOP 默认 0（上电电机不转）")
    check(params[P_DUTY_TEST] == 0, "DUTY_TEST 默认 0")

    check(params[P_TRIM_THETA] == 135, "TRIM_THETA 默认 135（对齐官方固件读数）")

    check(params[P_CENTER_ANGLE] == (3075 if current_defaults else 3086 if screenshot_defaults else 3070 if v >= 0x011E else 3095),
          "CENTER_ANGLE default (current source: 3075)")

    check(params[P_THETA_SWITCH] == (30 if v >= 0x011F else 15),
          "THETA_SWITCH 默认 30（0x011F 起）")
    check(params[P_OMEGA_SWITCH] == (600 if v >= 0x011F else 500),
          "OMEGA_SWITCH 默认 600（0x011F 起）")
    if v >= 0x0117:
        check(params[P_CAPTURE_MS] == (55 if v >= 0x011F else 10),
              "CAPTURE_MS 默认 55（0x011F 起；之前是 10）")

    if len(params) > P_WAITUP_BRAKE:
        check(params[P_WAITUP_BRAKE] == 0,
              "WAITUP_BRAKE 默认 0（刹车关 = 冷启动等于 v1.25）")

    if len(params) > P_PHASE_B_MS:
        check(params[P_GAIN_A_KP] == kp_arm_default, "DUMP GAIN_A_KP matches steady default")
        check(params[P_GAIN_A_KD] == kd_arm_default, "DUMP GAIN_A_KD matches steady default")
        check(params[P_GAIN_B_KP] == kp_arm_default, "DUMP GAIN_B_KP matches steady default")
        check(params[P_GAIN_B_KD] == kd_arm_default, "DUMP GAIN_B_KD matches steady default")
        check(params[P_PHASE_A_MS] == 0, "DUMP PHASE_A_MS 默认 0（调度关）")
        check(params[P_PHASE_B_MS] == 0, "DUMP PHASE_B_MS 默认 0（二段式）")

    if len(params) > P_SETTLE_DAMP:
        check(params[P_SETTLE_DAMP] == 0x0002_0000,
              "DUMP SETTLE_DAMP 默认 +2.0（表尾那一格，不是越界）")
    if len(params) > P_MOVE_DEC:
        soft_edges = EXPECTED_MAGIC>>8 == 0x0207 and (EXPECTED_MAGIC&255)>=11
        user_motion = EXPECTED_MAGIC>>8 == 0x0207 and (EXPECTED_MAGIC&255)>=12
        check(params[40] == (13107 if user_motion else 8192), "DUMP move slew default")
        check(params[P_MOVE_ACC] == (98 if user_motion else 10 if soft_edges else 0), "DUMP startup acceleration default")
        check(params[P_MOVE_DEC] == (30 if soft_edges else 0), "DUMP terminal deceleration default")

    link.arm_capture(ch=1, edge=0, level=500, pre_len=4)

    st, idx = link.cap_status()
    check(st & 0x02, "CAP_STATUS 的 done 位为 1")
    check((st & 0x01) == 0, "CAP_STATUS 的 armed 位在采完之后清零")
    check(idx == 5, f"trig_idx == 5（实际 {idx}）")

    rows = link.read_capture()

    check(len(rows) == 64, f"READ_CAPTURE 返回 64 个采样（实际 {len(rows)}）")
    check(all(len(r) == NCH for r in rows), "每个采样有 4 个通道")

    check(rows[0][0] == 1, f"逻辑 0 号点 ch0 == 1（实际 {rows[0][0]}）")
    check(rows[0][1] == 100, f"逻辑 0 号点 ch1 == 100（实际 {rows[0][1]}）")
    check(rows[4][1] == 500, f"触发点落在逻辑第 4 号（ch1 == 500，实际 {rows[4][1]}）")
    check(all(rows[i][0] == 1 + i for i in range(len(rows))),
          "采集数据逐点连贯（ch0 每点 +1）")

    check(rows[-1][0] == len(rows),
          "末点是逻辑最后一个采样")

    st_arm = link.arm_capture(ch=1, edge=0, level=500, pre_len=4)
    check((st_arm & 0x01) == 0, "ARM 响应里报的是 arm 之前的状态（应为未 arm）")
    st_dis = link.disarm()
    check((st_dis & 0x01) == 1,
          "DISARM 响应里报的也是执行之前的状态（仍为已 arm）")

    check(link.ser.pos == len(data),
          f"字节流完整消费（{link.ser.pos}/{len(data)}）")

    w = link.ser.written
    check(len(w) >= 6 and w[-6:-4] == bytes([0xAA, 0x55]),
          "最后一帧帧头是 AA 55")
    check(len(w) >= 6 and w[-4] == CMD_DISARM,
          f"最后一帧命令码是 DISARM（实际 {w[-4]:#04x}）")
    check(len(w) >= 6 and w[-3:-1] == bytes([0x00, 0x00]),
          "DISARM 帧长为 0")

if __name__ == "__main__":
    test_basics()
    test_fuzzy_status()

    EXPECTED_VER = expected_ver()
    EXPECTED_MAGIC = expected_magic()

    cand = [
        os.path.join(os.environ.get("WORKDIR", "sim/merged_regression"), "dut_tx.hex"),
        os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "..", "sim", "regression", "dut_tx.hex"),
        os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "..", "sim", "work", "dut_tx.hex"),
        os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "..", "sim", "dut_tx.hex"),
    ]
    hexf = next((os.path.normpath(p) for p in cand if os.path.exists(p)), None)

    if hexf is None:
        print("\n== 2. 跳过回放（没找到 dut_tx.hex）==")
        print("   先跑一次仿真：")
        print("     cd sim/work && vsim -c -do \"run -all; quit -f\" work.tb_dbg_top")
    else:
        try:
            test_replay(hexf)
        except DbgError as e:
            FAIL.append(f"回放异常：{e}")
            print(f"  [FAIL] 回放异常：{e}")

    print("\n================================")
    print(f"  检查 {NCHK[0]} 项, 失败 {len(FAIL)} 项")
    if FAIL:
        for f in FAIL:
            print(f"    - {f}")
        print("  *** FAIL ***")
        sys.exit(1)
    print("  *** PASS ***")
    print("================================")
