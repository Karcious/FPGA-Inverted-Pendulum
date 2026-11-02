import math

def _values(items):
    values = [float(value) for value in items]
    if not values:
        raise ValueError("metric input must not be empty")
    if not all(math.isfinite(value) for value in values):
        raise ValueError("metric input must contain only finite values")
    return values

def _rms(values):
    return math.sqrt(sum(value * value for value in values) / len(values))

def _std(values):
    mean = sum(values) / len(values)
    return math.sqrt(sum((value - mean) ** 2 for value in values) / len(values))

def _percentile_abs(values, percentile):
    ordered = sorted(abs(value) for value in values)
    position = (len(ordered) - 1) * percentile
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction

def _linear_rate(values, sample_hz):

    if len(values) < 2:
        return 0.0
    center = (len(values) - 1) / 2.0
    numerator = sum((index - center) * value
                    for index, value in enumerate(values))
    denominator = sum((index - center) ** 2 for index in range(len(values)))
    return sample_hz * numerator / denominator

def _slew_rms(values):
    if len(values) < 2:
        return 0.0
    steps = [right - left for left, right in zip(values, values[1:])]
    return _rms(steps)

def _reversal_count(values):

    previous = 0
    reversals = 0
    for value in values:
        sign = 1 if value > 0 else -1 if value < 0 else 0
        if not sign:
            continue
        if previous and sign != previous:
            reversals += 1
        previous = sign
    return reversals

def _longest_saturation_ms(values, limit, sample_hz):
    threshold = 0.99 * limit
    longest = current = 0
    for value in values:
        if abs(value) >= threshold:
            current += 1
            longest = max(longest, current)
        else:
            current = 0
    return 1000.0 * longest / sample_hz

def _window_metrics(adc, delta, pos, duty, center, limit, sample_hz):
    angle = [value - center for value in adc]
    reversals = _reversal_count(duty)
    seconds = len(adc) / sample_hz
    return {
        "samples": len(adc),
        "seconds": seconds,
        "angle_mean": sum(angle) / len(angle),
        "angle_rms": _rms(angle),
        "angle_std": _std(angle),
        "angle_p95_abs": _percentile_abs(angle, 0.95),
        "angle_max_abs": max(abs(value) for value in angle),
        "position_pp": max(pos) - min(pos),
        "position_drift": pos[-1] - pos[0],
        "position_drift_rate": _linear_rate(pos, sample_hz),
        "duty_rms": _rms(duty),
        "duty_slew_rms": _slew_rms(duty),
        "duty_reversal_count": reversals,
        "duty_reversals_per_s": reversals / seconds,
        "duty_sat_longest_ms": _longest_saturation_ms(
            duty, limit, sample_hz),
        "delta_rms": _rms(delta),
        "delta_p95_abs": _percentile_abs(delta, 0.95),
        "delta_max_abs": max(abs(value) for value in delta),
    }

def compute_motion_metrics(adc, delta, pos, duty, sample_hz, center,
                           duty_limit, steady_start_s=1.5):

    if not math.isfinite(sample_hz) or sample_hz <= 0:
        raise ValueError("sample_hz must be finite and positive")
    if not math.isfinite(duty_limit) or duty_limit <= 0:
        raise ValueError("duty_limit must be finite and positive")
    if not math.isfinite(steady_start_s) or steady_start_s < 0:
        raise ValueError("steady_start_s must be finite and non-negative")

    adc = _values(adc)
    delta = _values(delta)
    pos = _values(pos)
    duty = _values(duty)
    if len({len(adc), len(delta), len(pos), len(duty)}) != 1:
        raise ValueError("captured channels must have equal lengths")

    full = _window_metrics(adc, delta, pos, duty, float(center),
                           float(duty_limit), sample_hz)
    start = int(math.ceil(steady_start_s * sample_hz))
    steady = None
    if start < len(adc):
        steady = _window_metrics(adc[start:], delta[start:], pos[start:],
                                 duty[start:], float(center),
                                 float(duty_limit), sample_hz)
    return {
        "sample_hz": float(sample_hz),
        "steady_start_s": float(steady_start_s),
        "full": full,
        "steady": steady,
    }

def format_motion_metrics(result, indent="  "):

    full = result["full"]
    lines = [
        indent + "新增动态指标（全窗）：",
        indent + ("  pos 漂移速率 %+.3f cnt/s（线性拟合）" %
                  full["position_drift_rate"]),
        indent + ("  duty 变化RMS %.2f code/sample，换向 %.2f 次/s，"
                  "最长连续近限幅 %.1f ms" %
                  (full["duty_slew_rms"], full["duty_reversals_per_s"],
                   full["duty_sat_longest_ms"])),
        indent + ("  delta RMS %.3f，|delta| P95 %.3f，最大 %.0f cnt/tick" %
                  (full["delta_rms"], full["delta_p95_abs"],
                   full["delta_max_abs"])),
    ]
    steady = result["steady"]
    if steady is None:
        lines.append(indent + ("稳态窗口 t>=%.3fs：数据不足" %
                               result["steady_start_s"]))
        return lines
    lines.extend([
        indent + ("稳态窗口 t>=%.3fs（%.3fs / %d点）：" %
                  (result["steady_start_s"], steady["seconds"],
                   steady["samples"])),
        indent + ("  dev 均值 %+.2f，RMS %.2f，std %.2f，P95 %.2f，最大 %.0f code" %
                  (steady["angle_mean"], steady["angle_rms"],
                   steady["angle_std"], steady["angle_p95_abs"],
                   steady["angle_max_abs"])),
        indent + ("  pos 峰峰 %.0f，净位移 %+.0f，漂移速率 %+.3f cnt/s" %
                  (steady["position_pp"], steady["position_drift"],
                   steady["position_drift_rate"])),
        indent + ("  duty RMS %.2f，变化RMS %.2f code/sample，换向 %.2f 次/s，"
                  "最长连续近限幅 %.1f ms" %
                  (steady["duty_rms"], steady["duty_slew_rms"],
                   steady["duty_reversals_per_s"],
                   steady["duty_sat_longest_ms"])),
        indent + ("  delta RMS %.3f，|delta| P95 %.3f，最大 %.0f cnt/tick" %
                  (steady["delta_rms"], steady["delta_p95_abs"],
                   steady["delta_max_abs"])),
    ])
    return lines
