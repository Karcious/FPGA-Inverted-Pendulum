import json
import os
from pathlib import Path
import random

from furuta_sim import FuzzyGainScheduler

Q = 65536
BASE_INNER = (229376, 0, 6553600)
BASE_OUTER = (-98304, -1966, -2097152)

def sample_sequence():

    yield 1, 0
    for error in (0, Q, 0, -Q, 0)*20:
        yield 0, error
    yield 1, 4*Q
    for error in (5*Q, 4*Q)*100:
        yield 0, error

    for e in range(-12, 13):
        for ec in range(-12, 13):
            e_value = e*Q//2
            ec_value = ec*Q//4
            e_raw = e_value + (2*Q if e_value>0 else -2*Q if e_value<0 else 0)
            c_raw = ec_value + (Q//2 if ec_value>0 else -Q//2 if ec_value<0 else 0)
            yield 1, e_raw-4*c_raw
            yield 0, e_raw

    for sign in (-1, 1):
        yield 1, 0
        for i in range(1, 101):
            yield 0, sign*i*4*Q
        for _ in range(100):
            yield 0, sign*400*Q
        yield 1, 0
        for error in (8*Q, -8*Q)*100:
            yield 0, error
        for _ in range(100):
            yield 0, 0

    rng = random.Random(0x20006)
    for i in range(1000):
        yield int(i%79 == 0), rng.randrange(-4095*Q, 4095*Q)
    for error in (-(1<<31), (1<<31)-1, 0, (1<<31)-1, -(1<<31), 0):
        yield 0, error
    yield 1, 123*Q
    yield 0, 123*Q

def jitter_summary():
    model = FuzzyGainScheduler(*BASE_INNER, 2, 0)
    model.step(4*Q, True)
    errors = [e*Q for e in (5, 4)*100]
    gains = [model.step(e)[0] for e in errors]
    fixed_outputs = [((BASE_INNER[0]*e)>>16)/Q for e in errors]
    scheduled_outputs = [((kp*e)>>16)/Q for kp, e in zip(gains, errors)]
    fixed_span = max(fixed_outputs)-min(fixed_outputs)
    tuned_span = max(scheduled_outputs)-min(scheduled_outputs)
    return {
        'error_codes': [4, 5],
        'kp_effective_range': [min(gains)/Q, max(gains)/Q],
        'fixed_P_output_range': [min(fixed_outputs), max(fixed_outputs)],
        'scheduled_P_output_range': [min(scheduled_outputs), max(scheduled_outputs)],
        'P_output_peak_to_peak_amplification': tuned_span/fixed_span,
    }

def main():
    root = Path(__file__).resolve().parents[1]
    work = Path(os.environ.get('WORKDIR', root/'build/tests'))
    work.mkdir(parents=True, exist_ok=True)
    inner = FuzzyGainScheduler(*BASE_INNER, 2, 0)
    outer = FuzzyGainScheduler(*BASE_OUTER, 4, 1, tune_enable=False)
    lines = []
    kp_scales, kd_scales = [], []
    for clear, error in sample_sequence():
        gains_in = inner.step(error, bool(clear))
        gains_out = outer.step(error, bool(clear))
        lines.append(' '.join(map(str, (clear, error, *gains_in, *gains_out))))
        kp_scales.append(inner.kp_scale)
        kd_scales.append(inner.kd_scale)
    (work/'fuzzy_gain_vectors.txt').write_text('\n'.join(lines)+'\n', encoding='ascii')
    result = {
        'vector_format': 'clear,error,inner_kp,inner_ki,inner_kd,outer_kp,outer_ki,outer_kd',
        'vector_count': len(lines), 'effective_gain_comparisons': len(lines)*6,
        'audit_fixtures_only': True,
        'inner_base': [v/Q for v in BASE_INNER],
        'inner_at_rest': [v/Q for v in FuzzyGainScheduler(*BASE_INNER, 2, 0).step(0)],
        'outer_fixed': [v/Q for v in BASE_OUTER],
        'inner_Kp_scale_Q8_observed_range': [min(kp_scales), max(kp_scales)],
        'inner_Kd_scale_Q8_observed_range': [min(kd_scales), max(kd_scales)],
        'one_code_jitter': jitter_summary(),
        'limitations': 'Stateful numerical and synthetic noise checks; no loaded-plant acceptance claim.',
    }
    (root/'build').mkdir(parents=True, exist_ok=True)
    (root/'build/fuzzy_conservative_numeric.json').write_text(
        json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False))

if __name__ == '__main__':
    main()
