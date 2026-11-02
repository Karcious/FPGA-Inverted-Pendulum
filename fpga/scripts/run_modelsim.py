import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

ROOT=Path(__file__).resolve().parents[1]
BIN=Path(os.environ['SIMDIR']) if 'SIMDIR' in os.environ else None
BASE=ROOT/'build/tests'
BASE.mkdir(parents=True,exist_ok=True)
WORK=Path(tempfile.mkdtemp(prefix='modelsim_',dir=BASE))

def exe(name):
    result=str(BIN/(name+'.exe')) if BIN else shutil.which(name)
    if not result:raise RuntimeError('Install ModelSim and set SIMDIR to its executable directory')
    return result

def run(name,*args,timeout=240):
    p=subprocess.run([exe(name),*map(str,args)],cwd=WORK,capture_output=True,timeout=timeout)
    output=(p.stdout+p.stderr).decode(errors='replace')
    (WORK/(name+'.log')).write_text(output,encoding='utf-8')
    if p.returncode or '** Error' in output or '** Fatal' in output:raise RuntimeError(output[-3000:])
    return output

benches=[ROOT/'tests'/(n+'.v') for n in sys.argv[1:]] if sys.argv[1:] else sorted((ROOT/'tests').glob('tb_*.v'))
sources=sorted((ROOT/'src').rglob('*.v'))+sorted((ROOT/'tests').glob('tb_*.v'))
run('vlib','work')
(WORK/'sources.f').write_text('\n'.join('"'+p.as_posix()+'"' for p in sources),encoding='utf-8')
run('vlog','-sv','+incdir+'+str(ROOT/'src/debug'),'+incdir+'+str(ROOT/'tests'),'-f','sources.f')
subprocess.run([sys.executable,str(ROOT/'scripts/audit_fuzzy_pid.py')],check=True,env=dict(os.environ,WORKDIR=str(WORK)),stdout=subprocess.DEVNULL)
shutil.copy2(ROOT/'tests/swing_fault_replay.hex',BASE/'swing_fault_replay.hex')
results=[]
for bench in benches:
    dest=BASE/bench.stem;dest.mkdir(exist_ok=True)
    try:
        output=run('vsim','-c','-do','onerror {quit -code 1}; run -all; quit -f','work.'+bench.stem)
        passed=('ALL PASS' in output or '*** PASS ***' in output) and '[FAIL]' not in output and 'FAILED:' not in output
    except (RuntimeError,subprocess.TimeoutExpired) as exc:
        output=str(exc);passed=False
    (dest/'modelsim.log').write_text(output,encoding='utf-8')
    for filename in {'tb_dbg_top':'dut_tx.hex','tb_live_telemetry':'live_tx.hex','tb_fuzzy_telemetry':'fuzzy_tx.hex'}.get(bench.stem,'').split():
        if (WORK/filename).is_file():shutil.copy2(WORK/filename,dest/filename)
    results.append({'name':bench.stem,'passed':passed})
    print(bench.stem+(': PASS' if passed else ': FAIL\n'+output[-1800:]),flush=True)
(BASE/'modelsim_results.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
print(f'{sum(r["passed"] for r in results)}/{len(results)} passed',flush=True)
sys.exit(0 if all(r['passed'] for r in results) else 1)
