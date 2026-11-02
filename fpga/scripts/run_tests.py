import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]

def main():
    p=argparse.ArgumentParser()
    p.add_argument('benches',nargs='*')
    p.add_argument('--timeout',type=int,default=180)
    args=p.parse_args()
    bindir=os.environ.get('IVERILOG_BIN')
    iv=str(Path(bindir)/'iverilog') if bindir else shutil.which('iverilog')
    vv=str(Path(bindir)/'vvp') if bindir else shutil.which('vvp')
    if not iv or not vv:raise SystemExit('Install Icarus Verilog or set IVERILOG_BIN')
    work=ROOT/'build/tests';work.mkdir(parents=True,exist_ok=True)
    benches=[ROOT/'tests'/(n+'.v') for n in args.benches] if args.benches else sorted((ROOT/'tests').glob('tb_*.v'))
    if any(b.stem=='tb_fuzzy_gain_audit' for b in benches):
        subprocess.run([sys.executable,str(ROOT/'scripts/audit_fuzzy_pid.py')],check=True,env=dict(os.environ,WORKDIR=str(work)))
    shutil.copy2(ROOT/'tests/swing_fault_replay.hex',work.parent/'swing_fault_replay.hex')
    sources=sorted((ROOT/'src').rglob('*.v'))
    results=[]
    for bench in benches:
        name=bench.stem
        run_dir=work/name;run_dir.mkdir(exist_ok=True)
        out=run_dir/(name+'.vvp')
        if name=='tb_fuzzy_gain_audit':shutil.copy2(work/'fuzzy_gain_vectors.txt',run_dir/'fuzzy_gain_vectors.txt')
        shutil.copy2(ROOT/'tests/swing_fault_replay.hex',work/'swing_fault_replay.hex')
        extra=[ROOT/'tests/tb_spin_plant.v'] if name in ('tb_key_motion_plant','tb_move_chain') else []
        command=[iv,'-g2012','-Wall','-I',str(ROOT/'src/debug'),'-I',str(ROOT/'tests'),'-s',name,'-o',str(out),*[str(s) for s in sources+extra],str(bench)]
        c=subprocess.run(command,cwd=run_dir,capture_output=True)
        (work/(name+'_compile.log')).write_bytes(c.stdout+c.stderr)
        ok=False
        if not c.returncode:
            try:
                r=subprocess.run([vv,str(out)],cwd=run_dir,capture_output=True,timeout=args.timeout)
                content=(r.stdout+r.stderr).decode(errors='replace')
                ok=r.returncode==0 and ('ALL PASS' in content or '*** PASS ***' in content) and '[FAIL]' not in content and 'FAILED:' not in content
            except subprocess.TimeoutExpired as exc:
                content='TIMEOUT\n'+(exc.stdout or b'').decode(errors='replace')
            (work/(name+'.log')).write_text(content,encoding='utf-8')
        else:content=(c.stdout+c.stderr).decode(errors='replace')
        results.append({'name':name,'passed':ok})
        print(name+(': PASS' if ok else ': FAIL\n'+content[-1800:]),flush=True)
    (work/'results.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
    print(f'{sum(r["passed"] for r in results)}/{len(results)} passed')
    return 0 if all(r['passed'] for r in results) else 1

if __name__=='__main__':sys.exit(main())
