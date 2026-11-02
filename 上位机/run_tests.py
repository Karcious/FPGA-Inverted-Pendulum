import os
from pathlib import Path
import subprocess
import sys
import unittest

ROOT=Path(__file__).resolve().parent
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
sys.path.insert(0,str(ROOT/'src'))
sys.path.insert(0,str(ROOT/'tests'))
suite=unittest.defaultTestLoader.discover(str(ROOT/'tests'),pattern='test_*.py')
result=unittest.TextTestRunner(verbosity=2).run(suite)
if not result.wasSuccessful():sys.exit(1)
env=dict(os.environ,PYTHONPATH=str(ROOT/'src')+os.pathsep+str(ROOT/'tests'),WORKDIR=str(ROOT.parent/'fpga/build/tests/tb_dbg_top'))
subprocess.run([sys.executable,str(ROOT/'tests/test_dbg_host.py')],check=True,cwd=ROOT,env=env)
if (ROOT.parent/'fpga/build/tests/tb_live_telemetry/live_tx.hex').exists():
    env['WORKDIR']=str(ROOT.parent/'fpga/build/tests/tb_live_telemetry')
    subprocess.run([sys.executable,'-m','unittest','test_live_host'],check=True,cwd=ROOT/'tests',env=env)
