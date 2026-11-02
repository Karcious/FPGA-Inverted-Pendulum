import importlib.metadata
from pathlib import Path
import shutil
import subprocess
import sys

ROOT=Path(__file__).resolve().parent
NAME='FPGA-Balance-Studio-REV25'
DEST=ROOT/'dist'/NAME
if DEST.exists():raise SystemExit('Release directory already exists; move it before rebuilding')
subprocess.run([sys.executable,'-m','PyInstaller','--noconfirm','--clean','--windowed','--onedir',
    '--name',NAME,'--distpath',str(ROOT/'dist'),'--workpath',str(ROOT/'build'),
    '--specpath',str(ROOT/'build'),'--collect-submodules','bleak','--collect-submodules','winrt',
    '--exclude-module','tkinter','--exclude-module','matplotlib','--icon',str(ROOT/'assets/app.ico'),
    '--add-data',str(ROOT/'assets')+';assets',str(ROOT/'src/fpga_console.py')],check=True,cwd=ROOT)
shutil.copy2(ROOT.parent/'readme.md',DEST/'readme.md')
shutil.copytree(ROOT/'assets/licenses',DEST/'THIRD_PARTY_LICENSES/additional',dirs_exist_ok=True)
for name in ('PySide6-Essentials','shiboken6','pyserial','pyinstaller','bleak','winrt-runtime'):
    dist=importlib.metadata.distribution(name)
    for file in dist.files or []:
        if any(word in str(file).lower() for word in ('license','copying','notice')):
            source=Path(dist.locate_file(file))
            if source.is_file():
                target=DEST/'THIRD_PARTY_LICENSES'/name/str(file).replace('../','')
                target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,target)
(DEST/'THIRD_PARTY.txt').write_text(
    'Qt/PySide6 and Shiboken: LGPLv3/GPL/commercial; https://www.qt.io/licensing/\n'
    'Qt sources: https://download.qt.io/archive/qt/6.8/6.8.3/submodules/\n'
    'PySide sources: https://download.qt.io/official_releases/QtForPython/pyside6/\n'
    'Qt libraries in _internal are dynamically loaded and replaceable with compatible builds.\n'
    'pyserial: BSD; PyInstaller: GPL with bootloader exception; see THIRD_PARTY_LICENSES.\n',encoding='utf-8')
print(shutil.make_archive(str(ROOT/'dist'/(NAME+'-Windows-x64')),'zip',root_dir=ROOT/'dist',base_dir=NAME))
