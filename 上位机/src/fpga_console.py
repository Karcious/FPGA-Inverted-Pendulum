import gui_features
import gui_quiet
from gui_tof import TofPanel
import argparse
import csv
import json
import math
import os
from pathlib import Path
import queue
import sys
import time

from PySide6.QtCore import Qt, QThread, Signal, QTimer, QPointF
from PySide6.QtGui import QColor, QIcon, QPainter, QPen, QPolygonF, QFontDatabase
from PySide6.QtWidgets import (QApplication,QMainWindow,QWidget,QVBoxLayout,QHBoxLayout,
    QLabel,QPushButton,QComboBox,QTableWidget,QTableWidgetItem,QHeaderView,QSplitter,
    QPlainTextEdit,QMessageBox,QFileDialog,QTabWidget,QAbstractItemView,QLineEdit,QDoubleSpinBox,QSlider,QCheckBox,QScrollArea,QSizePolicy)
from serial.tools import list_ports
from dbg_host import N_PARAM, DbgLink, PARAM_TABLE, decode_status
from gui_trajectory import TrajectorySession,compile_curve
from gui_wave import (WaveSession, build_cycle, cycle_formula, SHAPES, WAVE_SEEDS,
                      DEFAULT_EXPRESSION, MAX_ACCEL, MAX_AMPLITUDE,
                      MIN_PERIOD, MAX_PERIOD, degrees_of)
from gui_motion import (request_key_move, request_zero_calibration, request_angle_move,
                        request_angle_move_counts, request_travel_move,
                        request_spin_start, request_spin_pause,
                        travel_move_counts, split_move_counts, plan_angle_move,
                        move_limit_counts,
                        spin_speed_code, spin_speed_rpm, decode_motion_status,
                        move_decel_code, move_brake_counts,
                        COUNTS_PER_REV, MOVE_LIMIT_COUNTS, MOVE_CHUNK_COUNTS,
                        LEG_LIMIT_COUNTS,
                        SPIN_TICK_MS, SPIN_W_LIMIT, SPIN_RPM_PER_CODE, SPIN_RPM_SCALE)
from gui_model import READ_ONLY,LABELS,encode,display,decode,channels,DemoLink
from gui_monitor import StabilityStats,balanced,error_series,window_error

def app_icon():

    base=Path(getattr(sys,'_MEIPASS',Path(__file__).resolve().parents[1]))
    path=base/'assets'/'app.ico'
    return QIcon(str(path)) if path.exists() else QIcon()

def raise_timer_resolution():

    if os.name != 'nt':
        return None
    try:
        import ctypes
        if ctypes.windll.winmm.timeBeginPeriod(1) == 0:
            return ctypes
    except Exception:
        pass
    return None

class Worker(QThread):
    event = Signal(str,object)
    def __init__(self):
        super().__init__()
        self.commands=queue.PriorityQueue()
        self.sequence=0
        self.running=True
        self.link=None
        self.monitor=False
        self.capture=None
        self.params=None
        self.next_status=0
        self.version=0x011e
        self.live=False
        self.next_live=0

        self.pid_supported=True
        self.next_pid=0

        self.next_motion=0
        self.trajectory=TrajectorySession();self.next_trajectory=0

        self.wave=WaveSession(); self.wave_active=False
        gui_features.init(self)
    def submit(self,kind,data=None):
        self.sequence+=1
        self.commands.put((0 if kind in ('stop','quit','disconnect','trajectory_stop','wave_stop') else 1,self.sequence,kind,data))
    def disconnect(self):
        self.tof_forward=False;self.tof_key=None
        self.event.emit("tof_enabled",False)
        self.monitor=False
        self.capture=None
        if self.link:
            if self.trajectory.active:
                try:self.trajectory.stop(self.link)
                except Exception as exc:self.event.emit('log','减速停止请求未确认，板上可能仍在运行：'+str(exc))
            if self.wave.active:

                try:
                    if self.link.get_param(16)==2: self.link.set_param(16,3)
                    self.link.set_param(13,0)
                    self.event.emit('wave_note','连接断开：往复已停，交回模式 3（自动平衡/起摆）。')
                except Exception as exc:
                    self.event.emit('log','往复未能切回模式 3：'+str(exc))
            try: self.link.disarm()
            except Exception: pass
            try: self.link.close()
            except Exception: pass
        self.link=None
        self.trajectory.active=False;self.trajectory.uploading=False
        self.event.emit("trajectory_active",False)
        self.wave.active=False; self.wave.state=WaveSession.IDLE
        if self.wave_active:
            self.wave_active=False
            self.event.emit('wave_active',False)
        self.event.emit('connection',False)
        self.event.emit('monitor',False)
    def discard_writes(self):
        retained=[]
        while True:
            try:
                entry=self.commands.get_nowait()
            except queue.Empty:
                break
            if entry[2] not in ('set','fuzzy','monitor','trajectory_start','trajectory_update',
                                'key_move','zero_cal','angle_move','angle_move_counts',
                                'travel_move','spin_start','spin_pause',
                                'wave_start'):
                retained.append(entry)
        for entry in retained:
            self.commands.put(entry)
    def refresh(self):
        self.params=self.link.dump_params()
        self.event.emit('params',self.params.copy())
    def run(self):
        timer=raise_timer_resolution()
        try:
            self._loop()
        finally:
            if timer is not None:
                timer.windll.winmm.timeEndPeriod(1)
    def _loop(self):
        while self.running:
            try:
                try: _,_,kind,data=self.commands.get(timeout=.003)
                except queue.Empty: kind=None
                if kind=='quit':
                    self.disconnect(); self.running=False; continue
                if kind=='connect':
                    self.disconnect()
                    self.link=DemoLink() if data=='DEMO' else DbgLink(data,timeout=.8)
                    version=self.link.ping()
                    if version not in (0x011e,0x011f,0x0200,0x0201,0x0202,0x0203,0x0206,0x0207):
                        raise ValueError(f'当前固件 0x{version:04X}，需要 0x011E / 0x011F / 0x0200 / 0x0201 / 0x0202 / 0x0203 / 0x0206 / 0x0207')
                    self.version=version
                    self.live=version in (0x011f,0x0200,0x0201,0x0202,0x0203,0x0206,0x0207)
                    self.pid_supported=True
                    self.refresh()
                    if self.params[0]>>8==0x0207 and self.params[0]&255>=2:
                        self.trajectory.revision=self.params[0]&255
                        self.trajectory.active=bool(self.link.get_param(64)&1)
                        self.event.emit('trajectory_active',self.trajectory.active)
                    self.event.emit('connection',data)
                    self.event.emit('capability',version)
                elif kind=='disconnect': self.disconnect()
                elif kind and gui_features.command(self,kind,data):pass
                elif kind and self.link:
                    if gui_features.guard(self,kind):continue
                    if self.wave.active and kind in ('key_move','zero_cal','angle_move',
                                                     'angle_move_counts','travel_move',
                                                     'spin_start','spin_pause'):

                        self.event.emit('motion_note','往复轨迹正在运行：先点「停止」，再走位 / 旋转 / 标定。')
                        continue
                    if kind=='refresh': self.refresh()
                    elif kind=='acquisition':
                        self.live=bool(data) and self.version in (0x011f,0x0200,0x0201,0x0202,0x0203,0x0206,0x0207)
                        if self.capture is not None: self.link.disarm()
                        self.capture=None
                        self.event.emit('reset_live',None)
                    elif kind=='key_move':
                        try:
                            if self.trajectory.active:
                                raise ValueError('请先停止速度轨迹并等待停稳，再使用左右90°按钮')
                            self.event.emit('motion_note',request_key_move(self.link,data))
                        except ValueError as exc:self.event.emit('motion_note',str(exc))
                    elif kind=='zero_cal':
                        try:
                            self.event.emit('motion_note',request_zero_calibration(self.link))
                        except ValueError as exc:self.event.emit('motion_note',str(exc))
                    elif kind=='angle_move':

                        try:
                            note,counts=request_angle_move(self.link,*data)
                            self.event.emit('motion_note',note)
                            self.event.emit('angle_move_sent',counts)
                        except ValueError as exc:self.event.emit('motion_note',str(exc))
                    elif kind=='angle_move_counts':

                        counts,gap=data
                        if gap: time.sleep(0.15)
                        try:
                            note,got=request_angle_move_counts(self.link,counts)
                            self.event.emit('motion_note',note)
                            self.event.emit('angle_move_sent',got)
                        except ValueError as exc:
                            self.event.emit('motion_note',str(exc))

                            self.event.emit('move_chain_failed',None)
                    elif kind=='travel_move':

                        try:
                            note,counts=request_travel_move(self.link,*data)
                            self.event.emit('motion_note',note)
                            self.event.emit('angle_move_sent',counts)
                        except ValueError as exc:self.event.emit('motion_note',str(exc))
                    elif kind=='spin_start':
                        try:
                            self.event.emit('motion_note',request_spin_start(self.link,*data))

                            self.event.emit('spin_started',None)
                            self.refresh()
                        except ValueError as exc:self.event.emit('motion_note',str(exc))
                    elif kind=='spin_pause':
                        try:
                            note,was_running=request_spin_pause(self.link)
                            self.event.emit('motion_note',note)
                            self.event.emit('spin_pause_sent',(was_running,bool(data)))
                            self.refresh()
                        except ValueError as exc:self.event.emit('motion_note',str(exc))
                    elif kind=='trajectory_update':
                        try:
                            self.trajectory.update(self.link,*data)
                            self.event.emit('trajectory_knobs_applied',data)
                            self.event.emit('trajectory_note','实时幅值/周期已写入并核对；速度按斜坡过渡。')
                        except ValueError as exc:self.event.emit('trajectory_note',str(exc))
                    elif kind in ('trajectory_start','trajectory_stop'):
                        if kind=='trajectory_stop':self.discard_writes()
                        try:
                            if kind=='trajectory_start':
                                if self.capture is not None:self.link.disarm();self.capture=None
                                self.trajectory.begin(self.link,data)
                            else:self.trajectory.stop(self.link)
                            self.event.emit('trajectory_note','正在上传曲线…' if kind=='trajectory_start' else '已请求减速停止，保持倒立。')
                        except ValueError as exc:self.event.emit('trajectory_note',str(exc))
                        self.event.emit('trajectory_active',self.trajectory.active)
                    elif kind=='wave_start':

                        if self.trajectory.active:
                            self.event.emit('wave_note','请先停止速度轨迹并等它停稳，再开始往复。')
                        else:
                            try:
                                self.wave.begin(self.link,data)
                            except ValueError as exc:
                                self.event.emit('wave_note',str(exc))
                            else:
                                self.wave_active=True
                                self.event.emit('wave_active',True)
                                self.event.emit('wave_note',self.wave.note)
                    elif kind=='wave_stop':
                        if not self.wave.active:
                            self.event.emit('wave_note','往复没有在运行。')
                        else:
                            self.wave.stop(self.link)
                            self.event.emit('wave_note',self.wave.note)
                            self.wave_active=self.wave.active
                            self.event.emit('wave_active',self.wave.active)
                    elif kind=='fuzzy':

                        if self.capture is not None:
                            self.link.disarm(); self.capture=None
                        cur=self.link.get_param(9)
                        want=(cur|0x100) if data else (cur&~0x100)
                        self.link.set_param(9,want)
                        if self.link.get_param(9)!=want:
                            raise ValueError('地址 9 回读不一致')
                        self.refresh()
                        self.event.emit('log',f"模糊 PID 已{'打开' if data else '关闭'}："
                                              f"地址 9 = {cur} → {want}（只动 bit8）")
                        self.event.emit('reset_live',None)
                    elif kind in ('set','stop'):
                        if kind=='stop': self.discard_writes()
                        addr,value=(16,0) if kind=='stop' else data
                        if self.trajectory.active and kind!='stop':
                            self.event.emit('trajectory_note','请先停止曲线再修改参数。');continue
                        if self.wave.active and kind!='stop':
                            self.event.emit('wave_note','请先停止往复轨迹再修改参数。');continue
                        if self.capture is not None:
                            self.link.disarm(); self.capture=None
                        self.link.set_param(addr,value)
                        if self.link.get_param(addr)!=value:
                            raise ValueError(f'地址 {addr} 回读不一致')
                        if kind=='stop':
                            self.trajectory.active=False;self.trajectory.uploading=False
                            self.event.emit('trajectory_active',False)
                        self.refresh()
                        self.event.emit('log' ,f'已写入并核对：{addr} {PARAM_TABLE[addr][0]} = {display(addr,value)}')
                        self.event.emit('written',addr)
                        self.event.emit('reset_live',None)
                    elif kind=='monitor':
                        self.monitor=bool(data)
                        if not self.monitor:
                            self.link.disarm(); self.capture=None
                        self.event.emit('monitor',self.monitor)
                if self.link:
                    now=time.monotonic()
                    gui_features.poll(self,now)
                    if self.trajectory.active and now>=self.next_trajectory:
                        try:self.event.emit('trajectory_note',self.trajectory.poll(self.link))
                        except ValueError as exc:
                            self.trajectory.stop(self.link);self.event.emit('trajectory_note',str(exc))
                        self.event.emit('trajectory_active',self.trajectory.active)
                        if self.trajectory.snapshot:self.event.emit('trajectory_progress',self.trajectory.snapshot.copy())
                        self.next_trajectory=now+(.005 if self.trajectory.uploading else .1)

                    if self.wave.active:
                        out=self.wave.poll(self.link,now)
                        if out is not None and not out.get('done'):
                            self.event.emit('wave_sample',out)
                        elif out is not None:
                            self.event.emit('wave_note',out['note'])
                        if self.wave.active!=self.wave_active:
                            self.wave_active=self.wave.active
                            self.event.emit('wave_active',self.wave.active)
                            if not self.wave.active:
                                self.event.emit('wave_note',self.wave.note)
                    if now>=self.next_status:
                        mode=self.link.get_param(16)
                        status=self.link.get_param(31)
                        if self.params is not None:
                            if self.params[16]!=mode: self.event.emit('reset_live',None)
                            self.params[16],self.params[31]=mode,status
                        self.event.emit('status',(mode,status))
                        self.next_status=now+.5

                    if (self.version>=0x0200 and not self.trajectory.uploading
                            and now>=self.next_motion):
                        self.next_motion=now+.1
                        try:
                            status=decode_motion_status(self.link.get_param(75))
                            try:
                                raw=self.link.get_param(77)
                                if raw&0x80000000: raw-=0x100000000
                                status['vel']=raw/65536.0
                            except Exception:
                                status['vel']=None
                            self.event.emit('motion_status',status)
                        except Exception:pass

                    if (self.monitor and self.live and self.capture is None and
                            not self.trajectory.uploading and now>=self.next_pid):
                        self.next_pid=now+.5
                        if self.pid_supported:
                            try:
                                sample=self.link.read_fuzzy()
                                sample['received']=time.monotonic()
                                self.event.emit('pid',sample)
                            except Exception:
                                self.pid_supported=False
                                self.event.emit('pid_unavailable',None)
                    if self.monitor and self.live:
                        if now>=self.next_live:
                            sample=self.link.read_live()
                            self.params[31]=sample['status']
                            self.event.emit('live',{**sample,'params':self.params.copy(),
                                                   'received':time.time()})
                            self.next_live=now+.020
                    elif self.monitor:
                        if self.capture is None:
                            self.refresh()
                            if not 0<=self.params[21]<=4:
                                raise ValueError('采集指数必须为 0～4，请先修正地址 21')
                            self.capture={'params':self.params.copy(),'started':time.time(),
                                          'clock':time.monotonic(),'sample_hz':1000/(1<<self.params[21])}
                            self.link.arm_capture(0,2,0,0)
                            self.event.emit('collecting',4096/self.capture['sample_hz'])
                        else:
                            state,_=self.link.cap_status()
                            if state & 2:
                                rows=self.link.read_capture(retries=0)
                                if len(rows)!=4096: raise ValueError('采集长度不是 4096 点')
                                self.event.emit('frame',{**self.capture,'rows':rows,'received':time.time()})
                                self.capture=None
                            elif now-self.capture['clock']>4096/self.capture['sample_hz']+8:
                                raise ValueError('采集触发或完成超时')
            except Exception as exc:
                self.event.emit('error',str(exc))
                self.disconnect()

class Plot(QWidget):
    def __init__(self,title,color,unit):
        super().__init__()
        self.title,self.color,self.unit=title,QColor(color),unit
        self.values=[]; self.fs=1000; self.times=None; self.marker=None

        self.overlay=None; self.overlay_color=QColor('#d97a16')
        self.setMinimumHeight(155)
    def set_data(self,values,fs,times=None):
        self.values,self.fs,self.times=values,fs,times; self.update()
    def set_overlay(self,values):
        self.overlay=values; self.update()
    def set_marker(self,t,target,actual=None):
        self.marker=(t,target,actual);self.update()
    def paintEvent(self,event):
        p=QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(),QColor('#fbf9fd'))
        p.setPen(QColor('#4b087b')); p.drawText(14,24,self.title+'  ['+self.unit+']')
        left,top,w,h=65,40,max(1,self.width()-90),max(1,self.height()-68)
        if not self.values:
            p.setPen(QColor('#9486a4')); p.drawText(70,85,'等待采集；离线演示可预览界面')
            return
        extent=self.values+([v for v in self.marker[1:] if v is not None] if self.marker else [])
        if self.overlay: extent=extent+[v for v in self.overlay if v is not None]
        lo,hi=min(extent),max(extent)
        margin=max((hi-lo)*.12,.2)
        lo-=margin; hi+=margin
        for j in range(5):
            y=top+h*j/4
            p.setPen(QColor('#e6dfef')); p.drawLine(left,int(y),left+w,int(y))
            p.setPen(QColor('#8a7b98')); p.drawText(4,int(y)+4,f'{hi-(hi-lo)*j/4:.1f}')

        p.setPen(QPen(self.color,1.2))
        n=len(self.values)
        for x in range(w if self.times is None else 0):
            chunk=self.values[x*n//w:max(x*n//w+1,(x+1)*n//w)]
            if not chunk: continue
            y1=top+(hi-max(chunk))/(hi-lo)*h
            y2=top+(hi-min(chunk))/(hi-lo)*h
            p.drawLine(QPointF(left+x,y1),QPointF(left+x,max(y1+.8,y2)))
        if self.times is not None and len(self.times)>1 and self.times[-1]>0:
            points=QPolygonF([QPointF(left+t/self.times[-1]*w,top+(hi-v)/(hi-lo)*h)
                              for t,v in zip(self.times,self.values)])
            p.drawPolyline(points)
        if self.overlay and self.times is not None and len(self.times)>1 and self.times[-1]>0:

            p.setPen(QPen(self.overlay_color,1.2))
            run=[]
            for t,v in zip(self.times,self.overlay):
                if v is None:
                    if len(run)>1: p.drawPolyline(QPolygonF(run))
                    run=[]
                else:
                    run.append(QPointF(left+t/self.times[-1]*w,top+(hi-v)/(hi-lo)*h))
            if len(run)>1: p.drawPolyline(QPolygonF(run))
        if self.overlay is not None:
            p.setPen(QColor('#0f9b7c')); p.drawText(left+w-110,24,'目标')
            p.setPen(QColor('#d97a16')); p.drawText(left+w-58,24,'实测')
        if self.marker and self.times and self.times[-1]>0:
            t,target,actual=self.marker
            x=left+max(0,min(1,t/self.times[-1]))*w

            p.setPen(QPen(QColor('#7b5c96'),1));p.drawLine(QPointF(x,top),QPointF(x,top+h))
            for v,color in ((target,'#0f9b7c'),(actual,'#d97a16')):
                if v is not None:
                    p.setBrush(QColor(color));p.setPen(QPen(QColor('#fbf9fd'),1.5))
                    p.drawEllipse(QPointF(x,top+(hi-v)/(hi-lo)*h),5,5)
            p.setBrush(Qt.NoBrush)
        p.setPen(QColor('#8a7b98'))
        p.drawText(left,top+h+20,'0 s')
        p.drawText(left+w-90,top+h+20,f'{(self.times[-1] if self.times else n/self.fs):.3f} s')

class Window(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('FPGA Balance Studio · REV25 · 实时遥测版')
        self.setWindowIcon(app_icon())
        self.resize(1400,920)
        self.params=None; self.frame=None; self.connected=False; self.demo=False
        self.dirty={}; self.updating=False; self.monitoring=False; self.closing=False
        self.last_status=None

        self.syncing=False
        self.trajectory_busy=False;self.trajectory_running=False;self.running_curve=None;self.running_amplitude=None;self.applied_gain=1;self.applied_rate=1

        self.wave_busy=False; self.wave_state=WaveSession.IDLE
        self.wave_times=[]; self.wave_targets=[]; self.wave_actuals=[]
        self.wave_t0=None; self.wave_plot_dirty=False
        self.live_rows=[]; self.live_ticks=[]; self.version=0x011e; self.live_origin=None

        self.stats=StabilityStats(); self.stats_time=0.0; self.stats_tick=None
        self.last_errors=None
        self.pid_sample=None; self.pid_available=True

        self.last_motion=None; self.move_pending=None; self.move_baseline=None

        self.move_queue=[]; self.move_chain_total=0
        self.move_chain_done=0; self.move_chain_net=0; self.move_chain_target=0
        self.pause_waiting=None

        self.mode4_ever_spun=False

        self.last_move_target=None
        self.spin_angle=0.0; self.spin_vel_prev=None; self.spin_vel_t=None
        self.spin_w_prev=0
        self.worker=Worker(); self.worker.event.connect(self.on_event); self.worker.start()
        root=QWidget(); self.setCentralWidget(root); layout=QVBoxLayout(root)

        header=QWidget(); header.setObjectName('appHeader')
        head_layout=QVBoxLayout(header); head_layout.setContentsMargins(18,12,18,12)
        head_layout.setSpacing(1)
        title=QLabel('FPGA  /  BALANCE STUDIO'); title.setObjectName('appTitle')
        subtitle=QLabel('倒立摆调试台   ·   0x020719 / REV25 · 定向走位（可多圈）/ 连续旋转 / 标定0点 / 模糊 PID   ·   实时遥测 / 完整波形 / 参数调试')
        subtitle.setObjectName('appSubtitle')
        head_layout.addWidget(title); head_layout.addWidget(subtitle)
        layout.addWidget(header)
        bar=QHBoxLayout(); layout.addLayout(bar)
        self.port=QComboBox(); self.port.setEditable(True); self.scan_ports()
        bar.addWidget(QLabel('串口')); bar.addWidget(self.port)
        self.button(bar,'刷新端口',self.scan_ports)
        self.connect_button=self.button(bar,'连接设备',self.connect_device,'primary')
        self.demo_button=self.button(bar,'离线演示',self.connect_demo)
        self.disconnect_button=self.button(bar,'断开',lambda:self.worker.submit('disconnect'))
        bar.addStretch()
        self.connection=QLabel('未连接'); bar.addWidget(self.connection)
        self.stop_button=self.button(bar,'停止电机',lambda:self.worker.submit('stop'))
        self.stop_button.setStyleSheet('background:#d83583;color:white;border:1px solid #d83583;font-weight:bold;padding:10px 22px')
        self.notice=QLabel('现有固件采用整窗回读：默认约 5 秒刷新；断开连接不会停止电机。')
        self.notice.setWordWrap(True); layout.addWidget(self.notice)

        cards=QHBoxLayout(); layout.addLayout(cards)
        self.angle=self.card(cards,'角度环误差 · 摆杆','— °')
        self.angle.note_label.setText('ch0 − 地址18')
        self.arm=self.card(cards,'位置环误差 · 摆臂','— °')
        self.arm.note_label.setText('编码器 − 地址13')
        self.duty=self.card(cards,'实际 duty','— / 1024')
        self.age=self.card(cards,'数据新鲜度','尚无波形')

        averages=QHBoxLayout(); layout.addLayout(averages)
        self.angle_avg=self.card(averages,'角度环平均误差 · RMS','— °',obj='rmseCard',value_name='rmseValue')
        self.arm_avg=self.card(averages,'位置环平均误差 · RMS','— °',obj='rmseCard',value_name='rmseValue')
        self.delay=self.card(averages,'平衡到稳态平均用时','— s')
        delay_tip=('从"确认平衡"到第一次连续 1 s 留在带内所用的时间。\n'
                   '先开监控再起摆才有起点；杆已经立住后才开监控，这一项永远没有数。')
        for w in (self.delay,self.delay.note_label):
            w.setToolTip(delay_tip)
        self.pid_card_widget(averages)

        average_bar=QHBoxLayout(); layout.addLayout(average_bar)
        self.avg_enable=QCheckBox('显示平均误差')
        self.avg_enable.setChecked(True)
        self.avg_enable.setToolTip('勾上 = 两张粉卡显示窗口内 RMS；\n时间窗口由右边的秒数决定。')
        average_bar.addWidget(self.avg_enable)
        self.avg_window=QDoubleSpinBox()
        self.avg_window.setRange(.2,60); self.avg_window.setDecimals(1)
        self.avg_window.setSingleStep(.5); self.avg_window.setValue(5.0)
        self.avg_window.setSuffix(' s'); self.avg_window.setFixedWidth(110)
        self.avg_window.setToolTip('平均误差的统计窗口，可自由输入 0.2~60 s。\n实时遥测保留最近约 60 秒数据；整窗回读按点数折算。')
        average_bar.addWidget(self.avg_window)
        average_bar.addWidget(QLabel('窗口内 RMS（相对目标）'))
        self.avg_enable.toggled.connect(lambda:self.refresh_average())
        self.avg_window.valueChanged.connect(lambda:self.refresh_average())
        self.button(average_bar,'清空稳态统计',self.clear_statistics)
        average_bar.addStretch()
        self.status=QLabel('运行状态：未连接'); layout.addWidget(self.status)
        split=QSplitter(Qt.Horizontal); layout.addWidget(split,1)
        panel=QWidget(); pl=QVBoxLayout(panel); split.addWidget(panel)
        controls=QHBoxLayout(); pl.addLayout(controls)
        self.monitor_button=self.button(controls,'开始波形监控',self.toggle_monitor)
        self.acquisition=QComboBox(); self.acquisition.addItems(['实时遥测 ~50Hz','完整窗口'])
        self.acquisition.setCurrentIndex(1); self.acquisition.setEnabled(False)
        self.acquisition.currentIndexChanged.connect(lambda i:self.worker.submit('acquisition',i==0))
        controls.addWidget(self.acquisition)
        self.button(controls,'导出波形 CSV',self.export_frame)
        self.button(controls,'导出参数 JSON',self.export_params)
        self.plots=[Plot('摆杆角度 · 相对 CENTER_ANGLE','#0f9b7c','°'),
                    Plot('摆臂位置 · 固件相对坐标','#3d6fd8','°'),
                    Plot('电机输出 · 采样值','#d97a16','duty')]
        for chart in self.plots: pl.addWidget(chart,1)
        self.capture_note=QLabel('角度零位以地址 18 为准；摆臂 1040 码/圈。'); self.capture_note.setWordWrap(True)
        pl.addWidget(self.capture_note)
        self.tabs=QTabWidget();split.addWidget(self.tabs)

        params_panel=QWidget(); right=QVBoxLayout(params_panel);self.tabs.addTab(params_panel,"参数调试")
        buttons=QHBoxLayout(); right.addLayout(buttons)
        self.refresh_button=self.button(buttons,'回读全部参数',lambda:self.worker.submit('refresh'))
        self.write_button=self.button(buttons,'写入选中行',self.write_selected)
        self.button(buttons,'撤销选中编辑',self.revert_selected)
        self.table=QTableWidget(N_PARAM,5)
        self.table.setHorizontalHeaderLabels(['地址 / 参数','功能','板上回读','待写值','状态'])
        self.table.verticalHeader().hide()
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(1,QHeaderView.Stretch)
        self.table.itemChanged.connect(self.edited)
        right.addWidget(self.table)
        for a in range(N_PARAM):
            for c,text in enumerate([f'{a:02d} {PARAM_TABLE[a][0]}',LABELS[a],'—','', '只读' if a in READ_ONLY else '未连接']):
                item=QTableWidgetItem(text)
                if c!=3 or a in READ_ONLY: item.setFlags(item.flags() & ~Qt.ItemIsEditable)
                self.table.setItem(a,c,item)
        right.addWidget(QLabel('修改后不会自动下发；点击“写入选中行”并核对回读。'))

        modes=QHBoxLayout(); right.addLayout(modes)
        self.auto_button=self.button(modes,'自动起摆（模式 3）',lambda:self.set_mode(3))
        self.closed_button=self.button(modes,'手扶闭环（模式 2）',lambda:self.set_mode(2))
        knobs=QHBoxLayout(); right.addLayout(knobs)

        self.fuzzy_switch=self.toggle_button(knobs,'模糊 PID（内环）')
        self.fuzzy_switch.setToolTip('按下 = 打开 mem[9] 的 bit8，其余滤波位一个都不动。\n'
                                     '板上电默认 271 = 15 | bit8，进来时这个开关就是按下的。\n'
                                     '实测：开模糊后 duty 抖动约翻倍、撞限幅次数多一个数量级，'
                                     '杆侧收益不明显 —— 想对照就再按一次关掉。')
        self.fuzzy_switch.toggled.connect(self.toggle_fuzzy)

        knobs.addSpacing(8)
        comp_label=QLabel('死区补偿形状')
        comp_label.setSizePolicy(QSizePolicy.Expanding,QSizePolicy.Preferred)
        comp_label.setMinimumWidth(140)
        knobs.addWidget(comp_label)
        self.comp_shape=QComboBox()
        for text,s in (('×2',0),('×3（默认）',1),('×5',2),('×9',3)):
            self.comp_shape.addItem(text,s)
        self.comp_shape.setSizePolicy(QSizePolicy.Fixed,QSizePolicy.Fixed)
        self.comp_shape.setFixedWidth(130)
        self.comp_shape.setToolTip('小信号时把 duty 再放大几倍，专治「低速大死区」。\n'
                                   '×2 是 REV12 的老斜坡（逐位相同）；\n'
                                   '×3/×5/×9 会让最小可动 duty 变小，代价是低速更硬、更容易抖。\n'
                                   'REV13 起上电默认是 ×3；写地址 50（参数表里也能直接改）。')
        self.comp_shape.currentIndexChanged.connect(self.toggle_comp_shape)
        knobs.addWidget(self.comp_shape)

        key_row=QHBoxLayout();right.addLayout(key_row)

        self.move_left_button=self.button(key_row,'逆时针 90°（SW2）',lambda:self.worker.submit('key_move',-1))

        self.zero_button=self.button(key_row,'标定 0 点（=长按 SW3）',lambda:self.worker.submit('zero_cal'))
        self.zero_button.setToolTip('当前摆臂位置记为 0 点，等同长按 SW3 约 2 秒。\n'
                                    '只在两种状态放行：滑行中（模式 0），或平衡且摆臂停稳。\n'
                                    '其余状态板上会拒绝，并把原因写在下面的提示里。')
        self.move_right_button=self.button(key_row,'顺时针 90°（SW4）',lambda:self.worker.submit('key_move',1))
        self.move_left_button.setToolTip('等同短按 SW2：编码器减少260码（逆时针90°）')
        self.move_right_button.setToolTip('等同短按 SW4：编码器增加260码（顺时针90°）')
        self.motion_note=QLabel('未连接或未平衡时，下面的按钮是灰的：先连接，再自动起摆到模式 3。逆时针对应 SW2，顺时针对应 SW4。')
        self.motion_note.setWordWrap(True);right.addWidget(self.motion_note)
        self.build_motion_panel()
        self.build_trajectory_panel()
        self.build_wave_panel()
        self.build_quiet_panel()
        self.tof_panel=TofPanel(self.worker);self.tabs.addTab(self.tof_panel,"蓝牙测距")
        self.log=QPlainTextEdit(); self.log.setReadOnly(True); self.log.setMaximumHeight(105)
        self.log.setMaximumBlockCount(300); layout.addWidget(self.log)
        split.setSizes([680,700])
        self.timer=QTimer(self); self.timer.timeout.connect(self.update_age); self.timer.start(250)
        self.set_controls(False)
    def button(self,parent,text,callback,role=None):
        b=QPushButton(text); b.clicked.connect(callback)
        if role: b.setProperty('role',role)
        parent.addWidget(b); return b
    def toggle_button(self,parent,text):

        b=QPushButton(text); b.setCheckable(True); b.setProperty('role','toggle')
        parent.addWidget(b); return b
    def build_motion_panel(self):

        panel=QWidget(); box=QVBoxLayout(panel)
        scroll=QScrollArea(); scroll.setWidgetResizable(True); scroll.setWidget(panel)
        self.tabs.addTab(scroll,'走位与旋转')
        note=QLabel('设定角度 → 电机自己走过去、停稳，停下的地方成为新 0 点。\n'
                    '「到点误差」= 停稳时实测位置 − 目标位置（地址 75）。')
        note.setWordWrap(True); box.addWidget(note)
        row=QHBoxLayout(); box.addLayout(row)
        row.addWidget(QLabel('走多少度'))
        self.move_angle=QDoubleSpinBox(); self.move_angle.setRange(.5,1800)
        self.move_angle.setDecimals(1); self.move_angle.setSingleStep(5)
        self.move_angle.setValue(30); self.move_angle.setSuffix(' °')
        self.move_angle.setFixedWidth(120)
        self.move_angle.setToolTip('≤180° 走位置环（命令 4）；超过 180° 自动改用速度环多圈\n'
                                   '走位（命令 5，需要 REV16 固件），最高 ±31 圈（±32767 码）。\n'
                                   '速度取下面「转速」框的值。')
        row.addWidget(self.move_angle)
        self.move_ccw_button=self.button(row,'逆时针走（SW2 方向）',lambda:self.start_angle_move(-1))
        self.move_cw_button=self.button(row,'顺时针走（SW4 方向）',lambda:self.start_angle_move(1))
        row.addStretch()

        frow=QHBoxLayout(); box.addLayout(frow)
        frow.addWidget(QLabel('走位转速'))
        self.move_slew=QDoubleSpinBox(); self.move_slew.setRange(.02,1.2)
        self.move_slew.setDecimals(3); self.move_slew.setSingleStep(.01)
        self.move_slew.setValue(.2); self.move_slew.setSuffix(' 码/ms')
        self.move_slew.setFixedWidth(150)
        self.move_slew.setToolTip('地址 40（MOVE_SLEW，Q16.16 码/ms）：走位参考爬多快。\n'
                                  '0.200 码/ms ≈ 0.19 圈/秒 ⟹ 走 4 圈约 21 秒。\n'
                                  '调大就快，但到点更容易冲过头；调小更稳更慢。\n'
                                  '写 0 = 回落到固件编译期默认（在参数表里写）。')
        frow.addWidget(self.move_slew)
        frow.addWidget(self.button(frow,'写进板子',self.write_move_slew))
        self.move_slew_note=QLabel(''); self.move_slew_note.setObjectName('muted')
        frow.addWidget(self.move_slew_note)
        frow.addStretch()

        self.move_slew.valueChanged.connect(lambda v:self.refresh_move_preview())
        self.move_preview=QLabel(); self.move_preview.setObjectName('muted'); self.move_preview.setWordWrap(True)
        box.addWidget(self.move_preview)

        self.move_angle.valueChanged.connect(lambda v:self.refresh_move_preview())
        self.move_result=QLabel('还没有走位记录。'); self.move_result.setWordWrap(True)
        box.addWidget(self.move_result)

        self.move_live=QLabel('实时：还没有走位在跑。'); self.move_live.setWordWrap(True)
        self.move_live.setObjectName('metricValue')
        box.addWidget(self.move_live)

        title=QLabel('连续旋转（模式 4 · 速度环）'); title.setStyleSheet('font-weight:bold;margin-top:10px')
        box.addWidget(title)

        spin_per_code=SPIN_RPM_PER_CODE*SPIN_RPM_SCALE
        spin=QLabel(f'mem[43] 是整数码/25ms（夹 ±{SPIN_W_LIMIT}）；这里的 rpm 已按 1.4 '
                    f'标定过（1 码 ≈ {spin_per_code:.1f} rpm），实用约 '
                    f'{12*spin_per_code:.0f}～{42*spin_per_code:.0f} rpm（mem[43] 别超 42）。')
        spin.setWordWrap(True); spin.setObjectName('muted'); box.addWidget(spin)
        srow=QHBoxLayout(); box.addLayout(srow)
        srow.addWidget(QLabel('转速'))
        self.spin_rpm=QDoubleSpinBox()
        self.spin_rpm.setRange(spin_per_code,SPIN_W_LIMIT*spin_per_code)
        self.spin_rpm.setDecimals(1); self.spin_rpm.setSingleStep(5)
        self.spin_rpm.setValue(30); self.spin_rpm.setSuffix(' rpm'); self.spin_rpm.setFixedWidth(130)
        srow.addWidget(self.spin_rpm)
        self.spin_ccw_button=self.button(srow,'逆时针连续转',lambda:self.start_spin(-1))
        self.spin_cw_button=self.button(srow,'顺时针连续转',lambda:self.start_spin(1))
        srow.addStretch()
        self.spin_preview=QLabel(); self.spin_preview.setObjectName('muted'); self.spin_preview.setWordWrap(True)
        box.addWidget(self.spin_preview)
        self.spin_rpm.valueChanged.connect(lambda v:self.refresh_spin_preview())

        self.spin_rpm.valueChanged.connect(lambda v:self.refresh_move_preview())
        self.refresh_spin_preview()
        self.refresh_move_preview()
        prow=QHBoxLayout(); box.addLayout(prow)
        self.spin_pause_button=self.button(prow,'暂停（停稳后定零）',self.pause_spin)
        self.spin_pause_button.setToolTip('把 mem[43] 写 0：板上自己软减速，等摆臂真停住、杆还立着，\n'
                                          '才把当前位置搬成新的 0 点（地址 75 的 bit0 = 已停靠）。\n'
                                          '等多久取决于转速，通常 0.5～2 秒；这期间别走位。')
        self.pause_switch=QCheckBox('停稳后切回普通平衡（模式 3）')
        self.pause_switch.setChecked(True)
        self.pause_switch.setToolTip('停靠确认后再写 16=3。零点已经搬好，切模式不会让摆臂一抖。\n'
                                     '不勾就留在模式 4（给定 0，位置环同样守在新零点），下次起转更快。')
        prow.addWidget(self.pause_switch)
        prow.addStretch()
        self.spin_state=QLabel('未在连续旋转。'); self.spin_state.setWordWrap(True)
        box.addWidget(self.spin_state)

        self.spin_live=QLabel('实时：未在转。'); self.spin_live.setWordWrap(True)
        self.spin_live.setObjectName('metricValue')
        box.addWidget(self.spin_live)
        self.spin_note=QLabel(''); self.spin_note.setWordWrap(True); box.addWidget(self.spin_note)
        help_text=QLabel('走位要模式 3；连续转是模式 4。'
                         '到点或暂停停稳后，零点都会搬到停下的地方。')
        help_text.setWordWrap(True); help_text.setObjectName('muted'); box.addWidget(help_text)
        box.addStretch()

    def move_plan(self,counts):

        p=self.params or []
        return plan_angle_move(counts,p[0] if p else 0)

    def refresh_move_preview(self):
        if not hasattr(self,'move_preview') or not hasattr(self,'spin_rpm'):
            return
        try:
            counts=travel_move_counts(self.move_angle.value(),1)
            limit,segs=self.move_plan(counts)
        except ValueError as exc:
            self.move_preview.setText(str(exc)); return
        if len(segs)==1:
            extra=('（一程走完，中途不搬零点）' if abs(counts)>MOVE_LIMIT_COUNTS else '')
            turns=(f' · {abs(counts)/COUNTS_PER_REV:.2f} 圈'
                   if abs(counts)>MOVE_LIMIT_COUNTS else '')
            sec=self.move_seconds(counts)
            tail=(f' · 按 {self.move_slew_value():.3f} 码/ms 约 {sec:.0f} 秒' if sec else '')
            self.move_preview.setText(f'= {counts} 码{turns} · 位置环（命令 4）一条{extra}{tail}')
            return
        text=(f'= {counts} 码（{abs(counts)/COUNTS_PER_REV:.2f} 圈）'
              f' · 位置环分段（命令 4）×{len(segs)}，每段约 '
              f'{segs[0]*360.0/COUNTS_PER_REV:.0f}°（段间停一下再走）')
        p=self.params or []
        if p and (p[0]>>8!=0x0207 or p[0]&255<0x0F):
            text+=' · 当前固件不支持，需要 REV15'
        sec=self.move_seconds(counts)
        if sec:
            text+=f' · 按 {self.move_slew_value():.3f} 码/ms 约 {sec:.0f} 秒（不含段间停顿）'
        self.move_preview.setText(text)

    def move_slew_value(self):

        return self.move_slew.value() if hasattr(self,'move_slew') else 0.0

    def move_seconds(self,counts):

        w=self.move_slew_value()
        if w<=0 or not counts:
            return None
        return abs(counts)/w/1000.0

    def write_move_slew(self):

        if not self.connected:
            return
        slew=self.move_slew_value()
        raw=int(round(slew*65536))
        decel=move_decel_code(slew)
        brake=move_brake_counts(slew,decel)
        self.move_slew_note.setText(
            f'→ 地址 40 = {raw}（{raw/65536.0:.3f} 码/ms）'
            f' · 地址 49 = {decel}（收尾减速，末段约 {brake:.0f} 码 / '
            f'{brake*360.0/COUNTS_PER_REV:.0f}°）')
        self.worker.submit('set',(40,raw))
        self.worker.submit('set',(49,decel))

    def sync_move_slew(self):

        if not hasattr(self,'move_slew') or not self.params or len(self.params)<=40:
            return
        if self.move_slew.hasFocus():
            return
        raw=self.params[40]
        if raw>0:
            self.move_slew.setValue(raw/65536.0)

    def refresh_spin_preview(self):
        try:
            w=spin_speed_code(self.spin_rpm.value(),1)
            self.spin_preview.setText(f'= mem[43] {w:+d} 码/25ms ≈ {spin_speed_rpm(w):.1f} rpm')
        except ValueError as exc:
            self.spin_preview.setText(str(exc))

    def start_angle_move(self,direction):

        if self.move_queue:
            self.move_result.setText(
                f'分段走位还没走完（{self.move_chain_done}/{self.move_chain_total} 段）'
                '：等最后一段到点再发下一条。')
            return
        if self.move_pending is not None:
            self.move_result.setText('上一次走位还没到点：等终点误差出来再发下一条。')
            return
        if self.pause_waiting is not None:
            self.move_result.setText('正在等停靠：等「已停靠」出现后再走位。')
            return
        degrees=self.move_angle.value()

        try:
            counts=travel_move_counts(degrees,direction)
            limit,segs=self.move_plan(counts)
        except ValueError as exc:
            self.move_result.setText(str(exc)); return

        if len(segs)==1:
            self.worker.submit('angle_move_counts',(segs[0],False))
            return
        self.move_queue=segs[1:]
        self.move_chain_total=len(segs)
        self.move_chain_done=0
        self.move_chain_net=0
        self.move_chain_target=counts
        self.move_result.setText(
            f'分段走位：{abs(counts)*360.0/COUNTS_PER_REV:.0f}° 拆成 {len(segs)} 段'
            f'（每段约 {segs[0]*360.0/COUNTS_PER_REV:.0f}°，固件单条上限 {limit} 码），'
            f'正在发第 1 段…')
        self.worker.submit('angle_move_counts',(segs[0],False))

    def start_spin(self,direction):
        if self.move_queue or self.move_pending is not None:
            self.spin_note.setText('走位还没走完（分段走位要一段一段走）：等最后一段到点、零点搬完再起转。')
            return
        self.worker.submit('spin_start',(self.spin_rpm.value(),direction))

    def pause_spin(self):
        if self.move_queue or self.move_pending is not None:
            self.spin_note.setText('走位正在走（分段走位要一段一段走）：等它走完再暂停。')
            return
        self.worker.submit('spin_pause',self.pause_switch.isChecked())

    def update_spin_state(self):
        if not hasattr(self,'spin_state'):
            return
        p=self.params or []
        mode=p[16] if len(p)>16 else None
        parked=bool((self.last_motion or {}).get('parked'))
        if mode==4:
            w=0
            if len(p)>43:
                w=p[43]&0xFFFFFFFF
                if w&0x80000000: w-=0x100000000
            if parked:
                self.spin_state.setText('模式 4 · 已停靠：零点已搬到停下的地方。')
            elif w:
                self.spin_state.setText(
                    f'模式 4 · 转动中：{w:+d} 码/25ms ≈ {spin_speed_rpm(abs(w)):.1f} rpm。')
            elif self.pause_waiting is not None:
                self.spin_state.setText('模式 4 · 给定 0，正在软减速、等停靠…')
            else:
                self.spin_state.setText('模式 4 · 给定 0（未在转）。')
        else:
            self.spin_state.setText('未在连续旋转。起转会切到模式 4；暂停停稳后可回到模式 3。')

    def on_motion_status(self,data):

        self.last_motion=data
        if not hasattr(self,'move_result'):
            return
        count=data['count']
        if self.move_baseline is None:
            self.move_baseline=count
        elif count!=self.move_baseline and self.move_pending is not None:
            target=self.move_pending; self.move_pending=None; self.move_baseline=count
            err=data['error']; measured=target+err
            tail=' · 中途杆歪过，油门被收' if data.get('aborted') else ''
            if self.move_chain_total:

                self.move_chain_done+=1
                self.move_chain_net+=measured
                if self.move_queue:
                    nxt=self.move_queue.pop(0)
                    self.move_result.setText(
                        f'第 {self.move_chain_done}/{self.move_chain_total} 段到点'
                        f'（走 {measured*360.0/COUNTS_PER_REV:+.1f}°·误差 {err:+d} 码）· '
                        f'累计 {self.move_chain_net*360.0/COUNTS_PER_REV:+.1f}°；'
                        f'正在发第 {self.move_chain_done+1} 段…'+tail)
                    self.worker.submit('angle_move_counts',(nxt,True))
                else:
                    left=self.move_chain_net-self.move_chain_target
                    self.move_result.setText(
                        f'分段走位完成：{self.move_chain_total} 段 · '
                        f'实测总位移 {self.move_chain_net*360.0/COUNTS_PER_REV:+.1f}°'
                        f'（{self.move_chain_net:+d} 码）· 目标 '
                        f'{self.move_chain_target*360.0/COUNTS_PER_REV:+.1f}° · '
                        f'总误差 {left:+d} 码'+tail)
                    self.move_chain_total=0; self.move_chain_done=0
                    self.move_chain_net=0; self.move_chain_target=0
            else:
                self.move_result.setText(
                    f'目标 {target*360.0/COUNTS_PER_REV:+.1f}°（{target:+d} 码） · '
                    f'实测 {measured*360.0/COUNTS_PER_REV:+.1f}°（{measured:+d} 码） · '
                    f'终点误差 {err*360.0/COUNTS_PER_REV:+.1f}°（{err:+d} 码） · 零点已更新'+tail)
        else:
            self.move_baseline=count

        if self.pause_waiting is not None and data['parked']:
            self.finish_pause(self.pause_waiting)
        self.update_spin_state()
        self.update_live_readouts(data)

    def update_live_readouts(self,data):

        if not hasattr(self,'spin_live'):
            return
        p=self.params or []
        mode=p[16] if len(p)>18 else None
        vel=data.get('vel')
        now=time.monotonic()

        if self.move_pending is not None:
            target=self.move_pending
            done=target+data['error']
            text=(f'实时：已走 {done*360.0/COUNTS_PER_REV:+.1f}°（{done:+d} 码）'
                  f' · 目标 {target*360.0/COUNTS_PER_REV:+.1f}°')
            if data['parked']:
                text+=' · 已停稳，零点已搬'
            else:
                left=abs(target-done)*360.0/COUNTS_PER_REV
                text+=f' · 还差 {left:.1f}°'
            if text!=getattr(self,'_move_live_text',None):
                self._move_live_text=text; self.move_live.setText(text)

        w=0
        if len(p)>43:
            w=p[43]&0xFFFFFFFF
            if w&0x80000000: w-=0x100000000
        if w and not self.spin_w_prev:
            self.spin_angle=0.0; self.spin_vel_prev=None; self.spin_vel_t=None
        self.spin_w_prev=w
        if vel is None or mode!=4 or not w:
            self.spin_vel_prev=None; self.spin_vel_t=None
        else:
            dt=0.0 if self.spin_vel_t is None else min(.5,max(0.0,now-self.spin_vel_t))
            if dt>0 and self.spin_vel_prev is not None:

                self.spin_angle+=0.5*(vel+self.spin_vel_prev)*dt/.025
            self.spin_vel_prev=vel; self.spin_vel_t=now
        if mode!=4:
            text='实时：未在转。'
        else:
            parts=[f'已转 {self.spin_angle*360.0/COUNTS_PER_REV:+.1f}°'
                   f'（{self.spin_angle/COUNTS_PER_REV:+.2f} 圈）']
            if vel is not None:
                rpm=abs(vel)*60000.0/(SPIN_TICK_MS*COUNTS_PER_REV)
                where='顺时针' if vel>.5 else '逆时针' if vel<-.5 else '静止'
                parts.append(f'现在 {rpm:.1f} rpm（实测 · {where}）')
            parts.append(f'给定 {spin_speed_rpm(abs(w)):.1f} rpm' if w else '给定 0')
            text='实时：'+' · '.join(parts)
        if text!=getattr(self,'_spin_live_text',None):
            self._spin_live_text=text; self.spin_live.setText(text)

    def on_spin_pause_sent(self,data):
        was_running,switch=data
        if not hasattr(self,'spin_state'):
            return
        if (self.last_motion or {}).get('parked'):
            self.finish_pause(switch)
            return

        if not was_running and not self.mode4_ever_spun:
            self.spin_note.setText('已停止（这次进模式 4 没转过，零点没动，直接切回模式 3）。')
            if switch: self.worker.submit('set',(16,3))
        else:
            self.pause_waiting=switch
            if was_running:
                self.spin_note.setText('正在软减速停靠… 等「已停靠」出现（通常 0.5～2 s）；这期间别走位。')
            else:
                self.spin_note.setText('给定已经是 0，但板上还没报「已停靠」：零点还没搬，'
                                       '这时候切模式 3 会按旧零点把摆臂拽回去，所以先等着。')
            QTimer.singleShot(6000,self.pause_timeout)

        self.update_trajectory_controls()

    def finish_pause(self,switch):
        self.pause_waiting=None
        self.spin_note.setText('已停靠：零点已搬到停下的地方；下一次走位/旋转都从这里算。')
        if switch: self.worker.submit('set',(16,3))
        self.update_trajectory_controls()

    def pause_timeout(self):
        if self.pause_waiting is None:
            return
        self.pause_waiting=None

        vel=(self.last_motion or {}).get('vel')
        extra=''
        if vel is not None:
            extra=f'（此刻实测还有 {abs(vel)*60000.0/(SPIN_TICK_MS*COUNTS_PER_REV):.1f} rpm）'
        self.spin_note.setText('停靠未确认：板上还没报「已停靠」'+extra+'，所以没有切模式 3'
                               '（零点没搬，切过去会按旧零点把摆臂拽回来）。'
                               '给定已经是 0，摆臂就停在这儿；等它报停靠，或稍后再点一次「暂停」。')

    def build_trajectory_panel(self):
        panel=QWidget();box=QVBoxLayout(panel);scroll=QScrollArea();scroll.setWidgetResizable(True);scroll.setWidget(panel);self.tabs.addTab(scroll,'平衡轨迹')
        note=QLabel('横摆臂角速度 v(t)，单位 °/s；t 为轨迹时间，T 为总时间。先自动起摆并确认平衡；REV4会接管并取消正在执行的按键指令。偏离过大时暂停轨迹计时，恢复后继续；终点继续平衡。')
        note.setWordWrap(True);box.addWidget(note)
        row=QHBoxLayout();box.addLayout(row)
        row.addWidget(QLabel('总时间 / 循环周期 s'));self.trajectory_time=QDoubleSpinBox();self.trajectory_time.setRange(.1,60);self.trajectory_time.setDecimals(3);self.trajectory_time.setValue(8);row.addWidget(self.trajectory_time)
        self.trajectory_direction=QComboBox();self.trajectory_direction.addItems(['正向（编码器增加）','反向（编码器减少）']);row.addWidget(self.trajectory_direction)
        self.trajectory_continuous=QCheckBox('持续运行：按周期循环，直到停止');box.addWidget(self.trajectory_continuous)
        template_row=QHBoxLayout();box.addLayout(template_row)
        self.trajectory_template=QComboBox();self.trajectory_template.addItems(['sin² 单向平滑','sin 正反往复','cos 升余弦','三角波','自定义表达式']);template_row.addWidget(self.trajectory_template)
        template_row.addWidget(QLabel('幅值 °/s'));self.trajectory_amplitude=QDoubleSpinBox();self.trajectory_amplitude.setRange(0,500);self.trajectory_amplitude.setValue(30);template_row.addWidget(self.trajectory_amplitude)
        self.amplitude_slider=QSlider(Qt.Horizontal);self.amplitude_slider.setRange(0,500);self.amplitude_slider.setValue(30);box.addWidget(self.amplitude_slider)
        box.addWidget(QLabel('周期滑块（秒，精确值可在上方输入）'))
        self.period_slider=QSlider(Qt.Horizontal);self.period_slider.setRange(1,600);self.period_slider.setValue(80);box.addWidget(self.period_slider)
        self.live_knobs=QCheckBox('运行中自动应用幅值 / 周期（400 ms 防抖，无需重传）');box.addWidget(self.live_knobs)
        box.addWidget(QLabel('v(t) ='));self.trajectory_expression=QLineEdit('30*sin(pi*t/T)**2');box.addWidget(self.trajectory_expression)
        help_text=QLabel('支持 sin、cos、sqrt、abs、min、max、pi、t、T 和 ** 乘方。起止速度为 0；已取消60°/s限制；|a|≤120°/s²，可连续多圈。曲线接入原有位置平衡环。预览为目标曲线，实际跟踪存在误差。')
        help_text.setWordWrap(True);box.addWidget(help_text)
        buttons=QHBoxLayout();box.addLayout(buttons)
        self.button(buttons,'检查 / 预览',self.preview_trajectory)
        self.trajectory_start_button=self.button(buttons,'上传并开始',self.start_trajectory)
        self.trajectory_stop_button=self.button(buttons,'停止轨迹并保持平衡',lambda:self.worker.submit('trajectory_stop'))
        self.validation_note=QLabel();self.validation_note.setWordWrap(True);box.addWidget(self.validation_note)
        self.trajectory_note=QLabel('循环与实时改参需要 0x020703 固件。');self.trajectory_note.setWordWrap(True);box.addWidget(self.trajectory_note)
        self.trajectory_progress_label=QLabel('绿圈：板上目标速度；橙圈：编码器实测速度（25 ms 窗口）。');box.addWidget(self.trajectory_progress_label)
        self.trajectory_vplot=Plot('目标角速度','#0f9b7c','°/s');box.addWidget(self.trajectory_vplot)
        self.trajectory_pplot=Plot('角速度积分 · 预计位移','#3d6fd8','°');box.addWidget(self.trajectory_pplot)
        self.preview_timer=QTimer(self);self.preview_timer.setSingleShot(True);self.preview_timer.setInterval(120);self.preview_timer.timeout.connect(self.preview_trajectory)
        self.live_timer=QTimer(self);self.live_timer.setSingleShot(True);self.live_timer.setInterval(400);self.live_timer.timeout.connect(self.apply_live_knobs)
        self.amplitude_slider.valueChanged.connect(self.trajectory_amplitude.setValue)
        self.period_slider.valueChanged.connect(lambda v:self.trajectory_time.setValue(v/10))
        self.trajectory_amplitude.valueChanged.connect(self.template_changed)
        self.trajectory_time.valueChanged.connect(self.template_changed)
        self.trajectory_template.currentIndexChanged.connect(self.template_changed)
        self.trajectory_expression.textChanged.connect(lambda:self.preview_timer.start())
        self.trajectory_direction.currentIndexChanged.connect(lambda:self.preview_timer.start())
        self.live_knobs.toggled.connect(lambda:self.live_timer.start())
        self.template_changed()
    def build_quiet_panel(self):
        panel=QWidget();box=QVBoxLayout(panel);self.tabs.addTab(panel,'静稳抑制')
        note=QLabel('仅在普通自动平衡且停稳到位时抑制驱动。连续无编码器运动、偏角小且变化很小时进入；同向偏离趋势超过阈值时恢复控制。编码器运动、大偏角或新运动任务会直接退出。开关默认关闭，尚需实机验证。')
        note.setWordWrap(True);box.addWidget(note)
        self.quiet_enable=QCheckBox('启用静稳抑制（勾选后点击应用）');box.addWidget(self.quiet_enable)

        self.quiet_enable.setChecked(True)
        self.quiet_seconds=QDoubleSpinBox();self.quiet_seconds.setRange(.5,5);self.quiet_seconds.setValue(1);self.quiet_seconds.setSingleStep(.5)
        self.quiet_angle=QLineEdit('1.0');self.quiet_angle.setToolTip('取消人为角度上限；保留32位寄存器范围及原有跌倒保护。')
        for label,widget in [('连续静止时间 / s',self.quiet_seconds),('趋势唤醒偏角 / °',self.quiet_angle)]:
            row=QHBoxLayout();row.addWidget(QLabel(label));row.addWidget(widget);box.addLayout(row)
        self.quiet_apply=self.button(box,'应用静稳开关与阈值',lambda:self.worker.submit('quiet_config',(self.quiet_enable.isChecked(),self.quiet_seconds.value(),self.quiet_angle.text())))
        self.quiet_note=QLabel('板上状态未读取；此处勾选是待写值，以回读状态为准。');self.quiet_note.setWordWrap(True);box.addWidget(self.quiet_note);box.addStretch()

    def build_wave_panel(self):

        panel=QWidget(); box=QVBoxLayout(panel)
        scroll=QScrollArea(); scroll.setWidgetResizable(True); scroll.setWidget(panel)
        self.tabs.addTab(scroll,'往复轨迹')
        note=QLabel('以 0 点为中心来回走：上位机每 10 ms 算一个位置点写进地址 13，'
                    '板上的位置环去追（模式 2）。下面实时画两条线：绿 = 目标，橙 = 实测。\n'
                    '注意：模式 2 不会自己起摆 —— 先自动起摆到平衡再来；跑的过程中杆歪超过 25°，'
                    '本页会立刻把板子交回模式 3。')
        note.setWordWrap(True); box.addWidget(note)
        row=QHBoxLayout(); box.addLayout(row)
        row.addWidget(QLabel('波形'))
        self.wave_shape=QComboBox()
        for key in ('sine','cosine','triangle','custom'):
            self.wave_shape.addItem(SHAPES[key],key)
        self.wave_shape.setFixedWidth(120)
        row.addWidget(self.wave_shape)
        row.addWidget(QLabel('幅值'))
        self.wave_amp=QDoubleSpinBox(); self.wave_amp.setRange(1,MAX_AMPLITUDE)
        self.wave_amp.setDecimals(0); self.wave_amp.setSingleStep(10); self.wave_amp.setValue(90)
        self.wave_amp.setSuffix(' °'); self.wave_amp.setFixedWidth(110)
        self.wave_amp.setToolTip('以 0 点为中心的**单边**幅度。\n'
                                 '中心±幅值要留在 ±500 码（≈±173°）以内：\n'
                                 '再往外摆臂读数会整圈搬家，波形就对不上了。')
        row.addWidget(self.wave_amp)
        row.addWidget(QLabel('周期'))
        self.wave_period=QDoubleSpinBox(); self.wave_period.setRange(MIN_PERIOD,MAX_PERIOD)
        self.wave_period.setDecimals(1); self.wave_period.setSingleStep(.5)
        self.wave_period.setValue(8); self.wave_period.setSuffix(' s')
        self.wave_period.setFixedWidth(110)
        row.addWidget(self.wave_period)

        row.addWidget(QLabel('位置(t) ='))
        self.wave_expression=QLineEdit(DEFAULT_EXPRESSION)
        self.wave_expression.setReadOnly(True)
        self.wave_expression.setMinimumWidth(200)
        self.wave_expression.setToolTip('一个周期内的位置形状，T = 周期。\n'
                                        '可用：t、T、pi、+ − * / **、sin、cos、sqrt、abs、min、max。\n'
                                        '只用它决定**形状**，幅度由「幅值」给（按峰值归一化，\n'
                                        '所以表达式里写 90 还是 1 结果一样）；直流分量会自动去掉。\n'
                                        '选内置波形时这里是只读的公式显示；选「自定义」才能改。')
        row.addWidget(self.wave_expression,1)

        self.wave_custom_text=DEFAULT_EXPRESSION
        self.wave_custom_dirty=False
        self.wave_last_builtin='sine'
        buttons=QHBoxLayout(); box.addLayout(buttons)
        self.wave_start_button=self.button(buttons,'开始往复',self.start_wave)
        self.wave_stop_button=self.button(buttons,'停止（这里变新 0 点）',
                                          lambda:self.worker.submit('wave_stop'))
        self.wave_preview=QLabel(); self.wave_preview.setWordWrap(True)
        box.addWidget(self.wave_preview)
        self.wave_note=QLabel('未连接或未平衡时按钮是灰的：先连接、自动起摆，等「按键已就绪」再来。')
        self.wave_note.setWordWrap(True); box.addWidget(self.wave_note)
        self.wave_plot=Plot('摆臂位置 · 目标 vs 实测','#0f9b7c','°')
        box.addWidget(self.wave_plot,1)
        hint=QLabel('绿线 = 写进地址 13 的目标；橙线 = 相对实测（实时遥测 ch2，1040 码/圈；\n'
                    '它在 ±180° 以外会绕圈折回来，所以超过半圈以后这条线只当参考）。\n'
                    '停止 = 余弦减速（幅值越大减得越慢，峰减速度不超 120 °/s²）→ '
                    '**等杆晃回来站住**（板子标零点那道门要求杆在 ±5.3° 以内）→ 切回模式 3 '
                    '并把**停下的地方**标成新 0 点。等的时候摆臂被按在原地，不会动。')
        hint.setWordWrap(True); hint.setObjectName('muted'); box.addWidget(hint)
        self.wave_shape.currentIndexChanged.connect(self.refresh_wave_preview)
        self.wave_amp.valueChanged.connect(self.refresh_wave_preview)
        self.wave_period.valueChanged.connect(self.refresh_wave_preview)

        self.wave_expression_timer=QTimer(self); self.wave_expression_timer.setSingleShot(True)
        self.wave_expression_timer.setInterval(300)
        self.wave_expression_timer.timeout.connect(self.refresh_wave_preview)

        self.wave_expression.textEdited.connect(self.wave_expression_edited)
        self.wave_plot_timer=QTimer(self); self.wave_plot_timer.setInterval(80)
        self.wave_plot_timer.timeout.connect(self.refresh_wave_plot)
        self.wave_plot_timer.start()
        self.refresh_wave_preview()
    def wave_expression_edited(self):

        if self.wave_shape.currentData()!='custom': return
        self.wave_custom_text=self.wave_expression.text()
        self.wave_custom_dirty=True
        self.wave_expression_timer.start()
    def refresh_wave_preview(self,*args):
        if not hasattr(self,'wave_preview'): return
        shape=self.wave_shape.currentData()
        custom=shape=='custom'
        amp=self.wave_amp.value(); period=self.wave_period.value()

        if custom:
            if not self.wave_custom_dirty:

                self.wave_custom_text=WAVE_SEEDS.get(self.wave_last_builtin,
                                                     DEFAULT_EXPRESSION)
            text=self.wave_custom_text
        else:
            self.wave_last_builtin=shape
            text=cycle_formula(shape,amp,period)
        if self.wave_expression.text()!=text:
            self.wave_expression.setText(text)
        self.wave_expression.setReadOnly(not custom)
        try:
            c=build_cycle(shape,amp,period,self.wave_expression.text())
        except ValueError as exc:
            self.wave_preview.setText(str(exc)); return
        extra=''
        if custom:
            extra='形状按峰值归一化到 ±%g°（表达式里的系数只影响形状）。' % amp
            if abs(c.get('dc',0.0))>1e-9:
                extra+='表达式带直流 %+.4g，已自动去掉（仍然以 0 点为中心）。' % c['dc']
        if c['ok']:
            self.wave_preview.setText(
                f"峰加速度 {c['accel']:.0f} °/s²（上限 {MAX_ACCEL:.0f}）· "
                f"峰速度 {c['speed']:.0f} °/s（≈{c['speed']*60/360:.0f} rpm）· 可以开始。{extra}")
        elif c['min_period'] > MAX_PERIOD:

            msg=(f"峰加速度 {c['accel']:.0f} °/s² 超过上限 {MAX_ACCEL:.0f}，"
                 f"周期拉到最长（{MAX_PERIOD:g} 秒）也放不下。")
            if c['seam'] > 5:
                msg+=(f"这个函数一圈首尾接不上：接缝处目标速度要跳 {c['seam']:.0f} °/s，"
                      '每圈都会猛拽一下摆臂 —— 换成 sin(2*pi*t/T) 这类写法。')
            self.wave_preview.setText(msg+extra)
        else:
            self.wave_preview.setText(
                f"峰加速度 {c['accel']:.0f} °/s² 超过上限 {MAX_ACCEL:.0f}："
                f"这个幅值下周期至少要 {c['min_period']:.1f} 秒。{extra}")
    def refresh_wave_plot(self):
        if not self.wave_plot_dirty: return
        self.wave_plot_dirty=False
        self.wave_plot.set_data(self.wave_targets,100,self.wave_times)
        self.wave_plot.set_overlay(self.wave_actuals)
    def start_wave(self):
        shape=self.wave_shape.currentData()
        self.worker.submit('wave_start',dict(shape=shape,
                                             amplitude_deg=self.wave_amp.value(),
                                             period_s=self.wave_period.value(),

                                             expression=self.wave_expression.text() if shape=='custom' else ''))
    def update_wave_controls(self):
        p=self.params or []

        supported=bool(self.connected and len(p)>31 and
                       p[0]>>8==0x0207 and p[0]&255>=0x0E)
        ready=bool(supported and p[16]==3 and p[31]&7==4 and not p[31]&0x18 and
                   p[31]&(1<<21) and not p[31]&(1<<24))
        self.wave_start_button.setEnabled(ready and not self.trajectory_busy
                                          and not self.wave_busy)
        self.wave_stop_button.setEnabled(self.wave_busy)
        for widget in (self.wave_shape,self.wave_amp,self.wave_period,self.wave_expression):
            widget.setEnabled(not self.wave_busy)
    def template_changed(self,*args):
        a=self.trajectory_amplitude.value();i=self.trajectory_template.currentIndex()
        for slider,value in ((self.amplitude_slider,round(a)),(self.period_slider,round(self.trajectory_time.value()*10))):
            slider.blockSignals(True);slider.setValue(value);slider.blockSignals(False)
        shapes=['sin(pi*t/T)**2','sin(2*pi*t/T)','(1-cos(2*pi*t/T))/2','(1-abs(2*t/T-1))']
        self.trajectory_expression.setReadOnly(i<4)
        if i<4:self.trajectory_expression.setText(f'{a:g}*{shapes[i]}')
        self.preview_timer.start()
        if self.trajectory_busy and self.live_knobs.isChecked():self.live_timer.start()
    def apply_live_knobs(self):
        if not self.trajectory_busy or not self.live_knobs.isChecked() or not self.running_curve:return
        if not self.running_amplitude:
            self.trajectory_note.setText('零幅值起始或自定义函数不能按模板实时缩放，请停止后重新上传。');return
        curve=self.preview_trajectory()
        if curve is None:return
        gain=self.trajectory_amplitude.value()/self.running_amplitude
        rate=self.running_curve['seconds']/self.trajectory_time.value()
        if not -8<=gain<=8 or not .125<=rate<=4:
            self.validation_note.setText('实时幅值最多为起始幅值的8倍；周期范围为起始周期的0.25～8倍。超出范围需停止后重新上传。');return
        self.worker.submit('trajectory_update',(gain,rate))
    def preview_trajectory(self):
        try:
            c=compile_curve(self.trajectory_expression.text(),self.trajectory_time.value(),-1 if self.trajectory_direction.currentIndex() else 1)
            c['continuous']=self.trajectory_continuous.isChecked()
            self.trajectory_vplot.set_data(c['speeds']+[0],1000/c['interval'],c['times'])
            self.trajectory_pplot.set_data(c['positions'],1000/c['interval'],c['times'])
            self.validation_note.setText(f"检查通过：{len(c['words'])} 段，更新间隔 {c['interval']}ms，预计净位移 {c['positions'][-1]:.3f}°；预览参数，是否已应用请看运行状态。")
            return c
        except ValueError as exc:self.validation_note.setText(str(exc));return None
    def start_trajectory(self):
        curve=self.preview_trajectory()
        if curve is not None:
            self.trajectory_running=False;self.applied_gain=1;self.applied_rate=1
            self.running_curve=curve;self.running_amplitude=self.trajectory_amplitude.value() if self.trajectory_template.currentIndex()<4 else None
            self.trajectory_vplot.marker=None
            self.worker.submit('trajectory_start',curve)
    def update_trajectory_controls(self):

        p=self.params or []
        supported=bool(self.connected and not self.demo and len(p)>31 and
                       p[0]>>8==0x0207 and p[0]&255>=2)
        self.trajectory_start_button.setEnabled(supported and not self.trajectory_busy
                                                and not self.wave_busy and p[16]==3)
        self.trajectory_stop_button.setEnabled(supported)

        motion_link=bool(self.connected and len(p)>31 and
                         p[0]>>8==0x0207 and p[0]&255>=2)
        key_supported=bool(motion_link and p[0]&255>=6 and p[16]==3 and
                           p[31]&7==4 and not p[31]&0x18 and
                           p[31]&(1<<21) and not p[31]&(1<<24) and not self.trajectory_busy
                           and not self.wave_busy)
        self.move_left_button.setEnabled(key_supported)
        self.move_right_button.setEnabled(key_supported)
        zero_supported=bool(self.connected and len(p)>0 and
                            p[0]>>8==0x0207 and p[0]&255>=0x0E)
        self.zero_button.setEnabled(zero_supported and not self.trajectory_busy
                                    and not self.wave_busy)

        motion_supported=bool(motion_link and p[0]&255>=0x0F)
        balance_ok=bool(motion_supported and p[31]&7==4 and not p[31]&0x18 and
                        p[31]&(1<<21) and not p[31]&(1<<24))
        move_ready=bool(motion_supported and p[16]==3 and balance_ok and
                        not self.trajectory_busy and not self.wave_busy and
                        self.pause_waiting is None)
        self.move_ccw_button.setEnabled(move_ready)
        self.move_cw_button.setEnabled(move_ready)

        spin_ready=bool(motion_supported and p[16] in (3,4) and
                        p[31]&7==4 and not p[31]&0x18 and not p[31]&(1<<24) and
                        (p[16]==4 or p[31]&(1<<21)) and
                        not self.trajectory_busy and not self.wave_busy and
                        self.pause_waiting is None)
        self.spin_ccw_button.setEnabled(spin_ready)
        self.spin_cw_button.setEnabled(spin_ready)
        self.spin_pause_button.setEnabled(bool(motion_supported and p[16]==4 and
                                               self.pause_waiting is None))
        self.move_angle.setEnabled(motion_supported)
        self.spin_rpm.setEnabled(motion_supported)
        self.pause_switch.setEnabled(motion_supported)
        self.refresh_move_preview()
        self.update_spin_state()
        if motion_link and p[31]&(1<<24):
            self.motion_note.setText('回零故障：稳定后长按SW3约2秒，以当前位置重新置零；等按键就绪后再走位。')
        elif motion_link and p[16]==3 and not p[31]&(1<<21):
            self.motion_note.setText('按键尚未就绪：正在等待回零和停稳；此时不会发送或暂存左右命令。')
        elif key_supported and self.motion_note.text().startswith(('回零故障','按键尚未就绪')):
            self.motion_note.setText('按键已就绪，可以逆时针/顺时针移动90°；停稳后以当前位置重定零点。')
        for widget in (self.trajectory_template,self.trajectory_direction,self.trajectory_expression,self.trajectory_continuous):widget.setEnabled(not self.trajectory_busy)
        live=bool(supported and self.params[0]&255>=3 and self.trajectory_running and self.running_curve and self.running_amplitude)
        self.live_knobs.setEnabled(live or not self.trajectory_busy)
        for widget in (self.trajectory_time,self.period_slider,self.trajectory_amplitude,self.amplitude_slider):widget.setEnabled(not self.trajectory_busy or live)
        if self.trajectory_busy or self.wave_busy:
            for button in (self.write_button,self.auto_button,self.closed_button):button.setEnabled(False)
        self.update_wave_controls()
    def card(self,parent,title,value,obj='metricCard',value_name='metricValue'):

        box=QWidget(); box.setObjectName(obj)
        l=QVBoxLayout(box); l.setContentsMargins(12,8,12,8); l.setSpacing(2)
        name=QLabel(title); l.addWidget(name)
        v=QLabel(value); v.setObjectName(value_name); l.addWidget(v)
        note=QLabel(''); note.setObjectName('muted'); note.setWordWrap(True)
        l.addWidget(note)
        v.note_label=note
        parent.addWidget(box,1)
        return v
    def pid_card_widget(self,parent):

        box=QWidget(); box.setObjectName('pidCard')
        l=QVBoxLayout(box); l.setContentsMargins(12,8,12,8); l.setSpacing(2)
        name=QLabel('内环 PD'); name.setObjectName('pidTitle'); l.addWidget(name)
        self.pid_set=QLabel('设定 —'); self.pid_set.setObjectName('muted'); l.addWidget(self.pid_set)
        self.pid_eff=QLabel('生效 —'); self.pid_eff.setObjectName('pidValue'); l.addWidget(self.pid_eff)
        self.pid_badge=QLabel('设定约 1 Hz / 生效约 2 Hz'); self.pid_badge.setObjectName('muted')
        l.addWidget(self.pid_badge)
        parent.addWidget(box,1)

    def update_average(self,angle,position,hz,times=None):

        self.last_errors=(angle,position,hz,times)
        self.refresh_average()
    def refresh_average(self):
        if not hasattr(self,'angle_avg') or self.last_errors is None:
            return
        if not self.avg_enable.isChecked():
            for label in (self.angle_avg,self.arm_avg):
                label.setText('— °'); label.note_label.setText('平均误差未启用')
            return
        angle,position,hz,times=self.last_errors
        for label,values in ((self.angle_avg,angle),(self.arm_avg,position)):
            result=window_error(values,hz,times,self.avg_window.value())
            if result is None:
                label.setText('— °')
                label.note_label.setText('窗口内不足 2 点，等数据')
                continue
            label.setText(f'{result["rms"]:.3f} °')
            source='遥测' if times is not None else '整窗'
            label.note_label.setText(
                f'{source} · 最近 {result["duration"]:.2f} s / {result["count"]} 点 · 相对目标')

    def reset_stability(self):

        self.stats.interrupt()
        self.stats_time=0.0; self.stats_tick=None
        self.update_stability()
    def clear_statistics(self):
        self.stats=StabilityStats(); self.stats_time=0.0; self.stats_tick=None
        self.last_errors=None
        self.update_stability()
        for label in (getattr(self,'angle_avg',None),getattr(self,'arm_avg',None)):
            if label is not None:
                label.setText('— °'); label.note_label.setText('等待数据')
        self.append_log('稳态统计已清空（不影响板上状态）')
    def observe_stability(self,data,error):
        tick=data['sequence']
        if self.stats_tick is not None:

            self.stats_time+=((tick-self.stats_tick)&0xffffffff)/1000
        self.stats_tick=tick
        self.stats.observe(self.stats_time,error,balanced(data['params'],data['status']))
        self.update_stability()
    def update_stability(self):
        if not hasattr(self,'delay'):
            return
        s=self.stats
        self.delay.setText('— s' if s.mean_delay is None else f'{s.mean_delay:.2f} s')
        note=f'连续保持 {s.dwell:g} s · 有效 {s.successes} / 进入平衡 {s.attempts} 次'
        if s.attempts==0 and s.center is not None:
            note+='；从平衡中途开始监控，没有起点'
        self.delay.note_label.setText(note)

    def update_pid_monitor(self,sample=None,available=None):
        if not hasattr(self,'pid_eff'):
            return
        if sample is not None:
            self.pid_sample=sample
        if available is not None:
            self.pid_available=available
        p=self.params or []
        if len(p)>4:
            self.pid_set.setText(f'设定 Kp {decode(3,p[3]):.3f} · Kd {decode(4,p[4]):.3f}')
        if not self.pid_available:
            self.pid_eff.setText('生效回读不可用')
            self.pid_badge.setText('固件未提供模糊调度状态')
            return
        s=self.pid_sample
        if s is None:

            self.pid_eff.setText('等待回读')
            self.pid_badge.setText('设定约 1 Hz / 生效约 2 Hz')
            return
        if time.monotonic()-s.get('received',0)>=2:
            self.pid_eff.setText('生效回读已过期')
            self.pid_badge.setText('设定约 1 Hz / 生效约 2 Hz')
            return
        self.pid_eff.setText(f'生效 Kp {s["kp"]:.3f} · Kd {s["kd"]:.3f}')
        if s.get('enabled'):
            self.pid_badge.setText(f'模糊调度开启 · P×{s["p_scale"]:.3f} / D×{s["d_scale"]:.3f}')
        else:
            self.pid_badge.setText('固定 PD')
    def scan_ports(self):

        keep=self.port.currentText().strip()
        self.port.clear()
        ports=list(list_ports.comports())
        for p in ports: self.port.addItem(p.device)
        names=[p.device for p in ports]
        board=[p.device for p in ports if 'CH340' in (p.description or '').upper()]

        wired=[p.device for p in ports if '蓝牙' not in (p.description or '')]
        for candidate in (board[:1]+[keep]+wired[:1]):
            if candidate and candidate in names:
                self.port.setCurrentText(candidate); return
        self.port.setCurrentText('')
        self.port.setPlaceholderText('未找到 CH340，请插线后点刷新端口')
    def set_controls(self,connected):
        for widget in (self.stop_button,self.refresh_button,self.write_button,self.monitor_button,
                       self.auto_button,self.closed_button,self.zero_button,self.fuzzy_switch,
                       self.comp_shape):
            widget.setEnabled(connected)
        self.disconnect_button.setEnabled(connected)
        self.connect_button.setEnabled(not connected)
        self.demo_button.setEnabled(not connected)
        self.update_trajectory_controls()
    def connect_device(self):
        self.demo=False
        self.worker.submit('connect',self.port.currentText().strip())
    def connect_demo(self):
        self.demo=True; self.worker.submit('connect','DEMO')
    def append_log(self,text): self.log.appendPlainText(time.strftime('%H:%M:%S')+'  '+text)
    def update_connection_label(self):

        if not self.connected:
            self.connection.setText('未连接'); return
        if self.demo:
            self.connection.setText('演示数据 · 未连接设备'); return
        port=self.port.currentText() or '串口'
        if self.params:
            self.connection.setText(f'{port} · 固件 0x{self.params[0]:08X}')
        else:
            self.connection.setText(port)
    def edited(self,item):
        if not self.updating and item.column()==3 and self.params is not None:
            self.dirty[item.row()]=item.text()
            self.table.item(item.row(),4).setText('待写入')
    def fill_params(self,params):

        previous=self.params
        if (previous is not None and len(previous)>18 and len(params)>18 and
                any(previous[a]!=params[a] for a in (13,16,18))):
            self.reset_stability()
        self.params=params; self.updating=True
        for a in range(N_PARAM):
            self.table.setRowHidden(a, a >= len(params))
            if a >= len(params):
                continue
            self.table.item(a,2).setText(display(a,params[a]))
            if a not in self.dirty:
                self.table.item(a,3).setText('' if a in READ_ONLY else display(a,params[a]))
                self.table.item(a,4).setText('只读' if a in READ_ONLY else '已回读')
        self.updating=False
        self.sync_switches()
        self.sync_move_slew()
        self.update_pid_monitor()
        self.update_trajectory_controls()
    def revert_selected(self):
        a=self.table.currentRow()
        self.dirty.pop(a,None)
        if self.params: self.fill_params(self.params)
    def write_selected(self):
        a=self.table.currentRow()
        if not self.connected or a<0 or a in READ_ONLY: return
        try: raw=encode(a,self.table.item(a,3).text())
        except ValueError as e: QMessageBox.warning(self,'数值无效',str(e)); return
        if a==16 and raw!=0:
            if not self.confirm_motion(f'写入运行模式 {raw}，电机可能立即运动。'): return
        elif a in (17,19):
            if not self.confirm_motion('修改开环输出或电机使能可能立即改变电机动作。'): return
        self.worker.submit('set',(a,raw))
    def confirm_motion(self,text):
        return QMessageBox.question(self,'确认电机动作',text+'\n请确认运动范围已清空。',
                                    QMessageBox.Yes|QMessageBox.No,QMessageBox.No)==QMessageBox.Yes
    def set_mode(self,mode):
        text='从自然下垂开始自动起摆。' if mode==3 else '请先把摆杆扶到竖直附近，再进入闭环。'
        if self.confirm_motion(text): self.worker.submit('set',(16,mode))
    def sync_switches(self):

        if not self.params:
            return
        p=self.params
        self.syncing=True
        try:
            self.fuzzy_switch.setChecked(len(p)>9 and bool(p[9]&0x100))
            if len(p)>50:
                idx=self.comp_shape.findData(p[50]&3)
                if idx>=0: self.comp_shape.setCurrentIndex(idx)
        finally:
            self.syncing=False
    def toggle_fuzzy(self,on):
        if self.syncing:
            return
        if not self.connected:
            self.syncing=True; self.fuzzy_switch.setChecked(not on); self.syncing=False
            return
        self.worker.submit('fuzzy',bool(on))
    def toggle_comp_shape(self,index):
        if self.syncing or not self.connected:
            return

        self.worker.submit('set',(50,int(self.comp_shape.itemData(index) or 0)))
    def toggle_monitor(self): self.worker.submit('monitor',not self.monitoring)
    def update_age(self):
        if self.frame:
            age=time.time()-self.frame['received']
            self.age.setText(f'{age:.1f} s 前回读'+(' · 已断开' if not self.connected else ''))

        self.update_pid_monitor()
    def on_event(self,kind,data):
        if kind=='tof_enabled':
            self.tof_panel.forward.blockSignals(True);self.tof_panel.forward.setChecked(data);self.tof_panel.forward.blockSignals(False);return
        if kind=='tof_note':self.tof_panel.forward_note.setText(data);return
        if kind=='avoid_note':self.tof_panel.avoid_note.setText(data);return
        if kind=='quiet_note':self.quiet_note.setText(data);return
        if kind=='motion_note':
            self.motion_note.setText(data);self.append_log(data)

            if hasattr(self,'spin_note'): self.spin_note.setText(data)
            return
        if kind=='angle_move_sent':

            self.move_pending=data; self.last_move_target=data
            self._move_live_text=None
            if hasattr(self,'move_live'):
                self.move_live.setText(f'实时：目标 {data*360.0/COUNTS_PER_REV:+.1f}°（{data:+d} 码），等待起步…')
            baseline=(self.last_motion or {}).get('count')
            if baseline is not None: self.move_baseline=baseline
            if self.move_chain_total:

                self.move_result.setText(
                    f'分段走位 第 {self.move_chain_done+1}/{self.move_chain_total} 段已下发'
                    f'（本段 {data*360.0/COUNTS_PER_REV:+.1f}°·{data:+d} 码）；'
                    f'累计已走 {self.move_chain_net*360.0/COUNTS_PER_REV:+.1f}°。')
            else:
                self.move_result.setText(
                    f'走位中… 目标 {data*360.0/COUNTS_PER_REV:+.1f}°（{data:+d} 码）；'
                    '到点停稳后这里显示实测终点与误差。')
            return
        if kind=='move_chain_failed':

            left=len(self.move_queue)
            done=self.move_chain_done; total=self.move_chain_total
            self.move_queue=[]
            if total:
                self.move_result.setText(
                    f'分段走位中断：已走 {done}/{total} 段'
                    f'（累计 {self.move_chain_net*360.0/COUNTS_PER_REV:+.1f}°），'
                    f'还剩 {left} 段没走。原因见上面的提示。')
            else:
                self.move_result.setText('走位被板子拒绝：原因见上面的提示。')
            self.move_chain_total=0; self.move_chain_done=0
            self.move_chain_net=0; self.move_chain_target=0
            return
        if kind=='motion_status':
            self.on_motion_status(data);return
        if kind=='spin_started':
            self.mode4_ever_spun=True; return
        if kind=='spin_pause_sent':
            self.on_spin_pause_sent(data);return
        if kind=='trajectory_active':
            self.trajectory_busy=data
            if not data:self.trajectory_running=False
            self.set_controls(self.connected);return
        if kind=='trajectory_knobs_applied':
            self.applied_gain,self.applied_rate=data;return
        if kind=='trajectory_progress':
            self.trajectory_running=bool(data['status']&1 and not data['status']&(8|32))
            self.update_trajectory_controls()
            if not self.running_curve:
                self.trajectory_vplot.marker=None
                self.trajectory_progress_label.setText(f"已恢复运行监控：板上相位 {data['elapsed']:.3f}s，目标 {data['target']:.2f}°/s；当前曲线预览未恢复。")
                return
            if self.running_amplitude and (abs(self.trajectory_amplitude.value()-self.running_amplitude*self.applied_gain)>.001 or abs(self.trajectory_time.value()-self.running_curve['seconds']/self.applied_rate)>.001):
                self.trajectory_vplot.marker=None;self.trajectory_vplot.update()
                self.trajectory_progress_label.setText(f"预览参数尚未应用；板上目标 {data['target']:.2f}°/s，实测 {data.get('actual',0):.2f}°/s")
                return
            duration=self.running_curve['seconds'] if self.running_curve else self.trajectory_time.value()
            phase=data['elapsed']/duration*self.trajectory_time.value()
            self.trajectory_vplot.set_marker(phase,data['target'],data.get('actual'))
            actual=f"{data['actual']:.2f}°/s" if 'actual' in data else '不可用'
            self.trajectory_progress_label.setText(f"第 {data.get('cycles',0)+1} 周期 · 相位 {phase:.3f}s · 绿圈目标 {data['target']:.2f}°/s · 橙圈实测 {actual}")
            return
        if kind=='trajectory_note':self.trajectory_note.setText(data);return
        if kind=='wave_active':
            self.wave_busy=bool(data)
            if data:

                self.wave_t0=None
                self.wave_times.clear(); self.wave_targets.clear(); self.wave_actuals.clear()
                self.wave_plot_dirty=True
            else:
                self.wave_state=WaveSession.IDLE
            self.set_controls(self.connected)
            return
        if kind=='wave_sample':
            if data['state']!=self.wave_state:
                self.wave_state=data['state']; self.update_wave_controls()
            if self.wave_t0 is None: self.wave_t0=data['t']
            self.wave_times.append(data['t']-self.wave_t0)
            self.wave_targets.append(degrees_of(data['target']))

            self.wave_actuals.append(None if data['actual'] is None
                                     else degrees_of(data['actual']))

            if len(self.wave_times)>8000:
                del self.wave_times[:500]; del self.wave_targets[:500]
                del self.wave_actuals[:500]
            self.wave_plot_dirty=True
            return
        if kind=='wave_note':
            self.wave_note.setText(data); self.append_log(data); return
        if kind=='connection':
            self.connected=bool(data); self.set_controls(self.connected)
            self.update_connection_label()
            if data:
                self.dirty.clear(); self.frame=None
                self.live_rows=[]; self.live_ticks=[]; self.live_origin=None

                self.stats=StabilityStats(); self.stats_time=0.0; self.stats_tick=None
                self.last_errors=None
                self.pid_sample=None; self.pid_available=True

                self.last_motion=None; self.move_pending=None; self.move_baseline=None
                self.move_queue=[]; self.move_chain_total=0; self.move_chain_done=0
                self.move_chain_net=0; self.move_chain_target=0
                self.pause_waiting=None
                self.mode4_ever_spun=False

                self.wave_busy=False; self.wave_state=WaveSession.IDLE
                self.wave_t0=None; self.wave_plot_dirty=True
                self.wave_times.clear(); self.wave_targets.clear(); self.wave_actuals.clear()
                if hasattr(self,'move_result'):
                    self.move_result.setText('还没有走位记录。')
                    self.spin_note.setText('')
                    self.update_spin_state()
                if self.params: self.fill_params(self.params)
                for p in self.plots: p.set_data([],1000)
                self.angle.setText('— °'); self.arm.setText('— °'); self.duty.setText('—'); self.age.setText('尚无波形')
                for label in (self.angle_avg,self.arm_avg):
                    label.setText('— °'); label.note_label.setText('等待数据')
                self.update_stability(); self.update_pid_monitor()
            else:
                self.status.setText('运行状态：已断开；板上电机状态未知')
                self.trajectory_vplot.marker=None;self.trajectory_vplot.update()
                self.trajectory_progress_label.setText('连接已断开，速度指示不可用。')
                self.running_curve=None
            self.append_log('离线演示已连接' if data=='DEMO' else ('串口已连接' if data else '连接已关闭；未发送停机命令'))
        elif kind=='capability':
            self.version=data
            self.acquisition.blockSignals(True)
            self.acquisition.setCurrentIndex(0 if data in (0x011f,0x0200,0x0201,0x0202,0x0203,0x0206,0x0207) else 1)
            self.acquisition.setEnabled(data in (0x011f,0x0200,0x0201,0x0202,0x0203,0x0206,0x0207))
            self.acquisition.blockSignals(False)
            self.notice.setText('实时遥测：目标约50Hz；高频振动分析仍用完整窗口。' if data in (0x011f,0x0200,0x0201,0x0202,0x0203,0x0206,0x0207) else '0x011E 仅支持约5秒整窗刷新；烧录0x011F后启用实时遥测。')
            self.update_connection_label()
        elif kind=='reset_live':

            self.live_rows=[]; self.live_ticks=[]; self.live_origin=None
            self.reset_stability(); self.last_errors=None
            if hasattr(self,'angle_avg'):
                for label in (self.angle_avg,self.arm_avg):
                    label.setText('— °'); label.note_label.setText('等待数据')
        elif kind=='live':
            tick=data['sequence']
            if self.live_ticks and tick==self.live_ticks[-1]: return
            if self.live_ticks and ((tick-self.live_ticks[-1]) & 0xffffffff)>5000:
                self.live_rows=[]; self.live_ticks=[]; self.live_origin=None
            self.live_rows.append(data['row']); self.live_ticks.append(tick)

            self.live_rows=self.live_rows[-3000:]; self.live_ticks=self.live_ticks[-3000:]
            times=[((t-self.live_ticks[0]) & 0xffffffff)/1000 for t in self.live_ticks]
            self.frame={'rows':list(self.live_rows),'params':data['params'],
                        'received':data['received'],'sample_hz':50,'times':times,'live':True}
            series=channels(self.live_rows,data['params'])
            for chart,values in zip(self.plots,series): chart.set_data(values,50,times)
            angle_error,position_error=error_series(self.live_rows,data['params'])
            self.angle.setText(f'{angle_error[-1]:+.3f} °')
            self.arm.setText(f'{position_error[-1]:+.2f} °')
            self.duty.setText(f'{series[2][-1]:+d} / 1024')
            hz=(len(times)-1)/times[-1] if len(times)>1 and times[-1] else 0
            self.capture_note.setText(f'实时快照 · 实际 {hz:.1f} Hz · 保留最近{len(self.live_rows)}点 · 非1kHz连续记录')
            self.status.setText(decode_status(data['status']))
            self.update_average(angle_error,position_error,hz,times)
            self.observe_stability(data,angle_error[-1])
        elif kind=='params':
            self.fill_params(data)
            self.update_connection_label()
        elif kind=='written':
            self.dirty.pop(data,None)
            if self.params: self.fill_params(self.params)
        elif kind=='status':
            mode,raw=data
            if self.move_pending is not None and mode!=3:

                self.move_pending=None
                if hasattr(self,'move_result'):
                    self.move_result.setText('走位被中断（模式已改变），这一次没有终点数据。')
            if self.params:
                self.params[16],self.params[31]=mode,raw

                if mode!=4: self.mode4_ever_spun=False
                self.sync_switches()
                self.update_trajectory_controls()
            self.status.setText(f'模式 {mode}  |  '+decode_status(raw))
        elif kind=='monitor':
            self.monitoring=data; self.monitor_button.setText('停止波形监控' if data else '开始波形监控')
        elif kind=='pid':
            self.update_pid_monitor(data,True)
        elif kind=='pid_unavailable':
            self.pid_sample=None
            self.update_pid_monitor(None,False)
        elif kind=='collecting':
            self.capture_note.setText(f'正在采集 {data:.3f} 秒窗口，完成后整段更新。停止监控不停止电机。')
        elif kind=='frame':
            self.frame=data
            series=channels(data['rows'],data['params'])
            for chart,values in zip(self.plots,series): chart.set_data(values,data['sample_hz'])
            angle_error,position_error=error_series(data['rows'],data['params'])
            self.angle.setText(f'{angle_error[-1]:+.3f} °')
            self.arm.setText(f'{position_error[-1]:+.2f} °')
            self.duty.setText(f'{series[2][-1]:+d} / 1024')

            self.update_average(angle_error,position_error,data['sample_hz'],None)

            self.stats.interrupt(); self.update_stability()
            rms=math.sqrt(sum(v*v for v in angle_error)/len(angle_error))
            self.append_log(f'{"[演示] " if self.demo else ""}回读4096点 / {data["sample_hz"]:g}Hz，角度RMS {rms:.3f}°')
        elif kind=='log': self.append_log(data)
        elif kind=='error':
            self.append_log('通信/采集失败：'+data)

            if '参数表长度' in data:
                note = ('板上固件比这个上位机新（参数表更长），这不是板子的故障：\n'
                        '按 SW5 不会有用。请换用与固件同版本的上位机\n'
                        '（或先用 tools/dbg_host.py 命令行读写参数）。\n')
            else:
                note = '串口停机无法保证，请检查板上状态，必要时按 SW5。\n'
            QMessageBox.warning(self,'连接已停止',data+'\n'+note)
    def export_params(self):
        if self.params is None: return
        path,_=QFileDialog.getSaveFileName(self,'导出参数','parameters.json','JSON (*.json)')
        if path:
            self.write_file(lambda:Path(path).write_text(json.dumps({'firmware':self.version,'demo':self.demo,
                'params_raw':dict(enumerate(self.params)),
                'values':{PARAM_TABLE[a][0]:display(a,v) for a,v in enumerate(self.params)}},
                ensure_ascii=False,indent=2),encoding='utf-8'))
    def write_file(self,callback):
        try: callback()
        except OSError as e: QMessageBox.warning(self,'保存失败',str(e))
    def export_frame(self):
        if not self.frame: return
        path,_=QFileDialog.getSaveFileName(self,'导出当前完整窗口','capture.csv','CSV (*.csv)')
        if not path: return
        frame=self.frame
        def save():
            with open(path,'w',newline='',encoding='utf-8') as f:
                w=csv.writer(f)
                w.writerow(['idx','adc','delta','pos','duty']+(['time_s'] if frame.get('live') else []))
                w.writerows([i,*r]+([frame['times'][i]] if frame.get('live') else []) for i,r in enumerate(frame['rows']))
            meta={'firmware':self.version,'demo':self.demo,'complete':True,'sample_hz':frame['sample_hz'],
                  'params_raw':dict(enumerate(frame['params'])),'received':frame['received'],
                  'acquisition':'live_snapshot' if frame.get('live') else 'capture',
                  'irregular_sampling':bool(frame.get('live')),
                  'filter':'none' if frame.get('live') else ('block_mean' if frame['params'][21] else 'none')}
            Path(path+'.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2),encoding='utf-8')
        self.write_file(save)
    def closeEvent(self,event):
        self.tof_panel.shutdown()
        if self.closing:
            event.accept(); return
        if self.connected:

            extra=('往复轨迹正在运行：断开时会把板子交回模式 3（自动平衡/起摆），'
                   '摆臂会按板上的 0 点慢慢回去。\n' if self.wave_busy else '')
            answer=QMessageBox.question(self,'关闭界面',extra+'关闭界面不会停止电机。确定断开并关闭？\n如需停机，请先点击“停止电机”。',
                                        QMessageBox.Yes|QMessageBox.No,QMessageBox.No)
            if answer!=QMessageBox.Yes: event.ignore(); return
        event.ignore()
        self.setEnabled(False)
        self.worker.finished.connect(self.finish_close)
        self.worker.submit('quit')

        QTimer.singleShot(3000, self.force_close)
    def force_close(self):
        if self.closing:
            return
        self.append_log('串口线程 3 秒没退出（多半选错了口，卡在系统调用里），强制结束。')
        self.worker.running=False
        self.worker.terminate(); self.worker.wait(1000)
        self.finish_close()
    def finish_close(self):
        self.closing=True; self.close()

STYLE='''
QWidget { background:#f1f2f6; color:#292538; font-family:"Microsoft YaHei UI"; font-size:12px; }
QPushButton { background:white; color:#504260; border:1px solid #d9d3e2; border-radius:5px; padding:7px 12px; }
QPushButton:hover {background:#f1e9fa;border-color:#9d77bc;}
QPushButton:disabled {color:#a29baa;background:#f4f2f7;border-color:#e4e0e9;}

QPushButton[role="primary"] { background:#64138f; color:white; border:1px solid #64138f; font-weight:600; }
QPushButton[role="primary"]:hover { background:#7a26a6; border-color:#7a26a6; }
QPushButton[role="primary"]:disabled { background:#e3d3ee; color:#a58cb6; border-color:#d3c0e2; }

QPushButton[role="toggle"] { background:#f4f2f7; color:#8a7b98; border:1px solid #ded5e8; }
QPushButton[role="toggle"]:hover { background:#ece2f6; color:#650f92; }
QPushButton[role="toggle"]:checked { background:#64138f; border-color:#64138f; color:white; font-weight:600; }
QPushButton[role="toggle"]:checked:hover { background:#7a26a6; }
QPushButton[role="toggle"]:checked:disabled { background:#e3d3ee; border-color:#d3c0e2; color:#a58cb6; }
QComboBox {padding:6px;background:white;border:1px solid #d9d3e2;border-radius:4px;min-width:80px;}
QTableWidget {background:white;alternate-background-color:#faf8fc;gridline-color:#e9e4ef;selection-background-color:#eee3f8;selection-color:#4b087b;border:1px solid #ded8e6;}
QHeaderView::section {background:#ede6f3;color:#5b4773;padding:8px;border:0;}
QPlainTextEdit {background:white;border:1px solid #ded8e6;color:#73677f;border-radius:5px;}
QSplitter::handle {background:#e1dbe9;}
QTabWidget::pane {background:white;border:1px solid #ddd5e6;}
QTabBar::tab {background:#ede9f2;color:#756481;padding:7px 15px;border:1px solid #ded5e8;}
QTabBar::tab:selected {background:white;color:#650f92;border-top:2px solid #7923a8;}
QScrollBar:vertical {background:#f4f1f7;width:10px;}
QScrollBar::handle:vertical {background:#cec2db;border-radius:4px;min-height:20px;}
QLineEdit,QDoubleSpinBox {background:white;color:#292538;border:1px solid #d9d3e2;padding:4px;}
QToolTip {background:#4b087b;color:white;}

QCheckBox { padding:2px 4px; }
QCheckBox::indicator { width:15px; height:15px; border:1px solid #b9a8cc;
                       border-radius:3px; background:white; }
QCheckBox::indicator:checked { background:#64138f; border-color:#64138f; }

QWidget#appHeader {background:qlineargradient(x1:0,y1:0,x2:1,y2:0,stop:0 #853866,stop:0.4 #65158a,stop:1 #41076d);border-radius:7px;}
QLabel#appTitle {color:white;background:transparent;font-size:24px;font-weight:700;}
QLabel#appSubtitle {color:white;background:transparent;font-size:11px;}
QWidget#metricCard {background:white;border-radius:6px;}
QWidget#rmseCard {background:#f8e9f2;border-radius:6px;}
QLabel#metricValue {font-size:25px;font-weight:600;color:#48314f;background:transparent;}
QLabel#rmseValue {font-size:27px;font-weight:700;color:#c82f79;background:transparent;}
QWidget#pidCard {background:white;border:1px solid #ded5e7;border-radius:6px;}
QWidget#pidCard QLabel {background:transparent;border:0;}
QLabel#pidTitle {font-size:13px;font-weight:700;color:#4b087b;}
QLabel#pidValue {font-size:14px;font-weight:600;color:#4b087b;}
QLabel#muted {font-size:11px;color:#867590;background:transparent;}
QWidget#metricCard QLabel,QWidget#rmseCard QLabel {background:transparent;}
'''

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--demo',action='store_true')
    parser.add_argument('--smoke-test',metavar='PNG',help='离线绘制、截图并退出，不访问串口')
    args=parser.parse_args()
    app=QApplication(sys.argv[:1])

    if not QFontDatabase.families():
        QFontDatabase.addApplicationFont(str(Path(os.environ.get('WINDIR','C:/Windows'))/'Fonts/msyh.ttc'))
    app.setStyle('Fusion'); app.setStyleSheet(STYLE)

    app.setWindowIcon(app_icon())
    window=Window(); window.show()
    if args.demo: QTimer.singleShot(0,window.connect_demo)
    if args.smoke_test:
        demo=DemoLink(); demo.shift=0
        window.demo=True
        window.fill_params(demo.dump_params())
        window.connection.setText('离线截图 · 模拟数据')
        window.on_event('capability',0x0207)
        for i,row in enumerate(demo.read_capture()[::20]):
            window.on_event('live',{'params':demo.dump_params(),'row':row,'sequence':i*20,
                                   'status':0x84,'received':time.time()})
        def snapshot():
            window.grab().save(args.smoke_test)
            window.close()
        QTimer.singleShot(600,snapshot)
    return app.exec()

if __name__=='__main__': raise SystemExit(main())
