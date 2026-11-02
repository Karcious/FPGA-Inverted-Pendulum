import asyncio, struct, threading, time
from PySide6.QtCore import QThread, Signal, QTimer
from PySide6.QtWidgets import QWidget,QVBoxLayout,QHBoxLayout,QLabel,QPushButton,QLineEdit,QCheckBox
SERVICE='6e400001-b5a3-f393-e0a9-e50e24dcca9e'
TX='6e400003-b5a3-f393-e0a9-e50e24dcca9e'
PACKET=struct.Struct('<2sBBIIHHBBH')
def decode(data):
    if len(data)!=PACKET.size:raise ValueError('测距数据长度错误')
    magic,version,present,seq,ms,left,right,ls,rs,reserved=PACKET.unpack(data)
    if magic!=b'TF' or version!=1 or reserved!=0 or present&~3:raise ValueError('测距协议不匹配')
    flags=(1 if present&1 and ls==0 else 0)|(2 if present&2 and rs==0 else 0)
    return dict(device_seq=seq,device_ms=ms,left=left,right=right,ls=ls,rs=rs,present=present,flags=flags)
class TofMailbox:
    def __init__(self):
        self.lock=threading.Lock();self.packet=None;self.counter=0
    def clear(self):
        with self.lock:self.packet=None
    def put(self,packet):
        with self.lock:
            self.counter=(self.counter+1)&0xffffffff
            self.packet={**packet,'seq':self.counter,'received':time.monotonic()}
    def snapshot(self,now=None):
        with self.lock:p=dict(self.packet) if self.packet else dict(seq=self.counter,left=0,right=0,flags=0,received=0)
        if (time.monotonic() if now is None else now)-p['received']>=.5:p['flags']=0
        return p

def pick(p,side='left'):

    l,r,f=p['left'],p['right'],p['flags']&3
    if side=='right':return r,l,((f&1)<<1)|((f>>1)&1)
    if side=='both':
        lf,rf=f&1,(f>>1)&1
        if lf and rf:return (l,r,f) if l<=r else (r,l,f)
        if rf:return r,l,((f&1)<<1)|((f>>1)&1)
    return l,r,f
def forward_snapshot(link,p,side='left'):

    if link.get_param(0) not in (0x020715,0x020716,0x020717,0x020718,0x020719) or link.get_param(80)!=0x544f4631:
        raise ValueError('FPGA不支持测距接收，未写入')
    lo,ro,fo=pick(p,side)
    for a,v in ((81,lo),(82,ro),(84,fo),(83,p['seq'])):link.set_param(a,v)
    if link.get_param(83)!=p['seq']:raise ValueError('FPGA测距提交未确认')

class BleWorker(QThread):
    state=Signal(str)
    sample=Signal(object)
    def __init__(self,name,mailbox):
        super().__init__();self.name=name;self.mailbox=mailbox;self.stop_event=threading.Event()
    def run(self):
        try:asyncio.run(self.session())
        except Exception as exc:self.state.emit('蓝牙连接失败／已断开：'+str(exc))
        finally:self.mailbox.clear()
    async def session(self):
        from bleak import BleakScanner,BleakClient
        self.state.emit('正在查找 '+self.name)
        device=None

        for _ in range(5):
            if self.stop_event.is_set():return
            device=await BleakScanner.find_device_by_filter(lambda d,a:a.local_name==self.name,timeout=2)
            if device:break
        if not device:raise ValueError('未找到ESP32，请确认供电及测距蓝牙固件')
        self.mailbox.clear();last_seq=None
        async with BleakClient(device,timeout=10) as client:
            def notify(_,data):
                nonlocal last_seq
                try:
                    p=decode(data)
                    if last_seq is not None and not 0<((p['device_seq']-last_seq)&0xffffffff)<0x80000000:return
                    last_seq=p['device_seq'];self.mailbox.put(p);self.sample.emit(p)
                except ValueError as exc:self.state.emit(str(exc))
            await client.start_notify(TX,notify)
            self.state.emit('蓝牙已连接：'+device.address)
            while not self.stop_event.is_set():
                if not client.is_connected:raise ValueError('ESP32连接丢失，距离已失效')
                await asyncio.sleep(.05)
            await client.stop_notify(TX)
        self.state.emit('蓝牙已断开，距离已失效')

class TofPanel(QWidget):
    def __init__(self,serial_worker):
        super().__init__();self.serial=serial_worker;self.ble=None
        box=QVBoxLayout(self)
        note=QLabel('ESP32 → 蓝牙 → 电脑 → FPGA串口。避障可选左／右／双侧：<6cm退避，>9cm持续200ms后停稳。超过0.5秒无新数据即失效。第二模块未接时单独显示未连接。')
        note.setWordWrap(True);box.addWidget(note)
        self.name=QLineEdit('FPGA-ESP32-TOF');box.addWidget(self.name)
        row=QHBoxLayout();box.addLayout(row)
        self.connect_button=QPushButton('连接测距蓝牙');row.addWidget(self.connect_button);self.connect_button.clicked.connect(self.connect_ble)
        self.disconnect_button=QPushButton('断开蓝牙');row.addWidget(self.disconnect_button);self.disconnect_button.clicked.connect(self.stop_ble)
        self.state_label=QLabel('蓝牙未连接');self.state_label.setWordWrap(True);box.addWidget(self.state_label)
        self.left=QLabel('左侧：—');self.right=QLabel('右侧：—')
        for label in (self.left,self.right):label.setStyleSheet('font-size:26px;font-weight:bold;color:#56d6c9');box.addWidget(label)
        self.detail=QLabel('等待测距数据');self.detail.setWordWrap(True);box.addWidget(self.detail)
        self.forward=QCheckBox('转发到FPGA（需连接REV23固件，最高10次/秒）');box.addWidget(self.forward)
        self.forward.toggled.connect(lambda enabled:self.serial.submit('tof_forward',enabled))
        self.forward_note=QLabel('串口转发未开启');self.forward_note.setWordWrap(True);box.addWidget(self.forward_note)
        side_row=QHBoxLayout();box.addLayout(side_row)
        self.avoid_left=QPushButton('启用左侧避障');side_row.addWidget(self.avoid_left)
        self.avoid_right=QPushButton('启用右侧避障');side_row.addWidget(self.avoid_right)
        self.avoid_both=QPushButton('双侧同时避障');side_row.addWidget(self.avoid_both)

        self.flip=QCheckBox('反转退避方向（不勾＝逆时针，勾选＝顺时针）')
        self.flip.setToolTip('默认逆时针。只有实测发现越退越靠近障碍物时才勾选。')
        box.addWidget(self.flip)

        for button,side in ((self.avoid_left,'left'),(self.avoid_right,'right'),(self.avoid_both,'both')):
            button.clicked.connect(lambda _=False,s=side:self.serial.submit('avoid_config',(s,1|(2 if self.flip.isChecked() else 0))))
        self.avoid_stop=QPushButton('关闭避障并停稳');box.addWidget(self.avoid_stop)
        self.avoid_stop.clicked.connect(lambda:self.serial.submit('avoid_config',0))

        self.clear_fault=QPushButton('清除轨迹故障（交还摆臂）');box.addWidget(self.clear_fault)
        self.clear_fault.clicked.connect(lambda:self.serial.submit('clear_fault',None))
        self.avoid_note=QLabel('需REV23；先竖直平衡并连接蓝牙。FPGA按距离调速约15～90°/s，平衡PID全程工作。默认逆时针退避（右侧来物即用此方向）。');self.avoid_note.setWordWrap(True);box.addWidget(self.avoid_note)
        box.addStretch();self.timer=QTimer(self);self.timer.timeout.connect(self.refresh);self.timer.start(100)
    def connect_ble(self):
        if self.ble and self.ble.isRunning():return
        self.ble=BleWorker(self.name.text().strip(),self.serial.tof)
        self.ble.state.connect(self.state_label.setText)
        self.connect_button.setEnabled(False)
        self.ble.finished.connect(lambda:self.connect_button.setEnabled(True))
        self.ble.start()
    def stop_ble(self):
        self.serial.tof.clear()
        if self.ble:self.ble.stop_event.set()
    def refresh(self):
        p=self.serial.tof.snapshot();age=time.monotonic()-p['received']
        for label,name,key,bit,stat in ((self.left,'左侧','left',1,'ls'),(self.right,'右侧','right',2,'rs')):
            if not p.get('present',0)&bit:text='未连接／未初始化'
            elif age>=.5:text='数据过期'
            elif not p['flags']&bit:text=f"测量无效（状态{p.get(stat,255)}）"
            else:text=f"{p[key]} mm / {p[key]/10:.1f} cm"
            label.setText(name+'：'+text)
        self.detail.setText(f"设备序号 {p.get('device_seq','—')} · 数据年龄 {age:.2f}s" if p['received'] else '等待测距数据')
    def shutdown(self):
        self.stop_ble()
        if self.ble and self.ble.isRunning():self.ble.wait(15000)
