import math
import time
from dbg_host import N_PARAM, PARAM_TABLE, float_to_q16, q16_to_float

READ_ONLY = {0, 1, 2, 31}
SIGNED = {17, 24, 29}
LABELS = ['固件标识','控制基准频率','PWM 频率','内环 P','内环 D','外环 P','外环 I',
          '捕获角度阈值','捕获角速度阈值','微分滤波位掩码','摩擦补偿幅度','输出限幅',
          '起摆位置 P','摆臂目标位置','起摆位置 D','传感器偏移','运行模式','开环输出',
          '竖直目标码','电机使能位掩码','外环 D','采集降采样指数','摩擦补偿模式',
          '运动摩擦补偿','起摆驱动','起摆时长','起摆超时','起摆行程上限','下垂角度码',
          '起摆目标位置','捕获确认时长','运行状态',
          '起摆刹车增益','A 段外环 P','A 段外环 D','B 段外环 P','B 段外环 D',
          'A 段时长','B 段时长','下垂按停阻尼',

          '走位斜坡','轨迹斜坡上限','斜坡加速度限',
          '转速给定','速度环 P','速度环 I','转速软启动',

          '回家限速','回正力上限',

          '走位减速度',

          '死区补偿形状',

          '走位绳子长度']

RANGES = {7:(0,180),8:(0,10000),9:(0,511),10:(0,1024),11:(1,1024),
          15:(-4096,4096),

          16:(0,4),17:(-1024,1024),18:(0,4092),19:(0,3),
          21:(0,4),22:(0,1),23:(0,1024),24:(-1024,1024),25:(0,60000),
          26:(0,60000),27:(0,8191),28:(0,4092),29:(-8191,8191),30:(0,60000),

          43:(-64,64),46:(0,1024),

          47:(0,8191),48:(0,8191),

          50:(0,3),

          51:(0,4096)}
SIGNED.add(15)

SIGNED.add(43)

def decode(addr, raw):
    if PARAM_TABLE[addr][2]:
        return q16_to_float(raw)
    return raw-2**32 if addr in SIGNED and raw & 0x80000000 else raw

def display(addr, raw):
    return f'{decode(addr,raw):.6f}'.rstrip('0').rstrip('.') if PARAM_TABLE[addr][2] else str(decode(addr,raw))

def encode(addr, text):
    if addr in READ_ONLY:
        raise ValueError('此项为只读')
    value = float(text)
    if not math.isfinite(value):
        raise ValueError('数值必须有限')
    if PARAM_TABLE[addr][2]:
        if not -32768 <= value < 32768:
            raise ValueError('超出 Q16.16 范围')
        return float_to_q16(value)
    if value != int(value):
        raise ValueError('此参数必须为整数')
    lo,hi = RANGES.get(addr,(0,2**32-1))
    if not lo <= value <= hi:
        raise ValueError(f'允许范围 {lo} 至 {hi}')
    return int(value) & 0xffffffff

def channels(rows, params):
    center = params[18]
    return ([ (r[0]-center)*360/4096 for r in rows],
            [r[2]*360/1040 for r in rows], [r[3] for r in rows])

class DemoLink:

    def __init__(self):
        self.raw = [0]*N_PARAM

        defaults = {1:1000,2:20000,3:4.5,4:120,5:-1.2,6:-.015,7:30,8:600,
                    9:271,10:130,11:1024,12:4,13:0,14:1.6,15:135,16:3,
                    18:3075,19:2,20:-40,21:0,23:50,24:-900,25:600,
                    26:1500,27:520,28:1017,29:350,30:55,32:0,33:-1.5,34:-32,35:-1.5,36:-32,37:0,38:0,39:2,

                    40:0.2,41:3787/65536,42:0.0015,43:0,44:-11,45:0,46:4,

                    47:0.25,48:0,49:30/65536,

                    50:1,51:0}
        for a,v in defaults.items():
            self.raw[a] = float_to_q16(v) if PARAM_TABLE[a][2] else int(v)&0xffffffff

        self.raw[0],self.raw[31] = 0x020712,(0x84|(1<<21))

        self.ext={}
        self.cmd_seq=0
        self.move_count=0
        self.move_due=None
        self.park_due=None
        self.parked=False
        self.started = None
        self.phase = 0

        self.demo_pos = 0.0
        self.demo_t = None
    def ping(self): return 0x0207
    def _demo_target(self):
        v=self.raw[13]&0xFFFFFFFF
        if v&0x80000000: v-=0x100000000
        return v/65536.0
    def read_live(self):
        t=time.monotonic()
        a=.9*math.sin(2*math.pi*1.1*t)+.3*math.sin(2*math.pi*7*t)
        if self.raw[16]==2:

            dt=0.0 if self.demo_t is None else min(.1,t-self.demo_t)
            self.demo_t=t
            alpha=1-math.exp(-dt/.08) if dt>0 else 0.0
            self.demo_pos+=(self._demo_target()-self.demo_pos)*alpha
            position=round(self.demo_pos)
        else:
            self.demo_t=None; self.demo_pos=0.0
            position=round(12*math.sin(2*math.pi*.6*t))
        return {'row':[self.raw[18]+round(a*4096/360),0,
                       position,round(100*math.sin(2*math.pi*7*t))],
                'sequence':int(t*1000)&0xffffffff,'status':self.raw[31]}
    def read_fuzzy(self):

        return {'kp': q16_to_float(self.raw[3]), 'kd': q16_to_float(self.raw[4]),
                'ec': 0.0, 'p_scale': 1.0, 'd_scale': 1.0,
                'p_scale_q8': 256, 'd_scale_q8': 256,
                'enabled': bool(self.raw[9] & 0x100)}
    def dump_params(self): return self.raw.copy()
    def get_param(self,a):

        if a==75:
            now=time.monotonic()

            if self.move_due is not None and now>=self.move_due:
                self.move_due=None
                self.move_count=(self.move_count+1)&0xFF
                self.parked=True
            if self.park_due is not None and now>=self.park_due:
                self.park_due=None
                self.parked=True
            return ((self.move_count&0xFF)<<8)|(1 if self.parked else 0)
        if a==79:

            return self.ext.get(79,0)
        if a==77:

            w=self.raw[43]&0xFFFFFFFF if self.raw[16]==4 else 0
            if w&0x80000000: w-=0x100000000
            return (w<<16)&0xFFFFFFFF
        if a>=N_PARAM: return 0
        return self.raw[a]
    def set_param(self,a,v):
        if a==79:
            self.cmd_seq=(self.cmd_seq+1)&0xffffff
            cmd=v&7
            self.ext[79]=((self.cmd_seq&0xffffff)<<8)|0x80|cmd
            if cmd in (1,2,4,5):
                self.move_due=time.monotonic()+1.2
                self.parked=False
            elif cmd==3:
                self.parked=True
            return
        self.raw[a] = v

        if a==16:
            self.raw[31] = (0x84|(1<<21)) if v in (3,4) else 0x80

            if v==2: self.demo_pos=self._demo_target(); self.demo_t=None

        if a==43 and self.raw[16]==4:
            if v: self.parked=False; self.park_due=None
            else: self.park_due=time.monotonic()+0.6
    def arm_capture(self,*args):
        self.started=time.monotonic()
        self.shift=self.raw[21]
    def cap_status(self):
        done=self.started is not None and time.monotonic()-self.started >= 4.096*(1<<self.shift)
        return (2 if done else 1),0
    def read_capture(self,**kwargs):
        fs=1000/(1<<self.shift)
        result=[]
        for i in range(4096):
            t=self.phase+i/fs
            a=.9*math.sin(2*math.pi*1.1*t)+.3*math.sin(2*math.pi*7*t)
            result.append([self.raw[18]+round(a*4096/360),0,
                           round(12*math.sin(2*math.pi*.6*t)),round(100*math.sin(2*math.pi*7*t))])
        self.phase+=4096/fs
        return result
    def disarm(self): self.started=None
    def close(self): pass
