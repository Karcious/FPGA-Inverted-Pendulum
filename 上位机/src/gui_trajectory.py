import ast
import math

FACTOR=1040/360*65536/1000
FUNCTIONS={'sin':math.sin,'cos':math.cos,'sqrt':math.sqrt,'abs':abs,'min':min,'max':max}

def build_function(expression,seconds):

    seconds=float(seconds)
    if len(expression)>256:raise ValueError('函数过长。')

    for ch in '·×÷−°π（）［］｛｝，；：':
        if ch in expression:
            raise ValueError('这段里有 ·、°、π 或全角符号 —— 那是界面上显示的写法。'
                             '表达式要写半角：乘号用 *、圆周率用 pi、去掉 °，'
                             '例如 90*cos(2*pi*t/T + pi/2)。')
    try:tree=ast.parse(expression,mode='eval')
    except SyntaxError as exc:raise ValueError('函数语法错误；乘方使用 **，例如 sin(pi*t/T)**2。') from exc
    if len(list(ast.walk(tree)))>80:raise ValueError('函数过于复杂。')
    def value(node,t):
        if isinstance(node,ast.Constant) and type(node.value) in (int,float) and abs(node.value)<=1e6:return float(node.value)
        if isinstance(node,ast.Name) and node.id in ('t','T','pi'):return {'t':t,'T':seconds,'pi':math.pi}[node.id]
        if isinstance(node,ast.UnaryOp) and isinstance(node.op,(ast.UAdd,ast.USub)):
            v=value(node.operand,t);return v if isinstance(node.op,ast.UAdd) else -v
        if isinstance(node,ast.BinOp):
            a,b=value(node.left,t),value(node.right,t)
            if isinstance(node.op,ast.Add):return a+b
            if isinstance(node.op,ast.Sub):return a-b
            if isinstance(node.op,ast.Mult):return a*b
            if isinstance(node.op,ast.Div):return a/b
            if isinstance(node.op,ast.Pow) and abs(b)<=8:return a**b
        if isinstance(node,ast.Call) and isinstance(node.func,ast.Name) and node.func.id in FUNCTIONS and not node.keywords and 1<=len(node.args)<=4:
            return FUNCTIONS[node.func.id](*[value(n,t) for n in node.args])
        raise ValueError('仅支持 t、T、pi、四则运算、** 和 sin/cos/sqrt/abs/min/max。')
    def evaluate(t):
        try:v=value(tree.body,t)
        except (ArithmeticError,TypeError,ValueError) as exc:raise ValueError(f't={t:.3f}s 时函数无效：{exc}') from exc
        if not isinstance(v,(int,float)) or not math.isfinite(v):
            raise ValueError(f't={t:.3f}s 时算出来的不是有限实数。')
        return float(v)
    return evaluate

def compile_curve(expression,seconds,direction=1):
    seconds=float(seconds)
    if not math.isfinite(seconds) or not .1<=seconds<=60 or direction not in (-1,1):
        raise ValueError('运动时间范围 0.1～60 秒，方向必须为正向或反向。')
    ms=round(seconds*1000)
    if abs(ms/1000-seconds)>1e-8:raise ValueError('时间最多保留三位小数。')
    f=build_function(expression,seconds)
    def evaluate(t):
        try:v=direction*f(t)
        except ValueError as exc:raise ValueError(str(exc)) from exc
        return v
    points=[evaluate(i/1000) for i in range(ms+1)]
    if abs(points[0])>.05 or abs(points[-1])>.05:raise ValueError('起点和终点速度应为 0（允许 ±0.05°/s），请给函数加入加减速段。')
    if any(abs(b-a)>.120001 for a,b in zip(points,points[1:])):raise ValueError('曲线加速度超过 120°/s² 或有速度突变。')
    dt=max(1,(ms+1023)//1024)
    words=[];positions=[0];speeds=[];times=[0]
    for start in range(0,ms,dt):
        end=min(ms,start+dt)

        avg=sum((points[i]+points[i+1])/2 for i in range(start,end))/(end-start)
        word=round(avg*FACTOR)
        if abs(word)>167772:raise ValueError(f'量化后速度超过当前固件数据范围（约 ±{167772/FACTOR:.2f}°/s），未上传。')
        words.append(word)
        speeds.append(word/FACTOR)
        positions.append(positions[-1]+word/FACTOR*(end-start)/1000)
        times.append(end/1000)
    return dict(expression=expression,seconds=seconds,direction=direction,duration=ms,interval=dt,
                words=words,positions=positions,times=times,speeds=speeds,
                checksum=sum(words)&0xffffffff)

class TrajectorySession:
    def __init__(self):self.active=False;self.uploading=False;self.curve=None;self.index=0;self.snapshot=None;self.revision=0
    def supported(self,link):
        magic=link.get_param(0);self.revision=magic&255
        if magic>>8!=0x0207 or magic&255<2:raise ValueError('自定义轨迹需要 ZL.10.1 0x020702 或更新修订固件。')
    def begin(self,link,curve):
        self.supported(link)
        if curve.get('continuous') and self.revision<3:raise ValueError('持续循环需要 0x020703 固件。')
        if self.active or link.get_param(64)&1:raise ValueError('轨迹已在运行。')
        if link.get_param(16)!=3:raise ValueError('请先自动起摆，并确认倒立平衡后再开始。')
        status=link.get_param(31)
        holding=bool(link.get_param(64)&16)
        if self.revision>=4:
            if status&7!=4 or status&0x18:
                raise ValueError('尚未确认倒立平衡或保护生效；请先完成起摆。')
            link.set_param(64,3)
            if not link.get_param(64)&32:raise ValueError('轨迹接管未确认，未上传。')
            self.active=True
        elif not holding and (not status&(1<<21) or not status&(1<<18) or status&(1<<25)):
            raise ValueError('当前固件需等按键走位到位；要立即接管，请烧录 0x020704 固件。')
        try:
            link.set_param(64,0)
            link.set_param(67,0x20702)
            link.set_param(65,curve['duration']);link.set_param(66,curve['interval'])
            if link.get_param(65)!=curve['duration'] or link.get_param(66)!=curve['interval']:raise ValueError('轨迹配置回读不一致。')
            if self.revision>=3:
                link.set_param(72,2|int(curve.get('continuous',False)))
            self.curve=curve;self.index=0;self.snapshot=None;self.active=True;self.uploading=True
        except Exception:
            if self.revision>=4 and self.active:
                link.set_param(64,2);self.uploading=False
            raise
    def update(self,link,gain,rate):
        if self.revision<3 or not self.active or self.uploading or not link.get_param(64)&1 or link.get_param(64)&(8|32):
            raise ValueError('仅在 REV3 曲线运行时允许实时改参。')
        if not math.isfinite(gain) or not -8<=gain<=8 or not math.isfinite(rate) or not .125<=rate<=4:
            raise ValueError('速度倍率范围 ±8，时间倍率范围 0.125～4。')
        g=round(gain*65536)&0xffffffff;r=round(rate*65536)
        link.set_param(73,g);link.set_param(74,r);link.set_param(75,1)
        if link.get_param(73)!=g or link.get_param(74)!=r:raise ValueError('实时参数回读不一致。')
    def stop(self,link):
        self.supported(link);link.set_param(64,2)
        if not link.get_param(64)&(8|16) and link.get_param(64)&1:raise ValueError('减速停止请求未确认。')
        self.active=bool(link.get_param(64)&1);self.uploading=False
    def poll(self,link):
        if self.uploading:
            for _ in range(4):
                if self.index==len(self.curve['words']):break
                link.set_param(68,self.curve['words'][self.index]);self.index+=1
            if self.index<len(self.curve['words']):return f'上传 {self.index}/{len(self.curve["words"])}，'+('已接管并保持当前位置' if self.revision>=4 else '电机保持原状态')
            if link.get_param(70)!=self.index or link.get_param(67)!=self.curve['checksum'] or link.get_param(64)>>8:
                raise ValueError('轨迹上传校验失败，未启动。')
            link.set_param(64,1);self.uploading=False
        s=link.get_param(64);elapsed=link.get_param(69)
        signed=lambda x:x-0x100000000 if x&0x80000000 else x
        self.snapshot=dict(status=s,elapsed=elapsed/1000,target=signed(link.get_param(71))/FACTOR)
        if self.revision>=3:
            self.snapshot.update(cycles=link.get_param(76),actual=signed(link.get_param(77))/65536*360/1040*40)
        if s>>8:
            self.active=bool(s&1)
            reason={1:'配置无效或启动时未稳定到位',3:'平衡状态丢失或按键接管',4:'恢复平衡等待超过30秒',5:'停车5秒仍未收敛，位置保持仍在工作；若仍连续转动，请全局停机',6:'等待启动窗口超过30秒，已取消轨迹并保持位置'}.get((s>>8)&255,'未知故障')
            return '轨迹中止：'+reason
        if s&32:
            if not s&64:return '已接管并保持位置，但尚未提交启动；请停止后重新上传。'
            return '已取消旧按键指令，保持当前位置；等待角度±10.55°、速度±55.38°/s内启动。'
        if s&4:self.active=False;return '轨迹已完成并停车到位：保持停止位置，继续平衡。'
        if s&8:return f'位置停车中，已冻结参考，等待实测稳定 · 轨迹时间 {elapsed/1000:.3f}s'
        if not s&1:self.active=False;return '已停止曲线并锁定停车参考，继续平衡。'
        return f'{"暂停计时，恢复平衡中" if s&2 else "运行中"} · 轨迹时间 {elapsed/1000:.3f}s'
