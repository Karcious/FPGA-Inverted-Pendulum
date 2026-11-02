import time
from gui_tof import TofMailbox,forward_snapshot
import gui_quiet
MAGIC=0x020719

MAGIC_OK=(0x020717,0x020718,0x020719)

TRAJ_FAULT={1:'启动条件不满足或曲线数据非法',3:'运动中途被安全条件掐断',
            4:'暂停超时',5:'刹车5秒没停住',6:'准备阶段超时'}

SWING_FAULT={1:'传感器',2:'配置',3:'编码器速度',4:'角度跳变',5:'行程',6:'准备超时',
             7:'起摆超时',8:'闭环保护/捕获偏离',9:'端点过渡超时',10:'捕获确认超时'}
def init(w):
 w.tof=TofMailbox();w.tof_forward=False;w.tof_key=None;w.tof_side='left';w.next_tof=0;w.next_features=0

def command(w,kind,data):
 if kind not in ('tof_forward','avoid_config','quiet_config','clear_fault'):return False
 try:
  if w.link is None:raise ValueError('请先连接FPGA')
  if w.link.get_param(0) not in MAGIC_OK:raise ValueError('新功能需要合并版REV23固件')
  if kind=='tof_forward':
   w.tof_forward=bool(data);w.tof_key=None
   w.event.emit('tof_enabled',w.tof_forward)
  elif kind=='quiet_config':w.event.emit('quiet_note',gui_quiet.configure(w.link,*data))
  elif kind=='clear_fault':

   c=w.link.get_param(64)
   if not c>>8:raise ValueError('现在没有轨迹故障')
   if c&1:raise ValueError('轨迹还在运行，先停止再清故障')
   w.link.set_param(64,0)
   c=w.link.get_param(64)
   if c>>8:raise ValueError('故障没清掉（码 %d），请复位板子'%(c>>8))
   w.event.emit('avoid_note','轨迹故障已清除，摆臂已交还位置环')
  else:

   side,value=('left',int(data)) if not isinstance(data,(tuple,list)) else (str(data[0]),int(data[1]))
   if side not in ('left','right','both'):side='left'
   if value:
    if w.wave.active or w.trajectory.active or w.link.get_param(64)&1:raise ValueError('先停止当前轨迹并停稳')
    if w.link.get_param(16)!=3 or w.link.get_param(31)&31!=4:raise ValueError('先完成竖直平衡')

    p=w.tof.snapshot()
    if not p.get('received') or time.monotonic()-p['received']>=.5:raise ValueError('测距数据未到达或已过期')
    if not p.get('present',0)&{'left':1,'right':2,'both':3}[side]:raise ValueError('所选那一路模块未连接')
    w.tof_side=side
    forward_snapshot(w.link,p,side)
   w.link.set_param(86,value)
   if w.link.get_param(86)!=value:raise ValueError('避障开关未确认')
   if value:
    w.tof_forward=True;w.tof_key=None;w.event.emit('tof_enabled',True)
    w.event.emit('avoid_note',{'left':'左侧避障已开启','right':'右侧避障已开启','both':'双侧避障已开启'}[side])
   else:w.event.emit('avoid_note','避障已关闭，等待停稳')
 except ValueError as exc:
  w.event.emit('log',str(exc));w.event.emit('avoid_note',str(exc))
 return True

def poll(w,now):
 if w.params is None or w.params[0] not in MAGIC_OK:return
 if w.tof_forward and now>=w.next_tof:
  p=w.tof.snapshot(now);key=(p['seq'],p['flags'],w.tof_side)
  if key!=w.tof_key:
   forward_snapshot(w.link,p,w.tof_side);w.tof_key=key
   w.event.emit('tof_note',f"FPGA已确认距离：帧{p['seq']}，有效标志{p['flags']}")
  w.next_tof=now+.1
 if now>=w.next_features:
  w.event.emit('quiet_note',gui_quiet.status_text(w.link))
  a=w.link.get_param(87);c=w.link.get_param(64)
  note='避障关闭' if not a&1 else '测距无效，停止退避' if not a&2 else '正在退避／等待平衡启动' if a&4 else '正在停稳' if c&9 else '检测中'

  if c>>8:note+='；轨迹故障码 %d：%s'%(c>>8,TRAJ_FAULT.get(c>>8,'未知'))
  st=w.link.get_param(31)
  if st&7==5:note+='；板上已进故障态，原因：'+SWING_FAULT.get((st>>10)&15,'未知')
  w.event.emit('avoid_note',note);w.next_features=now+.5

def guard(w,kind):
 if w.params is None or w.params[0] not in MAGIC_OK:return False
 conflicting=('key_move','zero_cal','angle_move','angle_move_counts','travel_move','spin_start','spin_pause','wave_start','trajectory_start','trajectory_update')
 if kind in conflicting and w.link.get_param(86)&1:
  w.event.emit('log','请先关闭避障并等待停稳，再执行其他运动。');return True
 if kind in ('stop','trajectory_stop','disconnect','quit'):
  w.link.set_param(86,0)
 return False
