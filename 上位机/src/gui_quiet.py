import math
REASONS={0:'静稳抑制中',1:'同向增大趋势唤醒',2:'编码器运动唤醒',3:'大偏角唤醒',4:'运动/保护状态变化',5:'开关更新'}
def configure(link,enabled,seconds,angle):
    if link.get_param(0) not in (0x02070F,0x020710,0x020711,0x020712,0x020713,0x020714,0x020715,0x020716,0x020717,0x020718,0x020719):raise ValueError('静稳抑制需要 0x02070F或0x020710 / REV15或REV16 固件')
    seconds=float(seconds);angle=float(angle)
    if not math.isfinite(seconds) or not math.isfinite(angle) or not .5<=seconds<=5 or angle<.35:
        raise ValueError('静止时间0.5～5秒，趋势唤醒角至少0.35°')
    if angle>0xffffffff*360/4096:raise ValueError('阈值超过32位寄存器可表示范围')
    codes=max(4,round(angle*4096/360))
    if codes>0xffffffff:raise ValueError('阈值超过32位寄存器可表示范围')
    for addr,value in [(88,0),(89,round(seconds*1000)),(90,codes),(88,int(bool(enabled)))]:
        link.set_param(addr,value)
        if link.get_param(addr)!=value:raise ValueError('静稳配置回读不一致，请检查板上状态')
    return f'静稳抑制已{"启用" if enabled else "关闭"}；趋势阈值实际 {codes*360/4096:.3f}°，大偏角 {2*codes*360/4096:.3f}°直接唤醒。'
def status_text(link):
    v=link.get_param(91)
    return ('关闭' if not v&1 else '正在抑制驱动' if v&2 else '已启用，正常控制/等待静稳')+' · '+('尚无退出事件' if not v&2 and (v>>9&255)==0 else REASONS.get(v>>9&255,'未知原因'))
