COUNTS_PER_REV=1080
MOVE_LIMIT_COUNTS=520
MOVE_CHUNK_COUNTS=500

LEG_LIMIT_COUNTS=8191
LEG_MIN_REV=0x12
TRAVEL_LIMIT_COUNTS=32767
SPIN_W_LIMIT=64
SPIN_TICK_MS=25

SPIN_RPM_PER_CODE=60000.0/(SPIN_TICK_MS*COUNTS_PER_REV)

SPIN_RPM_SCALE=1.4

MOVE_DECEL_PER_SLEW=150.0

MOVE_CRAWL_Q16=2048

def move_decel_code(slew_ms):

    return max(1,int(round(float(slew_ms)*MOVE_DECEL_PER_SLEW)))

def move_brake_counts(slew_ms,decel_code=None):

    q=int(round(float(slew_ms)*65536))
    a=move_decel_code(slew_ms) if decel_code is None else int(decel_code)
    v=min(max(q,0),0x100000)
    total=0; steps=0
    while v>MOVE_CRAWL_Q16 and a>0 and steps<65535:
        total+=v; v-=a; steps+=1
    total+=(MOVE_CRAWL_Q16<<6)+(q<<2)
    return total/65536.0

def angle_move_counts(degrees, direction):

    if direction not in (-1, 1):
        raise ValueError('移动方向必须为 -1 或 +1')
    if not float(degrees) > 0:
        raise ValueError('角度必须为正数：方向由「逆时针/顺时针」决定，不接受负角度')
    counts=direction*int(round(float(degrees)*COUNTS_PER_REV/360.0))

    if not 0 < abs(counts) <= MOVE_LIMIT_COUNTS:
        limit_deg=MOVE_LIMIT_COUNTS*360.0/COUNTS_PER_REV
        raise ValueError(f'角度需在 0.2°～{limit_deg:.1f}° 之间'
                         f'（命令 4 最多 ±{MOVE_LIMIT_COUNTS} 码）')
    return counts

def travel_move_counts(degrees, direction):

    if direction not in (-1, 1):
        raise ValueError('移动方向必须为 -1 或 +1')
    if not float(degrees) > 0:
        raise ValueError('角度必须为正数：方向由「逆时针/顺时针」决定，不接受负角度')
    counts=direction*int(round(float(degrees)*COUNTS_PER_REV/360.0))
    if not 0 < abs(counts) <= TRAVEL_LIMIT_COUNTS:
        raise ValueError('多圈走位需在 0.2°～11342°（±32767 码）之间')
    return counts

def move_limit_counts(magic):

    if magic>>8 == 0x0207 and magic&255 >= LEG_MIN_REV:
        return LEG_LIMIT_COUNTS
    return MOVE_LIMIT_COUNTS

def plan_angle_move(counts, magic):

    counts=int(counts)
    if counts==0:
        raise ValueError('走位码数不能为 0')
    limit=move_limit_counts(magic)
    if abs(counts)<=limit:
        return limit,[counts]
    chunk=limit if limit>MOVE_LIMIT_COUNTS else MOVE_CHUNK_COUNTS
    return limit,split_move_counts(counts,chunk=chunk,limit=limit)

def split_move_counts(counts, chunk=MOVE_CHUNK_COUNTS, limit=MOVE_LIMIT_COUNTS):

    if counts == 0:
        raise ValueError('走位码数不能为 0')
    sign=-1 if counts<0 else 1
    total=abs(int(counts))

    if total <= limit:
        return [sign*total]
    assert chunk<=limit, 'chunk 不能超过固件上限'
    n=-(-total//chunk)
    base,rem=divmod(total,n)

    out=[base+1]*rem+[base]*(n-rem)
    return [sign*c for c in out]

def spin_speed_code(rpm, direction):

    if direction not in (-1, 1):
        raise ValueError('旋转方向必须为 -1 或 +1')
    if not float(rpm) > 0:
        raise ValueError('转速必须为正数：方向由「逆时针/顺时针」决定，不接受负转速')
    per_code=SPIN_RPM_PER_CODE*SPIN_RPM_SCALE
    w=int(round(abs(float(rpm))/per_code))
    if not 1 <= w <= SPIN_W_LIMIT:
        raise ValueError(f'转速需在 {per_code:.1f}～{SPIN_W_LIMIT*per_code:.0f} rpm 之间'
                         f'（mem[43] 是整数码/25ms，夹在 ±64）')
    return direction*w

def spin_speed_rpm(code):

    return code*SPIN_RPM_PER_CODE*SPIN_RPM_SCALE

def decode_motion_status(raw):

    err=(raw>>16)&0xFFFF
    if err & 0x8000: err-=0x10000
    return {'raw':raw,'error':err,'count':(raw>>8)&0xFF,
            'aborted':bool(raw&2),'parked':bool(raw&1)}

def request_key_move(link, direction):
    if direction not in (-1, 1):
        raise ValueError('移动方向必须为 -1 或 +1')
    magic=link.get_param(0)
    if magic>>8 != 0x0207 or magic&255 < 6:
        raise ValueError('左右90°按钮需要 0x020706 或更新的兼容固件')
    mode,status=link.get_param(16),link.get_param(31)
    if mode != 3 or status&7 != 4 or status&0x18:
        raise ValueError('请先自动起摆并确认平衡，解除故障后再走位')
    if status & (1<<24):
        raise ValueError('回零故障：稳定后长按SW3约2秒，以当前位置重新置零；等按键就绪后再走位')
    if not status & (1<<21):
        raise ValueError('按键尚未就绪：等待回零停稳后重试，本次未发送走位命令')
    if link.get_param(64)&1:
        raise ValueError('请先停止速度轨迹并等待停稳，再使用左右90°按钮')
    command=1 if direction<0 else 2
    before=link.get_param(79)
    link.set_param(79,command)
    reply=link.get_param(79)
    if reply>>8 != ((before>>8)+1)&0xffffff or reply&3 != command:
        raise ValueError('按键命令回执不一致；请检查状态，不自动重发')
    if not reply&0x80:
        reason=(reply>>4)&7
        raise ValueError({1:'板上状态已改变：尚未平衡、正在走位（一程没走完）或存在保护/回零故障',
                          2:'速度轨迹仍占用控制，请先停止并等待停稳',
                          3:'板上拒绝了无效方向命令',
                          4:'板上按键尚未就绪，本次未排队；等待回零停稳后重试'}.get(reason,'板上拒绝走位命令'))
    return ('逆时针90°（SW2，编码器减少）' if direction<0 else '顺时针90°（SW4，编码器增加）')+'已接收；停稳后自动重定零点。'

def request_zero_calibration(link):

    magic=link.get_param(0)
    if magic>>8 != 0x0207 or magic&255 < 0x0E:
        raise ValueError('「标定0点」需要 REV14（0x0002070E）或更新的固件；当前是 0x%08X' % magic)
    before=link.get_param(79)
    link.set_param(79,3)
    reply=link.get_param(79)
    if reply>>8 != ((before>>8)+1)&0xffffff or reply&3 != 3:
        raise ValueError('标定命令回执不一致；请检查状态，不自动重发')
    if not reply&0x80:
        reason=(reply>>4)&7
        raise ValueError({5:'当前状态不允许标定：请先停摆滑行，或先稳稳平衡、让摆臂停住再标定'}.get(reason,'板上拒绝标定0点'))
    return '标定0点已接收：当前位置记为 0 点（等同长按 SW3）。'

def request_angle_move(link, degrees, direction):

    return request_angle_move_counts(link, angle_move_counts(degrees, direction))

def request_angle_move_counts(link, counts):

    counts=int(counts)
    magic=link.get_param(0)
    limit=move_limit_counts(magic)
    if not 0 < abs(counts) <= limit:
        raise ValueError('单段码数需在 ±%d 之内（当前 %+d）；'
                         'REV18 起一条命令 4 最多 ±8191 码'
                         % (limit, counts))
    if magic>>8 != 0x0207 or magic&255 < 0x0F:
        raise ValueError('「走任意角度」需要 REV15（0x0002070F）或更新的固件；当前是 0x%08X' % magic)
    mode,status=link.get_param(16),link.get_param(31)
    if mode != 3:
        raise ValueError('走位只在模式 3（自动起摆、平衡中）可用：请先起摆；'
                         '连续旋转（模式 4）请先暂停并回到模式 3')
    if status&7 != 4 or status&0x18:
        raise ValueError('请先自动起摆并确认平衡，解除故障后再走位')
    if status & (1<<24):
        raise ValueError('回零故障：稳定后长按SW3约2秒，以当前位置重新置零；等按键就绪后再走位')
    if not status & (1<<21):
        raise ValueError('按键尚未就绪：等待回零停稳后重试，本次未发送走位命令')
    if link.get_param(64)&1:
        raise ValueError('请先停止速度轨迹并等待停稳，再使用走位')
    before=link.get_param(79)
    link.set_param(79,((counts&0xFFFF)<<16)|4)
    reply=link.get_param(79)

    if reply>>8 != ((before>>8)+1)&0xffffff or reply&7 != 4:
        raise ValueError('走位命令回执不一致；请检查状态，不自动重发')
    if not reply&0x80:
        reason=(reply>>4)&7
        raise ValueError({1:'板上状态已改变：尚未平衡、正在走位（一程没走完）、正在连续旋转或存在保护/回零故障',
                          2:'速度轨迹仍占用控制，请先停止并等待停稳',
                          3:'板上拒绝了这组码数（0 或超出当前固件的上限）',
                          4:'板上按键尚未就绪，本次未排队；等待回零停稳后重试'}.get(reason,'板上拒绝走位命令'))
    name='逆时针' if counts<0 else '顺时针'
    return (f'{name} {abs(counts)*360.0/COUNTS_PER_REV:.1f}°（{counts:+d} 码）'
            '已接收；到点停稳后自动重定零点。'),counts

def request_travel_move(link, degrees, direction, rpm):

    counts=travel_move_counts(degrees, direction)
    magic=link.get_param(0)
    if magic>>8 != 0x0207 or magic&255 < 0x10:
        raise ValueError('超过半圈的走位需要 REV16（0x00020710）或更新的固件；当前是 0x%08X' % magic)
    mode,status=link.get_param(16),link.get_param(31)
    if mode != 3:
        raise ValueError('走位只在模式 3（自动起摆、平衡中）可用：请先起摆；'
                         '连续旋转（模式 4）请先暂停并回到模式 3')
    if status&7 != 4 or status&0x18:
        raise ValueError('请先自动起摆并确认平衡，解除故障后再走位')
    if status & (1<<24):
        raise ValueError('回零故障：稳定后长按SW3约2秒，以当前位置重新置零；等按键就绪后再走位')
    if not status & (1<<21):
        raise ValueError('按键尚未就绪：等待回零停稳后重试，本次未发送走位命令')
    if link.get_param(64)&1:
        raise ValueError('请先停止速度轨迹并等待停稳，再使用走位')
    w=spin_speed_code(rpm,1)
    link.set_param(43,w)
    if link.get_param(43)!=w:
        raise ValueError('转速给定回读不一致，本次未发送走位命令')
    before=link.get_param(79)
    link.set_param(79,((counts&0xFFFF)<<16)|5)
    reply=link.get_param(79)
    if reply>>8 != ((before>>8)+1)&0xffffff or reply&7 != 5:
        raise ValueError('走位命令回执不一致；请检查状态，不自动重发')
    if not reply&0x80:
        reason=(reply>>4)&7
        raise ValueError({1:'板上状态已改变：尚未平衡、另一条走位（含一程坐标的长程）还没到点，或存在保护/回零故障',
                          2:'速度轨迹仍占用控制，请先停止并等待停稳',
                          3:'板上拒绝了这组码数（0 或超出 ±32767）',
                          4:'板上按键尚未就绪，本次未排队；等待回零停稳后重试',
                          5:'转速给定是 0：先把转速设成非 0 再走位'}.get(reason,'板上拒绝走位命令'))
    name='逆时针' if direction<0 else '顺时针'
    turns=abs(counts)/float(COUNTS_PER_REV)
    return (f'{name} {float(degrees):.1f}°（{counts:+d} 码，{turns:.2f} 圈）已接收；'
            f'按 {spin_speed_rpm(abs(w)):.1f} rpm 走，停稳后自动重定零点。'),counts

def request_spin_start(link, rpm, direction):

    w=spin_speed_code(rpm,direction)
    magic=link.get_param(0)
    if magic>>8 != 0x0207 or magic&255 < 0x0F:
        raise ValueError('连续旋转需要 REV15（0x0002070F）或更新的固件；当前是 0x%08X' % magic)
    mode,status=link.get_param(16),link.get_param(31)
    if mode not in (3,4):
        raise ValueError('连续旋转需要先自动起摆进入平衡（模式 3）；当前模式 %d' % mode)
    if status&7 != 4 or status&0x18:
        raise ValueError('尚未确认倒立平衡或保护生效；请先完成起摆')
    if status & (1<<24):
        raise ValueError('回零故障：稳定后长按SW3约2秒，以当前位置重新置零后再转')

    if mode == 3 and not status & (1<<21):
        raise ValueError('按键尚未就绪：等待回零停稳后重试，本次未起转')
    if link.get_param(64)&1:
        raise ValueError('请先停止速度轨迹并等待停稳，再开始连续旋转')
    if mode==3:
        link.set_param(43,0)
        if link.get_param(43)!=0:
            raise ValueError('转速给定清零回读不一致，未切换模式')
        link.set_param(16,4)
        if link.get_param(16)!=4:
            raise ValueError('模式 4 切换未被板上确认，本次未起转')
    link.set_param(43,w&0xFFFFFFFF)
    if link.get_param(43)!=(w&0xFFFFFFFF):
        raise ValueError('转速给定回读不一致，板上可能没在按这个速度转')
    name='逆时针' if direction<0 else '顺时针'
    eff=spin_speed_rpm(abs(w))
    return f'{name}连续旋转已起转：{eff:.1f} rpm（mem[43] = {w:+d} 码/25ms）。'

def request_spin_pause(link):

    magic=link.get_param(0)
    if magic>>8 != 0x0207 or magic&255 < 0x0F:
        raise ValueError('暂停旋转需要 REV15（0x0002070F）或更新的固件；当前是 0x%08X' % magic)
    if link.get_param(16)!=4:
        raise ValueError('当前不在连续旋转（模式 4），没有需要暂停的旋转')
    raw=link.get_param(43)&0xFFFFFFFF
    w=raw-0x100000000 if raw&0x80000000 else raw
    link.set_param(43,0)
    if link.get_param(43)!=0:
        raise ValueError('转速给定清零回读不一致，板上可能仍在转；请按「停止电机」')
    if w==0:
        return '转速给定本来就是 0：没有需要停靠的旋转。',False
    return f'已请求减速停靠（原给定 {w:+d} 码/25ms）；等「已停靠」出现后零点才搬到停下的地方。',True
