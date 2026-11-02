import unittest
from gui_motion import (request_key_move, request_zero_calibration, request_angle_move,
                        request_angle_move_counts, request_travel_move,
                        request_spin_start, request_spin_pause,
                        angle_move_counts, travel_move_counts,
                        move_limit_counts, plan_angle_move, split_move_counts,
                        spin_speed_code, spin_speed_rpm, decode_motion_status,
                        move_decel_code, move_brake_counts)
class Link:
    def __init__(self):
        self.reg={0:0x20706,16:3,31:4|(1<<21),64:0,79:0,43:0};self.writes=[];self.reject=0;self.no_ack=False
        self.stuck43=False
    def get_param(self,a):return self.reg.get(a,0)
    def set_param(self,a,v):
        self.writes.append((a,v))
        if a==79:

            if not self.no_ack:
                seq=((self.reg[79]>>8)+1)&0xffffff
                low=((self.reject<<4) if self.reject else 0x80)|(v&7)
                self.reg[79]=(seq<<8)|low
            return
        if a==43 and self.stuck43:
            return
        self.reg[a]=v
class TestKeyMove(unittest.TestCase):
    def test_both_directions_and_repeat_receipts(self):
        l=Link()
        for d in (-1,1,-1):self.assertIn('已接收',request_key_move(l,d))
        self.assertEqual(l.writes,[(79,1),(79,2),(79,1)])
    def test_preconditions_never_write(self):
        for a,v in [(0,0x20705),(16,0),(31,6),(31,12),(31,4),(31,4|(1<<21)|(1<<24)),(64,1)]:
            l=Link();l.reg[a]=v
            with self.subTest(a=a,v=v),self.assertRaises(ValueError):request_key_move(l,1)
            self.assertFalse(l.writes)
    def test_reject_and_missing_ack_not_retried(self):
        for rejection in (1,2,3,4):
            l=Link();l.reject=rejection
            with self.assertRaises(ValueError):request_key_move(l,1)
            self.assertEqual(len(l.writes),1)
        l=Link();l.no_ack=True
        with self.assertRaises(ValueError):request_key_move(l,-1)
        self.assertEqual(l.writes,[(79,1)])
    def test_sequence_wrap(self):
        l=Link();l.reg[79]=0xffffff81
        request_key_move(l,1);self.assertEqual(l.reg[79],0x82)
class TestZeroCalibration(unittest.TestCase):

    def test_sends_command_three(self):
        l=Link();l.reg[0]=0x2070E
        self.assertIn('已接收',request_zero_calibration(l))
        self.assertEqual(l.writes,[(79,3)])
    def test_old_firmware_never_writes(self):
        for rev in (0x20705,0x2070D):
            l=Link();l.reg[0]=rev
            with self.subTest(rev=rev),self.assertRaises(ValueError):request_zero_calibration(l)
            self.assertFalse(l.writes)
    def test_reason_five_and_missing_ack_not_retried(self):
        l=Link();l.reg[0]=0x2070E;l.reject=5
        with self.assertRaises(ValueError) as ctx:request_zero_calibration(l)
        self.assertIn('不允许标定',str(ctx.exception))
        self.assertEqual(len(l.writes),1)
        l=Link();l.reg[0]=0x2070E;l.no_ack=True
        with self.assertRaises(ValueError):request_zero_calibration(l)
        self.assertEqual(l.writes,[(79,3)])

class TestAngleMove(unittest.TestCase):

    def test_degrees_to_signed_counts(self):

        self.assertEqual(angle_move_counts(90,-1),-270)
        self.assertEqual(angle_move_counts(90,1),270)
        self.assertEqual(angle_move_counts(30,1),90)
        self.assertEqual(angle_move_counts(173,1),519)
        for bad in (0,0.1,180.5,-5,174):
            with self.subTest(bad=bad),self.assertRaises(ValueError):angle_move_counts(bad,1)
        with self.assertRaises(ValueError):angle_move_counts(30,0)

    def test_writes_command_four_with_counts_in_the_top_half(self):
        l=Link();l.reg[0]=0x2070F
        note,counts=request_angle_move(l,30,-1)
        self.assertEqual(counts,-90)
        self.assertEqual(l.writes,[(79,(((-90)&0xFFFF)<<16)|4)])
        self.assertIn('逆时针',note);self.assertIn('已接收',note)
        l=Link();l.reg[0]=0x2070F
        request_angle_move(l,90,1)
        self.assertEqual(l.writes,[(79,(270<<16)|4)])

    def test_preconditions_never_write(self):
        for a,v in [(0,0x2070E),(16,4),(16,2),(31,6),(31,12),(31,4),
                    (31,4|(1<<21)|(1<<24)),(64,1)]:
            l=Link();l.reg[0]=0x2070F;l.reg[a]=v
            with self.subTest(a=a,v=v),self.assertRaises(ValueError):request_angle_move(l,30,1)
            self.assertEqual(l.writes,[],f'{a}={v} 不该发命令')

    def test_reject_reasons_and_missing_ack_not_retried(self):
        for rejection in (1,2,3,4):
            l=Link();l.reg[0]=0x2070F;l.reject=rejection
            with self.assertRaises(ValueError):request_angle_move(l,30,1)
            self.assertEqual(len(l.writes),1)
        l=Link();l.reg[0]=0x2070F;l.no_ack=True
        with self.assertRaises(ValueError):request_angle_move(l,30,1)
        self.assertEqual(len(l.writes),1)

    def test_bad_angle_fails_before_touching_the_link(self):
        l=Link();l.reg[0]=0x2070F
        with self.assertRaises(ValueError):request_angle_move(l,360,1)
        self.assertEqual(l.writes,[])

class TestTravelMove(unittest.TestCase):

    def test_degrees_to_signed_counts_beyond_half_turn(self):

        self.assertEqual(travel_move_counts(560,1),1680)
        self.assertEqual(travel_move_counts(360,-1),-1080)
        self.assertEqual(travel_move_counts(1800,1),5400)
        self.assertEqual(travel_move_counts(30,1),90)
        self.assertEqual(travel_move_counts(10900,1),32700)
        for bad in (0,0.1,-5,12000):
            with self.subTest(bad=bad),self.assertRaises(ValueError):travel_move_counts(bad,1)
        with self.assertRaises(ValueError):travel_move_counts(30,0)

    def test_writes_speed_then_command_five(self):
        l=Link();l.reg[0]=0x20710
        note,counts=request_travel_move(l,560,-1,30)
        self.assertEqual(counts,-1680)
        self.assertEqual(l.writes,[(43,10),(79,(((-1680)&0xFFFF)<<16)|5)])
        self.assertIn('逆时针',note);self.assertIn('已接收',note);self.assertIn('1.56 圈',note)

    def test_requires_rev16_and_never_writes(self):
        l=Link();l.reg[0]=0x2070F
        with self.assertRaises(ValueError) as ctx:request_travel_move(l,560,1,30)
        self.assertIn('REV16',str(ctx.exception))
        self.assertEqual(l.writes,[])

    def test_preconditions_never_write(self):
        for a,v in [(16,4),(16,2),(31,6),(31,12),(31,4),
                    (31,4|(1<<21)|(1<<24)),(64,1)]:
            l=Link();l.reg[0]=0x20710;l.reg[a]=v
            with self.subTest(a=a,v=v),self.assertRaises(ValueError):request_travel_move(l,560,1,30)
            self.assertEqual(l.writes,[])

    def test_bad_angle_fails_before_touching_the_link(self):
        l=Link();l.reg[0]=0x20710
        with self.assertRaises(ValueError):request_travel_move(l,12000,1,30)
        self.assertEqual(l.writes,[])

    def test_reject_reasons_and_missing_ack_not_retried(self):
        for rejection in (1,2,3,4,5):
            l=Link();l.reg[0]=0x20710;l.reject=rejection
            with self.assertRaises(ValueError):request_travel_move(l,560,1,30)

            self.assertEqual(len(l.writes),2)
        l=Link();l.reg[0]=0x20710;l.no_ack=True
        with self.assertRaises(ValueError):request_travel_move(l,560,1,30)
        self.assertEqual(l.writes,[(43,10),(79,(1680<<16)|5)])

    def test_speed_readback_mismatch_aborts_before_the_command(self):
        l=Link();l.reg[0]=0x20710;l.stuck43=True
        with self.assertRaises(ValueError) as ctx:request_travel_move(l,560,1,30)
        self.assertIn('转速给定回读不一致',str(ctx.exception))
        self.assertEqual(l.writes,[(43,10)])

class TestLongMovePlan(unittest.TestCase):

    def test_limit_follows_the_firmware(self):
        self.assertEqual(move_limit_counts(0x00020712),8191)
        self.assertEqual(move_limit_counts(0x00020713),8191)
        self.assertEqual(move_limit_counts(0x00020711),520)
        self.assertEqual(move_limit_counts(0x0002070F),520)
        self.assertEqual(move_limit_counts(0x00020706),520)

    def test_540_degrees_is_one_command_on_rev18(self):

        self.assertEqual(plan_angle_move(1620,0x00020712),(8191,[1620]))
        self.assertEqual(plan_angle_move(-1620,0x00020712),(8191,[-1620]))

    def test_540_degrees_is_four_segments_on_rev17(self):
        self.assertEqual(plan_angle_move(1620,0x00020711),(520,[405,405,405,405]))

    def test_whole_turn_and_more_still_fits_one_leg(self):
        self.assertEqual(plan_angle_move(5200,0x00020712),(8191,[5200]))
        limit,segs=plan_angle_move(8191,0x00020712)
        self.assertEqual((limit,segs),(8191,[8191]))

        limit,segs=plan_angle_move(32760,0x00020712)
        self.assertEqual(limit,8191)
        self.assertEqual(sum(segs),32760)
        self.assertTrue(all(0<c<=8191 for c in segs),segs)
        self.assertEqual(len(segs),4)

    def test_old_firmware_segments_never_exceed_520(self):
        _,segs=plan_angle_move(32760,0x00020711)
        self.assertEqual(sum(segs),32760)
        self.assertTrue(all(0<c<=500 for c in segs),'老固件每段还不许贴 520 的边')

        _,segs=plan_angle_move(-1620,0x00020711)
        self.assertEqual(segs,[-405]*4)

    def test_zero_is_rejected(self):
        with self.assertRaises(ValueError):plan_angle_move(0,0x00020712)

    def test_command_four_carries_the_long_count_on_rev18(self):
        l=Link();l.reg[0]=0x00020712
        note,counts=request_angle_move_counts(l,1620)
        self.assertEqual(counts,1620)
        self.assertEqual(l.writes,[(79,(1620<<16)|4)])
        self.assertIn('540.0°',note)
        l=Link();l.reg[0]=0x00020712
        note,_=request_angle_move_counts(l,-8191)
        self.assertEqual(l.writes,[(79,((-8191)&0xFFFF)<<16|4)])

    def test_command_four_refuses_a_long_count_on_rev17(self):
        l=Link();l.reg[0]=0x00020711
        with self.assertRaises(ValueError) as ctx:request_angle_move_counts(l,1620)
        self.assertIn('±520',str(ctx.exception))
        self.assertEqual(l.writes,[])

        l=Link();l.reg[0]=0x00020712
        with self.assertRaises(ValueError):request_angle_move_counts(l,8192)
        self.assertEqual(l.writes,[])

class TestMoveDecel(unittest.TestCase):

    def test_decel_tracks_the_speed(self):
        self.assertEqual(move_decel_code(.2),30)
        self.assertEqual(move_decel_code(.5),75)
        self.assertEqual(move_decel_code(1.0),150)
        self.assertEqual(move_decel_code(.02),3)

    def test_brake_distance_matches_the_firmware_recursion(self):

        self.assertAlmostEqual(move_brake_counts(.2),45.5,places=1)
        self.assertAlmostEqual(move_brake_counts(.5),113.0,places=1)

        self.assertAlmostEqual(move_brake_counts(.4)/move_brake_counts(.2),2.0,places=1)

    def test_decel_zero_or_negative_is_never_used(self):

        for v in (.02,.1,.33,.77,1.2):
            with self.subTest(v=v):
                self.assertGreater(move_decel_code(v),0)

class TestMotionStatus(unittest.TestCase):
    def test_decode_address_75(self):

        s=decode_motion_status(((-1&0xFFFF)<<16)|(4<<8)|0)
        self.assertEqual((s['error'],s['count'],s['parked']),(-1,4,False))
        s=decode_motion_status((7<<16)|(255<<8)|1)
        self.assertEqual((s['error'],s['count'],s['parked']),(7,255,True))

        s=decode_motion_status((0<<16)|(0<<8)|2)
        self.assertTrue(s['aborted']);self.assertFalse(s['parked'])
        s=decode_motion_status(3)
        self.assertTrue(s['aborted'] and s['parked'])

class TestSpin(unittest.TestCase):

    def test_rpm_quantization(self):
        self.assertEqual(spin_speed_code(30,1),10)
        self.assertEqual(spin_speed_code(30,-1),-10)
        self.assertAlmostEqual(spin_speed_rpm(10),31.111,places=3)
        self.assertAlmostEqual(spin_speed_rpm(1),3.111,places=3)
        self.assertEqual(spin_speed_code(3.3,1),1)
        self.assertEqual(spin_speed_code(199,1),64)
        for bad in (1,0,210,-30):
            with self.subTest(bad=bad),self.assertRaises(ValueError):spin_speed_code(bad,1)
        with self.assertRaises(ValueError):spin_speed_code(30,0)

    def test_start_from_mode_three_zeroes_the_old_setpoint_first(self):

        l=Link();l.reg[0]=0x2070F
        note=request_spin_start(l,30,1)
        self.assertEqual(l.writes,[(43,0),(16,4),(43,10)])
        self.assertIn('顺时针',note);self.assertIn('31.1 rpm',note)

    def test_start_in_mode_four_only_changes_speed(self):
        l=Link();l.reg[0]=0x2070F;l.reg[16]=4
        request_spin_start(l,60,-1)
        self.assertEqual(l.writes,[(43,((-19)&0xFFFFFFFF))])

    def test_start_preconditions_never_write(self):
        for a,v in [(0,0x2070E),(16,2),(31,6),(31,12),(31,4),
                    (31,4|(1<<21)|(1<<24)),(64,1)]:
            l=Link();l.reg[0]=0x2070F;l.reg[a]=v
            with self.subTest(a=a,v=v),self.assertRaises(ValueError):request_spin_start(l,30,1)
            self.assertEqual(l.writes,[])

    def test_pause_zeroes_the_setpoint_and_reports_whether_it_was_spinning(self):
        l=Link();l.reg[0]=0x2070F;l.reg[16]=4;l.reg[43]=13
        note,was=request_spin_pause(l)
        self.assertTrue(was)
        self.assertEqual(l.writes,[(43,0)])
        self.assertIn('停靠',note)
        l=Link();l.reg[0]=0x2070F;l.reg[16]=4;l.reg[43]=0
        _,was=request_spin_pause(l)
        self.assertFalse(was)

    def test_pause_outside_mode_four_never_writes(self):
        l=Link();l.reg[0]=0x2070F;l.reg[16]=3;l.reg[43]=13
        with self.assertRaises(ValueError):request_spin_pause(l)
        self.assertEqual(l.writes,[])

    def test_pause_refuses_when_the_zero_write_does_not_take(self):
        l=Link();l.reg[0]=0x2070F;l.reg[16]=4;l.reg[43]=13;l.stuck43=True
        with self.assertRaises(ValueError) as ctx:request_spin_pause(l)
        self.assertIn('仍在转',str(ctx.exception))
        self.assertEqual(l.writes,[(43,0)])
