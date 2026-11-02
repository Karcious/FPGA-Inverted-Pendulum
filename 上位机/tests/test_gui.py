import os
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
import time
import unittest
from unittest.mock import patch
from PySide6.QtWidgets import QApplication
from gui_model import encode,decode,channels,DemoLink,LABELS
from gui_motion import decode_motion_status, travel_move_counts
from dbg_host import N_PARAM,P_COMP_SHIFT,P_TRAVEL_FOLLOW
from fpga_console import Worker,Window

class TestModel(unittest.TestCase):
    def test_labels_cover_every_address(self):

        self.assertEqual(len(LABELS),N_PARAM)

        self.assertEqual(P_TRAVEL_FOLLOW,N_PARAM-1)
    def test_comp_shape_range(self):
        self.assertEqual(encode(P_COMP_SHIFT,'0'),0)
        self.assertEqual(encode(P_COMP_SHIFT,'3'),3)
        for bad in ('-1','4','1.5'):
            with self.subTest(value=bad):
                with self.assertRaises(ValueError): encode(P_COMP_SHIFT,bad)
    def test_fixed_point_and_signed(self):
        self.assertAlmostEqual(decode(5,encode(5,'-1.2')),-1.2,places=4)
        self.assertEqual(decode(24,encode(24,'-600')),-600)
        self.assertEqual(encode(9,'15'),15)
    def test_reject_invalid(self):
        for addr,value in ((0,'0'),(4,'nan'),(4,'inf'),(21,'5'),(9,'0.25'),(11,'0'),(17,'1025')):
            with self.subTest(addr=addr,value=value):
                with self.assertRaises(ValueError): encode(addr,value)
    def test_angle_scale(self):
        raw=DemoLink().raw
        a,p,d=channels([[raw[18]+1024,0,1040,-100]],raw)
        self.assertEqual((a,p,d),([90.],[360.],[-100]))

class TestQt(unittest.TestCase):
    @classmethod
    def setUpClass(cls): cls.app=QApplication.instance() or QApplication([])
    def wait_for(self,predicate,timeout=3):
        end=time.monotonic()+timeout
        while not predicate() and time.monotonic()<end:
            self.app.processEvents(); time.sleep(.01)
        self.assertTrue(predicate())
    def test_worker_demo_write_capture_stop(self):
        worker=Worker(); events=[]
        worker.event.connect(lambda kind,data: events.append((kind,data)))
        worker.start()
        try:
            worker.submit('connect','DEMO')
            self.wait_for(lambda:any(k=='connection' and d=='DEMO' for k,d in events))
            worker.submit('set',(4,encode(4,'190')))
            self.wait_for(lambda:any(k=='written' and d==4 for k,d in events))
            self.assertEqual(decode(4,worker.params[4]),190)
            worker.submit('monitor',True)
            self.wait_for(lambda:len([1 for k,d in events if k=='live'])>=5)
            worker.submit('acquisition',False)
            self.wait_for(lambda:any(k=='frame' for k,d in events),7)
            frame=next(d for k,d in events if k=='frame')
            self.assertEqual(len(frame['rows']),4096)
            worker.submit('stop')
            self.wait_for(lambda:any(k=='written' and d==16 for k,d in events))
            self.assertEqual(worker.params[16],0)
        finally:
            worker.submit('quit'); worker.wait(5000)
    def test_edit_survives_refresh_and_cancel(self):
        window=Window()
        try:
            raw=DemoLink().raw
            window.fill_params(raw)
            window.table.item(4,3).setText('210')
            window.fill_params(raw)
            self.assertEqual(window.table.item(4,3).text(),'210')
            window.table.setCurrentCell(4,3)
            window.revert_selected()
            self.assertEqual(window.table.item(4,3).text(),'120')
        finally:
            window.worker.submit('quit'); window.worker.wait(5000)
            window.closing=True; window.close()
    def test_hardware_mode_change_updates_curve_button(self):
        window=Window()
        try:
            raw=DemoLink().raw;raw[0]=0x20702;raw[16]=0
            window.connected=True;window.demo=False;window.fill_params(raw)
            self.assertFalse(window.trajectory_start_button.isEnabled())
            window.on_event('status',(3,0))
            self.assertTrue(window.trajectory_start_button.isEnabled())
            window.on_event('status',(0,0))
            self.assertFalse(window.trajectory_start_button.isEnabled())
        finally:
            window.worker.submit('quit');window.worker.wait(5000)
            window.closing=True;window.close()
    def test_templates_sliders_and_progress_marker(self):
        window=Window()
        try:
            window.trajectory_template.setCurrentIndex(1)
            window.amplitude_slider.setValue(45);window.period_slider.setValue(120)
            self.wait_for(lambda:len(window.trajectory_vplot.values)>0)
            c=window.preview_trajectory()
            self.assertEqual(c['seconds'],12);self.assertIn('45*sin(2*pi*t/T)',c['expression'])
            self.assertAlmostEqual(c['positions'][-1],0,places=2)
            window.running_curve=c
            window.on_event('trajectory_progress',dict(elapsed=3,target=45,actual=41,cycles=2,status=1))
            self.assertEqual(window.trajectory_vplot.marker,(3,45,41))
            self.assertIn('第 3 周期',window.trajectory_progress_label.text())
            window.trajectory_continuous.setChecked(True)
            self.assertTrue(window.preview_trajectory()['continuous'])
        finally:
            window.worker.submit('quit');window.worker.wait(5000);window.closing=True;window.close()
    def test_wave_formula_box_follows_the_shape_and_only_custom_is_editable(self):

        window=Window()
        try:
            pick=lambda key: window.wave_shape.setCurrentIndex(
                window.wave_shape.findData(key))
            pick('sine')
            self.assertTrue(window.wave_expression.isReadOnly())
            self.assertIn('sin',window.wave_expression.text())
            pick('cosine')
            self.assertTrue(window.wave_expression.isReadOnly())
            self.assertIn('cos',window.wave_expression.text())
            pick('custom')
            self.assertFalse(window.wave_expression.isReadOnly())
            self.assertEqual(window.wave_expression.text(),'cos(2*pi*t/T + pi/2)')

            window.wave_expression.setText('90·cos(2π·t/8 + π/2) °')
            window.wave_expression_edited()
            window.refresh_wave_preview()
            self.assertIn('90*cos(2*pi*t/T + pi/2)',window.wave_preview.text())

            pick('sine'); pick('custom')
            self.assertEqual(window.wave_expression.text(),'90·cos(2π·t/8 + π/2) °')

            pick('triangle'); pick('custom')
            self.assertTrue(window.wave_expression.text()!='90·tri(2π·t/8) °（尖角磨圆）')
        finally:
            window.worker.submit('quit');window.worker.wait(5000)
            window.closing=True;window.close()
    def test_stop_cancels_pending_motion(self):
        worker=Worker()
        worker.submit('set',(16,3))
        worker.submit('set',(17,500))
        worker.submit('key_move',1)
        worker.submit('quit')
        worker.discard_writes()
        entries=[]
        while not worker.commands.empty(): entries.append(worker.commands.get()[2])
        self.assertEqual(entries,['quit'])

    def test_remote_key_buttons_and_command_mapping(self):
        window=Window()
        try:
            raw=DemoLink().raw;raw[0]=0x20706;raw[16]=3;raw[31]=4|(1<<21)
            window.connected=True;window.demo=False;window.fill_params(raw)
            self.assertTrue(window.move_left_button.isEnabled())
            with patch.object(window.worker,'submit') as send:
                window.move_left_button.click();send.assert_called_with('key_move',-1)
                window.move_right_button.click();send.assert_called_with('key_move',1)
            window.on_event('status',(3,6));self.assertFalse(window.move_left_button.isEnabled())
            window.on_event('status',(3,4));self.assertFalse(window.move_right_button.isEnabled())
            window.on_event('status',(3,4|(1<<21)|(1<<24)));self.assertFalse(window.move_right_button.isEnabled())
            self.assertIn('回零故障',window.motion_note.text())
            window.on_event('status',(3,4|(1<<21)));self.assertTrue(window.move_right_button.isEnabled())
            window.on_event('trajectory_active',True);self.assertFalse(window.move_right_button.isEnabled())
            window.on_event('trajectory_active',False)
            raw[0]=0x20705;window.fill_params(raw);self.assertFalse(window.move_left_button.isEnabled())
            window.on_event('connection',False);self.assertFalse(window.move_right_button.isEnabled())
        finally:
            window.worker.submit('quit');window.worker.wait(5000);window.closing=True;window.close()

    def test_fuzzy_switch_touches_only_bit8(self):

        worker=Worker(); events=[]
        worker.event.connect(lambda kind,data: events.append((kind,data)))
        worker.start()
        try:
            worker.submit('connect','DEMO')
            self.wait_for(lambda:any(k=='connection' for k,d in events))

            before=worker.params[9]
            low=before&~0x100
            worker.submit('fuzzy',True)
            self.wait_for(lambda:any(k=='log' and '模糊 PID 已打开' in d for k,d in events))
            self.assertEqual(worker.params[9],low|0x100)
            worker.submit('fuzzy',False)
            self.wait_for(lambda:any(k=='log' and '模糊 PID 已关闭' in d for k,d in events))
            self.assertEqual(worker.params[9],low)
        finally:
            worker.submit('quit'); worker.wait(5000)
    def test_worker_polls_effective_gains_while_monitoring(self):

        worker=Worker(); events=[]
        worker.event.connect(lambda kind,data: events.append((kind,data)))
        worker.start()
        try:
            worker.submit('connect','DEMO')
            self.wait_for(lambda:any(k=='connection' for k,d in events))
            worker.submit('monitor',True)
            self.wait_for(lambda:any(k=='pid' for k,d in events))
            sample=next(d for k,d in events if k=='pid')
            self.assertAlmostEqual(sample['kp'],4.5,places=3)
            self.assertAlmostEqual(sample['kd'],120.0,places=3)
            self.assertIn('received',sample)
        finally:
            worker.submit('quit'); worker.wait(5000)
    def test_zero_button_needs_rev14_and_sends_zero_command(self):

        window=Window()
        try:
            raw=DemoLink().raw
            raw[16]=3;raw[31]=4|(1<<21)
            window.connected=True;window.demo=False
            raw[0]=0x02070D;window.fill_params(raw)
            self.assertFalse(window.zero_button.isEnabled())
            raw[0]=0x02070E;window.fill_params(raw)
            self.assertTrue(window.zero_button.isEnabled())
            with patch.object(window.worker,'submit') as send:
                window.zero_button.click();send.assert_called_with('zero_cal')
                window.on_event('trajectory_active',True)
                self.assertFalse(window.zero_button.isEnabled())
        finally:
            window.worker.submit('quit');window.worker.wait(5000);window.closing=True;window.close()
    def test_comp_shape_knob_maps_to_address_50(self):
        window=Window()
        try:
            raw=DemoLink().raw;window.fill_params(raw)
            self.assertEqual(window.comp_shape.currentData(),1)
            window.connected=True;window.demo=False
            with patch.object(window.worker,'submit') as send:
                window.comp_shape.setCurrentIndex(2)
                send.assert_called_with('set',(50,2))
            raw[50]=3;window.fill_params(raw)
            self.assertEqual(window.comp_shape.currentData(),3)
        finally:
            window.worker.submit('quit');window.worker.wait(5000);window.closing=True;window.close()
    def live(self, window, raw, tick, adc, pos=0, status=0x84, mode=3):

        params=list(raw); params[16]=mode
        window.on_event('live',{'row':[adc,0,pos,100],'sequence':tick,
                                'status':status,'params':params,
                                'received':time.time()})

    def test_error_cards_are_relative_to_their_targets(self):

        window=Window()
        try:
            raw=DemoLink().raw; raw[18]=2048; raw[13]=0
            window.fill_params(raw)
            self.live(window,raw,1000,3072,260)
            self.assertAlmostEqual(float(window.angle.text().split()[0]),90.0,places=3)
            self.assertAlmostEqual(float(window.arm.text().split()[0]),90.0,places=3)
            self.assertIn('地址18',window.angle.note_label.text())
            self.assertIn('地址13',window.arm.note_label.text())

            raw[13]=520*65536
            window.fill_params(raw)
            self.live(window,raw,1020,3072,260)
            self.assertAlmostEqual(float(window.arm.text().split()[0]),-90.0,places=3)
        finally:
            window.worker.submit('quit');window.worker.wait(5000);window.closing=True;window.close()

    def test_average_window_is_free_input_and_optional(self):
        window=Window()
        try:
            raw=DemoLink().raw; raw[18]=2048; raw[13]=0
            window.fill_params(raw)
            self.assertEqual((window.avg_window.minimum(),window.avg_window.maximum()),(.2,60))
            self.assertAlmostEqual(window.avg_window.value(),5.0)
            for i,tick in enumerate((1000,1020,1040)):
                self.live(window,raw,tick,3072,0)
            self.assertAlmostEqual(float(window.angle_avg.text().split()[0]),90.0,places=3)
            self.assertIn('遥测',window.angle_avg.note_label.text())
            window.avg_window.setValue(0.2)
            self.assertIn('最近 0.04 s / 3 点',window.angle_avg.note_label.text())
            window.avg_enable.setChecked(False)
            self.assertEqual(window.angle_avg.text(),'— °')
            self.assertIn('未启用',window.angle_avg.note_label.text())
            window.avg_enable.setChecked(True)
            self.assertAlmostEqual(float(window.angle_avg.text().split()[0]),90.0,places=3)
        finally:
            window.worker.submit('quit');window.worker.wait(5000);window.closing=True;window.close()

    def test_stability_needs_balance_and_stays_inside_the_band(self):
        window=Window()
        try:
            raw=DemoLink().raw; raw[18]=2048
            window.fill_params(raw)

            for i in range(10):
                self.live(window,raw,1000+i*20,2058,mode=2)
            self.assertAlmostEqual(window.stats.longest,0.0,places=2)

            for i in range(10,40):
                self.live(window,raw,1000+i*20,2058)
            self.assertAlmostEqual(window.stats.longest,.58,places=2)

            self.live(window,raw,1000+40*20,2048+300)
            self.assertAlmostEqual(window.stats.current,0.0,places=2)
            self.assertAlmostEqual(window.stats.longest,.58,places=2)
            window.clear_statistics()
            self.assertAlmostEqual(window.stats.longest,0.0,places=2)
        finally:
            window.worker.submit('quit');window.worker.wait(5000);window.closing=True;window.close()

    def test_steady_band_centres_on_the_lean_not_the_upright(self):

        window=Window()
        try:
            raw=DemoLink().raw; raw[18]=2048
            window.fill_params(raw)
            for i in range(21):
                self.live(window,raw,2000+i*20,2048+28)
            self.assertAlmostEqual(window.stats.longest,.40,places=2)
            self.assertAlmostEqual(window.stats.center,2.46,places=1)

            self.assertIn('没有起点',window.delay.note_label.text())
        finally:
            window.worker.submit('quit');window.worker.wait(5000);window.closing=True;window.close()

    def test_pid_card_shows_set_and_effective_gains(self):
        window=Window()
        try:
            raw=DemoLink().raw
            window.fill_params(raw)
            self.assertIn('设定 Kp 4.500',window.pid_set.text())
            window.on_event('pid',{'kp':3.6,'kd':110.0,'p_scale':1.06,'d_scale':1.12,
                                   'enabled':True,'received':time.monotonic()})
            self.assertIn('生效 Kp 3.600',window.pid_eff.text())
            self.assertIn('模糊调度开启',window.pid_badge.text())
            window.on_event('pid',{'kp':3.6,'kd':110.0,'p_scale':1.0,'d_scale':1.0,
                                   'enabled':False,'received':time.monotonic()})
            self.assertIn('固定 PD',window.pid_badge.text())
            window.on_event('pid_unavailable',None)
            self.assertIn('不可用',window.pid_eff.text())
        finally:
            window.worker.submit('quit');window.worker.wait(5000);window.closing=True;window.close()

    def test_motion_page_is_renamed_and_wired_to_rev16(self):

        window=Window()
        try:
            names=[window.tabs.tabText(i) for i in range(window.tabs.count())]
            self.assertIn('参数调试',names)
            self.assertNotIn('新版参数',names)
            self.assertIn('走位与旋转',names)
            raw=DemoLink().raw;window.connected=True;window.demo=False
            raw[16]=3;raw[31]=4|(1<<21)
            raw[0]=0x02070E;window.fill_params(raw)
            self.assertFalse(window.move_cw_button.isEnabled())
            self.assertFalse(window.spin_cw_button.isEnabled())
            raw[0]=0x02070F;window.fill_params(raw)
            self.assertTrue(window.move_cw_button.isEnabled())
            self.assertTrue(window.move_ccw_button.isEnabled())
            self.assertTrue(window.spin_cw_button.isEnabled())
            self.assertFalse(window.spin_pause_button.isEnabled())
            with patch.object(window.worker,'submit') as send:
                window.move_angle.setValue(30)

                window.move_cw_button.click();send.assert_called_with('angle_move_counts',(90,False))
                window.move_ccw_button.click();send.assert_called_with('angle_move_counts',(-90,False))
                window.spin_rpm.setValue(30)
                window.spin_cw_button.click();send.assert_called_with('spin_start',(30,1))

            raw[0]=0x020710;window.fill_params(raw)
            window.move_angle.setValue(560)
            self.assertIn('位置环分段（命令 4）',window.move_preview.text())
            with patch.object(window.worker,'submit') as send:
                window.move_cw_button.click()

                send.assert_called_with('angle_move_counts',(420,False))

            self.assertTrue(window.move_queue)
            with patch.object(window.worker,'submit') as send:
                window.move_ccw_button.click();send.assert_not_called()

            window.move_queue=[];window.move_chain_total=0
            with patch.object(window.worker,'submit') as send:
                window.move_ccw_button.click()
                send.assert_called_with('angle_move_counts',(-420,False))
            window.move_queue=[];window.move_chain_total=0
            window.move_angle.setValue(173)
            with patch.object(window.worker,'submit') as send:
                window.move_cw_button.click()

                send.assert_called_with('angle_move_counts',(519,False))
            self.assertIn('命令 4',window.move_preview.text())

            raw[0]=0x02070F;window.fill_params(raw)
            window.move_angle.setValue(560)
            self.assertIn('位置环分段（命令 4）',window.move_preview.text())
            self.assertNotIn('不支持',window.move_preview.text())

            window.on_event('status',(4,4|(1<<21)))
            self.assertFalse(window.move_cw_button.isEnabled())
            self.assertTrue(window.spin_pause_button.isEnabled())
            self.assertTrue(window.spin_cw_button.isEnabled())
            window.on_event('status',(3,4|(1<<21)))
        finally:
            window.worker.submit('quit');window.worker.wait(5000);window.closing=True;window.close()

    def test_angle_move_result_comes_from_address_75(self):
        window=Window()
        try:
            raw=DemoLink().raw;raw[0]=0x02070F;raw[16]=3;raw[31]=4|(1<<21)
            window.connected=True;window.demo=False;window.fill_params(raw)
            window.on_event('motion_status',decode_motion_status(5<<8))
            window.on_event('angle_move_sent',90)
            self.assertIn('走位中',window.move_result.text())

            window.on_event('motion_status',decode_motion_status(((-1&0xFFFF)<<16)|(6<<8)))
            text=window.move_result.text()
            self.assertIn('+30.0',text)
            self.assertIn('+29.7',text)
            self.assertIn('-0.3',text)
            self.assertIn('零点已更新',text)
            self.assertIsNone(window.move_pending)

            window.on_event('angle_move_sent',90)
            with patch.object(window.worker,'submit') as send:
                window.move_cw_button.click()
                send.assert_not_called()
            self.assertIn('还没到点',window.move_result.text())
        finally:
            window.worker.submit('quit');window.worker.wait(5000);window.closing=True;window.close()

    def test_live_readouts_show_travelled_degrees_and_measured_speed(self):

        window=Window()
        try:
            raw=DemoLink().raw;raw[0]=0x020710;raw[16]=3;raw[31]=4|(1<<21)
            window.connected=True;window.demo=False;window.fill_params(raw)
            window.on_event('motion_status',decode_motion_status(5<<8))
            window.on_event('angle_move_sent',1620)

            window.on_event('motion_status',dict(
                decode_motion_status(((-720&0xFFFF)<<16)|(5<<8)),vel=59.0))
            text=window.move_live.text()
            self.assertIn('+300.0',text)
            self.assertIn('+540.0',text)
            self.assertIn('还差',text)

            raw=list(raw); raw[16]=4; raw[31]=4|(1<<21); raw[43]=30
            window.fill_params(raw)
            window.spin_w_prev=0; window.spin_angle=0.0
            with patch('fpga_console.time.monotonic',side_effect=[100.0,100.5]):
                window.on_event('motion_status',dict(decode_motion_status(0),vel=30.0))
                window.on_event('motion_status',dict(decode_motion_status(0),vel=30.0))
            text=window.spin_live.text()
            self.assertIn('+200.0',text)
            self.assertIn('66.7',text)
            self.assertIn('顺时针',text)
        finally:
            window.worker.submit('quit');window.worker.wait(5000);window.closing=True;window.close()

    def test_multi_turn_result_reports_a_mid_way_abort(self):

        window=Window()
        try:
            raw=DemoLink().raw;raw[0]=0x020710;raw[16]=3;raw[31]=4|(1<<21)
            window.connected=True;window.demo=False;window.fill_params(raw)
            window.on_event('motion_status',decode_motion_status(5<<8))
            window.on_event('angle_move_sent',1620)
            window.on_event('motion_status',
                            decode_motion_status(((-40&0xFFFF)<<16)|(6<<8)|2))
            text=window.move_result.text()
            self.assertIn('+540.0',text)
            self.assertIn('-13.3',text)
            self.assertIn('杆歪过',text)
        finally:
            window.worker.submit('quit');window.worker.wait(5000);window.closing=True;window.close()

    def test_spin_pause_waits_for_park_then_switches_mode(self):
        window=Window()
        try:
            raw=DemoLink().raw;raw[0]=0x02070F;raw[16]=4;raw[31]=4|(1<<21)
            window.connected=True;window.demo=False;window.fill_params(raw)
            window.pause_switch.setChecked(True)
            window.on_event('motion_status',decode_motion_status(0))
            with patch.object(window.worker,'submit') as send:
                window.on_event('spin_pause_sent',(True,True))
                self.assertIsNotNone(window.pause_waiting)
                self.assertIn('减速',window.spin_state.text())

                send.assert_not_called()
                window.on_event('motion_status',decode_motion_status(1))
                send.assert_called_with('set',(16,3))
            self.assertIsNone(window.pause_waiting)
            self.assertIn('已停靠',window.spin_state.text())

            window.pause_switch.setChecked(False)
            window.on_event('motion_status',decode_motion_status(0))
            with patch.object(window.worker,'submit') as send:
                window.on_event('spin_pause_sent',(False,False))
                send.assert_not_called()
            self.assertIsNone(window.pause_waiting)
        finally:
            window.worker.submit('quit');window.worker.wait(5000);window.closing=True;window.close()

    def test_second_pause_click_must_not_switch_mode_without_a_park(self):

        window=Window()
        try:
            raw=DemoLink().raw;raw[0]=0x020710;raw[16]=4;raw[31]=4|(1<<21)
            window.connected=True;window.demo=False;window.fill_params(raw)
            window.pause_switch.setChecked(True)
            window.on_event('motion_status',decode_motion_status(0))
            with patch.object(window.worker,'submit') as send:
                window.on_event('spin_started',None)
                self.assertTrue(window.mode4_ever_spun)
                window.on_event('spin_pause_sent',(False,True))
                send.assert_not_called()
                self.assertIsNotNone(window.pause_waiting)
                self.assertIn('旧零点',window.spin_note.text())

                window.on_event('motion_status',decode_motion_status(1))
                send.assert_called_with('set',(16,3))
            self.assertIsNone(window.pause_waiting)

            window.on_event('status',(3,4))
            self.assertFalse(window.mode4_ever_spun)
        finally:
            window.worker.submit('quit');window.worker.wait(5000);window.closing=True;window.close()

    def test_offline_demo_can_press_the_motion_buttons(self):

        window=Window()
        try:
            link=DemoLink()
            window.demo=True;window.connected=True
            window.fill_params(link.dump_params())
            for button in (window.move_cw_button,window.move_ccw_button,
                           window.move_left_button,window.move_right_button,
                           window.zero_button,window.spin_cw_button,
                           window.spin_ccw_button):
                self.assertTrue(button.isEnabled(),button.text())
            with patch.object(window.worker,'submit') as send:
                window.move_cw_button.click()
            send.assert_called_with('angle_move_counts',
                                    (travel_move_counts(window.move_angle.value(),1),False))

            link.set_param(79,((87&0xFFFF)<<16)|4)
            reply=link.get_param(79)
            self.assertTrue(reply&0x80)
            self.assertEqual(reply&7,4)
            link.move_due=0
            word=decode_motion_status(link.get_param(75))
            self.assertEqual(word['count'],1)
            self.assertTrue(word['parked'])
        finally:
            window.worker.submit('quit');window.worker.wait(5000);window.closing=True;window.close()

    def test_move_speed_also_writes_the_matching_deceleration(self):

        window=Window()
        try:
            raw=DemoLink().raw;raw[0]=0x020712;raw[16]=3;raw[31]=4|(1<<21)
            window.connected=True;window.demo=False;window.fill_params(raw)
            with patch.object(window.worker,'submit') as send:
                window.move_slew.setValue(.2)
                window.write_move_slew()
                window.move_slew.setValue(.5)
                window.write_move_slew()
            calls=[c.args for c in send.call_args_list]
            self.assertEqual(calls,[('set',(40,13107)),('set',(49,30)),
                                    ('set',(40,32768)),('set',(49,75))])
            self.assertIn('113',window.move_slew_note.text())
        finally:
            window.worker.submit('quit');window.worker.wait(5000);window.closing=True;window.close()

    def test_no_params_does_not_raise(self):

        window=Window()
        try:
            self.assertIsNone(window.params)
            window.update_trajectory_controls()
            window.on_event('status',(3,0))
            window.on_event('connection',False)
        finally:
            window.worker.submit('quit');window.worker.wait(5000);window.closing=True;window.close()

if __name__=='__main__': unittest.main()
