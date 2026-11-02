import unittest,time
from types import SimpleNamespace
import gui_features as f
class Link:
 def __init__(self):self.p={0:f.MAGIC,16:3,31:4,64:0,80:0x544f4631,83:0,86:0,87:0,91:0};self.w=[]
 def get_param(self,a):return self.p.get(a,0)
 def set_param(self,a,v):self.w.append((a,v));self.p[a]=v
class Tests(unittest.TestCase):
 def worker(self):
  events=[];w=SimpleNamespace(link=Link(),params=[f.MAGIC],wave=SimpleNamespace(active=False),trajectory=SimpleNamespace(active=False),event=SimpleNamespace(emit=lambda *x:events.append(x)))
  f.init(w);w.events=events;w.tof.put(dict(left=40,right=0,flags=1,present=1));return w
 def test_start_only_extension_registers(self):
  w=self.worker();self.assertTrue(f.command(w,'avoid_config',1));self.assertTrue(w.tof_forward);self.assertEqual(w.link.p[86],1)
  self.assertEqual([a for a,v in w.link.w],[81,82,84,83,86])
 def test_busy_rejected(self):
  w=self.worker();w.wave.active=True;f.command(w,'avoid_config',1);self.assertEqual(w.link.w,[])
 def test_old_version_no_poll_or_guard_io(self):
  w=self.worker();w.params=[0x020712];w.link.get_param=lambda a:(_ for _ in ()).throw(AssertionError('unexpected IO'))
  f.poll(w,100);self.assertFalse(f.guard(w,'key_move'))
 def test_motion_conflict_and_stop(self):
  w=self.worker();w.link.p[86]=1;self.assertTrue(f.guard(w,'spin_start'));self.assertFalse(f.guard(w,'stop'));self.assertEqual(w.link.p[86],0)
 def test_stale_rejected(self):
  w=self.worker();w.tof.clear();f.command(w,'avoid_config',1);self.assertEqual(w.link.w,[])
 def test_no_target_still_arms(self):

  w=self.worker();w.tof.put(dict(left=2400,right=0,flags=0,present=1))
  self.assertTrue(f.command(w,'avoid_config',1));self.assertEqual(w.link.p[86],1)
 def test_absent_sensor_rejected(self):
  w=self.worker();w.tof.put(dict(left=0,right=0,flags=0,present=0))
  f.command(w,'avoid_config',1);self.assertEqual(w.link.w,[])
 def test_right_side_uses_second_sensor(self):

  w=self.worker();w.tof.put(dict(left=0,right=45,flags=2,present=2))
  self.assertTrue(f.command(w,'avoid_config',('right',1)))
  self.assertEqual(w.tof_side,'right');self.assertEqual(w.link.p[81],45);self.assertEqual(w.link.p[84],1)
  self.assertEqual(w.link.p[86],1)
 def test_fault_code_is_shown_not_just_the_word(self):

  w=self.worker();w.link.p[64]=5<<8
  f.poll(w,time.monotonic()+1)
  note=[e for e in w.events if e[0]=='avoid_note'][-1][1]
  self.assertIn('故障码 5',note);self.assertIn('刹车',note)
 def test_clear_fault_releases_arm(self):
  w=self.worker();w.link.p[64]=5<<8
  self.assertTrue(f.command(w,'clear_fault',None));self.assertEqual(w.link.p[64],0)
 def test_clear_fault_refused_while_running(self):
  w=self.worker();w.link.p[64]=(5<<8)|1
  f.command(w,'clear_fault',None);self.assertEqual(w.link.p[64],(5<<8)|1)
 def test_clear_fault_without_fault(self):
  w=self.worker();f.command(w,'clear_fault',None);self.assertEqual(w.link.w,[])
 def test_side_survives_into_poll(self):
  w=self.worker();w.tof.put(dict(left=0,right=45,flags=2,present=2))
  f.command(w,'avoid_config',('right',1));w.link.w=[]
  w.tof.put(dict(left=0,right=45,flags=2,present=2,device_seq=1))
  f.poll(w,time.monotonic()+1);self.assertIn((81,45),w.link.w)
