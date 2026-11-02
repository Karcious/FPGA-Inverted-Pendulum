import unittest
from gui_trajectory import compile_curve,TrajectorySession,FACTOR
class Link:
 def __init__(self):self.p={0:0x20702,16:3,31:(1<<21)|(1<<18),64:0,67:0,69:0,70:0,78:1}
 def get_param(self,a):return self.p.get(a,0)
 def set_param(self,a,v):
  if a==64:
   if v==0:self.p.update({64:self.p[64]&(16|33),67:0,70:0})
   elif v==3:self.p[64]=33
   elif v==1:self.p[64]=97 if self.p.get(78)==0 else 1
   elif v==2:self.p[64]=9
  elif a==67:assert v==0x20702
  elif a==68:self.p[70]+=1;self.p[67]=(self.p[67]+v)&0xffffffff
  else:self.p[a]=v
class Tests(unittest.TestCase):
 def test_units_and_multiple_turns(self):
  c=compile_curve('30*sin(pi*t/T)**2',24)
  self.assertAlmostEqual(c['positions'][-1],360,places=1)
  self.assertLessEqual(len(c['words']),1024)
  self.assertAlmostEqual(FACTOR,1040*65536/360/1000)
  self.assertAlmostEqual(compile_curve('30*sin(pi*t/T)**2',24,-1)['positions'][-1],-360,places=1)
 def test_above_sixty_allowed_in_both_directions(self):
  for peak in (61,120):
   for direction in (1,-1):
    c=compile_curve(f'{peak}*sin(pi*t/T)**2',30,direction)
    self.assertAlmostEqual(c['positions'][-1],direction*peak*15,delta=.1)
 def test_finite_error_and_firmware_range_are_distinct(self):
  with self.assertRaisesRegex(ValueError,'有限实数'):
   compile_curve('(-1)**0.5',30)
  with self.assertRaisesRegex(ValueError,'固件数据范围'):
   compile_curve('900*sin(pi*t/T)**2',30)
 def test_bad_inputs(self):
  for e in ['__import__("os")','t.__class__','1/(t-t)','900*sin(pi*t/T)**2','100','sin(10000*t)']:
   with self.subTest(e=e),self.assertRaises(ValueError):compile_curve(e,8)
 def test_display_formula_paste_is_explained(self):

  with self.assertRaises(ValueError) as ctx:
   compile_curve('90·cos(2π·t/8 + π/2) °',8)
  msg=str(ctx.exception)
  self.assertIn('*',msg);self.assertIn('pi',msg)
  self.assertIn('90*cos(2*pi*t/T + pi/2)',msg)
 def test_upload_pause_brake_hold(self):
  l=Link();s=TrajectorySession();s.begin(l,compile_curve('30*sin(pi*t/T)**2',24))
  while s.uploading:s.poll(l)
  self.assertEqual(l.p[64],1);l.p[64]=3;self.assertIn('暂停',s.poll(l))
  s.stop(l);self.assertTrue(s.active);self.assertIn('停车',s.poll(l))
  l.p[64]=20;self.assertIn('完成',s.poll(l));self.assertFalse(s.active)
  l.p[31]=0;s.begin(l,compile_curve('0',1));self.assertTrue(s.uploading)
 def test_live_update_does_not_reupload(self):
  l=Link();l.p[0]=0x20703;s=TrajectorySession();c=compile_curve('30*sin(pi*t/T)**2',8);c['continuous']=True
  s.begin(l,c)
  while s.uploading:s.poll(l)
  count=l.p[70];checksum=l.p[67];self.assertEqual(l.p[72],3)
  s.update(l,.5,2)
  self.assertEqual((l.p[73],l.p[74]),(32768,131072));self.assertEqual((count,checksum),(l.p[70],l.p[67]))
  with self.assertRaises(ValueError):s.update(l,9,1)
  l.p[64]=9
  with self.assertRaises(ValueError):s.update(l,1,1)
 def test_continuous_needs_revision3(self):
  c=compile_curve('0',1);c['continuous']=True
  with self.assertRaisesRegex(ValueError,'020703'):TrajectorySession().begin(Link(),c)
 def test_takeover_does_not_require_old_key_target_reached(self):
  l=Link();l.p.update({0:0x20704,31:4|(1<<25),78:0})
  s=TrajectorySession();s.begin(l,compile_curve('0',.1))
  self.assertEqual(l.p[64]&33,33)
  while s.uploading:s.poll(l)
  self.assertIn('等待',s.poll(l));self.assertTrue(s.active)
  s.stop(l);self.assertEqual(l.p[64],9)
 def test_reconnect_incomplete_upload_does_not_claim_auto_start(self):
  l=Link();l.p.update({0:0x20704,64:33});s=TrajectorySession();s.revision=4;s.active=True
  self.assertIn('尚未提交启动',s.poll(l))
 def test_takeover_requires_confirmed_balance(self):
  for status in (3,4|8,4|16):
   l=Link();l.p.update({0:0x20704,31:status})
   with self.assertRaises(ValueError):TrajectorySession().begin(l,compile_curve('0',.1))
   self.assertEqual(l.p[64],0)
 def test_legacy_rejected(self):
  l=Link();l.p[0]=0x20701
  with self.assertRaises(ValueError):TrajectorySession().begin(l,compile_curve('0',1))
if __name__=='__main__':unittest.main()
