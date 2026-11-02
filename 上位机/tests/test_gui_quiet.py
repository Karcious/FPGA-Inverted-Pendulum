import unittest
from gui_quiet import configure,status_text
class Link:
 def __init__(self):self.p={0:0x02070F,91:0};self.w=[]
 def get_param(self,a):return self.p.get(a,0)
 def set_param(self,a,v):self.p[a]=v;self.w.append((a,v))
class TestQuiet(unittest.TestCase):
 def test_config(self):
  l=Link();configure(l,True,1,.527);self.assertEqual(l.w,[(88,0),(89,1000),(90,6),(88,1)])
 def test_disable(self):
  l=Link();configure(l,False,1,.527);self.assertEqual(l.w[-1],(88,0))
 def test_old_firmware(self):
  l=Link();l.p[0]=0x02070D
  with self.assertRaises(ValueError):configure(l,True,1,.527)
  self.assertEqual(l.w,[])
 def test_invalid(self):
  for seconds,angle in [(0,.527),(1,.1),(float('nan'),.527),(1,float('inf'))]:
   l=Link()
   with self.assertRaises(ValueError):configure(l,True,seconds,angle)
   self.assertEqual(l.w,[])
 def test_large_threshold(self):
  l=Link();configure(l,True,1,30);self.assertEqual(l.p[90],341)
 def test_register_overflow(self):
  l=Link()
  with self.assertRaises(ValueError):configure(l,True,1,1e308)
  self.assertEqual(l.w,[])
 def test_status(self):
  l=Link();l.p[91]=3;self.assertIn('正在抑制',status_text(l));l.p[91]=1|(1<<9);self.assertIn('同向增大',status_text(l))
