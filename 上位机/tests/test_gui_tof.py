import unittest,time
from gui_tof import decode,PACKET,TofMailbox,forward_snapshot,pick
class Tests(unittest.TestCase):
 def test_decode(self):
  p=decode(PACKET.pack(b'TF',1,1,42,100,170,0,0,255,0))
  self.assertEqual(p['flags'],1)
  self.assertEqual(p['left'],170)
 def test_invalid(self):
  for data in (b'',PACKET.pack(b'XX',1,1,0,0,0,0,0,0,0)):
   with self.assertRaises(ValueError):decode(data)
  self.assertEqual(decode(PACKET.pack(b'TF',1,3,0,0,12,45,4,0,0))['flags'],2)
 def test_stale(self):
  m=TofMailbox();m.put(dict(left=1,right=2,flags=3));p=m.snapshot()
  self.assertEqual(m.snapshot(p['received']+.51)['flags'],0)
  m.clear();self.assertEqual(m.snapshot()['flags'],0)
 def test_old_firmware_never_written(self):
  class Link:
   def get_param(self,a):return 0x020712
   def set_param(self,a,v):raise AssertionError('Old firmware written')
  with self.assertRaises(ValueError):forward_snapshot(Link(),dict(seq=1,left=1,right=0,flags=1))
 def test_commit_last(self):
  class Link:
   def __init__(self):self.w=[]
   def get_param(self,a):return {0:0x020715,80:0x544f4631,83:9}[a]
   def set_param(self,a,v):self.w.append((a,v))
  l=Link();forward_snapshot(l,dict(seq=9,left=170,right=0,flags=1))
  self.assertEqual(l.w,[(81,170),(82,0),(84,1),(83,9)])
 def test_side_mapping(self):

  p=dict(seq=1,left=2400,right=55,flags=2)
  self.assertEqual(pick(p,'left')[0:2],(2400,55))
  self.assertEqual(pick(p,'right')[0:3],(55,2400,1))
  self.assertEqual(pick(p,'both')[0:3],(55,2400,1))
 def test_both_takes_nearer(self):
  p=dict(seq=1,left=120,right=48,flags=3)
  self.assertEqual(pick(p,'both'),(48,120,3))
 def test_both_without_valid_target(self):

  p=dict(seq=1,left=2400,right=2300,flags=0)
  self.assertEqual(pick(p,'both'),(2400,2300,0))
