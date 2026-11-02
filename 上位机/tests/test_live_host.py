import os
"""Replay the bytes actually emitted by RTL telemetry simulation."""
from pathlib import Path
import unittest
from dbg_host import DbgLink, DbgError
from test_dbg_host import ReplaySerial

class TestLive(unittest.TestCase):
    def test_rtl_bytes(self):
        p=Path(os.environ.get('WORKDIR', str(Path(__file__).resolve().parents[2]/'fpga/build/tests/tb_live_telemetry')))/'live_tx.hex'
        stream=ReplaySerial(bytes(int(x,16) for x in p.read_text().split()))
        link=DbgLink(transport=stream)
        self.assertEqual(link.read_live(),{'row':[3070,-2,1040,-100],
                                         'sequence':1,'status':0x12345684})
        self.assertEqual(link.read_live(),{'row':[4,3,2,1],'sequence':2,'status':0x80})
        self.assertEqual(stream.pos,len(stream.buf))
    def test_unsupported(self):
        from unittest.mock import patch
        with patch.object(DbgLink,'request',return_value=(0xa4,b'')):
            with self.assertRaises(DbgError): DbgLink(transport=ReplaySerial(b'')).read_live()

if __name__=='__main__': unittest.main()
