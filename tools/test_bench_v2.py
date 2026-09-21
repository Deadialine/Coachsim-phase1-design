import csv
from pathlib import Path
import tempfile
import unittest
from bench_v2 import generate, inspect_stream

class BenchTests(unittest.TestCase):
    def test_nominal_and_faults(self):
        with tempfile.TemporaryDirectory() as td:
            good=generate(Path(td)/'good',2,4)
            bad=generate(Path(td)/'bad',2,4,True)
            self.assertTrue(good['pass_all']);self.assertFalse(bad['pass_all'])
            self.assertFalse(bad['emg']['checks']['clock_rate_within_1pct'])
            self.assertFalse(bad['emg']['checks']['missing_below_0_5pct'])
            self.assertFalse(bad['emg']['checks']['clipping_each_channel_below_1pct'])
    def test_trailing_loss_and_duplicate(self):
        with tempfile.TemporaryDirectory() as td:
            path=Path(td)/'imu.csv'
            with path.open('w',newline='') as f:
                w=csv.writer(f);w.writerow(['sample_index','t_us','ax','ay','az','gx','gy','gz','sensor_flags'])
                for i in [0,1,1,3]:w.writerow([i,i*10000,0,0,1,0,0,0,0])
            r=inspect_stream(path,100,2)
            self.assertEqual(r['nonmonotonic'],1);self.assertEqual(r['missing_samples'],196);self.assertFalse(r['pass_all'])
    def test_never_overwrite_evidence(self):
        with tempfile.TemporaryDirectory() as td:
            with self.assertRaises(ValueError):generate(Path(td),2,1)

if __name__=='__main__': unittest.main()
