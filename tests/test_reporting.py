import csv
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class ReportingTests(unittest.TestCase):
    def test_percentile_and_summary(self):
        sys.path.insert(0, str(ROOT / 'scripts'))
        from summarize import percentile
        self.assertEqual(percentile([4, 1, 3, 2], 0.5), 2.5)
        self.assertEqual(percentile([4, 1, 3, 2], 1.0), 4)
        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp) / 'cpu.csv'
            with source.open('w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=['backend', 'callback_ms',
                    'filter_ms', 'n_input', 'map_size', 'reference_equal'])
                w.writeheader()
                for i in range(5):
                    w.writerow({'backend': 'cpu', 'callback_ms': i + 1,
                        'filter_ms': 1, 'n_input': 64, 'map_size': 32,
                        'reference_equal': 1})
            out = Path(temp) / 'comparison.csv'
            subprocess.run([sys.executable, str(ROOT/'scripts'/'summarize.py'),
                str(source), '--warmup', '0', '--deadline-ms', '3', '--out', str(out)],
                check=True, capture_output=True)
            with out.open() as f:
                data = list(csv.DictReader(f))
            self.assertEqual(len(data), 1)
            self.assertEqual(data[0]['deadline_misses'], '2')
            self.assertEqual(float(data[0]['callback_p50_ms']), 3.0)

    def test_run_level_aggregate(self):
        with tempfile.TemporaryDirectory() as temp:
            comparison = Path(temp) / 'comparison.csv'
            with comparison.open('w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=['backend', 'callback_p50_ms',
                    'callback_p95_ms', 'callback_p99_ms', 'filter_p50_ms', 'filter_p99_ms'])
                w.writeheader()
                for value in (1, 3, 5, 7, 9):
                    w.writerow(dict(backend='optimized', callback_p50_ms=value,
                        callback_p95_ms=value, callback_p99_ms=value,
                        filter_p50_ms=value, filter_p99_ms=value))
            out = Path(temp) / 'aggregate.csv'
            subprocess.run([sys.executable, str(ROOT/'scripts'/'summary_across_runs.py'),
                str(comparison), '--out', str(out)], check=True, capture_output=True)
            with out.open() as f:
                first = next(csv.DictReader(f))
            self.assertEqual(first['runs'], '5')
            self.assertEqual(float(first['run_level_mean']), 5.0)
            self.assertLess(float(first['ci95_low']), 5.0)
            self.assertGreater(float(first['ci95_high']), 5.0)


if __name__ == '__main__':
    unittest.main()
