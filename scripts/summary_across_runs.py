#!/usr/bin/env python3
"""Aggregate independent experimental runs, not autocorrelated frames."""
import argparse
import csv
import math
import statistics
from collections import defaultdict


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('comparison', help='CSV produced by summarize.py')
    parser.add_argument('--out', default='aggregate.csv')
    args = parser.parse_args()
    by_backend = defaultdict(list)
    with open(args.comparison, newline='') as file:
        for row in csv.DictReader(file):
            by_backend[row['backend']].append(row)
    # Conservative small-sample t critical values (two-sided 95%).
    t_crit = {1: 12.706, 2: 4.303, 3: 3.182, 4: 2.776,
              5: 2.571, 6: 2.447, 7: 2.365, 8: 2.306, 9: 2.262,
              10: 2.228, 11: 2.201, 12: 2.179, 13: 2.160, 14: 2.145,
              15: 2.131, 16: 2.120, 17: 2.110, 18: 2.101, 19: 2.093,
              20: 2.086, 21: 2.080, 22: 2.074, 23: 2.069, 24: 2.064,
              25: 2.060, 26: 2.056, 27: 2.052, 28: 2.048, 29: 2.045,
              30: 2.042}
    outputs = []
    for backend, rows in sorted(by_backend.items()):
        for metric in ('callback_p50_ms', 'callback_p95_ms', 'callback_p99_ms',
                       'filter_p50_ms', 'filter_p99_ms'):
            xs = [float(r[metric]) for r in rows]
            n = len(xs)
            mean = statistics.mean(xs)
            df = n - 1
            t = t_crit.get(df, 1.96) if df >= 1 else math.nan
            half = t * statistics.stdev(xs) / math.sqrt(n) if n > 1 else math.nan
            outputs.append({'backend': backend, 'metric': metric, 'runs': n,
                            'run_level_mean': round(mean, 5),
                            'ci95_low': round(mean-half, 5),
                            'ci95_high': round(mean+half, 5)})
    with open(args.out, 'w', newline='') as file:
        writer = csv.DictWriter(file, fieldnames=['backend', 'metric', 'runs',
            'run_level_mean', 'ci95_low', 'ci95_high'])
        writer.writeheader()
        writer.writerows(outputs)
    for row in outputs:
        print(row)


if __name__ == '__main__':
    main()
