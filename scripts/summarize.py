#!/usr/bin/env python3
"""Report frame-processing metrics and create comparison CSV without inventing measurements."""
import argparse
import csv
import math
import statistics
from pathlib import Path

def percentile(values, q):
    v = sorted(values)
    if not v: return float('nan')
    x = (len(v)-1)*q
    i = int(math.floor(x))
    j = int(math.ceil(x))
    return v[i] if i==j else v[i]+(v[j]-v[i])*(x-i)

def main():
    p=argparse.ArgumentParser()
    p.add_argument('csv', nargs='+', help='CSV files from separate runs')
    p.add_argument('--warmup', type=int, default=100)
    p.add_argument('--deadline-ms', type=float, default=100.0)
    p.add_argument('--out', default='comparison.csv')
    a=p.parse_args()
    result=[]
    for filename in a.csv:
        with open(filename, newline='') as f:
            rows=list(csv.DictReader(f))[a.warmup:]
        if not rows:
            print(f'SKIP {filename}: no steady-state samples')
            continue
        cb=[float(r['callback_ms']) for r in rows]
        filt=[float(r['filter_ms']) for r in rows]
        n=[int(r['n_input']) for r in rows]
        map_sz=[int(r['map_size']) for r in rows]
        # Source stamp to receiver clock only meaningful if clocks share the same time basis.
        summary={
            'file':str(filename), 'backend':rows[0]['backend'], 'frames':len(rows),
            'input_mean':round(statistics.mean(n),2),
            'map_size_last':map_sz[-1],
            'callback_p50_ms':round(percentile(cb,.50),4),
            'callback_p95_ms':round(percentile(cb,.95),4),
            'callback_p99_ms':round(percentile(cb,.99),4),
            'filter_p50_ms':round(percentile(filt,.50),4),
            'filter_p99_ms':round(percentile(filt,.99),4),
            'deadline_misses':sum(x>a.deadline_ms for x in cb),
            'correctness_failures':sum(r['reference_equal']=='0' for r in rows)
        }
        print(summary)
        result.append(summary)
    if result:
        with open(a.out,'w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(result[0]));w.writeheader();w.writerows(result)
        print('Wrote',a.out)
if __name__=='__main__':main()
