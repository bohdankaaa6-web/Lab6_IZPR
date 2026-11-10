# -*- coding: utf-8 -*-
"""Злиття журналів усіх компонентів в один хронологічний файл.
Запуск: python merge_logs.py [каталог_журналів] [вихідний_файл]"""
import csv
import glob
import os
import sys

from eventlog import HEADER


def merge(log_dir="logs", out=None):
    out = out or os.path.join(log_dir, "system_log.csv")
    rows = []
    for path in sorted(glob.glob(os.path.join(log_dir, "*.csv"))):
        if os.path.abspath(path) == os.path.abspath(out):
            continue
        with open(path, newline="", encoding="utf-8-sig") as f:
            rd = csv.reader(f, delimiter=";")
            next(rd, None)
            for k, r in enumerate(rd):
                if len(r) == len(HEADER):
                    rows.append((float(r[1]), os.path.basename(path), k, r))
    rows.sort(key=lambda x: (x[0], x[1], x[2]))
    with open(out, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f, delimiter=";")
        w.writerow(HEADER)
        for n, (_, _, _, r) in enumerate(rows, 1):
            r = list(r)
            r[0] = n
            w.writerow(r)
    return out, len(rows)


if __name__ == "__main__":
    d = sys.argv[1] if len(sys.argv) > 1 else "logs"
    o = sys.argv[2] if len(sys.argv) > 2 else None
    path, n = merge(d, o)
    print(f"Злито {n} записів -> {path}")
