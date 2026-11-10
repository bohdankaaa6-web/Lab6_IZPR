# -*- coding: utf-8 -*-
"""
Журнал подій компонента (CSV, UTF-8 з BOM, роздільник «;»).
Кожен компонент веде власний файл logs/<компонент>.csv; скрипт merge_logs.py зливає їх за часом
в один журнал system_log.csv, за яким можна відтворити взаємодію контролерів.

Колонки: №, Епоха (для злиття), Час (годин:хв:с.мс), Час симуляції, Компонент, Категорія,
         Операція № (ідентифікатор операції Modbus), Опис, Стан, Попередній стан, Наступний стан, Причина.
"""
import csv
import threading
import time
from collections import deque
from enum import Enum

HEADER = ["№", "Епоха, с", "Час", "Час сим., с", "Компонент", "Категорія", "Операція №",
          "Опис", "Стан", "Попередній стан", "Наступний стан", "Причина"]


def _nm(s):
    if isinstance(s, Enum):
        return s.name
    return s or ""


class EventLogger:
    def __init__(self, comp, path, echo=True):
        self.comp, self.path, self.echo = comp, path, echo
        self.n = 0
        self.tail = deque(maxlen=400)
        self._lock = threading.Lock()
        self._f = open(path, "w", newline="", encoding="utf-8-sig")
        self._w = csv.writer(self._f, delimiter=";")
        self._w.writerow(HEADER)
        self._f.flush()

    def log(self, cat, desc, op="", state="", prev="", nxt="", reason="", simt=""):
        now = time.time()
        stamp = time.strftime("%H:%M:%S", time.localtime(now)) + f".{int(now % 1 * 1000):03d}"
        with self._lock:
            self.n += 1
            row = [self.n, f"{now:.3f}", stamp, simt, self.comp, cat, op, desc,
                   _nm(state), _nm(prev), _nm(nxt), reason]
            self._w.writerow(row)
            self._f.flush()
            line = (f"{stamp} {self.comp:<11} {cat:<10} " + (f"op={op} " if op != "" else "")
                    + desc + (f"  ({reason})" if reason else ""))
            self.tail.append(line)
            if self.echo:
                print(line, flush=True)

    def close(self):
        with self._lock:
            try:
                self._f.close()
            except OSError:
                pass
