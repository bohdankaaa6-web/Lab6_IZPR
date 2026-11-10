# -*- coding: utf-8 -*-
"""
Доступ контролерів до об'єкта керування (сцена CoppeliaSim) через сигнали.

Бекенди:
  coppelia — справжня сцена CoppeliaSim (ZMQ Remote API, pip install coppeliasim-zmqremoteapi-client);
  mock     — програмна імітація сцени (mock_plant.py) для перевірки логіки без CoppeliaSim.
Кожен контролер відкриває власне з'єднання, тож контролери незалежні один від одного.
"""
import json
import socket


class Inputs:
    """Знімок вхідних сигналів за один цикл."""


class MockClient:
    """Клієнт імітатора сцени: той самий набір методів, що й у sim (JSON-рядки по TCP)."""

    def __init__(self, host, port):
        self.sock = socket.create_connection((host, port), timeout=3)
        self.f = self.sock.makefile("rwb")

    def _call(self, m, *a):
        self.f.write((json.dumps([m, list(a)]) + "\n").encode())
        self.f.flush()
        line = self.f.readline()
        if not line:
            raise ConnectionError("імітатор сцени закрив з'єднання")
        return json.loads(line)

    def getInt32Signal(self, n): return self._call("getInt32Signal", n)
    def setInt32Signal(self, n, v): return self._call("setInt32Signal", n, v)
    def getFloatSignal(self, n): return self._call("getFloatSignal", n)
    def getSimulationTime(self): return self._call("getSimulationTime")


class PlantIO:
    def __init__(self, backend="coppelia", host="localhost", port=None):
        self.backend = backend
        if backend == "mock":
            self.sim = MockClient(host, port or 23000)
        else:
            from coppeliasim_zmqremoteapi_client import RemoteAPIClient
            client = RemoteAPIClient(host, port or 23000)
            self.sim = client.require("sim")
        self._cache = {}

    def reset_cache(self):
        self._cache.clear()

    def time(self):
        return float(self.sim.getSimulationTime())

    def gi(self, name):
        v = self.sim.getInt32Signal(name)
        return 0 if v is None else int(v)

    def gf(self, name):
        v = self.sim.getFloatSignal(name)
        return 0.0 if v is None else float(v)

    def set(self, name, value):
        """Записати int-сигнал, якщо значення змінилось. Повертає True, якщо записано."""
        if self._cache.get(name) == value:
            return False
        self._cache[name] = value
        self.sim.setInt32Signal(name, int(value))
        return True
