# -*- coding: utf-8 -*-
"""
Імітатор об'єкта керування (спрощена модель ur5_sorter_plant.lua) — для перевірки логіки
розподіленої системи БЕЗ CoppeliaSim (наприклад, на комп'ютері без симулятора або в автотестах).

Реалізує той самий інтерфейс сигналів, що й Lua-скрипт сцени (див. його заголовок):
стрічка, датчик присутності, камера (колір найближчої до зони захоплення деталі),
цикл робота з фазами 1..5, лічильники, тестові впливи inj_*, скидання сцени.
Сигнали доступні по TCP (JSON-рядки), клієнт — plant_io.MockClient.
Параметр speed прискорює час симуляції відносно реального.
"""
import argparse
import json
import math
import random
import socketserver
import threading
import time

DT = 0.05
BELT = {"x0": -1.2, "x1": 1.2, "y": 0.68}
BELT_V, SPAWN_X, SPAWN_GAP, PICK_X = 0.12, -1.0, 0.35, 0.0
COLOR_CODE = {"red": 1, "green": 2, "blue": 3}


class MockPlant:
    def __init__(self, speed=4.0, seed=5, n_cubes=12):
        self.speed, self.n = speed, n_cubes
        self.lock = threading.RLock()
        self.sig, self.fsig = {}, {}
        self.rnd = random.Random(seed)
        self.running = False
        self._init_model()

    # ---------- інтерфейс сигналів ----------
    def api_getInt32Signal(self, n): return self.sig.get(n)
    def api_setInt32Signal(self, n, v): self.sig[n] = int(v)
    def api_getFloatSignal(self, n): return self.fsig.get(n)
    def api_getSimulationTime(self): return self.t

    def gi(self, n): return self.sig.get(n, 0)

    # ---------- модель ----------
    def _init_model(self):
        names = ["red", "green", "blue"]
        self.queue = [names[k % 3] for k in range(self.n)]
        self.rnd.shuffle(self.queue)
        self.t, self.steps = 0.0, 0
        self.cubes = []
        self.spawned = self.missed = self.delivered = self.correct = self.cleared = 0
        self.beltRun = False
        self.atHome, self.phase, self.doneCount, self.lastResult = False, 0, 0, 0
        self.lastBlob, self.visSeq, self.robotFault = None, 0, False
        self.lastPick = self.lastReset = 0
        self.cur = None
        self.seq, self.seq_t, self.seq_done = [], 0.0, None
        self.mode = "IDLE"
        for k in ("cmd_belt", "cmd_halt", "cmd_analyze", "cmd_pick_seq", "cmd_pick_color", "cmd_reset_seq",
                  "inj_overload", "inj_robot_hang", "inj_sensor_blind", "vis_seq", "vis_color"):
            self.sig.pop(k, None)
        self.sig["plant_ready"] = 0
        self.sig["total_cubes"] = self.n
        self._start_seq([(1.5, 0, None)], self._after_init_home)

    # ---------- рух робота як послідовність фаз ----------
    def _start_seq(self, steps, done):
        self.seq, self.seq_t, self.seq_done, self.mode = list(steps), 0.0, done, "MOVING"
        self._enter_step()

    def _enter_step(self):
        d, ph, fn = self.seq[0]
        self.phase = ph
        if fn:
            fn()

    def _follow(self):
        self.seq_t += DT
        if self.seq_t >= self.seq[0][0]:
            self.seq.pop(0)
            self.seq_t = 0.0
            if self.seq:
                self._enter_step()
            else:
                self.mode = "IDLE"
                f, self.seq_done = self.seq_done, None
                if f:
                    f()

    def _after_init_home(self):
        self.atHome, self.phase = True, 0

    def _after_cycle_home(self):
        self.atHome, self.phase, self.cur = True, 0, None
        self.doneCount += 1

    def _grasp_done(self):
        self.phase = 3
        self.cur["cube"]["state"] = "carried"

    def _release(self):
        self.phase = 4
        c, col = self.cur["cube"], self.cur["color"]
        c["state"] = "done"
        self.delivered += 1
        ok = COLOR_CODE[c["color"]] == col
        self.correct += 1 if ok else 0
        self.lastResult = 1 if ok else -1

    def _start_pick(self, blob, color):
        cube = None
        for c in self.cubes:
            if c["state"] == "belt" and math.hypot(c["x"] - blob["x"], c["y"] - blob["y"]) < 0.10:
                cube = c
        if not cube:
            return False
        self.cur = {"cube": cube, "color": color}
        self.atHome, self.phase = False, 1
        self._start_seq([(2.5, 1, None), (0.3, 2, None), (3.0, 3, self._grasp_done),
                         (0.3, 4, self._release), (2.0, 5, None)], self._after_cycle_home)
        return True

    def _reset_plant(self):
        n = 0
        for c in self.cubes:
            if c["state"] in ("belt", "carried"):
                c["state"] = "cleared"
                n += 1
        self.cleared += n
        self.robotFault, self.cur, self.atHome, self.phase = False, None, False, 0
        self._start_seq([(1.5, 0, None)], self._after_init_home)

    # ---------- такт ----------
    def _spawn(self):
        self.spawned += 1
        lat = (0.025 + self.rnd.random() * 0.025) * (-1 if self.rnd.random() < 0.5 else 1)
        self.cubes.append({"x": SPAWN_X, "y": BELT["y"] + lat, "color": self.queue[self.spawned - 1],
                           "state": "belt"})

    def _update_belt(self):
        if not self.beltRun:
            return
        for c in self.cubes:
            if c["state"] == "belt":
                c["x"] += BELT_V * DT
                if c["x"] > BELT["x1"] - 0.05:
                    c["state"] = "missed"
                    self.missed += 1
        if self.spawned < self.n and not any(c["state"] == "belt" and c["x"] < SPAWN_X + SPAWN_GAP
                                             for c in self.cubes):
            self._spawn()

    def _analyze(self):
        pick = None
        for c in self.cubes:
            if c["state"] == "belt" and abs(c["x"]) < 0.5 and c["x"] >= PICK_X - 0.15:
                if pick is None or abs(c["x"] - PICK_X) < abs(pick["x"] - PICK_X):
                    pick = c
        self.lastBlob = {"x": pick["x"], "y": pick["y"]} if pick else None
        self.visSeq += 1
        self.sig["vis_color"] = COLOR_CODE[pick["color"]] if pick else 0
        self.sig["vis_seq"] = self.visSeq

    def step(self):
        self.steps += 1
        self.t += DT
        if self.steps == 10:
            self.sig["plant_ready"] = 1
        rs = self.gi("cmd_reset_seq")
        if rs != self.lastReset:
            self.lastReset = rs
            self._reset_plant()
        if self.gi("cmd_halt") == 1:
            self.beltRun = False
            return self._publish()
        self.beltRun = self.gi("cmd_belt") == 1
        self._update_belt()
        if self.mode == "MOVING":
            self._follow()
        else:
            if self.gi("cmd_analyze") == 1 and not self.beltRun and self.steps % 3 == 0:
                self._analyze()
            ps = self.gi("cmd_pick_seq")
            if ps != self.lastPick:
                self.lastPick = ps
                if self.gi("inj_robot_hang") == 1:
                    pass                                          # робот ігнорує команду
                elif not self.atHome:
                    self.robotFault = True
                else:
                    col = self.gi("cmd_pick_color")
                    if not (1 <= col <= 3 and self.lastBlob and self._start_pick(self.lastBlob, col)):
                        self.robotFault = True
        self._publish()

    def _publish(self):
        blind = self.gi("inj_sensor_blind") == 1
        present, on_belt = 0, 0
        for c in self.cubes:
            if c["state"] == "belt":
                on_belt += 1
                if not blind and PICK_X - 0.02 <= c["x"] <= PICK_X + 0.30:
                    present = 1
        cur = (1.4 + 0.15 * math.sin(3 * self.t)) if self.beltRun else 0.1
        if self.gi("inj_overload") == 1:
            cur += 5.0
        s = self.sig
        s["plant_alive"] = self.steps
        s["robot_home"] = 1 if (self.atHome and self.mode == "IDLE") else 0
        s["robot_phase"], s["robot_fault"] = self.phase, 1 if self.robotFault else 0
        s["done_count"], s["last_result"], s["presence"] = self.doneCount, self.lastResult, present
        s["spawned"], s["belt_count"], s["missed"] = self.spawned, on_belt, self.missed
        s["delivered"], s["correct"], s["cleared"] = self.delivered, self.correct, self.cleared
        self.fsig["belt_speed"] = BELT_V if self.beltRun else 0.0
        self.fsig["motor_current"] = cur

    # ---------- потік симуляції ----------
    def run(self):
        self.running = True
        nxt = time.time()
        while self.running:
            with self.lock:
                self.step()
            nxt += DT / self.speed
            time.sleep(max(0.0, nxt - time.time()))
            if nxt < time.time() - 1:
                nxt = time.time()

    def start_thread(self):
        threading.Thread(target=self.run, daemon=True).start()


def make_server(plant, host="127.0.0.1", port=23000):
    class H(socketserver.StreamRequestHandler):
        def handle(self):
            try:
                for line in self.rfile:
                    try:
                        m, a = json.loads(line)
                        with plant.lock:
                            r = getattr(plant, "api_" + m)(*a)
                    except Exception:
                        r = None
                    self.wfile.write((json.dumps(r) + "\n").encode())
                    self.wfile.flush()
            except (OSError, ValueError):
                pass                                              # клієнт відключився

    class S(socketserver.ThreadingTCPServer):
        allow_reuse_address = True
        daemon_threads = True

    return S((host, port), H)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Імітатор сцени сортування")
    ap.add_argument("--port", type=int, default=23000)
    ap.add_argument("--speed", type=float, default=1.0)
    a = ap.parse_args()
    p = MockPlant(a.speed)
    p.start_thread()
    print(f"Імітатор сцени слухає порт {a.port}, швидкість x{a.speed}")
    make_server(p, "0.0.0.0", a.port).serve_forever()
