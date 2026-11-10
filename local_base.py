# -*- coding: utf-8 -*-
"""
Базовий клас локального контролера пристрою (процес-«slave» Modbus TCP).

Що робить база (однаково для стрічки, маніпулятора і підсистеми зору):
  * Modbus-сервер з єдиною картою регістрів (protocol.R);
  * приймання команд з ідентифікацією операції: підтвердження прийняття (ACK_OP),
    підтвердження виконання (DONE_OP + RESULT), відхилення (REJECT_OP + REJECT_ERR);
  * захист від повторів: команда з номером, який уже бачили, ВИКОНУЄТЬСЯ ЛИШЕ ОДИН РАЗ,
    повтор лише лічиться (DUP) і журналюється;
  * автономні захисти: втрата координатора (серцебиття), втрата об'єкта (plant_alive);
  * тестові впливи (імітація несправностей, затримки підтвердження, обриву зв'язку);
  * журнал подій із часом, ідентифікатором компонента та номером операції.
Конкретний пристрій задає свій автомат у підкласі (device_step, validate, start, ...).
"""
import argparse
import os
import sys
import time
from collections import deque

from eventlog import EventLogger
from modbus_tcp import ModbusServer
from plant_io import PlantIO
from protocol import (R, Cfg, C_HALT, C_RESET, RES_NONE, RES_OK, RES_FAILED, E_BUSY, E_PLANT_LINK,
                      E_COORD_LOST, E_HALTED, ERR_NAME, F_BUSY, F_FAULT, F_READY, cmd_name,
                      DEFAULT_PORTS)


class LocalController:
    DEV = "device"
    INIT_STATE = READY_STATE = FAULT_STATE = None
    INJ_MAP = {}          # {біт: (ім'я сигналу сцени, опис)}

    def __init__(self, args):
        self.args, self.cfg = args, Cfg
        os.makedirs(args.log_dir, exist_ok=True)
        self.log = EventLogger(self.DEV, os.path.join(args.log_dir, f"{self.DEV}.csv"))
        self.server = ModbusServer(args.bind, args.port, write_hook=self._on_write, size=R.SIZE)
        self.io = None
        self.i = None
        self.state, self.prev = None, None
        self.t = self.t_enter = 0.0
        self.wall = time.time()
        # обмін
        self.rx_seen = 0
        self.seen_ops = deque(maxlen=128)
        self.ack_op = self.done_op = self.reject_op = 0
        self.reject_err = 0
        self.cur_op = 0
        self.cur_busy = False
        self.result = RES_NONE
        self.error = 0
        self.hb = 0
        self.dups = 0
        self.accepted = 0
        self.delayed = None               # (час, op, code, p1, p2): відкладене підтвердження (тест)
        self.fault_count = 0
        # контроль зв'язку
        self.coord_hb_last = None
        self.coord_hb_t = self.wall
        self.coord_armed = False
        self.coord_lost = False
        self.alive_last, self.alive_t = None, self.wall
        self.plant_err = False
        self.inj_prev = 0

    # ---------- службові ----------
    def L(self, cat, desc, reason="", op=None):
        self.log.log(cat, desc, op=(self.cur_op if op is None else op) or "", state=self.state,
                     prev=self.prev, reason=reason, simt=f"{self.t:.2f}")

    def cmd(self, name, value, note=None):
        if self.io.set(name, value) and note:
            self.L("КОМАНДА", note)

    def in_state(self):
        return self.t - self.t_enter

    def go(self, new, reason=""):
        old = self.state
        self.log.log("ПЕРЕХІД", f"{old.name if old else '-'} -> {new.name}", op=self.cur_op or "",
                     state=old, prev=self.prev, nxt=new, reason=reason, simt=f"{self.t:.2f}")
        self.prev, self.state, self.t_enter = old, new, self.t
        self.on_enter(new)

    def fault(self, code, reason=""):
        if self.state == self.FAULT_STATE:
            return
        self.fault_count += 1
        self.error = code
        self.L("АВАРІЯ", ERR_NAME.get(code, str(code)), reason)
        self.go(self.FAULT_STATE, reason)
        if self.cur_busy:
            self.finish(RES_FAILED)

    def finish(self, result):
        """Операцію завершено: публікуємо DONE_OP/RESULT."""
        if not self.cur_busy:
            return
        self.cur_busy = False
        self.done_op, self.result = self.cur_op, result
        from protocol import RES_NAME
        self.L("ВИКОНАНО", f"Операцію завершено: {RES_NAME.get(result, result)}")

    # ---------- Modbus ----------
    def _on_write(self, addr, count):
        """Хук сервера (під замком): лічильник передавань блоку команди."""
        if addr <= R.OP < addr + count:
            self.server.regs[R.RX_COUNT] = (self.server.regs[R.RX_COUNT] + 1) & 0xFFFF

    def publish(self):
        flags = (F_BUSY if self.cur_busy else 0) | (F_FAULT if self.state == self.FAULT_STATE else 0) \
            | (F_READY if self.state == self.READY_STATE else 0) | self.extra_flags()
        self.hb = self.hb % 65535 + 1
        vals = [max(0, min(65535, int(v))) for v in self.values()]
        block = [int(self.state), self.ack_op, self.done_op, self.result, self.error, self.hb,
                 flags, self.reject_op, self.dups & 0xFFFF, self.reject_err] + vals + [self.accepted & 0xFFFF]
        self.server.set_block(R.STATUS_FIRST, block)

    # ---------- контроль зв'язку ----------
    def _plant_watchdog(self, i):
        if i.alive != self.alive_last:
            self.alive_last, self.alive_t = i.alive, self.wall
        elif self.state not in (self.INIT_STATE, self.FAULT_STATE) \
                and self.wall - self.alive_t > self.cfg.PLANT_TIMEOUT:
            self.fault(E_PLANT_LINK, f"plant_alive не змінюється {self.cfg.PLANT_TIMEOUT:.0f} с")

    def _coord_watchdog(self):
        hb, w = self.server.get(R.COORD_HB), self.wall
        if hb != self.coord_hb_last:
            self.coord_hb_last, self.coord_hb_t = hb, w
            if hb == 0:
                if self.coord_armed:
                    self.L("ЗВ'ЯЗОК", "Координатор штатно завершив роботу")
                self.coord_armed = False
            else:
                if self.coord_lost:
                    self.L("ЗВ'ЯЗОК", "Зв'язок з координатором відновлено")
                elif not self.coord_armed:
                    self.L("ЗВ'ЯЗОК", "Координатор підключився")
                self.coord_armed, self.coord_lost = True, False
        elif self.coord_armed and not self.coord_lost and w - self.coord_hb_t > self.cfg.COORD_TIMEOUT:
            self.coord_lost = True
            self.L("ЗВ'ЯЗОК", f"Втрата зв'язку з координатором: серцебиття не змінюється "
                              f"{self.cfg.COORD_TIMEOUT:.0f} с")
            if self.state not in (self.INIT_STATE, self.FAULT_STATE):
                self.fault(E_COORD_LOST, "автономна зупинка: координатор не відповідає")

    def _inject(self):
        mute = self.server.get(R.INJ_MUTE)
        if mute > 0:
            self.server.set(R.INJ_MUTE, 0)
            self.server.mute(mute)
            self.L("ІМІТАЦІЯ", f"Обрив зв'язку на {mute} с: Modbus-сервер не відповідає")
        flags = self.server.get(R.INJ_FLAGS)
        for bit, (sig, text) in self.INJ_MAP.items():
            on = bool(flags & bit)
            if on != bool(self.inj_prev & bit):
                self.L("ІМІТАЦІЯ", f"Тестовий вплив: {text} {'УВІМК.' if on else 'ВИМК.'}")
            self.io.set(sig, int(on))
        self.inj_prev = flags

    # ---------- команди ----------
    def _commands(self):
        if self.delayed and self.wall >= self.delayed[0]:
            _, op, code, p1, p2 = self.delayed
            self.delayed = None
            self.L("ІМІТАЦІЯ", "Затримку підтвердження завершено", op=op)
            self._decide(op, code, p1, p2)
        regs = self.server.get_block(0, 10)
        delta = (regs[R.RX_COUNT] - self.rx_seen) & 0xFFFF
        if delta == 0:
            return
        self.rx_seen = regs[R.RX_COUNT]
        code, op, p1, p2 = regs[R.CMD], regs[R.OP], regs[R.P1], regs[R.P2]
        if op == 0:
            return
        name = cmd_name(self.DEV, code)
        if op in self.seen_ops:                                   # повтор: не виконувати вдруге
            self.dups += delta
            self.L("ДУБЛЬ", f"Повторне надходження команди {name}: проігноровано, "
                            f"уже оброблялась (повторів: {delta})", op=op)
            return
        self.seen_ops.append(op)
        if delta > 1:
            self.dups += delta - 1
            self.L("ДУБЛЬ", f"Команду {name} отримано {delta} рази поспіль, виконується одна", op=op)
        self.L("ОТРИМАНО", f"Команда {name} (P1={p1}, P2={p2})", op=op)
        delay = regs[R.INJ_ACK_DELAY]
        if delay > 0 and code not in (C_HALT, C_RESET):
            self.server.set(R.INJ_ACK_DELAY, 0)
            self.delayed = (self.wall + delay / 1000.0, op, code, p1, p2)
            self.L("ІМІТАЦІЯ", f"Підтвердження затримано на {delay} мс", op=op)
            return
        self._decide(op, code, p1, p2)

    def _decide(self, op, code, p1, p2):
        name = cmd_name(self.DEV, code)
        if code in (C_HALT,):
            ok, err = True, 0
        elif code == C_RESET:
            ok, err = self.can_reset()
        elif self.cur_busy:
            ok, err = False, E_BUSY
        else:
            ok, err = self.validate(code, p1, p2)
        if not ok:
            self.reject_op, self.reject_err = op, err
            self.L("ВІДХИЛЕНО", f"Команду {name} відхилено", ERR_NAME.get(err, str(err)), op=op)
            return
        if self.cur_busy:                                         # HALT/RESET перериває поточну операцію
            self.finish(RES_FAILED)
        self.ack_op, self.cur_op, self.cur_busy = op, op, True
        self.accepted += 1
        self.result = RES_NONE
        self.L("ПРИЙНЯТО", f"Команду {name} прийнято до виконання", op=op)
        if code == C_HALT:
            self.do_halt()
        elif code == C_RESET:
            self.do_reset()
        else:
            self.start(code, p1, p2)

    def do_halt(self):
        self.cur_busy = False                                     # HALT сам не може «зазнати невдачі»
        if self.state != self.FAULT_STATE:
            self.fault(E_HALTED, "аварійна зупинка за командою координатора")
        self.cur_busy = True
        self.finish(RES_OK)

    def can_reset(self):
        return True, 0

    # ---------- головний цикл ----------
    def step(self):
        self.wall = time.time()
        try:
            i = self.read_plant()
            if self.plant_err:
                self.L("СИСТЕМА", "Зв'язок із сценою відновлено")
            self.plant_err = False
        except Exception as e:
            if not self.plant_err:
                self.L("СИСТЕМА", f"Помилка читання сигналів сцени: {e}")
            self.plant_err = True
            i = self.i
        if i is None:
            return
        if self.i is not None and i.t < self.t - 1.0:             # симуляцію перезапущено
            self.L("СИСТЕМА", "Виявлено перезапуск симуляції: повторна ініціалізація")
            self.io.reset_cache()
            self.cur_busy = False
            self.state = self.INIT_STATE
            self.on_reinit(i)
        self.i, self.t = i, i.t
        self._plant_watchdog(i)
        self._coord_watchdog()
        self._inject()
        self._commands()
        self.device_step(i)
        self.publish()

    def run(self):
        self.io = PlantIO(self.args.backend, self.args.plant_host, self.args.plant_port)
        self.server.start()
        self.state = self.INIT_STATE
        self.L("СТАН", f"Контролер «{self.DEV}» запущено, Modbus TCP порт {self.args.port}, початковий стан "
                       f"{self.state.name}")
        self.on_enter(self.state)
        try:
            while True:
                t0 = time.time()
                try:
                    self.step()
                except Exception as e:                            # цикл не повинен падати
                    self.L("СИСТЕМА", f"Внутрішня помилка циклу: {type(e).__name__}: {e}")
                    time.sleep(0.3)
                time.sleep(max(0.0, self.cfg.POLL - (time.time() - t0)))
        except KeyboardInterrupt:
            pass
        finally:
            self.L("СТАН", "Контролер зупинено")
            self.server.stop()
            self.log.close()

    # ---------- що перевизначає підклас ----------
    def read_plant(self): raise NotImplementedError
    def device_step(self, i): raise NotImplementedError
    def validate(self, code, p1, p2): return False, 3
    def start(self, code, p1, p2): pass
    def do_reset(self): self.finish(RES_OK)
    def on_enter(self, s): pass
    def on_reinit(self, i): pass
    def values(self): return [0] * 6
    def extra_flags(self): return 0


def parse_args(dev):
    ap = argparse.ArgumentParser(description=f"Локальний контролер «{dev}»")
    ap.add_argument("--bind", default="0.0.0.0", help="адреса, на якій слухає Modbus-сервер")
    ap.add_argument("--port", type=int, default=DEFAULT_PORTS[dev], help="порт Modbus TCP")
    ap.add_argument("--backend", choices=["coppelia", "mock"], default="coppelia")
    ap.add_argument("--plant-host", default="localhost", help="де працює CoppeliaSim / імітатор")
    ap.add_argument("--plant-port", type=int, default=None)
    ap.add_argument("--log-dir", default="logs")
    return ap.parse_args()
