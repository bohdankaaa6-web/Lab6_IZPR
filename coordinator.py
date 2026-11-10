# -*- coding: utf-8 -*-
"""
КООРДИНАТОР робототехнічного комплексу сортування (верхній рівень ієрархії).

Координатор не торкається сцени напряму. Усе, що він знає про пристрої, — регістри Modbus TCP
трьох локальних контролерів: conveyor (стрічка), vision (технічний зір), manipulator (робот UR5).

Цикл роботи (FSM із практикуму №5, адаптований до розподіленої схеми):
    INIT -> TRANSPORTING -> ANALYZING -> SORTING -> TRANSPORTING ...
    будь-який стан -> FAULT -> («Продовжити») -> INIT (RESET усіх пристроїв) -> ...
  TRANSPORTING : команда conveyor.ADVANCE   (стрічка їде до появи деталі і зупиняється)
  ANALYZING    : команда vision.ANALYZE     (колір деталі)
  SORTING      : команда manipulator.PICK_AND_SORT(колір)

Кожна команда має номер операції. Координатор чекає ПІДТВЕРДЖЕННЯ ПРИЙНЯТТЯ (ACK_OP) і ПІДТВЕРДЖЕННЯ
ВИКОНАННЯ (DONE_OP). Немає підтвердження за ACK_TIMEOUT — команда повторюється З ТИМ САМИМ НОМЕРОМ
(пристрій повтор не виконує). Працездатність зв'язку: успішні обміни + «серцебиття» пристрою.

Запуск:  python coordinator.py                    (вікно оператора)
         python coordinator.py --test normal      (без вікна)
Сценарії: normal | device_fail | overload | missed | comm_loss | ack_delay | duplicate
"""
import argparse
import os
import sys
import threading
import time
from enum import Enum

from eventlog import EventLogger
from modbus_tcp import ModbusClient, ModbusError
from protocol import (R, Cfg, C_HALT, C_RESET, C_ADVANCE, C_ANALYZE, C_PICK, RES_OK, RES_NO_PARTS,
                      RES_WRONG_BIN, RES_NAME, ERR_NAME, E_COMM, E_NO_ACK, E_SUPERVISOR, E_REJECTED,
                      E_COORD_LOST, F_FAULT, F_READY, F_OVERLOAD, INJ_OVERLOAD, INJ_BLIND,
                      INJ_ROBOT_HANG, DEFAULT_PORTS, COLOR_NAME, cmd_name, state_name)

DEVICES = ["conveyor", "vision", "manipulator"]


class State(Enum):
    INIT = "INIT"
    TRANSPORTING = "TRANSPORTING"
    ANALYZING = "ANALYZING"
    SORTING = "SORTING"
    FAULT = "FAULT"


# ============================================================
#  ПРИСТРІЙ НА ЛІНІЇ ЗВ'ЯЗКУ
# ============================================================
class Status:
    """Розібраний блок стану пристрою (регістри 10..26)."""
    def __init__(self, regs):
        (self.state, self.ack_op, self.done_op, self.result, self.error, self.hb, self.flags,
         self.reject_op, self.dups, self.reject_err) = regs[:10]
        self.v = regs[10:16]
        self.accepted = regs[16]


class Pending:
    """Відправлена команда, що очікує підтвердження/виконання."""
    def __init__(self, dev, code, p1, p2, op, now, best_effort):
        self.dev, self.code, self.p1, self.p2, self.op = dev, code, p1, p2, op
        self.t_first = self.t_sent = now
        self.tries = 0
        self.acked = self.done = self.rejected = self.gave_up = self.late_dup = False
        self.result, self.error, self.values = 0, 0, []
        self.best_effort = best_effort


class DeviceLink:
    def __init__(self, dev, host, port, cfg):
        self.dev, self.cfg = dev, cfg
        self.cli = ModbusClient(host, port, timeout=cfg.MB_TIMEOUT)
        self.status = None
        self.fresh = False            # статус оновлено в поточному циклі
        self.ever = False             # хоч раз був зв'язок
        self.last_ok = self.hb_t = 0.0
        self.coord_hb = 0
        self.lost = False
        self.pending = None
        self.fault_seen = False
        self.inj_flags_sent = None

    def exchange(self, now):
        self.fresh = False
        self.coord_hb = self.coord_hb % 65535 + 1
        try:
            self.cli.write_single(R.COORD_HB, self.coord_hb)
            regs = self.cli.read_holding(R.STATUS_FIRST, R.STATUS_COUNT)
        except ModbusError:
            return False
        st = Status(regs)
        if self.status is None or st.hb != self.status.hb:
            self.hb_t = now
        self.status, self.fresh, self.ever, self.last_ok = st, True, True, now
        return True

    def link_ok(self, now):
        return self.ever and now - self.last_ok <= self.cfg.COMM_TIMEOUT \
            and now - self.hb_t <= self.cfg.HB_TIMEOUT


# ============================================================
#  ОПЕРАТОР (GUI або сценарій)
# ============================================================
class Operator:
    def __init__(self):
        self.start_req = self.continue_req = self.belt_cleared = False
        self.inj_overload = self.inj_robot_hang = self.inj_sensor_blind = False
        self.dup_mode = False         # тест: кожну команду надсилати двічі
        self.ack_delay_req = False    # тест: затримати підтвердження наступної команди маніпулятора
        self.mute_req = False         # тест: обрив зв'язку зі стрічкою на 5 с
        self.stop_req = False
        self.sc = {}

    def press_start(self): self.start_req = True
    def press_continue(self): self.continue_req = True


# ============================================================
#  КООРДИНАТОР
# ============================================================
class Coordinator:
    def __init__(self, log, op, addrs, cfg=Cfg):
        self.log, self.op, self.cfg = log, op, cfg
        self.links = {d: DeviceLink(d, addrs[d][0], addrs[d][1], cfg) for d in DEVICES}
        self.state = self.prev = None
        self.t0 = self.now = time.time()
        self.t_enter = self.now
        self.op_seq = int(time.time() * 10) % 60000
        self.started = self.recovering = self.job_complete = False
        self.color = 0
        self.part_no = 0
        self.faults = self.rejected = 0
        self.fault_cause = ""
        self.fault_log = []
        self.sorted_ok = self.pick_ops = self.wrong_bin = 0
        self.dup_sent = self.retx = 0
        self.autonomous_stop_seen = False
        self._init_req_t = None

    # ---------- службові ----------
    def L(self, cat, desc, reason="", op=""):
        self.log.log(cat, desc, op=op, state=self.state, prev=self.prev, reason=reason)

    def in_state(self):
        return self.now - self.t_enter

    def go(self, new, reason):
        old = self.state
        self.log.log("ПЕРЕХІД", f"{old.name if old else '-'} -> {new.name}", state=old, prev=self.prev,
                     nxt=new, reason=reason)
        self.prev, self.state, self.t_enter = old, new, self.now
        self._on_enter(new)

    def fault(self, code, who, reason):
        if self.state == State.FAULT:
            return
        self.faults += 1
        self.fault_cause = f"{who}: {ERR_NAME.get(code, code)}"
        self.fault_log.append((self.now - self.t0, self.fault_cause))
        self.L("АВАРІЯ", self.fault_cause, reason)
        self.go(State.FAULT, reason)

    def all_ok(self):
        return all(l.ever and l.link_ok(self.now) for l in self.links.values())

    # ---------- передавання команд ----------
    def send(self, dev, code, p1=0, p2=0, best_effort=False):
        link = self.links[dev]
        self.op_seq = self.op_seq % 65535 + 1
        p = Pending(dev, code, p1, p2, self.op_seq, self.now, best_effort)
        link.pending = p
        self._tx(link, p, first=True)
        return p

    def _tx(self, link, p, first):
        p.tries += 1
        p.t_sent = self.now
        extra = f", P1={p.p1}" if p.code in (C_PICK,) and p.dev == "manipulator" else ""
        try:
            link.cli.write_multiple(R.CMD, [p.code, p.op, p.p1, p.p2])
            ok = True
        except ModbusError:
            ok = False
        name = cmd_name(p.dev, p.code)
        if first:
            self.L("КОМАНДА", f"{p.dev}: {name}{extra}" + ("" if ok else " (передати не вдалося)"), op=p.op)
        else:
            self.retx += 1
            self.L("ПОВТОР", f"{p.dev}: повторне передавання {name}, спроба {p.tries}/{self.cfg.ACK_TRIES}"
                   + ("" if ok else " (передати не вдалося)"), reason="підтвердження не надійшло", op=p.op)
        if first and ok and self.op.dup_mode:
            self._send_duplicate(link, p, "одразу після передавання")

    def _send_duplicate(self, link, p, when):
        try:
            link.cli.write_multiple(R.CMD, [p.code, p.op, p.p1, p.p2])
            self.dup_sent += 1
            p.late_dup = True
            self.L("ТЕСТ", f"{p.dev}: навмисний дублікат {cmd_name(p.dev, p.code)} ({when})", op=p.op)
        except ModbusError:
            pass

    def _tick_pendings(self):
        for dev, l in self.links.items():
            p, st = l.pending, l.status
            if p is None or p.done:
                continue
            if l.fresh and st:
                if st.ack_op == p.op and not p.acked:
                    p.acked = True
                    self.L("ПІДТВЕРДЖЕНО", f"{dev}: команду {cmd_name(dev, p.code)} прийнято "
                           f"(через {self.now - p.t_first:.2f} с, передавань: {p.tries})", op=p.op)
                    if self.op.dup_mode and not p.late_dup:
                        self._send_duplicate(l, p, "після підтвердження")
                if st.reject_op == p.op:
                    p.done = p.rejected = True
                    p.error = st.reject_err
                    self.L("ВІДХИЛЕНО", f"{dev}: команду {cmd_name(dev, p.code)} відхилено пристроєм",
                           ERR_NAME.get(st.reject_err, st.reject_err), op=p.op)
                    continue
                if st.done_op == p.op:
                    p.acked = p.done = True
                    p.result, p.error, p.values = st.result, st.error, list(st.v)
                    self.L("ВИКОНАНО", f"{dev}: {cmd_name(dev, p.code)} завершено, результат: "
                           f"{RES_NAME.get(st.result, st.result)} (за {self.now - p.t_first:.2f} с)", op=p.op)
                    continue
            if not p.acked and self.now - p.t_sent > self.cfg.ACK_TIMEOUT:
                if p.tries >= self.cfg.ACK_TRIES:
                    p.done = p.gave_up = True
                    self.L("ТАЙМ-АУТ", f"{dev}: команду {cmd_name(dev, p.code)} не підтверджено після "
                           f"{p.tries} передавань", op=p.op)
                    if not p.best_effort and self.state != State.FAULT:
                        self.fault(E_NO_ACK, dev, f"команда {cmd_name(dev, p.code)} не підтверджена")
                else:
                    self._tx(l, p, first=False)

    # ---------- контроль зв'язку та станів пристроїв ----------
    def _track_links(self):
        for dev, l in self.links.items():
            if not l.ever:
                continue
            ok = l.link_ok(self.now)
            if not ok and not l.lost:
                l.lost = True
                self.L("ЗВ'ЯЗОК", f"Втрата зв'язку з пристроєм {dev}",
                       f"немає відповіді {self.now - l.last_ok:.1f} с")
            elif ok and l.lost:
                l.lost = False
                self.L("ЗВ'ЯЗОК", f"Зв'язок з пристроєм {dev} відновлено")
            st = l.status
            if l.fresh and st:
                fl = bool(st.flags & F_FAULT)
                if fl and not l.fault_seen:
                    self.L("ПРИСТРІЙ", f"{dev}: перейшов у стан FAULT", ERR_NAME.get(st.error, st.error))
                    if st.error == E_COORD_LOST:
                        self.autonomous_stop_seen = True
                        self.L("ПРИСТРІЙ", f"{dev}: автономно зупинився після втрати координатора")
                l.fault_seen = fl

    def _global_checks(self):
        for dev, l in self.links.items():
            if l.ever and l.lost:
                return self.fault(E_COMM, dev, "обмін з пристроєм неможливий")
        for dev, l in self.links.items():
            if l.fresh and l.status and l.status.flags & F_FAULT:
                return self.fault(l.status.error, dev, "пристрій повідомив про аварію")

    # ---------- дії оператора / тестові впливи ----------
    def _sync_operator(self):
        conv, mn = self.links["conveyor"], self.links["manipulator"]
        want = {"conveyor": (INJ_OVERLOAD if self.op.inj_overload else 0)
                | (INJ_BLIND if self.op.inj_sensor_blind else 0),
                "manipulator": INJ_ROBOT_HANG if self.op.inj_robot_hang else 0}
        for dev, val in want.items():
            l = self.links[dev]
            if l.fresh and l.inj_flags_sent != val:
                try:
                    l.cli.write_single(R.INJ_FLAGS, val)
                    if l.inj_flags_sent is not None or val:
                        self.L("ОПЕРАТОР", f"Тестові впливи для {dev}: маска {val}")
                    l.inj_flags_sent = val
                except ModbusError:
                    pass
        if self.op.ack_delay_req and mn.fresh:
            try:
                mn.cli.write_single(R.INJ_ACK_DELAY, 2500)
                self.op.ack_delay_req = False
                self.L("ОПЕРАТОР", "Тест: підтвердження наступної команди маніпулятора буде затримано на 2.5 с")
            except ModbusError:
                pass
        if self.op.mute_req and conv.fresh:
            try:
                conv.cli.write_single(R.INJ_MUTE, 5)
                self.op.mute_req = False
                self.L("ОПЕРАТОР", "Тест: обрив зв'язку зі стрічкою на 5 с")
            except ModbusError:
                pass

    # ---------- дії при вході в стан ----------
    def _on_enter(self, s):
        if s == State.INIT:
            self._init_req_t = self.now if self.started else None
            if self.recovering:
                for d, l in self.links.items():
                    if l.ever:
                        self.send(d, C_RESET)
        elif s == State.TRANSPORTING:
            self.send("conveyor", C_ADVANCE)
        elif s == State.ANALYZING:
            self.send("vision", C_ANALYZE)
        elif s == State.SORTING:
            self.send("manipulator", C_PICK, self.color)
        elif s == State.FAULT:
            for d, l in self.links.items():
                if l.ever:
                    self.send(d, C_HALT, best_effort=True)

    # ---------- головний крок ----------
    def start(self):
        self.now = time.time()
        self.state = State.INIT
        self.t_enter = self.now
        self.L("СТАН", "Координатор запущено, початковий стан INIT")
        self._on_enter(State.INIT)

    def update(self):
        self.now = time.time()
        for l in self.links.values():
            l.exchange(self.now)
        self._track_links()
        self._sync_operator()
        self._tick_pendings()
        if self.state != State.FAULT:
            self._global_checks()
        {State.INIT: self._st_init, State.TRANSPORTING: self._st_transporting,
         State.ANALYZING: self._st_analyzing, State.SORTING: self._st_sorting,
         State.FAULT: self._st_fault}[self.state]()

    def shutdown(self):
        """Штатне завершення: серцебиття = 0, щоб пристрої не вважали це втратою зв'язку."""
        for l in self.links.values():
            try:
                l.cli.write_single(R.COORD_HB, 0)
            except ModbusError:
                pass
            l.cli.close()

    # ---------- стани ----------
    def _await(self, dev, code, backstop):
        """Повертає Pending, якщо він завершений; None — ще чекаємо; при проблемі — аварія."""
        p = self.links[dev].pending
        if p is None or p.code != code:
            return None
        if p.done:
            if p.rejected:
                self.fault(E_REJECTED, dev, f"команду {cmd_name(dev, code)} відхилено: {ERR_NAME.get(p.error)}")
                return None
            return p
        if self.now - p.t_first > backstop:
            self.fault(E_SUPERVISOR, dev, f"{cmd_name(dev, code)} не завершено за {backstop:.0f} с")
        return None

    def _st_init(self):
        c = self.cfg
        if self.job_complete:
            return
        if self.recovering:
            ps = [l.pending for l in self.links.values() if l.pending and l.pending.code == C_RESET]
            if ps and all(p.done for p in ps):
                if any(p.rejected or p.result != RES_OK for p in ps):
                    return self.fault(E_REJECTED, "комплекс", "пристрій не зміг виконати RESET")
                self.recovering = False
                self.L("СТАН", "Усі пристрої скинуто, сцену очищено")
            elif self.in_state() > c.BACKSTOP_RESET:
                self.fault(E_SUPERVISOR, "комплекс", f"RESET не завершено за {c.BACKSTOP_RESET:.0f} с")
            return
        if self.op.start_req:
            self.op.start_req = False
            if not self.started:
                self.started, self._init_req_t = True, self.now
                self.L("ОПЕРАТОР", "Натиснуто «Пуск», перевірка систем")
        if self._init_req_t is None:
            return
        if self.all_ok() and all(l.status.flags & F_READY for l in self.links.values()):
            self.L("СТАН", "Перевірка систем пройдена: зв'язок з усіма пристроями є, усі готові")
            self.go(State.TRANSPORTING, "системи в нормі")
        elif self.now - self._init_req_t > c.INIT_WAIT:
            self.fault(E_SUPERVISOR, "комплекс", f"пристрої не готові за {c.INIT_WAIT:.0f} с")

    def _st_transporting(self):
        p = self._await("conveyor", C_ADVANCE, self.cfg.BACKSTOP_CONV)
        if not p:
            return
        if p.result == RES_OK:
            self.part_no += 1
            self.L("ПОДІЯ", f"Деталь №{self.part_no} у зоні захоплення")
            self.go(State.ANALYZING, "деталь у зоні захоплення")
        elif p.result == RES_NO_PARTS:
            self._finish_job()
        else:
            self.fault(p.error or E_SUPERVISOR, "conveyor", "ADVANCE завершено з помилкою")

    def _st_analyzing(self):
        p = self._await("vision", C_ANALYZE, self.cfg.BACKSTOP_VIS)
        if not p:
            return
        if p.result == RES_OK and p.values and p.values[0] in COLOR_NAME:
            self.color = p.values[0]
            self.L("ПОДІЯ", f"Підсистема зору повідомила колір: {COLOR_NAME[self.color]}")
            self.go(State.SORTING, f"колір {COLOR_NAME[self.color]}")
        else:
            self.fault(p.error or E_SUPERVISOR, "vision", "аналіз завершено без результату")

    def _st_sorting(self):
        p = self._await("manipulator", C_PICK, self.cfg.BACKSTOP_MANIP)
        if not p:
            return
        if p.result in (RES_OK, RES_WRONG_BIN):
            self.sorted_ok += 1
            if p.result == RES_WRONG_BIN:
                self.wrong_bin += 1
                self.L("ПОПЕРЕДЖЕННЯ", "Деталь потрапила в кошик іншого кольору (помилка класифікації)")
            self.go(State.TRANSPORTING, "цикл сортування завершено")
        else:
            self.fault(p.error or E_SUPERVISOR, "manipulator", "цикл сортування завершено з помилкою")

    def _st_fault(self):
        if not self.op.continue_req:
            return
        self.op.continue_req = False
        problems = []
        for d, l in self.links.items():
            if not (l.ever and l.link_ok(self.now)):
                problems.append(f"немає зв'язку з пристроєм {d}")
            elif l.status and l.status.flags & F_OVERLOAD:
                problems.append("двигун стрічки усе ще перевантажений")
        if not self.op.belt_cleared:
            problems.append("оператор не підтвердив очищення стрічки")
        if problems:
            self.rejected += 1
            self.L("ОПЕРАТОР", "«Продовжити» відхилено", "; ".join(problems))
            return
        self.L("ОПЕРАТОР", "«Продовжити»: стрічку очищено, аварію усунуто")
        self.op.belt_cleared = False
        self.recovering, self.started = True, True
        self.go(State.INIT, "відновлення після аварії")

    def _finish_job(self):
        self.job_complete = True
        s = {d: l.status for d, l in self.links.items()}
        self.L("ПОДІЯ", f"Завдання виконано: надійшло {s['conveyor'].v[2]} з {s['conveyor'].v[3]}, "
               f"відсортовано {s['manipulator'].v[1]}, правильно {s['manipulator'].v[2]}, "
               f"пропущено {s['conveyor'].v[4]}, знято оператором {s['manipulator'].v[3]}")
        self.go(State.INIT, "завдання виконано")

    # ---------- знімок для GUI ----------
    def snapshot(self):
        s = {d: l.status for d, l in self.links.items()}
        devs = {}
        for d, l in self.links.items():
            st = l.status
            devs[d] = {"state": state_name(d, st.state) if st else "—",
                       "link": "ok" if (l.ever and l.link_ok(self.now)) else ("ВТРАТА" if l.ever else "очікування"),
                       "op": l.pending.op if l.pending else 0}
        cv, mn = s["conveyor"], s["manipulator"]
        return {"state": self.state.name if self.state else "-", "fault": self.fault_cause if self.state == State.FAULT else "",
                "devs": devs, "done": self.job_complete,
                "spawned": cv.v[2] if cv else 0, "total": cv.v[3] if cv else 0,
                "delivered": mn.v[1] if mn else 0, "correct": mn.v[2] if mn else 0,
                "missed": cv.v[4] if cv else 0, "cleared": mn.v[3] if mn else 0,
                "current": cv.v[0] / 100.0 if cv else 0.0, "t": self.now - self.t0}


# ============================================================
#  СЦЕНАРІЇ ІНТЕГРАЦІЙНОГО ТЕСТУВАННЯ
# ============================================================
SCENARIOS = {
    "normal":      "штатний цикл без аварій",
    "device_fail": "відмова пристрою: робот не реагує на команду -> тайм-аут, відновлення",
    "overload":    "відмова пристрою: перевантаження двигуна стрічки -> аварія, відновлення",
    "missed":      "відмова датчика: деталь доїхала до кінця стрічки -> аварія, відновлення",
    "comm_loss":   "втрата зв'язку зі стрічкою на 5 с -> аварія, автономна зупинка, відновлення",
    "ack_delay":   "затримка підтвердження -> повтор команди, повтор не виконується вдруге",
    "duplicate":   "кожну команду надіслано двічі -> виконується лише один раз",
}
EXPECT_FAULT = {"normal": False, "device_fail": True, "overload": True, "missed": True,
                "comm_loss": True, "ack_delay": False, "duplicate": False}
EXPECT_DUP = {"ack_delay", "duplicate"}


def scenario_step(name, op, co):
    s, t = op.sc, co.now - co.t0
    if name == "duplicate":
        op.dup_mode = True
    if not s.get("start") and t >= 0.5:
        s["start"] = True
        op.press_start()
    if name == "device_fail" and not s.get("inj"):
        s["inj"] = True
        op.inj_robot_hang = True
    if name == "missed" and not s.get("inj"):
        s["inj"] = True
        op.inj_sensor_blind = True
    if name == "ack_delay" and not s.get("inj"):
        s["inj"] = True
        op.ack_delay_req = True
    if name == "overload" and not s.get("inj") and co.state == State.TRANSPORTING and co.in_state() > 1.0:
        s["inj"] = True
        op.inj_overload = True
    if name == "comm_loss" and not s.get("inj") and co.state == State.TRANSPORTING and co.in_state() > 1.0:
        s["inj"] = True
        op.mute_req = True
    # дії оператора після аварії: спроба без очищення (відхиляється) -> усунення причини -> очищення -> «Продовжити»
    if co.state == State.FAULT:
        s.setdefault("ft", co.now)
        d = co.now - s["ft"]
        if d >= 1.0 and not s.get("try1"):
            s["try1"] = True
            op.press_continue()
        if d >= 2.0 and not s.get("fix"):
            s["fix"] = True
            op.inj_overload = op.inj_robot_hang = op.inj_sensor_blind = False
        if d >= 4.0 and not s.get("clr"):
            s["clr"] = True
            op.belt_cleared = True
        if d >= 5.0 and not s.get("try2"):
            s["try2"] = True
            op.press_continue()
    else:
        for k in ("ft", "try1", "fix", "clr", "try2"):
            s.pop(k, None)


def verdict(name, co):
    notes, ok = [], co.job_complete
    sts = {d: l.status for d, l in co.links.items()}
    cv, mn = sts["conveyor"], sts["manipulator"]
    if not ok:
        notes.append("завдання не завершено")
    total, delivered, correct, cleared, missed = cv.v[3], mn.v[1], mn.v[2], mn.v[3], cv.v[4]
    if delivered + missed + cleared != total:
        ok = False
        notes.append(f"баланс деталей не зійшовся: {delivered}+{missed}+{cleared} з {total}")
    if delivered != correct:
        notes.append(f"помилкових кошиків: {delivered - correct}")
    if mn.v[4] != co.sorted_ok or delivered != co.sorted_ok:
        ok = False
        notes.append(f"подвійне виконання: циклів робота {mn.v[4]}, підтверджено операцій {co.sorted_ok}")
    if EXPECT_FAULT[name] != (co.faults > 0):
        ok = False
        notes.append("аварія не виникла" if EXPECT_FAULT[name] else "були аварії у штатному сценарії")
    if EXPECT_FAULT[name] and co.rejected == 0:
        ok = False
        notes.append("спроба відновлення без очищення не була відхилена")
    dups = sum(st.dups for st in sts.values())
    if name in EXPECT_DUP and dups == 0:
        ok = False
        notes.append("повторних команд не зафіксовано")
    if name not in EXPECT_DUP and dups != 0:
        ok = False
        notes.append(f"несподівані повтори команд: {dups}")
    if name == "ack_delay" and co.retx == 0:
        ok = False
        notes.append("повторного передавання не було")
    if name == "comm_loss" and not co.autonomous_stop_seen:
        ok = False
        notes.append("автономна зупинка пристрою не зафіксована")
    return ok, notes


def print_summary(co, log):
    sts = {d: l.status for d, l in co.links.items()}
    cv, mn = sts["conveyor"], sts["manipulator"]
    log.log("ПІДСУМОК", f"Деталей: {cv.v[2]}/{cv.v[3]}; відсортовано {mn.v[1]} (правильно {mn.v[2]}); "
            f"пропущено {cv.v[4]}; знято оператором {mn.v[3]}; аварій {co.faults}; "
            f"відхилених спроб відновлення {co.rejected}")
    log.log("ПІДСУМОК", f"Повторних передавань: {co.retx}; навмисних дублікатів: {co.dup_sent}; "
            f"повторів, проігнорованих пристроями: " + ", ".join(f"{d}={s.dups}" for d, s in sts.items()))
    log.log("ПІДСУМОК", "Прийнято операцій: " + ", ".join(f"{d}={s.accepted}" for d, s in sts.items())
            + f"; циклів робота {mn.v[4]}")
    for t, c in co.fault_log:
        log.log("ПІДСУМОК", f"Аварія на {t:.1f} с: {c}")


# ============================================================
#  ЦИКЛ КООРДИНАТОРА
# ============================================================
def coordinator_loop(co, op, scenario, snap, max_time=300.0):
    co.start()
    try:
        while not op.stop_req:
            t0 = time.time()
            co.update()
            snap[0] = co.snapshot()
            if scenario:
                scenario_step(scenario, op, co)
                if co.job_complete or co.now - co.t0 > max_time:
                    break
            time.sleep(max(0.0, co.cfg.POLL - (time.time() - t0)))
    finally:
        co.shutdown()


# ============================================================
#  ВІКНО ОПЕРАТОРА
# ============================================================
def run_gui(co, op, log):
    import tkinter as tk
    snap = [None]
    th = threading.Thread(target=coordinator_loop, args=(co, op, None, snap), daemon=True)
    th.start()
    root = tk.Tk()
    root.title("Комплекс сортування: панель оператора (координатор)")
    colors = {"INIT": "#9aa0a6", "TRANSPORTING": "#1a73e8", "ANALYZING": "#f9ab00",
              "SORTING": "#188038", "FAULT": "#d93025", "-": "#9aa0a6"}
    lbl = tk.Label(root, text="-", font=("Arial", 20, "bold"), fg="white", bg="#9aa0a6", width=22, pady=8)
    lbl.pack(fill="x", padx=8, pady=8)
    info = tk.Label(root, text="", justify="left", font=("Consolas", 10))
    info.pack(anchor="w", padx=8)
    fr = tk.Frame(root)
    fr.pack(pady=6)
    tk.Button(fr, text="Пуск", width=14, command=op.press_start).grid(row=0, column=0, padx=4)
    tk.Button(fr, text="Продовжити", width=14, command=op.press_continue).grid(row=0, column=1, padx=4)
    v_clr = tk.BooleanVar()
    tk.Checkbutton(root, text="Стрічку очищено оператором", variable=v_clr).pack(anchor="w", padx=8)
    tk.Label(root, text="Тестові впливи (імітація відмов):", font=("Arial", 9, "bold")).pack(anchor="w", padx=8, pady=(8, 0))
    v_ov, v_hg, v_bl, v_dup = tk.BooleanVar(), tk.BooleanVar(), tk.BooleanVar(), tk.BooleanVar()
    for txt, v in (("Перевантаження двигуна стрічки", v_ov), ("Робот не реагує на команди", v_hg),
                   ("Датчик присутності не працює", v_bl), ("Надсилати кожну команду двічі (дублікати)", v_dup)):
        tk.Checkbutton(root, text=txt, variable=v).pack(anchor="w", padx=8)
    fr2 = tk.Frame(root)
    fr2.pack(pady=4)
    tk.Button(fr2, text="Затримати підтвердження (2.5 с)", command=lambda: setattr(op, "ack_delay_req", True)).grid(row=0, column=0, padx=4)
    tk.Button(fr2, text="Обрив зв'язку зі стрічкою (5 с)", command=lambda: setattr(op, "mute_req", True)).grid(row=0, column=1, padx=4)
    txt = tk.Text(root, width=110, height=16, font=("Consolas", 9), state="disabled")
    txt.pack(padx=8, pady=8, fill="both", expand=True)
    shown = [0]

    def tick():
        op.belt_cleared = v_clr.get()
        op.inj_overload, op.inj_robot_hang, op.inj_sensor_blind = v_ov.get(), v_hg.get(), v_bl.get()
        op.dup_mode = v_dup.get()
        s = snap[0]
        if s:
            lbl.config(text=s["state"], bg=colors.get(s["state"], "#9aa0a6"))
            dv = "\n".join(f"  {d:<12} стан {v['state']:<10} зв'язок {v['link']:<10} остання операція №{v['op']}"
                           for d, v in s["devs"].items())
            info.config(text=(f"Струм двигуна стрічки: {s['current']:.1f} А\n{dv}\n"
                              f"Деталей: {s['spawned']}/{s['total']}   відсортовано {s['delivered']} "
                              f"(правильно {s['correct']})   пропущено {s['missed']}\n"
                              + (f"АВАРІЯ: {s['fault']}" if s["fault"] else "")
                              + ("ЗАВДАННЯ ВИКОНАНО" if s["done"] else "")))
        lines = list(log.tail)
        if len(lines) != shown[0]:
            shown[0] = len(lines)
            txt.config(state="normal")
            txt.delete("1.0", "end")
            txt.insert("end", "\n".join(lines[-200:]))
            txt.see("end")
            txt.config(state="disabled")
        root.after(150, tick)

    def on_close():
        op.stop_req = True
        th.join(timeout=5)
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    tick()
    root.mainloop()


# ============================================================
def parse_addr(s, default_port):
    host, _, port = s.partition(":")
    return host or "localhost", int(port) if port else default_port


def main(argv=None):
    ap = argparse.ArgumentParser(description="Координатор комплексу сортування")
    ap.add_argument("--test", choices=list(SCENARIOS), help="автоматичний сценарій без вікна")
    ap.add_argument("--log-dir", default="logs")
    for d in DEVICES:
        ap.add_argument(f"--{d}", default=f"localhost:{DEFAULT_PORTS[d]}",
                        help=f"адреса контролера {d} (хост:порт)")
    args = ap.parse_args(argv)
    os.makedirs(args.log_dir, exist_ok=True)
    addrs = {d: parse_addr(getattr(args, d), DEFAULT_PORTS[d]) for d in DEVICES}
    log = EventLogger("coordinator", os.path.join(args.log_dir, "coordinator.csv"))
    op = Operator()
    co = Coordinator(log, op, addrs)
    if args.test:
        log.log("ПОДІЯ", f"Сценарій «{args.test}»: {SCENARIOS[args.test]}")
        coordinator_loop(co, op, args.test, [None])
        if not all(l.status for l in co.links.values()):
            log.log("ПІДСУМОК", "ТЕСТ НЕ ПРОЙДЕНО: немає зв'язку з контролерами")
            log.close()
            return 2
        print_summary(co, log)
        ok, notes = verdict(args.test, co)
        log.log("ПІДСУМОК", ("ТЕСТ ПРОЙДЕНО" if ok else "ТЕСТ НЕ ПРОЙДЕНО") + (": " + "; ".join(notes) if notes else ""))
        log.close()
        return 0 if ok else 1
    run_gui(co, op, log)
    if all(l.status for l in co.links.values()):
        print_summary(co, log)
    log.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
