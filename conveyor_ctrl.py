# -*- coding: utf-8 -*-
"""
Локальний контролер КОНВЕЄРА (стрічка + оптичний датчик присутності + контроль струму двигуна).

Власні сигнали сцени:  пише cmd_belt, inj_overload, inj_sensor_blind
                       читає presence, belt_speed, motor_current, belt_count, spawned, total_cubes,
                             missed, robot_phase, plant_alive, plant_ready
Команди координатора:  ADVANCE (1) — їхати до появи деталі в зоні захоплення і зупинитись;
                       HALT (90), RESET (91).
Стани:  INIT -> STOPPED -> ADVANCING -> STOPPING -> STOPPED ...;   будь-який стан -> FAULT -> (RESET) -> STOPPED
Локальні аварії: перевантаження двигуна, пропущена деталь, тайм-аут очікування деталі,
                 втрата об'єкта керування, втрата координатора.
"""
from local_base import LocalController, parse_args
from plant_io import Inputs
from protocol import (Cfg, CvState, C_ADVANCE, RES_OK, RES_NO_PARTS, E_INTERLOCK, E_NOT_READY,
                      E_OVERLOAD, E_PART_MISSED, E_ARRIVE_TIMEOUT, F_OVERLOAD, F_PART, INJ_OVERLOAD,
                      INJ_BLIND)


class Conveyor(LocalController):
    DEV = "conveyor"
    INIT_STATE, READY_STATE, FAULT_STATE = CvState.INIT, CvState.STOPPED, CvState.FAULT
    INJ_MAP = {INJ_OVERLOAD: ("inj_overload", "імітація перевантаження двигуна стрічки"),
               INJ_BLIND: ("inj_sensor_blind", "імітація несправності датчика присутності")}

    def __init__(self, args):
        super().__init__(args)
        self._cur_cnt = self._pres_cnt = self._pres_prev = 0
        self.overload = False
        self._missed_seen = 0

    def read_plant(self):
        g, i = self.io, Inputs()
        i.t = g.time()
        i.alive, i.ready = g.gi("plant_alive"), g.gi("plant_ready")
        i.presence, i.belt_speed, i.current = g.gi("presence"), g.gf("belt_speed"), g.gf("motor_current")
        i.belt_count, i.spawned, i.total = g.gi("belt_count"), g.gi("spawned"), g.gi("total_cubes")
        i.missed, i.phase = g.gi("missed"), g.gi("robot_phase")
        return i

    def values(self):
        i = self.i
        return [i.current * 100, i.belt_count, i.spawned, i.total, i.missed, i.belt_speed * 1000]

    def extra_flags(self):
        return (F_OVERLOAD if self.overload else 0) | (F_PART if self.i and self.i.presence else 0)

    def on_enter(self, s):
        if s == CvState.ADVANCING:
            self._pres_cnt = 0
            self.cmd("cmd_belt", 1, "Стрічку увімкнено")
        elif s == CvState.FAULT:
            self.cmd("cmd_belt", 0, "Стрічку зупинено (аварія)")
        else:
            self.cmd("cmd_belt", 0, "Стрічку вимкнено")

    def on_reinit(self, i):
        self._missed_seen = i.missed

    # ---------- команди ----------
    def can_reset(self):
        if self.overload:
            return False, E_NOT_READY                            # двигун усе ще перевантажений
        return True, 0

    def validate(self, code, p1, p2):
        if code != C_ADVANCE:
            return False, 3
        if self.state != CvState.STOPPED:
            return False, E_NOT_READY
        if self.i.phase in (1, 2):                               # інтерлок спільної зони: робот біля стрічки
            return False, E_INTERLOCK
        return True, 0

    def start(self, code, p1, p2):
        self.go(CvState.ADVANCING, "команда ADVANCE")

    def do_reset(self):
        self.error = 0
        self._cur_cnt = 0
        self._missed_seen = self.i.missed
        self.go(CvState.STOPPED, "скидання за командою координатора")
        self.finish(RES_OK)

    # ---------- автомат ----------
    def device_step(self, i):
        c = self.cfg
        self._cur_cnt = self._cur_cnt + 1 if i.current > c.CURRENT_LIMIT else 0
        self.overload = self._cur_cnt >= c.CURRENT_TICKS
        self._pres_cnt = self._pres_cnt + 1 if i.presence else 0
        if i.presence and not self._pres_prev:
            self.L("СЕНСОР", "Датчик присутності: деталь у зоні захоплення")
        self._pres_prev = i.presence
        if self.state not in (CvState.INIT, CvState.FAULT):
            if self.overload:
                self.fault(E_OVERLOAD, f"струм двигуна {i.current:.1f} А > {c.CURRENT_LIMIT:.1f} А")
            elif i.missed > self._missed_seen:
                self._missed_seen = i.missed
                self.fault(E_PART_MISSED, "деталь дійшла до кінця стрічки, її не забрав робот")
        else:
            self._missed_seen = i.missed

        if self.state == CvState.INIT:
            if i.ready == 1 and i.alive and i.current <= c.CURRENT_LIMIT:
                self._missed_seen = i.missed
                self.L("СТАН", "Перевірка пройшла: сцена готова, струм у нормі")
                self.go(CvState.STOPPED, "системи в нормі")
        elif self.state == CvState.ADVANCING:
            if self._pres_cnt >= c.PRESENT_TICKS:
                self.go(CvState.STOPPING, "датчик присутності підтвердив деталь")
            elif i.total > 0 and i.spawned >= i.total and i.belt_count == 0:
                self.L("ПОДІЯ", f"Деталей на стрічці більше немає: надійшло {i.spawned} з {i.total}")
                self.go(CvState.STOPPED, "усі деталі оброблено")
                self.finish(RES_NO_PARTS)
            elif self.in_state() > c.T_ARRIVE:
                self.L("ТАЙМ-АУТ", f"Деталь не надійшла за {c.T_ARRIVE:.0f} с")
                self.fault(E_ARRIVE_TIMEOUT, f"за {c.T_ARRIVE:.0f} с деталь не з'явилась у зоні захоплення")
        elif self.state == CvState.STOPPING:
            if self.in_state() >= c.T_SETTLE and i.belt_speed < 0.001:
                self.go(CvState.STOPPED, "стрічка зупинилась, деталь у зоні захоплення")
                self.finish(RES_OK)


if __name__ == "__main__":
    Conveyor(parse_args("conveyor")).run()
