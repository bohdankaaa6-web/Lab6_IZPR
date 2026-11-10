# -*- coding: utf-8 -*-
"""
Локальний контролер МАНІПУЛЯТОРА UR5 (присоска, цикл «взяти зі стрічки — покласти в кошик»).

Власні сигнали сцени:  пише cmd_halt, cmd_pick_color, cmd_pick_seq, cmd_reset_seq, inj_robot_hang
                       читає robot_home, robot_phase, robot_fault, done_count, last_result,
                             delivered, correct, cleared, plant_alive, plant_ready
Команди координатора:  PICK_AND_SORT (1), P1 = колір 1/2/3;  HALT (90);  RESET (91).
Стани: INIT -> READY -> STARTING -> EXECUTING -> READY ...;
       будь-який -> FAULT -> (RESET) -> RESETTING -> READY
ВАЖЛИВО (ідемпотентність): cmd_pick_seq збільшується лише при ПРИЙНЯТТІ нової операції. Повторна команда з
тим самим номером операції відсікається базовим класом і цикл вдруге не запускається.
RESET також скидає сцену (прибирає деталі зі стрічки і з присоски, робот їде в початкову позицію).
"""
from local_base import LocalController, parse_args
from plant_io import Inputs
from protocol import (MnState, C_PICK, RES_OK, RES_WRONG_BIN, E_NOT_READY, E_BAD_PARAM, E_ROBOT_FAULT,
                      E_ROBOT_NO_START, E_ROBOT_TIMEOUT, E_INIT_TIMEOUT, F_HOME, INJ_ROBOT_HANG, COLOR_NAME)


class Manipulator(LocalController):
    DEV = "manipulator"
    INIT_STATE, READY_STATE, FAULT_STATE = MnState.INIT, MnState.READY, MnState.FAULT
    INJ_MAP = {INJ_ROBOT_HANG: ("inj_robot_hang", "імітація зависання робота")}

    def __init__(self, args):
        super().__init__(args)
        self.pick_seq = self.reset_seq = 0
        self.done_seen = 0
        self.rb_started = False
        self.color = 0

    def read_plant(self):
        g, i = self.io, Inputs()
        i.t = g.time()
        i.alive, i.ready = g.gi("plant_alive"), g.gi("plant_ready")
        i.home, i.phase, i.robot_fault = g.gi("robot_home"), g.gi("robot_phase"), g.gi("robot_fault")
        i.done_count, i.last_result = g.gi("done_count"), g.gi("last_result")
        i.delivered, i.correct, i.cleared = g.gi("delivered"), g.gi("correct"), g.gi("cleared")
        return i

    def values(self):
        i = self.i
        return [i.phase, i.delivered, i.correct, i.cleared, i.done_count, 1 if i.last_result == 1 else (2 if i.last_result == -1 else 0)]

    def extra_flags(self):
        return F_HOME if self.i and self.i.home == 1 else 0

    def on_enter(self, s):
        if s == MnState.FAULT:
            self.cmd("cmd_halt", 1, "Аварійна зупинка об'єкта керування (робот і стрічка)")

    def _sync_counters(self, i):
        self.done_seen = i.done_count
        self.pick_seq = self.io.gi("cmd_pick_seq")               # продовжуємо нумерацію сцени
        self.reset_seq = self.io.gi("cmd_reset_seq")

    def on_reinit(self, i):
        self._sync_counters(i)

    # ---------- команди ----------
    def validate(self, code, p1, p2):
        if code != C_PICK:
            return False, 3
        if p1 not in (1, 2, 3):
            return False, E_BAD_PARAM
        if self.state != MnState.READY or self.i.home != 1:
            return False, E_NOT_READY
        return True, 0

    def start(self, code, p1, p2):
        self.color = p1
        self.pick_seq += 1
        self.io.set("cmd_pick_color", p1)
        self.io.set("cmd_pick_seq", self.pick_seq)
        self.rb_started = False
        self.L("КОМАНДА", f"Сортувати деталь: колір {COLOR_NAME[p1]} (цикл сцени №{self.pick_seq})")
        self.go(MnState.STARTING, f"колір {COLOR_NAME[p1]}")

    def do_reset(self):
        self.error = 0
        self.cmd("cmd_halt", 0, "Аварійну зупинку знято")
        self.reset_seq += 1
        self.io.set("cmd_reset_seq", self.reset_seq)
        self.L("КОМАНДА", "Скинути сцену: прибрати деталі, робот у початкову позицію")
        self.go(MnState.RESETTING, "скидання за командою координатора")

    # ---------- автомат ----------
    def device_step(self, i):
        c = self.cfg
        s = self.state
        if s == MnState.INIT:
            if i.ready == 1 and i.alive and i.home == 1 and self.in_state() >= 0.3:
                self._sync_counters(i)
                self.L("СТАН", "Перевірка пройшла: сцена готова, робот у початковій позиції")
                self.go(MnState.READY, "системи в нормі")
            elif self.in_state() > c.T_INIT:
                self.fault(E_INIT_TIMEOUT, f"системи не готові за {c.T_INIT:.0f} с")
        elif s == MnState.STARTING:
            if i.robot_fault:
                self.fault(E_ROBOT_FAULT, "робот повідомив про неможливість виконати цикл")
            elif i.phase > 0:
                self.L("РОБОТ", "Робот розпочав цикл")
                self.go(MnState.EXECUTING, "робот розпочав цикл")
            elif self.in_state() > c.T_START:
                self.L("ТАЙМ-АУТ", f"Робот не розпочав цикл за {c.T_START:.0f} с")
                self.fault(E_ROBOT_NO_START, "робот не реагує на команду")
        elif s == MnState.EXECUTING:
            if i.robot_fault:
                self.fault(E_ROBOT_FAULT, "робот повідомив про неможливість виконати цикл")
            elif i.done_count > self.done_seen:
                self.done_seen = i.done_count
                wrong = i.last_result == -1
                self.L("ПОПЕРЕДЖЕННЯ" if wrong else "РОБОТ",
                       "Деталь потрапила в кошик іншого кольору (помилка класифікації)" if wrong
                       else "Деталь у відповідному кошику")
                self.go(MnState.READY, "цикл сортування завершено")
                self.finish(RES_WRONG_BIN if wrong else RES_OK)
            elif self.in_state() > c.T_SORT:
                self.L("ТАЙМ-АУТ", f"Цикл сортування не завершено за {c.T_SORT:.0f} с")
                self.fault(E_ROBOT_TIMEOUT, "робот не завершив цикл")
        elif s == MnState.RESETTING:
            if i.home == 1 and self.in_state() >= c.T_SETTLE:
                self._sync_counters(i)
                self.L("РОБОТ", "Робот у початковій позиції, сцену очищено")
                self.go(MnState.READY, "скидання завершено")
                self.finish(RES_OK)
            elif self.in_state() > c.T_INIT:
                self.fault(E_INIT_TIMEOUT, "робот не повернувся в початкову позицію")


if __name__ == "__main__":
    Manipulator(parse_args("manipulator")).run()
