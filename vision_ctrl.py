# -*- coding: utf-8 -*-
"""
ДОДАТКОВА ПІДСИСТЕМА: технічний зір (камера над стрічкою) — окремий процес зі своїм Modbus-інтерфейсом.

Вхід  (від координатора): команда ANALYZE (1) з номером операції.
Вихід (до координатора):  колір деталі в зоні захоплення (регістр V1: 1 червоний, 2 зелений, 3 синій),
                          кількість кадрів, що проголосували (V2), номер останнього кадру (V3),
                          результат RESULT і код помилки ERROR.
Алгоритм: камера в сцені розпізнає колір на кожному кадрі; підсистема збирає VOTES кадрів поспіль
і видає рішення лише якщо всі вони однакові та ненульові (як у практикумі №5, але тепер це окремий
компонент, а не частина FSM).
Власні сигнали сцени: пише cmd_analyze; читає vis_seq, vis_color, belt_speed.
Стани: INIT -> READY -> ANALYZING -> READY;  будь-який -> FAULT -> (RESET) -> READY.
"""
from local_base import LocalController, parse_args
from plant_io import Inputs
from protocol import (VsState, C_ANALYZE, RES_OK, E_INTERLOCK, E_CLASSIFY_TIMEOUT, E_NOT_RECOGNIZED,
                      E_UNSTABLE, COLOR_NAME)


class Vision(LocalController):
    DEV = "vision"
    INIT_STATE, READY_STATE, FAULT_STATE = VsState.INIT, VsState.READY, VsState.FAULT

    def __init__(self, args):
        super().__init__(args)
        self.votes, self.vis_seen, self.color = [], 0, 0

    def read_plant(self):
        g, i = self.io, Inputs()
        i.t = g.time()
        i.alive, i.ready = g.gi("plant_alive"), g.gi("plant_ready")
        i.vis_seq, i.vis_color, i.belt_speed = g.gi("vis_seq"), g.gi("vis_color"), g.gf("belt_speed")
        return i

    def values(self):
        return [self.color, len(self.votes), self.i.vis_seq, 0, 0, 0]

    def on_enter(self, s):
        if s != VsState.ANALYZING:
            self.cmd("cmd_analyze", 0)

    def validate(self, code, p1, p2):
        if code != C_ANALYZE:
            return False, 3
        if self.state != VsState.READY:
            return False, 16
        if self.i.belt_speed > 0.001:                            # аналізувати можна лише при нерухомій стрічці
            return False, E_INTERLOCK
        return True, 0

    def start(self, code, p1, p2):
        self.votes, self.color = [], 0
        self.vis_seen = self.i.vis_seq
        self.go(VsState.ANALYZING, "команда ANALYZE")
        self.cmd("cmd_analyze", 1, "Запит аналізу зображення")

    def do_reset(self):
        self.error = 0
        self.go(VsState.READY, "скидання за командою координатора")
        self.finish(RES_OK)

    def device_step(self, i):
        c = self.cfg
        if self.state == VsState.INIT:
            if i.ready == 1 and i.alive:
                self.go(VsState.READY, "камера готова")
        elif self.state == VsState.ANALYZING:
            if self.in_state() > c.T_CLASSIFY:
                self.L("ТАЙМ-АУТ", f"Аналіз зображення не завершено за {c.T_CLASSIFY:.0f} с")
                return self.fault(E_CLASSIFY_TIMEOUT, "камера не дала стабільного результату")
            if self.in_state() < c.T_SETTLE or i.belt_speed > 0.001:
                return
            if i.vis_seq != self.vis_seen:
                self.vis_seen = i.vis_seq
                self.votes.append(i.vis_color)
                if len(self.votes) < c.VOTES:
                    return
                votes = self.votes[-c.VOTES:]
                if len(set(votes)) == 1 and votes[0] != 0:
                    self.color = votes[0]
                    self.L("КАМЕРА", f"Розпізнано колір: {COLOR_NAME[self.color]} ({c.VOTES} однакових кадрів)")
                    self.cmd("cmd_analyze", 0)
                    self.go(VsState.READY, f"колір {COLOR_NAME[self.color]}")
                    self.finish(RES_OK)
                elif set(votes) == {0}:
                    self.fault(E_NOT_RECOGNIZED, "камера не бачить деталі, хоча датчик її фіксував")
                else:
                    self.fault(E_UNSTABLE, f"результати камери: {votes}")


if __name__ == "__main__":
    Vision(parse_args("vision")).run()
