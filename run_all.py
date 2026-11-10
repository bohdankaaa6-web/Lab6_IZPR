# -*- coding: utf-8 -*-
"""
Запуск усього комплексу однією командою: три контролери + (за потреби) імітатор сцени + координатор.
Контролери та координатор працюють В ОКРЕМИХ ПРОЦЕСАХ і спілкуються лише через Modbus TCP.

  python run_all.py                       CoppeliaSim (сцену відкрито), вікно оператора
  python run_all.py --start-sim           те саме, але симуляцію запустить сам скрипт
  python run_all.py --backend mock        без CoppeliaSim (імітатор сцени), вікно оператора
  python run_all.py --backend mock --test normal --speed 5    автотест
Для запуску на кількох комп'ютерах стартуйте компоненти окремо (див. README.md).
"""
import argparse
import os
import subprocess
import sys
import threading
import time

import coordinator
import merge_logs
from protocol import DEFAULT_PORTS

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--backend", choices=["coppelia", "mock"], default="coppelia")
    ap.add_argument("--test", choices=list(coordinator.SCENARIOS))
    ap.add_argument("--speed", type=float, default=4.0, help="прискорення імітатора (лише mock)")
    ap.add_argument("--start-sim", action="store_true", help="запустити симуляцію CoppeliaSim")
    ap.add_argument("--plant-port", type=int, default=23000)
    ap.add_argument("--log-dir", default="logs")
    a = ap.parse_args()
    os.makedirs(a.log_dir, exist_ok=True)

    if a.backend == "mock":
        import mock_plant
        plant = mock_plant.MockPlant(a.speed)
        plant.start_thread()
        srv = mock_plant.make_server(plant, "127.0.0.1", a.plant_port)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        print(f"[run_all] імітатор сцени запущено (x{a.speed})", flush=True)
    elif a.start_sim:
        from coppeliasim_zmqremoteapi_client import RemoteAPIClient
        sim = RemoteAPIClient().require("sim")
        if sim.getSimulationState() == sim.simulation_stopped:
            sim.startSimulation()
        print("[run_all] симуляцію CoppeliaSim запущено", flush=True)

    procs = []
    for mod in ("conveyor_ctrl", "manipulator_ctrl", "vision_ctrl"):
        dev = {"conveyor_ctrl": "conveyor", "manipulator_ctrl": "manipulator", "vision_ctrl": "vision"}[mod]
        cmd = [sys.executable, os.path.join(HERE, mod + ".py"), "--backend", a.backend,
               "--port", str(DEFAULT_PORTS[dev]), "--plant-port", str(a.plant_port), "--log-dir", a.log_dir]
        procs.append(subprocess.Popen(cmd))
    time.sleep(1.0)
    cargs = ["--log-dir", a.log_dir] + (["--test", a.test] if a.test else [])
    try:
        code = coordinator.main(cargs)
    finally:
        time.sleep(0.5)
        for p in procs:
            p.terminate()
        for p in procs:
            try:
                p.wait(timeout=3)
            except subprocess.TimeoutExpired:
                p.kill()
        path, n = merge_logs.merge(a.log_dir)
        print(f"[run_all] об'єднаний журнал: {path} ({n} записів)")
    return code


if __name__ == "__main__":
    sys.exit(main())
