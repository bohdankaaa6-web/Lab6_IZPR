# -*- coding: utf-8 -*-
"""
Мінімальна реалізація Modbus TCP (кадр MBAP + функції 0x03, 0x06, 0x10).
Лише стандартна бібліотека Python, тому нічого встановлювати не потрібно.

Карта регістрів (holding registers) та формат кадрів відповідають специфікації Modbus,
тому ті самі регістри можна читати/писати будь-яким стороннім клієнтом
(pymodbus, ModbusPoll, QModMaster) — це зручно для демонстрації й діагностики.

ModbusServer  — «пристрій» (slave): контролер маніпулятора, стрічки, підсистема зору.
ModbusClient  — «майстер» (master): координатор комплексу.
"""
import socket
import struct
import threading
import time

FC_READ_HOLDING = 0x03
FC_WRITE_SINGLE = 0x06
FC_WRITE_MULTIPLE = 0x10

EX_ILLEGAL_FUNCTION = 1
EX_ILLEGAL_ADDRESS = 2
EX_ILLEGAL_VALUE = 3


class ModbusError(Exception):
    """Базова помилка обміну."""


class ModbusTimeout(ModbusError):
    """Немає відповіді за відведений час (обрив лінії, зависання пристрою)."""


class ModbusException(ModbusError):
    """Пристрій відповів кодом винятку Modbus."""

    def __init__(self, fc, code):
        super().__init__(f"виняток Modbus: функція 0x{fc:02X}, код {code}")
        self.fc, self.code = fc, code


def _recv_exact(sock, n, patient=False, idle_ok=False):
    """Прочитати рівно n байт.
    patient=True  — на тайм-ауті сокета продовжувати чекати (сервер усередині кадру);
    idle_ok=True  — тайм-аут до першого байта означає «клієнт просто мовчить» (повторно кидаємо його);
    інакше (клієнт) будь-який тайм-аут — це помилка обміну."""
    buf = b""
    while len(buf) < n:
        try:
            chunk = sock.recv(n - len(buf))
        except socket.timeout:
            if patient or (idle_ok and buf):
                continue
            raise
        if not chunk:
            raise ConnectionError("з'єднання закрито")
        buf += chunk
    return buf


# ============================================================
#  СЕРВЕР (пристрій)
# ============================================================
class ModbusServer:
    def __init__(self, host="0.0.0.0", port=5020, unit=1, size=64, write_hook=None):
        self.host, self.port, self.unit = host, port, unit
        self.regs = [0] * size
        self.lock = threading.Lock()
        self.write_hook = write_hook          # write_hook(addr, count) — викликається під замком
        self.mute_until = 0.0                 # тестовий режим: до цього моменту не відповідати
        self.requests = 0
        self._run = False
        self._srv = None

    # ---------- доступ до регістрів з боку контролера ----------
    def get(self, addr):
        with self.lock:
            return self.regs[addr]

    def set(self, addr, value):
        with self.lock:
            self.regs[addr] = int(value) & 0xFFFF

    def get_block(self, addr, count):
        with self.lock:
            return list(self.regs[addr:addr + count])

    def set_block(self, addr, values):
        with self.lock:
            for k, v in enumerate(values):
                self.regs[addr + k] = int(v) & 0xFFFF

    def mute(self, seconds):
        """Імітація обриву зв'язку: запити надходять, але відповіді немає."""
        self.mute_until = time.time() + seconds

    # ---------- мережа ----------
    def start(self):
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind((self.host, self.port))
        self._srv.listen(8)
        self._srv.settimeout(0.5)
        self._run = True
        threading.Thread(target=self._accept_loop, daemon=True).start()

    def stop(self):
        self._run = False
        try:
            self._srv.close()
        except OSError:
            pass

    def _accept_loop(self):
        while self._run:
            try:
                conn, _ = self._srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            conn.settimeout(0.5)
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            threading.Thread(target=self._serve, args=(conn,), daemon=True).start()

    def _serve(self, conn):
        try:
            while self._run:
                try:
                    hdr = _recv_exact(conn, 7, idle_ok=True)
                except socket.timeout:
                    continue
                tid, pid, length, unit = struct.unpack(">HHHB", hdr)
                pdu = _recv_exact(conn, length - 1, patient=True)
                self.requests += 1
                if time.time() < self.mute_until or unit != self.unit:
                    continue                                   # відповіді немає
                resp = self._process(pdu)
                conn.sendall(struct.pack(">HHHB", tid, 0, len(resp) + 1, unit) + resp)
        except (ConnectionError, OSError):
            pass
        finally:
            try:
                conn.close()
            except OSError:
                pass

    def _process(self, pdu):
        fc = pdu[0]
        n = len(self.regs)
        try:
            if fc == FC_READ_HOLDING:
                addr, cnt = struct.unpack(">HH", pdu[1:5])
                if not 1 <= cnt <= 125:
                    return bytes([fc | 0x80, EX_ILLEGAL_VALUE])
                if addr + cnt > n:
                    return bytes([fc | 0x80, EX_ILLEGAL_ADDRESS])
                vals = self.get_block(addr, cnt)
                return bytes([fc, cnt * 2]) + struct.pack(f">{cnt}H", *vals)
            if fc == FC_WRITE_SINGLE:
                addr, val = struct.unpack(">HH", pdu[1:5])
                if addr >= n:
                    return bytes([fc | 0x80, EX_ILLEGAL_ADDRESS])
                with self.lock:
                    self.regs[addr] = val
                    if self.write_hook:
                        self.write_hook(addr, 1)
                return pdu[:5]
            if fc == FC_WRITE_MULTIPLE:
                addr, cnt, bc = struct.unpack(">HHB", pdu[1:6])
                if not 1 <= cnt <= 123 or bc != cnt * 2 or len(pdu) < 6 + bc:
                    return bytes([fc | 0x80, EX_ILLEGAL_VALUE])
                if addr + cnt > n:
                    return bytes([fc | 0x80, EX_ILLEGAL_ADDRESS])
                vals = struct.unpack(f">{cnt}H", pdu[6:6 + bc])
                with self.lock:
                    for k, v in enumerate(vals):
                        self.regs[addr + k] = v
                    if self.write_hook:
                        self.write_hook(addr, cnt)
                return struct.pack(">BHH", fc, addr, cnt)
        except struct.error:
            return bytes([fc | 0x80, EX_ILLEGAL_VALUE])
        return bytes([fc | 0x80, EX_ILLEGAL_FUNCTION])


# ============================================================
#  КЛІЄНТ (майстер)
# ============================================================
class ModbusClient:
    def __init__(self, host, port, unit=1, timeout=0.4):
        self.host, self.port, self.unit, self.timeout = host, port, unit, timeout
        self.sock = None
        self.tid = 0

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None

    def _txn(self, pdu):
        try:
            if self.sock is None:
                self.sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
                self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.sock.settimeout(self.timeout)
            self.tid = self.tid % 65535 + 1
            self.sock.sendall(struct.pack(">HHHB", self.tid, 0, len(pdu) + 1, self.unit) + pdu)
            hdr = _recv_exact(self.sock, 7)
            tid, pid, length, unit = struct.unpack(">HHHB", hdr)
            resp = _recv_exact(self.sock, length - 1)
        except socket.timeout:
            self.close()
            raise ModbusTimeout("тайм-аут відповіді")
        except (OSError, ConnectionError) as e:
            self.close()
            raise ModbusTimeout(f"помилка з'єднання: {e}")
        if tid != self.tid:
            self.close()
            raise ModbusError("невідповідність ідентифікатора транзакції")
        if resp[0] & 0x80:
            raise ModbusException(resp[0] & 0x7F, resp[1])
        return resp

    def read_holding(self, addr, count):
        r = self._txn(struct.pack(">BHH", FC_READ_HOLDING, addr, count))
        return list(struct.unpack(f">{r[1] // 2}H", r[2:2 + r[1]]))

    def write_single(self, addr, value):
        self._txn(struct.pack(">BHH", FC_WRITE_SINGLE, addr, int(value) & 0xFFFF))

    def write_multiple(self, addr, values):
        values = [int(v) & 0xFFFF for v in values]
        self._txn(struct.pack(f">BHHB{len(values)}H", FC_WRITE_MULTIPLE, addr, len(values),
                              len(values) * 2, *values))
