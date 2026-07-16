"""
imu_receiver.py  –  Package 1 & 2
------------------------------------
Reads IMU data from STM32F3Discovery over USB CDC at 100 Hz.

STM32 CSV format (8 fields):
    timestamp_ms, ax_g, ay_g, az_g, gx_rads, gy_rads, gz_rads, stationary

ax/ay/az are in g-units from firmware → multiplied by 9.81 here → m/s²
gx/gy/gz are in rad/s (firmware already subtracts calibrated bias)
stationary is 0/1 hardware ZUPT flag

"""

import threading, queue, time, glob
from collections import namedtuple, deque
import numpy as np

try:
    import serial, serial.tools.list_ports
    _SERIAL_OK = True
except ImportError:
    _SERIAL_OK = False

IMUSample = namedtuple("IMUSample", ["t_s", "accel", "gyro", "hw_stationary"])

GYRO_MAX   = 2.0   # rad/s hard clamp (~115°/s — far above any ground robot turn rate)
MEDIAN_WIN = 3     # median filter window


def _find_port():
    if not _SERIAL_OK:
        return None
    for p in serial.tools.list_ports.comports():
        if p.vid == 0x0483:          # STMicroelectronics VID
            return p.device
    candidates = glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")
    return candidates[0] if candidates else None


class IMUReceiver:
    G = 9.81

    def __init__(self, port=None, baud=115200, queue_size=500):
        self._port      = port or _find_port()
        self._baud      = baud
        self._queue     = queue.Queue(maxsize=queue_size)
        self._stop      = threading.Event()
        self._ser       = None
        self._t0_ms     = None
        self._gbuf      = deque(maxlen=MEDIAN_WIN)
        self._last_g    = np.zeros(3)
        if self._port:
            print(f"[IMUReceiver] Using port: {self._port}")
        else:
            print("[IMUReceiver] WARNING: no STM32 port found")

    def start(self):
        self._stop.clear()
        threading.Thread(target=self._loop, daemon=True).start()

    def stop(self):
        self._stop.set()
        if self._ser:
            try:
                if self._ser.is_open:
                    self._ser.close()
            except OSError:
                pass

    def __iter__(self):
        while not self._stop.is_set():
            try:
                yield self._queue.get(timeout=0.1)
            except queue.Empty:
                continue

    def get_nowait(self):
        try:
            return self._queue.get_nowait()
        except queue.Empty:
            return None

    def _loop(self):
        if not _SERIAL_OK or not self._port:
            print("[IMUReceiver] Cannot start.")
            return

        # Port open with retry
        for attempt in range(5):
            try:
                self._ser = serial.Serial(self._port, self._baud, timeout=1.0)
                break
            except serial.SerialException as e:
                if attempt < 4:
                    print(f"[IMUReceiver] Busy, retry {attempt+1}/5… ({e})")
                    time.sleep(2.0)
                else:
                    print(f"[IMUReceiver] Cannot open {self._port}: {e}")
                    print(f"  Try: sudo fuser -k {self._port}")
                    return

        print(f"[IMUReceiver] Opened {self._port} @ {self._baud} baud")

        while not self._stop.is_set():
            try:
                raw = self._ser.readline()
            except OSError as e:
                if e.errno == 9:
                    break
                print(f"[IMUReceiver] Read error: {e}")
                break
            except Exception as e:
                if not self._stop.is_set():
                    print(f"[IMUReceiver] Error: {e}")
                break

            line = raw.decode("ascii", errors="ignore").strip()
            if not line:
                continue

            parts = line.split(",")
            if len(parts) not in (7, 8):
                continue

            try:
                ms  = int(parts[0])
                ax  = float(parts[1])
                ay  = float(parts[2])
                az  = float(parts[3])
                gx  = float(parts[4])
                gy  = float(parts[5])
                gz  = float(parts[6])
                hw  = bool(int(parts[7])) if len(parts) == 8 else False
            except ValueError:
                continue

            if self._t0_ms is None:
                self._t0_ms = ms
            t_s = (ms - self._t0_ms) / 1000.0

            # Accel: g-units → m/s²
            accel = np.array([ax, ay, az]) * self.G

            # Gyro spike filter
            g_raw = np.array([gx, gy, gz])
            if np.any(np.abs(g_raw) > GYRO_MAX):
                g_raw = self._last_g.copy()
            else:
                self._last_g = g_raw.copy()

            self._gbuf.append(g_raw.copy())
            gyro = (np.median(np.array(self._gbuf), axis=0)
                    if len(self._gbuf) == MEDIAN_WIN else g_raw)

            sample = IMUSample(t_s=t_s, accel=accel,
                               gyro=gyro, hw_stationary=hw)
            try:
                self._queue.put_nowait(sample)
            except queue.Full:
                try:
                    self._queue.get_nowait()
                except queue.Empty:
                    pass
                self._queue.put_nowait(sample)

        try:
            if self._ser and self._ser.is_open:
                self._ser.close()
        except OSError:
            pass
        print("[IMUReceiver] Reader thread exited.")
