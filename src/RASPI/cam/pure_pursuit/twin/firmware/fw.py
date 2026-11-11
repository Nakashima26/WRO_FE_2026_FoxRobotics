"""ctypes wrapper for firmware SIL shared library."""

from __future__ import annotations

import ctypes
import shutil
import sys
import tempfile
from pathlib import Path
from typing import Callable

from . import build as _build_mod

_SERVO_PIN = 13
_PWMA = 23
_A1 = 18
_A2 = 19


class _NativeHistory:
    """Read-only ring view, refreshed only outside firmware execution.

    Keep the library owner alive. No history drain may run while this view is
    used: the simulator selects either this view or the copying API.
    """
    def __init__(self, firmware):
        self._owner = firmware
        self._meta = (ctypes.c_size_t * 3)()
        fn = firmware._lib.sil_history_view
        fn.argtypes = [ctypes.c_void_p]
        fn.restype = ctypes.POINTER(ctypes.c_double * 4)
        self._fn = fn
        self._pose = (ctypes.c_double * 3)()
        self._pose_fn = firmware._lib.sil_pose_at
        self._pose_fn.argtypes = [ctypes.c_double, ctypes.c_void_p]
        self._pose_fn.restype = None
        self.refresh()

    def pose_at(self, t):
        self._pose_fn(t, self._pose)
        return tuple(self._pose)

    def refresh(self):
        self._rows = self._fn(self._meta)
        self._count, self._start, self._capacity = self._meta

    def __len__(self):
        return self._count

    def __getitem__(self, index):
        if index < 0:
            index += self._count
        if not 0 <= index < self._count:
            raise IndexError(index)
        return tuple(self._rows[(self._start + index) % self._capacity])

    def __reversed__(self):
        for index in range(self._count - 1, -1, -1):
            yield self[index]


class FirmwareSetupError(RuntimeError):
    pass


class FirmwareSIL:
    def __init__(
        self,
        overrides: dict[str, str] | None = None,
        defines: dict[str, str] | None = None,
        force_rebuild: bool = False,
        source: str = "worktree",
    ) -> None:
        lib_path = _build_mod.build(
            overrides=overrides,
            defines=defines,
            force=force_rebuild,
            source=source,
        )
        # Copia propia por instancia: cada Sim arranca con los globals del .ino
        # en cero. En Windows hay que cerrar el handle antes de LoadLibrary.
        self._tmp = tempfile.NamedTemporaryFile(suffix=lib_path.suffix, delete=False)
        self._tmp.close()
        shutil.copy2(lib_path, self._tmp.name)
        self._lib = ctypes.CDLL(self._tmp.name)

        self._native_sync = None
        self._advance_cb = None
        self._sonar_cb = None
        self._gyro_cb = None
        self._encoder_cb = None
        self._tof_cb = None

        self._bind()
        # Buffers por instancia: las lecturas son síncronas y devuelven copias.
        self._serial2_buf = ctypes.create_string_buffer(4096)
        self._serial2_time = ctypes.c_uint64()
        self._debug_buf = ctypes.create_string_buffer(65536)
        self._actuator_buf = (ctypes.c_double * 3)()
        self._advance_clock = (ctypes.c_uint64 * 1)()
        self._advance_actuators = (ctypes.c_double * 3)()
        self.set_callbacks(
            advance=lambda _us: None,
            sonar=lambda _id: 0,
            gyro=lambda: 0.0,
        )

    def _bind(self) -> None:
        lib = self._lib

        lib.sil_set_callbacks.argtypes = [
            ctypes.CFUNCTYPE(None, ctypes.c_uint64),
            ctypes.CFUNCTYPE(ctypes.c_uint32, ctypes.c_int),
            ctypes.CFUNCTYPE(ctypes.c_float),
            ctypes.CFUNCTYPE(ctypes.c_int32),
            ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_int),
        ]
        lib.sil_set_callbacks.restype = None

        lib.sil_setup.argtypes = []
        lib.sil_setup.restype = ctypes.c_int
        lib.sil_loop.argtypes = []
        lib.sil_loop.restype = None
        lib.sil_now_us.argtypes = []
        lib.sil_now_us.restype = ctypes.c_uint64
        lib.sil_set_now_us.argtypes = [ctypes.c_uint64]
        lib.sil_set_now_us.restype = None
        lib.sil_set_param.argtypes = [ctypes.c_char_p, ctypes.c_double]
        lib.sil_set_param.restype = None
        lib.sil_last_error.argtypes = []
        lib.sil_last_error.restype = ctypes.c_char_p

        lib.sil_serial2_rx_push.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.c_uint64]
        lib.sil_serial2_rx_push.restype = None
        lib.sil_serial2_tx_pop_line.argtypes = [
            ctypes.c_char_p,
            ctypes.c_int,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        lib.sil_serial2_tx_pop_line.restype = ctypes.c_int
        lib.sil_serial_tx_pop.argtypes = [ctypes.c_char_p, ctypes.c_int]
        lib.sil_serial_tx_pop.restype = ctypes.c_int

        lib.sil_ledc_duty.argtypes = [ctypes.c_uint8]
        lib.sil_ledc_duty.restype = ctypes.c_int
        lib.sil_digital.argtypes = [ctypes.c_uint8]
        lib.sil_digital.restype = ctypes.c_int
        lib.sil_set_advance_snapshot.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        lib.sil_set_advance_snapshot.restype = None
        lib.sil_actuators.argtypes = [ctypes.c_void_p]
        lib.sil_actuators.restype = None

    def enable_advance_snapshot(self):
        """Reloj y actuadores válidos durante advance, sin consultas ctypes adicionales.

        Los buffers pertenecen a esta instancia y no se comparten con actuators().
        """
        self._lib.sil_set_advance_snapshot(self._advance_clock, self._advance_actuators)
        return self._advance_clock, self._advance_actuators

    def enable_native_vehicle(self, vehicle, encoder, max_wheel, collision, sync):
        """Use the native kernel directly, retaining every shared buffer owner."""
        ep = encoder.params
        e = (ctypes.c_double * 11)(encoder._frac, encoder.count, 0, 0,
            encoder._slip_bias, ep.slip_c_acc_cap, ep.slip_c_acc,
            ep.slip_c_scrub, ep.counts_per_mm, max_wheel, 0)
        cb = ctypes.CFUNCTYPE(None, ctypes.c_uint64)(collision)
        self._native_owners = vehicle, encoder, e, cb
        lib = self._lib
        lib.sil_set_native_vehicle.argtypes = [ctypes.c_void_p] * 5
        lib.sil_set_native_vehicle.restype = None
        lib.sil_drain_history.argtypes = [ctypes.c_void_p]
        lib.sil_drain_history.restype = ctypes.c_size_t
        # ctypes releases the GIL during firmware execution. The kernel accesses
        # only these instance-owned buffers; callbacks reacquire the GIL normally.
        lib.sil_set_native_vehicle(vehicle._fn, vehicle._state, vehicle._p, e, cb)
        self._native_sync = sync
        return e

    def history_view(self):
        return _NativeHistory(self)

    def drain_history(self):
        n = self._lib.sil_drain_history(None)
        if not n:
            return ()
        rows = ((ctypes.c_double * 4) * n)()
        self._lib.sil_drain_history(rows)
        return [tuple(row) for row in rows]

    def set_callbacks(
        self,
        advance: Callable[[int], None],
        sonar: Callable[[int], int],
        gyro: Callable[[], float],
        encoder: Callable[[], int] | None = None,
        tof: Callable[[int], int] | None = None,
    ) -> None:
        # ctypes ya convierte uint64 a int de Python: sin envoltura por subpaso.
        _advance = ctypes.CFUNCTYPE(None, ctypes.c_uint64)(advance)

        @ctypes.CFUNCTYPE(ctypes.c_uint32, ctypes.c_int)
        def _sonar(sid: int) -> int:
            return int(sonar(int(sid)))

        @ctypes.CFUNCTYPE(ctypes.c_float)
        def _gyro() -> float:
            return float(gyro())

        @ctypes.CFUNCTYPE(ctypes.c_int32)
        def _encoder() -> int:
            return int(encoder()) if encoder else 0

        @ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_int)
        def _tof(idx: int) -> int:
            return int(tof(idx)) if tof else -1

        self._advance_cb = _advance
        self._sonar_cb = _sonar
        self._gyro_cb = _gyro
        self._encoder_cb = _encoder
        self._tof_cb = _tof
        self._lib.sil_set_callbacks(_advance, _sonar, _gyro, _encoder, _tof)

    def setup(self) -> None:
        rc = self._lib.sil_setup()
        if self._native_sync:
            self._native_sync()
        if rc != 0:
            err = self._lib.sil_last_error()
            msg = err.decode("utf-8") if err else "sil_setup failed"
            raise FirmwareSetupError(msg)

    def loop(self) -> None:
        self._lib.sil_loop()
        if self._native_sync:
            self._native_sync()

    @property
    def now_us(self) -> int:
        return int(self._lib.sil_now_us())

    def set_now_us(self, t: int) -> None:
        self._lib.sil_set_now_us(ctypes.c_uint64(t))

    def set_param(self, name: str, value: float) -> None:
        self._lib.sil_set_param(name.encode("utf-8"), float(value))

    def push_serial2(self, line: str, arrival_us: int | None = None) -> None:
        if not line.endswith("\n"):
            line += "\n"
        t = self.now_us if arrival_us is None else int(arrival_us)
        self._lib.sil_serial2_rx_push(line.encode("utf-8"), len(line), ctypes.c_uint64(t))

    def pop_serial2_lines(self) -> list[tuple[int, str]]:
        out: list[tuple[int, str]] = []
        buf = self._serial2_buf
        t_done = self._serial2_time
        while True:
            n = self._lib.sil_serial2_tx_pop_line(buf, len(buf), ctypes.byref(t_done))
            if n < 0:
                break
            out.append((int(t_done.value), buf.value[:n].decode("utf-8")))
        return out

    def pop_debug(self) -> str:
        chunks: list[bytes] = []
        buf = self._debug_buf
        while True:
            n = self._lib.sil_serial_tx_pop(buf, len(buf))
            if n <= 0:
                break
            chunks.append(buf.raw[:n])
        return b"".join(chunks).decode("utf-8", errors="replace")

    def servo_angle(self) -> float:
        duty = self._lib.sil_ledc_duty(_SERVO_PIN)
        pulse_us = duty * 20000.0 / 65535.0
        return (pulse_us - 500.0) * 180.0 / 2000.0

    def actuators(self):
        self._lib.sil_actuators(self._actuator_buf)
        return self._actuator_buf[0], self._actuator_buf[1], int(self._actuator_buf[2])

    def motor_pwm(self) -> int:
        return int(self._lib.sil_ledc_duty(_PWMA))

    def motor_dir(self) -> int:
        a1 = self._lib.sil_digital(_A1)
        a2 = self._lib.sil_digital(_A2)
        if a1 and not a2:
            return 1
        if not a1 and a2:
            return -1
        return 0

    def close(self) -> None:
        self._tmp.close()
        lib = getattr(self, "_lib", None)
        if lib is not None and sys.platform == "win32":
            # Windows no deja borrar una DLL cargada.
            ctypes.windll.kernel32.FreeLibrary.argtypes = [ctypes.c_void_p]
            ctypes.windll.kernel32.FreeLibrary(ctypes.c_void_p(lib._handle))
            self._lib = None
        try:
            Path(self._tmp.name).unlink(missing_ok=True)
        except PermissionError:
            pass

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            pass
