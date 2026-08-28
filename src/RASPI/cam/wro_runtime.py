import argparse
import os
import re
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from queue import Empty, Full, Queue

import cv2
import numpy as np



REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
RASPI_CAM_DIR = os.path.dirname(os.path.abspath(__file__))
if RASPI_CAM_DIR not in sys.path:
	sys.path.append(RASPI_CAM_DIR)

from vision import Vision


WOOD_LOWER = np.array([5, 25, 40])
WOOD_UPPER = np.array([35, 180, 255])
OUTPUT_PATTERN = re.compile(r"^orillas(\d+)\.mp4$")
CAM_FRAME_PATH = "/tmp/wro_cam_frame.jpg"


class ThreadedFrameGrabber:
	def __init__(self, cap):
		self.cap = cap
		self.lock = threading.Lock()
		self.frame = None
		self.stopped = False
		self.thread = threading.Thread(target=self._run, daemon=True)

	def start(self):
		self.thread.start()
		return self

	def _run(self):
		failures = 0
		print("[CAM] Hilo de captura iniciado", flush=True)
		while not self.stopped:
			ret, frame = self.cap.read()
			if not ret:
				failures += 1
				print(f"[CAM] cap.read() fallo #{failures}/100", flush=True)
				if failures >= 100:
					print("[CAM] Demasiados fallos, deteniendo hilo", flush=True)
					self.stopped = True
					break
				time.sleep(0.05)
				continue
			if failures > 0:
				print(f"[CAM] Frame recibido despues de {failures} fallos", flush=True)
			failures = 0
			with self.lock:
				self.frame = frame
		print("[CAM] Hilo de captura terminado", flush=True)

	def read(self):
		with self.lock:
			if self.frame is None:
				return False, None
			return True, self.frame.copy()

	def stop(self):
		self.stopped = True
		if self.thread.is_alive():
			self.thread.join(timeout=1.0)


def create_writer(output_path: str, frame_width: int, frame_height: int, fps: float):
	fourcc = cv2.VideoWriter_fourcc(*"mp4v")
	return cv2.VideoWriter(output_path, fourcc, fps, (frame_width, frame_height))


class AsyncVideoWriter:
	def __init__(self, output_path: str, frame_width: int, frame_height: int, fps: float, max_queue_size: int = 8):
		self.writer = create_writer(output_path, frame_width, frame_height, fps)
		self.queue = Queue(maxsize=max_queue_size)
		self.stopped = False
		self.thread = threading.Thread(target=self._run, daemon=True)

	def start(self):
		self.thread.start()
		return self

	def _run(self):
		while not self.stopped or not self.queue.empty():
			try:
				frame = self.queue.get(timeout=0.05)
			except Empty:
				continue
			try:
				self.writer.write(frame)
			finally:
				self.queue.task_done()

	def write(self, frame):
		if self.stopped:
			return
		try:
			self.queue.put_nowait(frame)
		except Full:
			try:
				self.queue.get_nowait()
				self.queue.task_done()
			except Empty:
				pass
			try:
				self.queue.put_nowait(frame)
			except Full:
				pass

	def stop(self):
		self.stopped = True
		if self.thread.is_alive():
			self.thread.join(timeout=2.0)
		self.writer.release()


def next_output_path(directory: Path) -> Path:
	directory.mkdir(parents=True, exist_ok=True)

	highest_index = 0
	for candidate in directory.glob("orillas*.mp4"):
		match = OUTPUT_PATTERN.match(candidate.name)
		if match:
			highest_index = max(highest_index, int(match.group(1)))

	return directory / f"orillas{highest_index + 1}.mp4"


def resolve_output_path(output_path: str | None) -> Path:
	if output_path:
		path = Path(output_path)
		if path.suffix.lower() == ".mp4" and OUTPUT_PATTERN.match(path.name):
			return next_output_path(path.parent)
		if path.is_dir() or not path.suffix:
			return next_output_path(path)
		path.parent.mkdir(parents=True, exist_ok=True)
		return path
	return next_output_path(Path.cwd())


@dataclass
class Config:
	cam_index: int = 0
	serial_port: str = "/dev/ttyS0"
	baudrate: int = 115200
	protocol: str = "signed"
	show_window: bool = True
	process_every_n: int = 3
	threaded_capture: bool = True
	warmup_frames: int = 40

	red_target_px: int = 140
	green_target_px: int = 500
	neutral_x: int = 700

	pid_kp: float = 0.012
	pid_kd: float = 0.004
	correction_limit_px: float = 160.0

	obstacle_hold_band: float = 24.0
	obstacle_memory_frames: int = 18
	obstacle_clear_frames: int = 10
	obstacle_fade_factor: float = 0.85
	obstacle_max_correction_px: float = 35.0

	turn_top_ratio: float = 0.30
	turn_drop_ratio: float = 0.85

	record_orillas: bool = False
	record_output: str | None = None
	record_every_n: int = 6
	record_fps: float = 5.0


class PID:
	def __init__(self, kp: float, kd: float):
		self.kp = kp
		self.kd = kd
		self.prev_error = 0.0

	def compute(self, error: float) -> float:
		derivative = error - self.prev_error
		self.prev_error = error
		return (self.kp * error) + (self.kd * derivative)


class TurnDetector:
	def __init__(self, top_ratio: float = 0.30, drop_ratio: float = 0.85):
		self.top_ratio = top_ratio
		self.drop_ratio = drop_ratio
		self.prev_left = None
		self.prev_right = None
		self.kernel = np.ones((3, 3), np.uint8)

	def detect(self, frame_bgr: np.ndarray):
		h, w = frame_bgr.shape[:2]
		top_limit = max(1, int(h * self.top_ratio))
		roi = frame_bgr[:top_limit, :]

		split = w // 4
		left_roi = roi[:, :split]
		right_roi = roi[:, 3 * split :]

		left_hsv = cv2.cvtColor(left_roi, cv2.COLOR_BGR2HSV)
		right_hsv = cv2.cvtColor(right_roi, cv2.COLOR_BGR2HSV)

		left_mask = cv2.inRange(left_hsv, WOOD_LOWER, WOOD_UPPER)
		right_mask = cv2.inRange(right_hsv, WOOD_LOWER, WOOD_UPPER)

		left_mask = cv2.morphologyEx(left_mask, cv2.MORPH_OPEN, self.kernel)
		right_mask = cv2.morphologyEx(right_mask, cv2.MORPH_OPEN, self.kernel)

		left_ratio = cv2.countNonZero(left_mask) / max(1, left_mask.size)
		right_ratio = cv2.countNonZero(right_mask) / max(1, right_mask.size)

		left_drop = self.prev_left is not None and left_ratio < (self.prev_left * self.drop_ratio)
		right_drop = self.prev_right is not None and right_ratio < (self.prev_right * self.drop_ratio)

		self.prev_left = left_ratio
		self.prev_right = right_ratio

		if left_drop and not right_drop:
			return +1, left_ratio, right_ratio
		if right_drop and not left_drop:
			return -1, left_ratio, right_ratio
		return 0, left_ratio, right_ratio


class SerialLink:
	def __init__(self, port: str, baudrate: int):
		self.port = port
		self.baudrate = baudrate
		self.fd = None
		self._queue = Queue(maxsize=4)
		self._last_rx = ""
		self._thread = threading.Thread(target=self._run, daemon=True)
		self._thread.start()

	def _run(self):
		print(f"[SERIAL] Thread iniciado, abriendo {self.port}...", flush=True)
		try:
			import termios
			self.fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY)
			iflag, oflag, cflag, lflag, ispeed, ospeed, cc = termios.tcgetattr(self.fd)

			# Modo raw completo — sin esto ICANON sigue activo y VMIN/VTIME
			# no aplican como se espera; ECHO puede reinyectar bytes recibidos
			# de vuelta al ESP32 y corromper el siguiente mensaje saliente.
			iflag &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK | termios.ISTRIP |
			           termios.INLCR  | termios.IGNCR  | termios.ICRNL  | termios.IXON)
			oflag &= ~termios.OPOST
			lflag &= ~(termios.ECHO | termios.ECHONL | termios.ICANON |
			           termios.ISIG | termios.IEXTEN)
			cflag &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB)
			cflag |= termios.CS8 | termios.CLOCAL | termios.CREAD

			cc[termios.VMIN]  = 0
			cc[termios.VTIME] = 2

			ispeed = ospeed = termios.B115200

			termios.tcsetattr(self.fd, termios.TCSANOW,
			                   [iflag, oflag, cflag, lflag, ispeed, ospeed, cc])
			time.sleep(1.0)
			print(f"[SERIAL] UART abierto en {self.port} @ {self.baudrate}", flush=True)
			# El READY lo manda run() DESPUES del warmup de camara (ver
			# IntegratedRuntime.run / PPRuntime.run). Antes se enviaba aqui
			# "READY,pi=1" ~1s tras abrir el UART -- ANTES de que la vision
			# estuviera lista -- y el ESP32 salia de setup() y arrancaba a rodar
			# en wall-PID de fallback mientras la Pi seguia calentando la camara.
		except Exception as exc:
			self.fd = None
			print(f"[SERIAL] No se pudo abrir UART ({self.port}): {exc}", flush=True)
			return

		while True:
			try:
				line = self._queue.get(timeout=0.1)
			except Empty:
				continue
			if line is None:
				break
			try:
				os.write(self.fd, f"{line}\n".encode("utf-8"))
				print(f"[SERIAL] TX: {line}", flush=True)
			except Exception as e:
				print(f"[SERIAL] Error TX: {e}", flush=True)
			try:
				data = b""
				while True:
					chunk = os.read(self.fd, 1)
					if not chunk or chunk == b"\n":
						break
					data += chunk
				rx = data.decode("utf-8", errors="ignore").strip()
				if rx:
					self._last_rx = rx
					print(f"[SERIAL] RX: {rx}", flush=True)
			except Exception:
				pass

	def open(self):
		pass

	def send_line(self, line: str):
		try:
			self._queue.put_nowait(line)
		except Full:
			pass

	def try_readline(self):
		rx = self._last_rx
		self._last_rx = ""
		return rx

	def close(self):
		try:
			self._queue.put_nowait(None)
		except Full:
			pass
		if self.fd is not None:
			try:
				os.close(self.fd)
			except Exception:
				pass
			self.fd = None


class IntegratedRuntime:
	def __init__(self, cfg: Config):
		self.cfg = cfg
		self.vision = Vision(cfg.cam_index)
		self.turn_detector = TurnDetector(cfg.turn_top_ratio, cfg.turn_drop_ratio)
		self.pid_obs = PID(cfg.pid_kp, cfg.pid_kd)
		self.serial_link = SerialLink(cfg.serial_port, cfg.baudrate)
		self.loop_count = 0
		self.persistencia_obstaculo = 0
		self.obstaculo_clear_frames = 0
		self.ultimo_color_obstaculo = None
		self.ultima_correccion_px = 0.0
		self.frame_grabber = None
		self.video_writer = None
		self.record_count = 0
		self.output_file = resolve_output_path(cfg.record_output) if cfg.record_orillas else None

		try:
			self.vision.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
		except Exception:
			pass

	@staticmethod
	def largest_object(objects):
		if not objects:
			return None, None
		x, y, w, h = max(objects, key=lambda obj: obj[2] * obj[3])
		return (x, y, w, h), x + (w // 2)

	def obstacle_decision(self, positions, frame_width: int):
		red_obj, red_x = self.largest_object(positions.get("Red", []))
		green_obj, green_x = self.largest_object(positions.get("Green", []))

		if red_obj and green_obj:
			red_area = red_obj[2] * red_obj[3]
			green_area = green_obj[2] * green_obj[3]
			use_red = red_area >= green_area
		else:
			use_red = bool(red_obj)

		if use_red and red_obj:
			x = red_x
			error_px = float(self.cfg.red_target_px - x)
			color = "Red"
			mode = "avoid_red"
		elif green_obj:
			x = green_x
			error_px = float(self.cfg.green_target_px - x)
			color = "Green"
			mode = "avoid_green"
		else:
			return {
				"detected": False,
				"color": "None",
				"x": None,
				"error_px": 0.0,
				"vision_error_norm": 0.0,
				"mode": "no_obstacle",
				"priority": False,
				"memory_frames": 0,
			}

		if abs(error_px) > self.cfg.obstacle_hold_band:
			correction_px = self.pid_obs.compute(error_px)
			correction_px = max(-self.cfg.correction_limit_px, min(self.cfg.correction_limit_px, correction_px))
			self.ultima_correccion_px = correction_px
		else:
			correction_px = 0.0
			self.pid_obs.prev_error = 0.0
			self.ultima_correccion_px = 0.0

		self.persistencia_obstaculo = self.cfg.obstacle_memory_frames
		self.obstaculo_clear_frames = 0
		self.ultimo_color_obstaculo = color

		norm = -(correction_px / max(1.0, self.cfg.correction_limit_px))

		return {
			"detected": True,
			"color": color,
			"x": x,
			"error_px": error_px,
			"vision_error_norm": float(norm),
			"mode": mode,
			"priority": True,
			"memory_frames": self.persistencia_obstaculo,
		}

	def obstacle_memory_decision(self):
		if self.persistencia_obstaculo <= 0:
			self.obstaculo_clear_frames += 1
			return {
				"detected": False,
				"color": "None",
				"x": None,
				"error_px": 0.0,
				"vision_error_norm": 0.0,
				"mode": "no_obstacle",
				"priority": False,
				"memory_frames": 0,
			}

		self.persistencia_obstaculo -= 1
		self.obstaculo_clear_frames = 0

		correction_px = self.ultima_correccion_px * self.cfg.obstacle_fade_factor
		correction_px = max(-self.cfg.obstacle_max_correction_px, min(self.cfg.obstacle_max_correction_px, correction_px))
		self.ultima_correccion_px = correction_px

		if self.ultimo_color_obstaculo == "Red":
			mode = "memory_red"
		elif self.ultimo_color_obstaculo == "Green":
			mode = "memory_green"
		else:
			mode = "memory_unknown"

		norm = -(correction_px / max(1.0, self.cfg.correction_limit_px)) if self.ultimo_color_obstaculo else 0.0

		return {
			"detected": False,
			"color": self.ultimo_color_obstaculo or "None",
			"x": None,
			"error_px": 0.0,
			"vision_error_norm": norm,
			"mode": mode,
			"priority": True,
			"memory_frames": self.persistencia_obstaculo,
		}

	def build_serial_message(self, obs_info, turn_hint):
		if self.cfg.protocol == "legacy_x":
			if obs_info["detected"]:
				return str(int(obs_info["x"]))
			return str(self.cfg.neutral_x)

		obs = obs_info["vision_error_norm"]
		state = obs_info["mode"]
		prio = 1 if obs_info.get("priority", False) else 0
		mem = int(obs_info.get("memory_frames", 0))
		return f"V1,obs={obs:+.3f},turn={int(turn_hint)},state={state},prio={prio},mem={mem}"

	def annotate(self, frame, obs_info, turn_hint, left_ratio, right_ratio, serial_msg, fps=None):
		lines = [
			f"obs_color={obs_info['color']} x={obs_info['x']}",
			f"obs_err_px={obs_info['error_px']:.1f} obs_norm={obs_info['vision_error_norm']:+.3f}",
			f"turn_hint={turn_hint} wood_L={left_ratio:.3f} wood_R={right_ratio:.3f}",
			f"mode={obs_info['mode']} prio={int(obs_info.get('priority', False))} mem={obs_info.get('memory_frames', 0)}",
			f"serial={serial_msg}",
		]
		if fps is not None:
			lines.append(f"fps={fps:.1f}")
		y = 22
		for txt in lines:
			cv2.putText(frame, txt, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
			y += 22

	def start_capture(self):
		print(f"[CAM] cap.isOpened()={self.vision.cap.isOpened()} threaded={self.cfg.threaded_capture}", flush=True)
		if self.cfg.threaded_capture:
			self.frame_grabber = ThreadedFrameGrabber(self.vision.cap).start()
			print("[CAM] ThreadedFrameGrabber iniciado", flush=True)

	def read_frame(self):
		if self.frame_grabber is not None:
			return self.frame_grabber.read()
		return self.vision.cap.read()

	def maybe_record_frame(self, frame, fps):
		if not self.cfg.record_orillas:
			return
		self.record_count += 1
		if self.record_count % max(1, self.cfg.record_every_n) != 0:
			return

		if self.video_writer is None:
			out_fps = self.cfg.record_fps
			if fps > 0:
				out_fps = max(out_fps, fps / max(1, self.cfg.record_every_n))
			self.video_writer = AsyncVideoWriter(
				str(self.output_file),
				frame.shape[1],
				frame.shape[0],
				out_fps,
			).start()
			print(f"[INFO] Grabando orillas en {self.output_file}", flush=True)

		self.video_writer.write(frame.copy())

	def _write_cam_frame(self, frame):
		try:
			_, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 65])
			with open(CAM_FRAME_PATH, "wb") as f:
				f.write(buf.tobytes())
		except Exception:
			pass

	def run(self, on_ready=None):
		self.serial_link.open()
		self.start_capture()

		# Descarta los primeros frames hasta que la exposicion de la camara se estabilice
		print(f"[INFO] Calentando camara ({self.cfg.warmup_frames} frames)...", flush=True)
		warmed = 0
		while warmed < self.cfg.warmup_frames:
			ret, _ = self.read_frame()
			if ret:
				warmed += 1
			else:
				time.sleep(0.01)
		print("[INFO] Camara estabilizada.", flush=True)

		# READY explicito: recien ahora que la camara esta lista y el pipeline
		# va a empezar a mandar control, avisar al ESP32 para que salga de
		# setup() y arranque motores. x3 por si se pierde el primer byte al
		# arrancar el UART. (Antes lo mandaba el hilo de SerialLink ~1s tras
		# abrir el UART, ANTES del warmup.)
		for _ in range(3):
			self.serial_link.send_line("READY")
			time.sleep(0.05)
		print("[INFO] READY enviado al ESP32.", flush=True)

		if on_ready is not None:
			on_ready()

		print("[INFO] Iniciando control WRO integrado. ESC para salir.", flush=True)

		last_fps_time = time.perf_counter()
		fps_count = 0
		fps = 0.0

		try:
			while True:
				ret, frame = self.read_frame()
				if not ret:
					print("[WARN] No frame capturado.", flush=True)
					time.sleep(0.01)
					continue

				self.loop_count += 1
				if self.loop_count % self.cfg.process_every_n != 0:
					time.sleep(0.001)
					continue

				fps_count += 1
				now = time.perf_counter()
				elapsed = now - last_fps_time
				if elapsed >= 1.0:
					fps = fps_count / elapsed
					fps_count = 0
					last_fps_time = now

				processed_frame, positions = self.vision.process_frame(frame)
				obs_now = self.obstacle_decision(positions, processed_frame.shape[1])
				if obs_now["detected"]:
					obs_info = obs_now
				else:
					self.pid_obs.prev_error = 0.0
					obs_info = self.obstacle_memory_decision()

				turn_hint_raw, left_ratio, right_ratio = self.turn_detector.detect(processed_frame)
				can_turn = (not obs_info.get("priority", False)) and (self.obstaculo_clear_frames >= self.cfg.obstacle_clear_frames)
				turn_hint = turn_hint_raw if can_turn else 0

				serial_msg = self.build_serial_message(obs_info, turn_hint)
				self.serial_link.send_line(serial_msg)
				serial_ack = self.serial_link.try_readline()

				if serial_ack:
					print(f"TX: {serial_msg} | RX: {serial_ack}", flush=True)
				else:
					print(f"TX: {serial_msg}", flush=True)

				if self.cfg.show_window:
					self.annotate(processed_frame, obs_info, turn_hint, left_ratio, right_ratio, serial_msg, fps=fps)
					cv2.imshow("WRO Runtime", processed_frame)
					if cv2.waitKey(1) & 0xFF == 27:
						break

				self.maybe_record_frame(processed_frame, fps)
				self._write_cam_frame(processed_frame)

		finally:
			if self.frame_grabber is not None:
				self.frame_grabber.stop()
			self.vision.cap.release()
			if self.video_writer is not None:
				self.video_writer.stop()
			cv2.destroyAllWindows()
			self.serial_link.close()


def parse_args():
	parser = argparse.ArgumentParser(description="Runtime integrado profesional para WRO Future Engineers.")
	parser.add_argument("--cam-index", type=int, default=0)
	parser.add_argument("--serial-port", type=str, default="/dev/ttyS0")
	parser.add_argument("--baudrate", type=int, default=115200)
	parser.add_argument("--protocol", choices=["signed", "legacy_x"], default="signed")
	parser.add_argument("--process-every", type=int, default=3)
	parser.add_argument("--threaded-capture", action="store_true")
	parser.add_argument("--no-threaded-capture", action="store_true")
	parser.add_argument("--record-orillas", action="store_true")
	parser.add_argument("--record-output", type=str, default=None)
	parser.add_argument("--record-every", type=int, default=6)
	parser.add_argument("--record-fps", type=float, default=5.0)
	parser.add_argument("--no-window", action="store_true")
	return parser.parse_args()


def main():
	_gpio = None
	try:
		import RPi.GPIO as GPIO
		_gpio = GPIO
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(27, GPIO.OUT)
		GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
		GPIO.output(27, GPIO.HIGH)  # LED encendido: la Pi ya prendio
		print("[GPIO] LED encendido - Pi prendida.", flush=True)
		print("[GPIO] Esperando boton en GPIO17...", flush=True)
		while GPIO.input(17) == GPIO.LOW:
			time.sleep(0.05)
		print("[GPIO] Boton detectado. Iniciando...", flush=True)
	except ImportError:
		print("[GPIO] RPi.GPIO no disponible, omitiendo espera de boton.", flush=True)

	def led_on():
		if _gpio is not None:
			_gpio.output(27, _gpio.HIGH)
			print("[GPIO] LED encendido - sistema listo.", flush=True)

	args = parse_args()
	threaded_capture = True
	if args.no_threaded_capture:
		threaded_capture = False
	elif args.threaded_capture:
		threaded_capture = True

	cfg = Config(
		cam_index=args.cam_index,
		serial_port=args.serial_port,
		baudrate=args.baudrate,
		protocol=args.protocol,
		process_every_n=max(1, args.process_every),
		threaded_capture=threaded_capture,
		record_orillas=bool(args.record_orillas),
		record_output=args.record_output,
		record_every_n=max(1, args.record_every),
		record_fps=max(1.0, args.record_fps),
		show_window=not args.no_window,
	)
	runtime = IntegratedRuntime(cfg)
	runtime.run(on_ready=led_on)


if __name__ == "__main__":
	main()
