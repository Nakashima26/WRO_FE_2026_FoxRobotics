import os
import time
import threading

import cv2
import numpy as np


# Rango HSV para madera/beige/cafe claro.
# Ajustable segun la luz real de la pista.
WOOD_LOWER = np.array([5, 25, 40])
WOOD_UPPER = np.array([35, 180, 255])
ANALYSIS_WIDTH = 1640
ANALYSIS_HEIGHT = 1232
PREVIEW_WIDTH = 640
PREVIEW_HEIGHT = 480
WINDOW_WIDTH = 320
WINDOW_HEIGHT = 240
PROCESS_EVERY_N = 3
ROI_SEGMENTS = 4
WINDOW_NAME = "Pista"


cv2.setUseOptimized(True)
cpu_count = os.cpu_count() or 1
fox_cv_threads = int(os.environ.get("FOX_CV_THREADS", str(min(4, cpu_count))))
cv2.setNumThreads(fox_cv_threads)


def open_camera(cam_index: int):
    if os.name == "nt":
        cap = cv2.VideoCapture(cam_index, cv2.CAP_DSHOW)
        if not cap.isOpened():
            cap = cv2.VideoCapture(cam_index)
    else:
        pipeline = (
            "libcamerasrc ! queue max-size-buffers=1 leaky=downstream "
            "! video/x-raw, width=1640, height=1232, framerate=30/1 "
            "! videoconvert ! videoscale ! video/x-raw, width=320, height=240 "
            "! appsink drop=true max-buffers=1 sync=false"
        )
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

        if not cap.isOpened():
            cap = cv2.VideoCapture(cam_index)

    if not cap.isOpened():
        raise RuntimeError("No se pudo abrir la camara.")

    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


class FrameGrabber:
    def __init__(self, cam_index: int):
        self.cap = open_camera(cam_index)
        self.lock = threading.Lock()
        self.frame = None
        self.stopped = False
        self.thread = threading.Thread(target=self._update, daemon=True)

    def start(self):
        self.thread.start()
        return self

    def _update(self):
        while not self.stopped:
            ret, frame = self.cap.read()
            if not ret:
                self.stopped = True
                break
            with self.lock:
                self.frame = frame

    def read(self):
        with self.lock:
            if self.frame is None:
                return False, None
            return True, self.frame.copy()

    def stop(self):
        self.stopped = True
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
        self.cap.release()


def get_window_image_size(window_name: str, fallback_width: int, fallback_height: int):
    try:
        _, _, window_width, window_height = cv2.getWindowImageRect(window_name)
    except cv2.error:
        return fallback_width, fallback_height

    if window_width > 0 and window_height > 0:
        return window_width, window_height

    return fallback_width, fallback_height


def build_segment_bounds(width: int, segment_count: int):
    boundaries = [int(round(index * width / segment_count)) for index in range(segment_count + 1)]
    bounds = []

    for index in range(segment_count):
        left = boundaries[index]
        right = boundaries[index + 1] - 1 if index < segment_count - 1 else width - 1
        bounds.append((left, max(left, right)))

    return bounds


def detect_turn_direction(frame, previous_left=None, previous_right=None, top_ratio=0.30):
    height, width = frame.shape[:2]
    top_limit = max(1, int(height * top_ratio))
    roi = frame[:top_limit, :]

    segment_bounds = build_segment_bounds(width, ROI_SEGMENTS)

    left_left, left_right = segment_bounds[0]
    right_left, right_right = segment_bounds[-1]

    left_roi = roi[:, left_left : left_right + 1]
    right_roi = roi[:, right_left : right_right + 1]

    left_hsv = cv2.cvtColor(left_roi, cv2.COLOR_BGR2HSV)
    right_hsv = cv2.cvtColor(right_roi, cv2.COLOR_BGR2HSV)

    left_mask = cv2.inRange(left_hsv, WOOD_LOWER, WOOD_UPPER)
    right_mask = cv2.inRange(right_hsv, WOOD_LOWER, WOOD_UPPER)

    left_mask = cv2.morphologyEx(left_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    right_mask = cv2.morphologyEx(right_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

    left_count = int(cv2.countNonZero(left_mask))
    right_count = int(cv2.countNonZero(right_mask))

    left_ratio = left_count / max(1, left_mask.size)
    right_ratio = right_count / max(1, right_mask.size)

    left_drop = previous_left is not None and left_ratio < previous_left * 0.85
    right_drop = previous_right is not None and right_ratio < previous_right * 0.85

    turn_text = None
    if left_drop and not right_drop:
        turn_text = "girar a la izquierda"
        print(turn_text, flush=True)
    elif right_drop and not left_drop:
        turn_text = "girar a la derecha"
        print(turn_text, flush=True)

    debug = {
        "mask": None,
        "left_mask": left_mask,
        "right_mask": right_mask,
        "edge_bounds": ((left_left, left_right), (right_left, right_right)),
        "left_ratio": left_ratio,
        "right_ratio": right_ratio,
        "top_ratio": top_ratio,
        "segment_count": ROI_SEGMENTS,
        "turn_text": turn_text,
    }
    return debug


def draw_debug(frame, debug, fps=None):
    height, width = frame.shape[:2]
    top_limit = max(1, int(height * debug["top_ratio"]))
    segment_bounds = build_segment_bounds(width, debug.get("segment_count", ROI_SEGMENTS))
    edge_mask = np.zeros((top_limit, width), dtype=np.uint8)

    left_bounds, right_bounds = debug["edge_bounds"]
    edge_mask[:, left_bounds[0] : left_bounds[1] + 1] = debug["left_mask"]
    edge_mask[:, right_bounds[0] : right_bounds[1] + 1] = debug["right_mask"]

    for index, (left, right) in enumerate(segment_bounds):
        if index == 0:
            color = (0, 255, 0)
            thickness = 2
        elif index == len(segment_bounds) - 1:
            color = (0, 255, 255)
            thickness = 2
        else:
            color = (100, 100, 100)
            thickness = 1
        cv2.rectangle(frame, (left, 0), (right, top_limit - 1), color, thickness)

    overlay = frame.copy()
    resized_mask = cv2.resize(edge_mask, (width, height), interpolation=cv2.INTER_NEAREST)
    colored_mask = cv2.cvtColor(resized_mask, cv2.COLOR_GRAY2BGR)
    overlay[:top_limit, :] = cv2.addWeighted(
        frame[:top_limit, :],
        0.65,
        colored_mask[:top_limit, :],
        0.35,
        0,
    )
    frame[:top_limit, :] = overlay[:top_limit, :]

    font_scale = max(0.5, height / 900.0)
    font_thickness = max(1, int(round(height / 500.0)))
    text_y = min(height - 10, top_limit + int(25 * (height / 480.0)))

    cv2.putText(
        frame,
        f"L={debug['left_ratio']:.3f} R={debug['right_ratio']:.3f}",
        (10, text_y),
        cv2.FONT_HERSHEY_SIMPLEX,
        font_scale,
        (255, 255, 255),
        font_thickness,
    )
    if debug["turn_text"]:
        cv2.putText(
            frame,
            debug["turn_text"],
            (10, min(height - 10, text_y + int(30 * font_scale))),
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            (0, 0, 255),
            font_thickness,
        )
    if fps is not None:
        cv2.putText(
            frame,
            f"FPS: {fps:.1f}",
            (10, max(20, int(20 * (height / 480.0)))),
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            (0, 255, 0),
            font_thickness,
        )


def run(cam_index=1):
    grabber = FrameGrabber(cam_index).start()
    previous_left = None
    previous_right = None
    last_fps_time = time.perf_counter()
    fps = 0.0
    frame_count = 0
    loop_index = 0
    last_debug = None

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, WINDOW_WIDTH, WINDOW_HEIGHT)

    while True:
        ret, frame = grabber.read()
        if not ret:
            time.sleep(0.005)
            continue

        frame_count += 1
        now = time.perf_counter()
        elapsed = now - last_fps_time
        if elapsed >= 1.0:
            fps = frame_count / elapsed
            frame_count = 0
            last_fps_time = now

        analysis = cv2.resize(frame, (ANALYSIS_WIDTH, ANALYSIS_HEIGHT), interpolation=cv2.INTER_AREA)

        if loop_index % PROCESS_EVERY_N == 0 or last_debug is None:
            last_debug = detect_turn_direction(analysis, previous_left, previous_right)
            previous_left = last_debug["left_ratio"]
            previous_right = last_debug["right_ratio"]

        debug = last_debug
        loop_index += 1

        display_frame = analysis.copy()
        draw_debug(display_frame, debug, fps=fps)
        cv2.imshow(WINDOW_NAME, display_frame)

        if cv2.waitKey(1) & 0xFF == 27:
            break

    grabber.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    run()
