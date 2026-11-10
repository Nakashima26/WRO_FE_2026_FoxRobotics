"""Publish the latest preview without JPEG/disk work on the control thread."""
import os
import threading
import uuid
from pathlib import Path

import cv2


class LiveJpegWriter:
    def __init__(self, path, quality=65):
        self.path = Path(path)
        self.quality = quality
        self._tmp = self.path.with_name(self.path.name + '.' + uuid.uuid4().hex + '.tmp')
        self._condition = threading.Condition()
        self._pending = None
        self._closed = False
        self._thread = threading.Thread(target=self._run, name='live-jpeg', daemon=True)
        self._thread.start()

    def write(self, frame):
        # Own the pixels: the runtime may annotate/reuse its frame immediately.
        snapshot = frame.copy()
        with self._condition:
            if self._closed:
                return
            self._pending = snapshot  # bounded to one pending frame
            self._condition.notify()

    def stop(self):
        with self._condition:
            self._closed = True
            self._condition.notify()
        self._thread.join()

    def _run(self):
        try:
            while True:
                with self._condition:
                    self._condition.wait_for(lambda: self._pending is not None or self._closed)
                    if self._pending is None:
                        return
                    frame, self._pending = self._pending, None
                try:
                    ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
                    if ok:
                        with self._tmp.open('wb') as f:
                            f.write(buf.tobytes())
                        os.replace(self._tmp, self.path)
                except Exception:
                    # A preview failure must not stop vehicle control.
                    pass
        finally:
            self._tmp.unlink(missing_ok=True)
