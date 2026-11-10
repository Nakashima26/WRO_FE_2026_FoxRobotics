import threading
import cv2
import numpy as np
from pure_pursuit.live_jpeg import LiveJpegWriter


def test_latest_pending_and_owned_pixels(tmp_path, monkeypatch):
    entered, release = threading.Event(), threading.Event()
    seen=[]
    encode=cv2.imencode
    def blocked(ext, frame, params):
        seen.append(int(frame[0,0,0]))
        if len(seen)==1:
            entered.set()
            assert release.wait(5)
        return encode(ext,frame,params)
    monkeypatch.setattr(cv2,'imencode',blocked)
    path=tmp_path/'preview.jpg'
    writer=LiveJpegWriter(path)
    try:
        writer.write(np.full((32,32,3),10,np.uint8))
        assert entered.wait(5)
        writer.write(np.full((32,32,3),20,np.uint8))
        last=np.full((32,32,3),30,np.uint8)
        writer.write(last)
        last[:]=255
        assert not path.exists()  # incomplete JPEG never published
    finally:
        release.set()
        writer.stop()
    assert seen==[10,30]
    assert np.all(cv2.imread(str(path))==30)
    assert not list(tmp_path.glob('*.tmp'))
    writer.stop()  # idempotent


def test_failed_encoding_keeps_previous_image(tmp_path, monkeypatch):
    path=tmp_path/'preview.jpg';path.write_bytes(b'previous')
    monkeypatch.setattr(cv2,'imencode',lambda *args: (False,None))
    writer=LiveJpegWriter(path)
    writer.write(np.zeros((8,8,3),np.uint8));writer.stop()
    assert path.read_bytes()==b'previous'
