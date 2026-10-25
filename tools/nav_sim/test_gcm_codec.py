"""Run: python3 -B test_gcm_codec.py"""
import numpy as np

import gcm_codec as gc


def test_a_frame_survives_encode_decode_and_is_small_when_mostly_free():
    data = np.zeros(520 * 280, np.int8)
    data[1000:1010] = 100
    data[5] = -1
    s = gc.encode(data)
    assert isinstance(s, str) and len(s) < 3000          # 145 kB of mostly zeros
    assert np.array_equal(gc.decode(s), data)


def test_frame_at_returns_the_last_frame_not_after_t():
    frames = [[1.0, "a"], [2.0, "b"], [4.0, "c"]]
    assert gc.frame_at(frames, 0.5) is None
    assert gc.frame_at(frames, 2.0) == "b" and gc.frame_at(frames, 3.9) == "b" and gc.frame_at(frames, 9.0) == "c"


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
    print("ok")
