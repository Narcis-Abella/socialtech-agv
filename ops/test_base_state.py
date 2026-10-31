"""Checks of base_state.verdict (no CAN needed): python3 ops/test_base_state.py"""
from base_state import verdict

NORMAL_CAN = (0, 1, 27.3, 0)
assert verdict(NORMAL_CAN, True)[0]
assert verdict(NORMAL_CAN, False)[0]
assert not verdict((2, 3, 27.3, 4), False)[0]      # as seen with the remote controller switches wrong: EXCEPTION, COMMANDED, fault 0x0004
assert not verdict((1, 1, 27.3, 0), True)[0]       # emergency stop
assert not verdict((0, 0, 27.3, 0), True)[0]       # NORMAL but in REMOTE mode: would ignore /cmd_vel
assert verdict((0, 0, 27.3, 0), False)[0]          # before tracer_base has asked for CAN mode that is fine
assert not verdict(None, False)[0]
assert "switches UP" in verdict((2, 3, 27.3, 4), False)[1]
print("ok")
