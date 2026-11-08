"""Is the Tracer base ready to be driven over CAN? Reads one system-state frame (0x211) and decodes it as ugv_sdk does (src/protocol_v1/agilex_protocol_v1.h:
vehicle_state, control_mode, battery_voltage x0.1 V, fault word). Exit 0 = ready, 1 = not, with what to do about it.
  base_state.py                 the vehicle must be NORMAL (no e-stop, no exception)
  base_state.py --require-can   and its control mode must be CAN: tracer_base asks for it (0x421) when it starts, so check AFTER the node is up
Seen on the robot: remote controller switches wrong -> state EXCEPTION, mode COMMANDED, fault 0x0004, every /cmd_vel ignored; switches up -> NORMAL, mode CAN."""
import argparse
import socket
import struct
import sys

STATE = {0: "NORMAL", 1: "EMERGENCY STOP", 2: "EXCEPTION"}
MODE = {0: "REMOTE", 1: "CAN", 2: "UART", 3: "COMMANDED"}
CAN_CMD = 1


def read_state(iface="can_tracer", timeout=3.0):
    """-> (vehicle_state, control_mode, volts, fault) from the next 0x211 frame, or None if none arrives in `timeout` s."""
    with socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as s:
        s.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, struct.pack("=II", 0x211, 0x7FF))
        s.bind((iface,))
        s.settimeout(timeout)
        try:
            frame = s.recv(16)
        except socket.timeout:
            return None
    _, _, data = struct.unpack("=IB3x8s", frame)
    return data[0], data[1], struct.unpack(">H", data[2:4])[0] / 10, struct.unpack(">H", data[4:6])[0]


def verdict(state, require_can):
    """-> (ready, message) for a read_state() result."""
    if state is None:
        return False, "no 0x211 frame in 3 s: the base is off or can_tracer is down (ip -br link show can_tracer)"
    vehicle, mode, volts, fault = state
    seen = f"state {STATE.get(vehicle, vehicle)}, mode {MODE.get(mode, mode)}, {volts:.1f} V, fault 0x{fault:04X}"
    if vehicle != 0:
        return False, f"base not NORMAL ({seen}): release the emergency stop and put the remote controller switches UP"
    if require_can and mode != CAN_CMD:
        return False, f"base not in CAN mode ({seen}): it would ignore /cmd_vel; put the remote controller switches UP"
    return True, f"base ok ({seen})"


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--require-can", action="store_true")
    ap.add_argument("--iface", default="can_tracer")
    args = ap.parse_args()
    ready, message = verdict(read_state(args.iface), args.require_can)
    print(("OK: " if ready else "NOT READY: ") + message)
    sys.exit(0 if ready else 1)
