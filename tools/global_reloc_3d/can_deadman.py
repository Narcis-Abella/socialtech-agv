"""Last software line behind the gate: watches the motion commands on the CAN bus itself and overrides a retained one with zeros.

tracer_base writes one motion frame (ID 0x111) per Twist it receives and nothing otherwise. If the last such frame is non-zero and
no newer one follows within DEADMAN_S, whoever was commanding is gone (gate, outlet, driver or DDS): this process then writes
all-zero 0x111 frames (exactly what tracer_base writes for a zero Twist) at 20 Hz until a newer frame from the driver appears.
It cannot help if this process, the computer or the CAN adapter dies: that case belongs to the base firmware's own timeout.

VALIDATED ON A VIRTUAL BUS ONLY. It refuses any interface whose name does not start with "vcan": putting a second transmitter on
the robot's bus is a decision for the maintainer, after the firmware timeout has been measured on hardware.
  can_deadman.py vcan0 [LOG.json]"""
import json
import select
import signal
import socket
import struct
import sys
import time

MOTION_ID = 0x111
DEADMAN_S = .3
PERIOD_S = .05
FRAME = struct.Struct('=IB3x8s')


def main():
    iface, log_path = sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None
    if not iface.startswith('vcan'):
        sys.exit(f'can_deadman: refusing {iface}: only virtual CAN interfaces (not validated on hardware)')
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, struct.pack('=II', MOTION_ID, socket.CAN_SFF_MASK))
    s.bind((iface,))
    stop = []
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.append(1))
    last_t, last_nonzero, last_tx, overriding = None, False, 0., False
    log = {'seen': 0, 'zeros_written': 0, 'overrides': []}
    while not stop:
        if select.select([s], [], [], .01)[0]:
            can_id, dlc, data = FRAME.unpack(s.recv(16))          # frames of other sockets only (own are not looped back)
            last_t, last_nonzero = time.monotonic(), any(data[:dlc])
            log['seen'] += 1
            if overriding:
                overriding = False
                log['overrides'][-1]['ended_epoch'] = time.time()
        now = time.monotonic()
        if last_t is not None and last_nonzero and now - last_t > DEADMAN_S and now - last_tx >= PERIOD_S:
            if not overriding:
                overriding = True
                log['overrides'].append({'started_epoch': time.time(), 'silence_s': now - last_t})
            s.send(FRAME.pack(MOTION_ID, 8, bytes(8)))
            last_tx = now; log['zeros_written'] += 1
    if log_path:
        json.dump(log, open(log_path, 'w'), indent=1)


if __name__ == '__main__':
    main()
