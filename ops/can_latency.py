"""Start and stop latency of the Tracer base from a `candump -L can_tracer` capture (ops/tracer_test.sh writes one): motion commands are 0x111, the base's
motion feedback is 0x221 (bytes 0-1: linear speed, signed mm/s). Latencies use the bus timestamps, so they do not include the start-up time of any ROS client.
  can_latency.py CAPTURE [MIN_MM_S=20]"""
import re
import sys


def parse(path):
    """-> ([time of each NON-zero 0x111 frame], [(time, linear speed mm/s) of each 0x221 frame])"""
    commands, feedback = [], []
    for line in open(path):
        m = re.match(r"\((\d+\.\d+)\) \S+ ([0-9A-F]+)#([0-9A-F]+)", line)
        if not m:
            continue
        t, ident, data = float(m[1]), m[2], m[3]
        if ident == "111" and data[:8] != "00000000":   # a motion command; the all-zero ones are stops (the test's final zero) and are not counted
            commands.append(t)
        elif ident == "221":
            v = int(data[:4], 16)
            feedback.append((t, v - 65536 if v > 32767 else v))
    return commands, feedback


def latencies(commands, feedback, min_speed):
    """-> (start s, stop s, peak mm/s): first command -> first speed >= min_speed; last command -> last speed >= min_speed (negative = it stopped before the last command)."""
    moving = [t for t, v in feedback if abs(v) >= min_speed and t > commands[0] - 1]
    if not moving:
        return None, None, max((abs(v) for _, v in feedback), default=0)
    return moving[0] - commands[0], moving[-1] - commands[-1], max(abs(v) for _, v in feedback)


if __name__ == "__main__":
    commands, feedback = parse(sys.argv[1])
    floor = float(sys.argv[2]) if len(sys.argv) > 2 else 20
    if not commands:
        sys.exit("no 0x111 command frame in the capture: nothing was sent")
    start, stop, peak = latencies(commands, feedback, floor)
    span = commands[-1] - commands[0]
    print(f"commands: {len(commands)} frames over {span:.2f} s (mean period {1000 * span / max(len(commands) - 1, 1):.0f} ms); peak feedback {peak} mm/s")
    if start is None:
        print(f"the base never moved at >= {floor:.0f} mm/s")
    else:
        print(f"start latency (first command -> first speed >= {floor:.0f} mm/s): {1000 * start:.0f} ms")
        print(f"stop latency (last command -> last speed >= {floor:.0f} mm/s, no zero sent): {1000 * stop:.0f} ms")
