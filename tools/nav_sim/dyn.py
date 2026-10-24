"""Moving obstacles for the simulation: a model walks a polyline at constant speed (Gazebo set_pose at 20 Hz), and the offline distance between two oriented rectangles.
A trial's "dyn" list (see trials.py) holds {"model", "path", "speed", "trigger": [x, y, r], "size": [sx, sy], "parked": [x, y]}: the model waits parked out of the lidar's reach and starts walking when the
robot's ground-truth position enters the trigger circle, so the encounter does not depend on how fast Nav2 drives."""
import math
import threading
import time

HZ = 20.0


def path_pose(path, speed, t):
    """(x, y, yaw) after walking `path` at `speed` for t seconds: waits at the start for t < 0, stays at the end; yaw is the heading of the current leg."""
    pts = [path[0]] + [p for a, p in zip(path, path[1:]) if math.hypot(p[0] - a[0], p[1] - a[1]) > 0]
    if len(pts) == 1:
        return pts[0][0], pts[0][1], 0.0
    s, yaw = max(0.0, speed * t), 0.0
    for a, b in zip(pts, pts[1:]):
        leg = math.hypot(b[0] - a[0], b[1] - a[1])
        yaw = math.atan2(b[1] - a[1], b[0] - a[0])
        if s <= leg:
            f = s / leg
            return a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]), yaw
        s -= leg
    return pts[-1][0], pts[-1][1], yaw


def triggered(gt, trigger):
    return gt is not None and math.hypot(gt[0] - trigger[0], gt[1] - trigger[1]) <= trigger[2]


def corners(r):
    x, y, yaw, sx, sy = r
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * dx - s * dy, y + s * dx + c * dy) for dx, dy in ((sx / 2, sy / 2), (-sx / 2, sy / 2), (-sx / 2, -sy / 2), (sx / 2, -sy / 2))]


def _inside(p, poly):
    side = [(b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]) for a, b in zip(poly, poly[1:] + poly[:1])]
    return all(v >= 0 for v in side) or all(v <= 0 for v in side)


def _point_segment(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    f = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (dx * dx + dy * dy)))
    return math.hypot(p[0] - a[0] - f * dx, p[1] - a[1] - f * dy)


def _cross(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _segments_cross(a, b, c, d):
    return _cross(a, b, c) * _cross(a, b, d) < 0 and _cross(c, d, a) * _cross(c, d, b) < 0


def rect_distance(r1, r2):
    """Gap in metres between two oriented rectangles (x, y, yaw, size_x, size_y); 0 when they overlap or touch."""
    p, q = corners(r1), corners(r2)
    if any(_inside(v, q) for v in p) or any(_inside(v, p) for v in q):
        return 0.0
    ep, eq = list(zip(p, p[1:] + p[:1])), list(zip(q, q[1:] + q[:1]))
    if any(_segments_cross(a, b, c, d) for a, b in ep for c, d in eq):
        return 0.0
    return min([_point_segment(v, a, b) for v in p for a, b in eq] + [_point_segment(v, a, b) for v in q for a, b in ep])


class Mover:
    """Walks models along their paths from a thread, on the simulation clock `now()`; each model has its own start time. `log` collects [t, model, x, y, yaw] for every command sent."""

    def __init__(self, world, now):
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.pose_pb2 import Pose
        from gz.transport13 import Node
        self.node, self.Pose, self.Boolean, self.service, self.now = Node(), Pose, Boolean, f"/world/{world}/set_pose", now
        self.active, self.log, self.lock, self.alive = {}, [], threading.Lock(), True
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def put(self, model, x, y, yaw, z):
        m = self.Pose()
        m.name, m.position.x, m.position.y, m.position.z = model, x, y, z
        m.orientation.z, m.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.node.request(self.service, m, self.Pose, self.Boolean, 500)

    def park(self, items):
        with self.lock:
            self.active.clear()
        for d in items:
            self.put(d["model"], d["parked"][0], d["parked"][1], 0.0, d["size"][2] / 2)

    def start(self, d):
        with self.lock:
            self.active[d["model"]] = (d, self.now())

    def run(self):
        while self.alive:
            time.sleep(1.0 / HZ)
            with self.lock:
                todo = list(self.active.values())
            for d, t0 in todo:
                x, y, yaw = path_pose(d["path"], d["speed"], self.now() - t0)
                self.put(d["model"], x, y, yaw, d["size"][2] / 2)
                self.log.append([round(self.now(), 3), d["model"], round(x, 3), round(y, 3), round(yaw, 3)])

    def close(self):
        """Stops the thread before the interpreter exits: a thread still inside gz.transport at shutdown segfaults."""
        self.alive = False
        self.thread.join(timeout=3.0)
        self.node = None
