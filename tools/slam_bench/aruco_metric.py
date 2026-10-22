"""M3DGR ArUco end-pose error (same formula as ArUco_evaluate.py). usage: aruco_metric.py <GT.txt> <est.tum> [end_time_s]"""
import sys
import numpy as np
from scipy.spatial.transform import Rotation as Rot

L = [l for l in open(sys.argv[1]).read().splitlines() if l.strip()]  # blank-line count differs between GT files
R_gt = np.array([[float(v) for v in L[i].split()] for i in (0, 1, 2)])
t_gt = np.array([float(L[i]) for i in (3, 4, 5)])
gt_time = float(L[-1].split(":")[1].replace("s", ""))
e = np.loadtxt(sys.argv[2])
T = lambda row: np.block([[Rot.from_quat(row[4:8]).as_matrix(), row[1:4, None]], [np.zeros((1, 3)), 1]])
end = float(sys.argv[3]) if len(sys.argv) > 3 else gt_time
for label, tend in (("at GT time", e[0, 0] + end), ("last pose", e[-1, 0])):
    row = e[np.searchsorted(e[:, 0], tend, side="right") - 1]
    rel = np.linalg.inv(T(e[0])) @ T(row)
    print(f"{label:11s} t={row[0]-e[0,0]:6.1f}s  trans_err={np.linalg.norm(t_gt-rel[:3,3]):.3f} m  rot_err(F)={np.linalg.norm(R_gt-rel[:3,:3]):.3f}  "
          f"est_t={np.round(rel[:3,3],3)}  yaw/pitch/roll_deg={np.round(Rot.from_matrix(rel[:3,:3]).as_euler('zyx',degrees=True),1)}")
print("GT t", t_gt, "GT ypr_deg", np.round(Rot.from_matrix(R_gt).as_euler('zyx', degrees=True), 1))
