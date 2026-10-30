"""Model JSON (livox_model_fit.py fit) -> the text file of socialtech_livox_sim: 'f_a f_z phi0 psi' then 24 c_az then 24 c_ze. usage: livox_model_to_txt.py MODEL.json OUT.lvm"""
import json
import sys

M = json.load(open(sys.argv[1])); U = M["unit"]
assert len(M["c_az"]) == 24 and len(M["c_ze"]) == 24
with open(sys.argv[2], "w") as f:
    f.write(" ".join(repr(float(U[k])) for k in ("f_a_hz", "f_z_hz", "phi0_rad", "psi_rad")) + "\n")
    for k in ("c_az", "c_ze"): f.write(" ".join(repr(float(v)) for v in M[k]) + "\n")
