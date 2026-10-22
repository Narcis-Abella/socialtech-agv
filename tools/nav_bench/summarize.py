"""One row per run directory under <root> (each with a report.json from pose_eval.py): summarize.py <root>"""
import json
import pathlib
import sys

HEAD = f"{'run':<28}{'conv_s':>7}{'pos_med':>8}{'pos_p95':>8}{'pos_max':>8}{'yaw_p95':>8}{'losses':>7}{'jumps':>6}{'corr_p95':>9}{'std_med':>8}{'drops':>6}"


def table(root):
    rows = [HEAD]
    for d in sorted(p for p in pathlib.Path(root).iterdir() if p.is_dir()):
        f = d / "report.json"
        if not f.exists():
            rows.append(f"{d.name:<28}NO REPORT")
            continue
        r = json.loads(f.read_text())
        log = d / "launch.log"
        drops = log.read_text(errors="replace").count("dropping") if log.exists() else 0  # messages AMCL lost: replay too fast for this configuration
        conv = f"{r['convergence_s']:.0f}" if r["converged"] else "never"
        rows.append(f"{d.name:<28}{conv:>7}{r['pos_err_m']['median']:>8.2f}{r['pos_err_m']['p95']:>8.2f}{r['pos_err_m']['max']:>8.2f}"
                    f"{r['yaw_err_deg']['p95']:>8.1f}{r['losses']:>7}{r['map_odom_jumps']:>6}{r.get('correction_at_robot_cm', {}).get('p95', float('nan')):>9.1f}{r.get('amcl_std_xy_m', {}).get('median', float('nan')):>8.2f}{drops:>6}")
    return "\n".join(rows)


if __name__ == "__main__":
    print(table(sys.argv[1]))
