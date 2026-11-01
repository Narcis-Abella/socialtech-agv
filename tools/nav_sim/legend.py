"""Text of the on-screen legend rviz2 shows while a job runs: the LEGEND env of the job (set by agx_queue.sh: name | overlays | overrides), one line per ' | ' part, then which trial is running."""


def legend_lines(legend, trial_id, k, n, wake=None):
    parts = [p.strip() for p in (legend or "").split(" | ") if p.strip()] or ["(no legend set)"]
    return parts + [f"trial {trial_id}  ({k}/{n})"] + ([] if wake is None else [f"estela en el costmap global: {wake} celdas"])
