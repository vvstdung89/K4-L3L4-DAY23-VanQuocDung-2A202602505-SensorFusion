"""Bonus study: camera-update visualisation and camera-calibration sensitivity.

One pass over the graded frame range. The detector runs once per frame; several
tracker variants then run side by side on the same detections:

- ``lidar``: lidar-only tracking (must reproduce ``metrics_lidar.json``).
- ``fused``: lidar + camera with the true calibration (must reproduce
  ``metrics_fused.json``).
- ``yaw_*`` / ``pitch_*``: lidar + camera where the tracker assumes a camera
  extrinsic rotated by a known error, while the camera measurements still come
  from the real image. This mimics a mis-calibrated camera.

The script only imports platform helpers; it does not modify ``platform/`` or
write to ``student/artifacts/``. Outputs go next to this file.

Usage (repo root, venv active):
    python student/bonus/bonus_study.py --config student/config/paths.yaml
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402
from scipy.stats import chi2  # noqa: E402

from fusion_lab import tracking_params  # noqa: E402
from fusion_lab.evaluation import TRACK_GATE_METERS  # noqa: E402
from fusion_lab.evaluation import _partial_assignment  # noqa: E402
from fusion_lab.evaluation import tracking_counts  # noqa: E402
from fusion_lab.evaluation import valid_ground_truth  # noqa: E402
from fusion_lab.scripts.run_lab import _front_observations  # noqa: E402
from fusion_lab.scripts.run_lab import _lidar_observations  # noqa: E402
from fusion_lab.scripts.run_lab import _load_paths_config  # noqa: E402
from fusion_lab.scripts.run_lab import _resolve_weights  # noqa: E402
from fusion_lab.scripts.run_lab import _setup_import_paths  # noqa: E402
from fusion_lab.tracking.filter import Filter  # noqa: E402
from fusion_lab.tracking.manager import TrackManager  # noqa: E402
from fusion_lab.tracking.sensors import Sensor  # noqa: E402
from fusion_lab.workspace_loader import load_workspace  # noqa: E402

OUT = Path(__file__).resolve().parent
ARTIFACTS = OUT.parent / "artifacts"
GATE_CAMERA = float(chi2.ppf(tracking_params.gating_threshold, 2))

# (name, uses camera, yaw error deg, pitch error deg)
VARIANTS = [
    ("lidar", False, 0.0, 0.0),
    ("fused", True, 0.0, 0.0),
    ("yaw_-2", True, -2.0, 0.0),
    ("yaw_-1", True, -1.0, 0.0),
    ("yaw_-0.5", True, -0.5, 0.0),
    ("yaw_0.25", True, 0.25, 0.0),
    ("yaw_0.5", True, 0.5, 0.0),
    ("yaw_1", True, 1.0, 0.0),
    ("yaw_2", True, 2.0, 0.0),
    ("yaw_5", True, 5.0, 0.0),
    ("pitch_0.5", True, 0.0, 0.5),
    ("pitch_1", True, 0.0, 1.0),
    ("pitch_2", True, 0.0, 2.0),
]


def _rotation(axis: str, degrees: float) -> np.ndarray:
    a = math.radians(degrees)
    c, s = math.cos(a), math.sin(a)
    if axis == "z":
        return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def perturb_extrinsic(sensor: Sensor, yaw_deg: float, pitch_deg: float) -> None:
    """Rotate the assumed vehicle-to-camera transform in the camera frame."""
    delta = np.eye(4)
    delta[:3, :3] = _rotation("z", yaw_deg) @ _rotation("y", pitch_deg)
    sensor.veh_to_sens = np.asmatrix(delta @ np.asarray(sensor.veh_to_sens))
    sensor.sens_to_veh = np.asmatrix(np.linalg.inv(np.asarray(sensor.veh_to_sens)))


class RecordingFilter(Filter):
    """Filter that logs every camera update: innovation, d², prior/posterior pixels."""

    def __init__(self, kalman_mod, association) -> None:
        super().__init__(kalman_mod)
        self._assoc = association
        self.frame = -1
        self.camera_updates: list[dict] = []

    def update(self, track, meas) -> None:
        if meas.sensor.name != "camera":
            super().update(track, meas)
            return
        gamma = np.asarray(self._k.innovation(track.x, meas)).ravel()
        d2 = self._assoc.mahalanobis_distance(track, meas)
        prior = np.asarray(meas.sensor.get_hx(track.x)).ravel()
        state = track.state
        super().update(track, meas)
        try:
            post = np.asarray(meas.sensor.get_hx(track.x)).ravel().tolist()
        except ValueError:
            post = None
        self.camera_updates.append({
            "frame": self.frame, "track": track.id, "state": state,
            "gamma_u": float(gamma[0]), "gamma_v": float(gamma[1]), "d2": float(d2),
            "z": np.asarray(meas.z).ravel().tolist(), "h_prior": prior.tolist(), "h_post": post,
        })


class Variant:
    def __init__(self, name, uses_camera, yaw, pitch, ws, seed):
        self.name, self.uses_camera, self.yaw, self.pitch = name, uses_camera, yaw, pitch
        self.filter = RecordingFilter(ws["kalman"], ws["association"])
        self.manager = TrackManager(ws["track_management"])
        self.rng = np.random.default_rng(seed)
        self.camera: Sensor | None = None
        self.totals = {"confirmed": 0, "matches": 0, "sum_sq_err": 0.0, "ghosts": 0, "misses": 0}
        self.nearest: list[dict] = []          # ungated nearest camera pair per visible confirmed track
        self.visible_confirmed = 0             # confirmed tracks inside camera FOV at camera pass
        self.errors: list[dict] = []           # per matched (frame, track) error vector
        self.snapshots: dict[int, list] = {}   # frame -> [(id, state, x, y, z)]


def matched_errors(tracks, labels):
    """Pair confirmed tracks with GT exactly like the grader (2 m XY gate)."""
    confirmed = [t for t in tracks if t.state == "confirmed"]
    if not confirmed or not labels:
        return []
    pos = np.array([np.asarray(t.x).reshape(-1)[:3] for t in confirmed])
    cen = np.array([(l.box.center_x, l.box.center_y, l.box.center_z) for l in labels])
    delta = pos[:, None, :] - cen[None, :, :]
    dist = np.linalg.norm(delta[:, :, :2], axis=2)
    pairs = _partial_assignment(dist, np.isfinite(dist) & (dist <= TRACK_GATE_METERS))
    return [(confirmed[i].id, delta[i, j]) for i, j in pairs]


def nearest_camera_pairs(tracks, observations, camera, kalman, association, frame):
    """Ungated diagnostics: each visible confirmed track's closest camera measurement."""
    rows, visible = [], 0
    for track in tracks:
        if track.state != "confirmed" or not camera.in_fov(track.x):
            continue
        visible += 1
        best = None
        for meas in observations:
            d2 = association.mahalanobis_distance(track, meas)
            if best is None or d2 < best[0]:
                best = (d2, np.asarray(kalman.innovation(track.x, meas)).ravel())
        if best is not None:
            rows.append({"frame": frame, "track": track.id, "d2": float(best[0]),
                         "gamma_u": float(best[1][0]), "gamma_v": float(best[1][1])})
    return rows, visible


def run(config_path: Path, seed: int) -> dict:
    _setup_import_paths()
    cfg = _load_paths_config(config_path)
    ws = load_workspace()
    kalman, assoc, cam = ws["kalman"], ws["association"], ws["camera_fusion"]
    bev, det_pipe = ws["bev_mapping"], ws["detection_pipeline"]

    from simple_waymo_open_dataset_reader import WaymoDataFileReader, dataset_pb2, label_pb2
    from simple_waymo_open_dataset_reader import utils as waymo_utils
    from fusion_lab.lidar_pcl import pcl_from_range_image

    tfrecord = Path(cfg["waymo_dir"]) / cfg["segment"]
    start, end = int(cfg.get("frame_start", 0)), int(cfg.get("frame_end", 20))
    weights = _resolve_weights(cfg)
    det_cfg = det_pipe.load_fpn_resnet_config(str(weights) if weights else None)
    model = det_pipe.create_fpn_model(det_cfg, str(weights) if weights else None)

    variants = [Variant(n, c, y, p, ws, seed) for n, c, y, p in VARIANTS]
    front, vehicle = dataset_pb2.CameraName.FRONT, label_pb2.Label.Type.TYPE_VEHICLE
    lidar_sensor = None
    frames = {}

    for cnt, frame in enumerate(WaymoDataFileReader(str(tfrecord))):
        if cnt < start:
            continue
        if cnt > end:
            break
        if lidar_sensor is None:
            lidar_sensor = Sensor("lidar", waymo_utils.get(
                frame.context.laser_calibrations, dataset_pb2.LaserName.TOP), cam)
            calib = waymo_utils.get(frame.context.camera_calibrations, front)
            for v in variants:
                if v.uses_camera:
                    v.camera = Sensor("camera", calib, cam)
                    perturb_extrinsic(v.camera, v.yaw, v.pitch)
        points = pcl_from_range_image(frame, dataset_pb2.LaserName.TOP)
        tensor = torch.from_numpy(bev.bev_maps_from_pcl(points, det_cfg)).unsqueeze(0).float()
        detections = det_pipe.detect_objects_from_bev(tensor, model, det_cfg)
        labels = valid_ground_truth(frame.laser_labels, det_cfg, vehicle)

        group = next((g for g in frame.camera_labels if g.name == front), None)
        image = next((im.image for im in frame.images if im.name == front), None)
        frames[cnt] = {
            "detections": [list(map(float, d[1:4])) for d in detections],
            "gt": [(l.box.center_x, l.box.center_y, l.box.center_z, l.box.length,
                    l.box.width, l.box.heading) for l in labels],
            "cam_boxes": [] if group is None else [
                (l.box.center_x, l.box.center_y, l.box.length, l.box.width)
                for l in group.labels if l.type == vehicle],
            "image": image,
        }

        for v in variants:
            v.filter.frame = cnt
            for track in v.manager.track_list:
                v.filter.predict(track)
                track.set_t(cnt * tracking_params.dt)
            observations = _lidar_observations(cnt, detections, lidar_sensor, det_cfg)
            assoc.associate_and_update(v.manager, observations, v.filter, lidar_sensor)
            if v.uses_camera:
                cam_obs = _front_observations(frame, cnt, v.camera, v.rng, front, vehicle)
                if cam_obs is not None:
                    rows, visible = nearest_camera_pairs(
                        v.manager.track_list, cam_obs, v.camera, kalman, assoc, cnt)
                    v.nearest.extend(rows)
                    v.visible_confirmed += visible
                    assoc.associate_and_update(v.manager, cam_obs, v.filter, v.camera)
            counts = tracking_counts(v.manager.track_list, labels)
            for key in v.totals:
                v.totals[key] += counts[key]
            for track_id, delta in matched_errors(v.manager.track_list, labels):
                v.errors.append({"frame": cnt, "track": track_id, "delta": delta.tolist()})
            v.snapshots[cnt] = [(t.id, t.state, *np.asarray(t.x).ravel()[:3].tolist())
                                for t in v.manager.track_list]
        if cnt % 20 == 0:
            print(f"frame {cnt}: done", flush=True)

    return {"variants": variants, "frames": frames, "range": [start, end]}


def summarise(variants) -> list[dict]:
    rows = []
    for v in variants:
        t = v.totals
        rmse = math.sqrt(t["sum_sq_err"] / t["matches"]) if t["matches"] else None
        deltas = np.array([e["delta"] for e in v.errors]).reshape(-1, 3)
        axis_rmse = np.sqrt((deltas ** 2).mean(axis=0)).tolist() if len(deltas) else [None] * 3
        upd = [u for u in v.filter.camera_updates if u["state"] == "confirmed"]
        near = v.nearest
        rows.append({
            "variant": v.name, "yaw_deg": v.yaw, "pitch_deg": v.pitch, "rmse": rmse,
            "rmse_x": axis_rmse[0], "rmse_y": axis_rmse[1], "rmse_z": axis_rmse[2],
            "matches": t["matches"], "sum_sq_err": t["sum_sq_err"],
            "ghosts": t["ghosts"], "misses": t["misses"],
            "camera_updates_all": len(v.filter.camera_updates),
            "camera_updates_confirmed": len(upd),
            "visible_confirmed": v.visible_confirmed,
            "update_rate": (len(upd) / v.visible_confirmed) if v.visible_confirmed else None,
            "applied_mean_gamma_u": float(np.mean([u["gamma_u"] for u in upd])) if upd else None,
            "applied_mean_gamma_v": float(np.mean([u["gamma_v"] for u in upd])) if upd else None,
            "applied_median_d2": float(np.median([u["d2"] for u in upd])) if upd else None,
            "nearest_mean_gamma_u": float(np.mean([n["gamma_u"] for n in near])) if near else None,
            "nearest_mean_gamma_v": float(np.mean([n["gamma_v"] for n in near])) if near else None,
            "nearest_median_d2": float(np.median([n["d2"] for n in near])) if near else None,
            "nearest_over_gate": (float(np.mean([n["d2"] >= GATE_CAMERA for n in near]))
                                  if near else None),
        })
    return rows


def check_reproduces_graded_run(rows) -> list[str]:
    """The lidar/fused variants must equal the committed graded artifacts."""
    notes = []
    for name in ("lidar", "fused"):
        path = ARTIFACTS / f"metrics_{name}.json"
        if not path.is_file():
            notes.append(f"{name}: {path.name} not found, skipped")
            continue
        graded = json.loads(path.read_text())["tracking"][name]
        row = next(r for r in rows if r["variant"] == name)
        same = (graded["matches"] == row["matches"]
                and math.isclose(graded["sum_sq_err"], row["sum_sq_err"], rel_tol=1e-9)
                and graded["ghost_track_frames"] == row["ghosts"]
                and graded["missed_gt_frames"] == row["misses"])
        notes.append(f"{name}: reproduces graded {path.name} = {same}")
    return notes


def write_table(rows, notes) -> None:
    def f(value, fmt):
        return "–" if value is None else format(value, fmt)

    lines = [
        "| Variant | yaw err (°) | pitch err (°) | RMSE (m) | ΔRMSE vs fused (m) | matches | ghosts | misses "
        "| cam updates (confirmed) | update rate | mean γu applied (px) | mean γv applied (px) "
        "| median d² applied | mean γu nearest (px) | mean γv nearest (px) | median d² nearest "
        "| nearest d² ≥ gate |",
        "|" + "---|" * 17,
    ]
    base = next(r["rmse"] for r in rows if r["variant"] == "fused")
    for r in rows:
        delta = None if r["rmse"] is None or r["variant"] == "lidar" else r["rmse"] - base
        lines.append(
            f"| {r['variant']} | {r['yaw_deg']:g} | {r['pitch_deg']:g} | {f(r['rmse'], '.4f')} "
            f"| {f(delta, '+.4f')} | {r['matches']} | {r['ghosts']} | {r['misses']} "
            f"| {r['camera_updates_confirmed']} | {f(r['update_rate'], '.3f')} "
            f"| {f(r['applied_mean_gamma_u'], '+.2f')} | {f(r['applied_mean_gamma_v'], '+.2f')} "
            f"| {f(r['applied_median_d2'], '.2f')} | {f(r['nearest_mean_gamma_u'], '+.2f')} "
            f"| {f(r['nearest_mean_gamma_v'], '+.2f')} | {f(r['nearest_median_d2'], '.2f')} "
            f"| {f(r['nearest_over_gate'], '.1%')} |")
    lines += ["", f"Camera χ² gate (2 dof, p = {tracking_params.gating_threshold}): d² < {GATE_CAMERA:.2f}",
              ""] + [f"- {n}" for n in notes]
    (OUT / "calibration_table.md").write_text("\n".join(lines) + "\n")


# ----------------------------------------------------------------------------- figures

def pick_frame(fused: Variant) -> int:
    """Frame with the most camera updates on confirmed tracks (ties: largest pixel shift)."""
    per_frame: dict[int, list] = {}
    for u in fused.filter.camera_updates:
        if u["state"] == "confirmed" and u["h_post"] is not None:
            per_frame.setdefault(u["frame"], []).append(u)

    def score(item):
        frame, ups = item
        shift = sum(np.hypot(*(np.subtract(u["h_post"], u["h_prior"]))) for u in ups)
        return (len(ups), shift)

    return max(per_frame.items(), key=score)[0]


def fig_bev(result, frame: int, lidar: Variant, fused: Variant) -> Path:
    data = result["frames"][frame]
    fig, ax = plt.subplots(figsize=(8, 9))
    for (x, y, _z, length, width, heading) in data["gt"]:
        corners = np.array([[length, width], [length, -width], [-length, -width],
                            [-length, width], [length, width]]) / 2
        rot = np.array([[math.cos(heading), -math.sin(heading)], [math.sin(heading), math.cos(heading)]])
        pts = corners @ rot.T + [x, y]
        ax.plot(-pts[:, 1], pts[:, 0], color="0.55", lw=1)
    if data["gt"]:
        gt = np.array(data["gt"])
        ax.scatter(-gt[:, 1], gt[:, 0], marker="+", s=120, c="k", label="GT vehicle centre (box in grey)")
    if data["detections"]:
        det = np.array(data["detections"])
        ax.scatter(-det[:, 1], det[:, 0], marker="o", s=30, facecolors="none", edgecolors="tab:orange",
                   label="LiDAR detection (FPN-ResNet)")
    err = {name: {e["track"]: np.linalg.norm(e["delta"]) for e in v.errors if e["frame"] == frame}
           for name, v in (("lidar", lidar), ("fused", fused))}
    for name, v, marker, color in (("lidar", lidar, "s", "tab:blue"), ("fused", fused, "^", "tab:red")):
        conf = [s for s in v.snapshots[frame] if s[1] == "confirmed"]
        if conf:
            arr = np.array([s[2:] for s in conf])
            ax.scatter(-arr[:, 1], arr[:, 0], marker=marker, s=45, facecolors="none", edgecolors=color,
                       lw=1.5, label=f"confirmed track — {'LiDAR-only' if name == 'lidar' else 'LiDAR + camera'}")
    for tid, _state, x, y, _z in [s for s in fused.snapshots[frame] if s[1] == "confirmed"]:
        el, ef = err["lidar"].get(tid), err["fused"].get(tid)
        text = f"T{tid}"
        if el is not None and ef is not None:
            text += f"\nerr L {el:.2f} m / F {ef:.2f} m"
        ax.annotate(text, (-y, x), xytext=(8, 6), textcoords="offset points", fontsize=8)
    cam = fused.camera
    origin = np.asarray(cam.sens_to_veh)[:3, 3]
    for angle in cam.fov:
        direction = np.asarray(cam.sens_to_veh)[:3, :3] @ [math.cos(angle), math.sin(angle), 0.0]
        tip = origin + 55 * direction
        ax.plot([-origin[1], -tip[1]], [origin[0], tip[0]], "g--", lw=1)
    ax.plot([], [], "g--", lw=1, label="FRONT camera FOV")
    pts = [(-g[1], g[0]) for g in data["gt"]] + [(-s[3], s[2]) for s in fused.snapshots[frame]]
    pts = np.array(pts) if pts else np.array([[0.0, 25.0]])
    ax.set_xlim(pts[:, 0].min() - 6, pts[:, 0].max() + 6)
    ax.set_ylim(max(pts[:, 1].min() - 6, -2), pts[:, 1].max() + 6)
    ax.set_aspect("equal")
    ax.set_xlabel("−y (m, right of ego →)")
    ax.set_ylabel("x (m, forward)")
    ax.set_title(f"BEV frame {frame} (zoomed): LiDAR-only vs LiDAR + camera tracks against GT\n"
                 "(err = 3D distance from track to its matched GT centre)")
    ax.legend(loc="lower left", fontsize=8)
    ax.grid(alpha=0.3)
    path = OUT / f"fig1_bev_frame{frame:03d}.png"
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return path


def fig_camera(result, frame: int, fused: Variant) -> Path:
    data = result["frames"][frame]
    image = cv2.cvtColor(cv2.imdecode(np.frombuffer(data["image"], np.uint8), cv2.IMREAD_COLOR),
                         cv2.COLOR_BGR2RGB)
    ups = [u for u in fused.filter.camera_updates
           if u["frame"] == frame and u["state"] == "confirmed" and u["h_post"] is not None]
    n = len(ups)
    fig = plt.figure(figsize=(4.2 * max(n, 2), 9))
    grid = fig.add_gridspec(2, max(n, 2), height_ratios=[1.3, 1])
    ax = fig.add_subplot(grid[0, :])
    ax.imshow(image)
    for (cx, cy, length, width) in data["cam_boxes"]:
        ax.add_patch(plt.Rectangle((cx - length / 2, cy - width / 2), length, width,
                                   fill=False, ec="yellow", lw=1))
    for u in ups:
        ax.plot(*u["z"], "g+", ms=10, mew=2)
        ax.annotate(f"T{u['track']}", u["z"], xytext=(6, -8), textcoords="offset points",
                    color="white", fontsize=9, weight="bold")
    ax.set_title(f"FRONT camera, frame {frame}: GT 2D boxes (yellow) and camera measurements z "
                 f"(green +) for {n} confirmed tracks updated by the camera")
    ax.axis("off")
    for k, u in enumerate(ups):
        sub = fig.add_subplot(grid[1, k])
        cx, cy = u["z"]
        spread = max(np.hypot(*np.subtract(p, u["z"])) for p in (u["h_prior"], u["h_post"]))
        half = max(25, 1.4 * spread)
        x0, x1 = int(max(cx - half, 0)), int(min(cx + half, image.shape[1]))
        y0, y1 = int(max(cy - half, 0)), int(min(cy + half, image.shape[0]))
        sub.imshow(image[y0:y1, x0:x1], extent=(x0, x1, y1, y0))
        sub.plot(*u["z"], "g+", ms=14, mew=2, label="z (camera measurement)")
        sub.plot(*u["h_prior"], "o", mfc="none", mec="red", ms=10, mew=2, label="h(x⁻) before camera update")
        sub.plot(*u["h_post"], "o", mfc="none", mec="cyan", ms=10, mew=2, label="h(x⁺) after camera update")
        sub.annotate("", xy=u["h_post"], xytext=u["h_prior"],
                     arrowprops={"arrowstyle": "->", "color": "white", "lw": 1.5})
        before = np.hypot(*np.subtract(u["z"], u["h_prior"]))
        after = np.hypot(*np.subtract(u["z"], u["h_post"]))
        sub.set_title(f"T{u['track']}: |z − h(x)| {before:.1f} px → {after:.1f} px\n"
                      f"d² = {u['d2']:.2f} (gate {GATE_CAMERA:.1f})", fontsize=9)
        sub.set_xlim(x0, x1)
        sub.set_ylim(y1, y0)
        sub.tick_params(labelsize=7)
        if k == 0:
            sub.legend(fontsize=7, loc="lower left")
    path = OUT / f"fig2_camera_update_frame{frame:03d}.png"
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return path


def track_summary(lidar: Variant, fused: Variant, min_frames: int = 30) -> list[dict]:
    """Per-track mean 3D error in both modes over frames matched in both."""
    def by_track(v):
        out: dict[int, dict[int, np.ndarray]] = {}
        for e in v.errors:
            out.setdefault(e["track"], {})[e["frame"]] = np.array(e["delta"])
        return out

    lt, ft = by_track(lidar), by_track(fused)
    rows = []
    for tid in sorted(set(lt) & set(ft)):
        frames = sorted(set(lt[tid]) & set(ft[tid]))
        if len(frames) < min_frames:
            continue
        dl = np.array([lt[tid][f] for f in frames])
        df = np.array([ft[tid][f] for f in frames])
        rows.append({
            "track": tid, "frames": len(frames), "first": frames[0], "last": frames[-1],
            "mean_err_lidar": float(np.linalg.norm(dl, axis=1).mean()),
            "mean_err_fused": float(np.linalg.norm(df, axis=1).mean()),
            "rmse_axis_lidar": np.sqrt((dl ** 2).mean(axis=0)).tolist(),
            "rmse_axis_fused": np.sqrt((df ** 2).mean(axis=0)).tolist(),
            "_series": (frames, np.linalg.norm(dl, axis=1), np.linalg.norm(df, axis=1)),
        })
    return rows


def fig_track_error(lidar: Variant, fused: Variant, tracks: list[dict]) -> tuple[Path, list[int]]:
    best = min(tracks, key=lambda r: r["mean_err_fused"] - r["mean_err_lidar"])
    worst = max(tracks, key=lambda r: r["mean_err_fused"] - r["mean_err_lidar"])
    fig, axes = plt.subplots(1, 3, figsize=(17, 4.6), gridspec_kw={"width_ratios": [1.6, 1.6, 1]})
    for ax, row, tag in ((axes[0], best, "camera helps most"), (axes[1], worst, "camera hurts most")):
        frames, el, ef = row["_series"]
        cam_frames = sorted({u["frame"] for u in fused.filter.camera_updates if u["track"] == row["track"]})
        ax.plot(frames, el, color="tab:blue", lw=1.2, label=f"LiDAR-only (mean {el.mean():.3f} m)")
        ax.plot(frames, ef, color="tab:red", lw=1.2, label=f"LiDAR + camera (mean {ef.mean():.3f} m)")
        ax.plot(cam_frames, np.full(len(cam_frames), -0.01), "|", color="green", ms=8,
                label="camera update on this track")
        rl, rf = row["rmse_axis_lidar"], row["rmse_axis_fused"]
        ax.set_title(f"T{row['track']} ({tag}), {row['frames']} matched frames\n"
                     f"RMSE x/y/z: L {rl[0]:.3f}/{rl[1]:.3f}/{rl[2]:.3f}  F {rf[0]:.3f}/{rf[1]:.3f}/{rf[2]:.3f} m",
                     fontsize=10)
        ax.set_xlabel("frame")
        ax.set_ylabel("3D position error to matched GT (m)")
        ax.legend(fontsize=8)
        ax.grid(alpha=0.3)

    rl = [math.sqrt(np.mean([np.array(e["delta"])[i] ** 2 for e in lidar.errors])) for i in range(3)]
    rf = [math.sqrt(np.mean([np.array(e["delta"])[i] ** 2 for e in fused.errors])) for i in range(3)]
    idx = np.arange(3)
    ax2 = axes[2]
    ax2.bar(idx - 0.2, rl, 0.4, color="tab:blue", label="LiDAR-only")
    ax2.bar(idx + 0.2, rf, 0.4, color="tab:red", label="LiDAR + camera")
    for i in range(3):
        ax2.text(i - 0.2, rl[i], f"{rl[i]:.3f}", ha="center", va="bottom", fontsize=8)
        ax2.text(i + 0.2, rf[i], f"{rf[i]:.3f}", ha="center", va="bottom", fontsize=8)
    ax2.set_xticks(idx, ["x (forward)", "y (lateral)", "z (height)"], fontsize=8)
    ax2.set_ylabel("RMSE per axis, all 502 matched pairs (m)")
    ax2.set_title("All tracks: per-axis RMSE")
    ax2.legend(fontsize=8)
    ax2.grid(alpha=0.3, axis="y")
    path = OUT / "fig3_track_error_and_axis_rmse.png"
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return path, [best["track"], worst["track"]]


def fig_calibration(rows) -> Path:
    lidar_rmse = next(r["rmse"] for r in rows if r["variant"] == "lidar")
    base = next(r for r in rows if r["variant"] == "fused")
    yaw = sorted([base] + [r for r in rows if r["variant"].startswith("yaw_")], key=lambda r: r["yaw_deg"])
    pitch = [base] + [r for r in rows if r["variant"].startswith("pitch_")]

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.4))
    for series, key, color, label in ((yaw, "yaw_deg", "tab:purple", "yaw error"),
                                      (pitch, "pitch_deg", "tab:brown", "pitch error")):
        x = [r[key] for r in series]
        axes[0].plot(x, [r["rmse"] for r in series], "o-", color=color, label=label)
        axes[1].plot(x, [r["update_rate"] for r in series], "o-", color=color, label=f"update rate, {label}")
        axes[2].plot(x, [r["nearest_median_d2"] for r in series], "o-", color=color, label=label)
    axes[0].axhline(lidar_rmse, color="tab:blue", ls="--", label="LiDAR-only")
    axes[0].set_ylabel("tracking RMSE (m)")
    axes[0].set_title("RMSE vs injected extrinsic error")
    axes[1].set_ylabel("camera updates / visible confirmed tracks")
    axes[1].set_title("How often the camera still passes the χ² gate")
    axes[2].axhline(GATE_CAMERA, color="k", ls="--", label=f"χ² gate {GATE_CAMERA:.1f}")
    axes[2].set_yscale("log")
    axes[2].set_ylabel("median d² of nearest camera pair (ungated)")
    axes[2].set_title("Innovation size seen by the gate")
    for ax in axes:
        ax.set_xlabel("injected rotation error (°)")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    path = OUT / "fig4_calibration_sweep.png"
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    result = run(args.config, args.seed)
    variants = {v.name: v for v in result["variants"]}
    rows = summarise(result["variants"])
    notes = check_reproduces_graded_run(rows)
    write_table(rows, notes)
    frame = pick_frame(variants["fused"])
    figures = [fig_bev(result, frame, variants["lidar"], variants["fused"]),
               fig_camera(result, frame, variants["fused"])]
    tracks = track_summary(variants["lidar"], variants["fused"])
    path, tid = fig_track_error(variants["lidar"], variants["fused"], tracks)
    figures += [path, fig_calibration(rows)]
    (OUT / "calibration_results.json").write_text(json.dumps({
        "frames": result["range"], "seed": args.seed, "camera_gate_d2": GATE_CAMERA,
        "figure_frame": frame, "figure_tracks": tid, "reproduction_check": notes, "variants": rows,
        "tracks_lidar_vs_fused": [{k: v for k, v in r.items() if k != "_series"} for r in tracks],
    }, indent=2))
    print("\n".join(notes))
    print((OUT / "calibration_table.md").read_text())
    print("figures:", *[p.name for p in figures])


if __name__ == "__main__":
    main()
