#!/usr/bin/env python3
from __future__ import annotations

import argparse
import itertools
import json
import hashlib
import os
import random
import subprocess
import sys
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple


def _import_compute_score(repo_root: str):
    import sys as _sys

    src_dir = os.path.join(repo_root, "src")
    if src_dir not in _sys.path:
        _sys.path.insert(0, src_dir)
    from ext_metrics import compute_score  # type: ignore

    return compute_score


def _load_json(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _safe_get(d: Dict[str, Any], keys: List[str], default=None):
    cur: Any = d
    for k in keys:
        if not isinstance(cur, dict) or k not in cur:
            return default
        cur = cur[k]
    return cur


def _load_yaml(path: str) -> Dict[str, Any]:
    import yaml
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def _dump_yaml(path: str, data: Dict[str, Any]) -> None:
    import yaml
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)


def _find_latest_log(log_dir: str, prefix: str) -> Optional[str]:
    if not os.path.isdir(log_dir):
        return None
    cand = []
    for fn in os.listdir(log_dir):
        if not fn.endswith(".json"):
            continue
        if prefix and not fn.startswith(prefix):
            continue
        p = os.path.join(log_dir, fn)
        try:
            cand.append((os.path.getmtime(p), p))
        except OSError:
            pass
    if not cand:
        return None
    cand.sort(reverse=True)
    return cand[0][1]


def _run(cmd: List[str], cwd: Optional[str] = None, env: Optional[Dict[str, str]] = None) -> int:
    p = subprocess.run(cmd, cwd=cwd, env=env)
    return p.returncode


@dataclass
class TrialResult:
    idx: int
    params: Dict[str, Any]
    # aggregated across repeats
    repeats: int
    success_count: int
    status: str  # e.g. "DONE@2/3" or "ABORT@0/3"
    survival_mean: float
    survival_min: float
    score_mean: float
    tilt_mean: Optional[float]
    energy_mean: Optional[float]
    abort_reasons: List[str]
    log_paths: List[str]


def _score(tr: TrialResult) -> float:
    return float(tr.score_mean)


def _success_rate(tr: TrialResult) -> float:
    if tr.repeats <= 0:
        return 0.0
    return float(tr.success_count) / float(tr.repeats)


def _best_key(tr: TrialResult) -> tuple:
    """
    Selection policy (practical/stable):
      1) success_rate (DONE fraction)  [higher is better]
      2) survival_min (worst-case)     [higher is better]
      3) score_mean (average score)    [higher is better]
      4) tilt_mean (tie-breaker)       [lower is better]
    """
    sr = _success_rate(tr)
    sm = float(tr.survival_min)
    sc = float(tr.score_mean)
    tilt = float(tr.tilt_mean) if tr.tilt_mean is not None else 1e9
    return (sr, sm, sc, -tilt)


def _extract_single(
    log_path: str, compute_score_fn
) -> Tuple[str, Dict[str, Any], float, Optional[float], Optional[float], Optional[str]]:
    payload = _load_json(log_path)
    result = payload.get("result", {})
    metrics = result.get("metrics", {}) if isinstance(result, dict) else {}
    abort = result.get("abort", {}) if isinstance(result, dict) else {}

    status = str(result.get("status", "UNKNOWN"))
    survival = float(metrics.get("survival_time", 0.0) or 0.0)
    tilt = metrics.get("tilt_max_abs", None)
    energy = metrics.get("energy_abs_tau_dq", None)
    reason = abort.get("reason") if isinstance(abort, dict) else None

    _ = float(compute_score_fn(metrics if isinstance(metrics, dict) else {}, status))
    return (
        status,
        metrics if isinstance(metrics, dict) else {},
        survival,
        float(tilt) if tilt is not None else None,
        float(energy) if energy is not None else None,
        str(reason) if reason is not None else None,
    )


def _mk_grid(values: List[float]) -> List[float]:
    # remove duplicates while preserving order
    out = []
    seen = set()
    for v in values:
        if v in seen:
            continue
        seen.add(v)
        out.append(v)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description="Parameter sweep for standing-pd-ext using agent_tuning.yaml")
    ap.add_argument("--config", default="config/agent_tuning.yaml", help="Base YAML config to modify")
    ap.add_argument("--best-out", default="config/agent_tuning_best.yaml", help="Where to write best YAML")
    ap.add_argument("--log-dir", default="runs", help="Run logs directory")
    ap.add_argument("--prefix", default="standing_pd_ext", help="Log prefix")
    ap.add_argument("--trials", type=int, default=24, help="Number of trials (random mode)")
    ap.add_argument("--repeats", type=int, default=1, help="Repeat each parameter set N times and aggregate")
    ap.add_argument("--mode", choices=["grid", "random"], default="grid", help="Sweep mode")
    ap.add_argument("--seed", type=int, default=0, help="Random seed (random mode)")
    ap.add_argument("--duration", type=float, default=None, help="Override runner.seconds (optional)")
    ap.add_argument("--no-gui", action="store_true", help="Force --no-gui for main_simulation.py")
    ap.add_argument("--dry-run", action="store_true", help="Print planned trials but do not run")
    ap.add_argument("--warmup", default="1.0", help="Comma-separated warmup_seconds candidates (two_stage)")
    ap.add_argument("--blend", default="0.0", help="Comma-separated blend_seconds candidates (two_stage)")
    ap.add_argument("--grav", default="0,1", help="Comma-separated use_gravity_comp candidates: 0 or 1 (two_stage)")
    ap.add_argument("--grav-scale", default="1.0", help="Comma-separated gravity_scale candidates (two_stage)")
    ap.add_argument("--kd-blend", default="1.0", help="Comma-separated kd_blend_factor candidates (two_stage)")
    ap.add_argument("--pos-kp", default=None, help="Comma-separated warmup position kp candidates (two_stage)")
    ap.add_argument("--pos-kd", default=None, help="Comma-separated warmup position kd candidates (two_stage)")
    ap.add_argument(
        "--stance",
        default="standing",
        help="Comma-separated stance candidates: standing,crouch_pos,crouch_neg",
    )
    ap.add_argument(
        "--crouch-knee",
        default=None,
        help="Comma-separated crouch knee magnitudes (used when stance is crouch_*)",
    )
    ap.add_argument(
        "--crouch-ankle",
        default=None,
        help="Comma-separated crouch ankle magnitudes (used when stance is crouch_*)",
    )
    ap.add_argument("--max-roll", default=None, help="Comma-separated safety max_roll (rad) overrides")
    ap.add_argument("--max-pitch", default=None, help="Comma-separated safety max_pitch (rad) overrides")
    ap.add_argument("--base-height", default=None, help="Comma-separated runner base_height overrides (m)")
    ap.add_argument(
        "--grid-sample",
        default="spread",
        choices=["spread", "random"],
        help="How to downsample grid when --trials < full grid size",
    )

    ap.add_argument("--kp", default="20,40,60,80,100,120", help="Comma-separated kp candidates (grid)")
    ap.add_argument("--kd", default="0.5,1.0,1.5,2.0,3.0,4.0", help="Comma-separated kd candidates (grid)")
    ap.add_argument("--tau", default="30,60,90,120", help="Comma-separated tau_limit candidates (grid)")
    ap.add_argument("--control-dt", default="0.01,0.001", help="Comma-separated control_dt candidates (grid)")
    ap.add_argument("--settle", default="0,300,800", help="Comma-separated settle_steps candidates (grid)")
    args = ap.parse_args()

    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    src_dir = os.path.join(repo_root, "src")

    compute_score_fn = _import_compute_score(repo_root)

    base_cfg = _load_yaml(os.path.join(repo_root, args.config))
    runner_cfg = (base_cfg.get("runner") or {})
    ctrl_cfg = (base_cfg.get("controller") or {})

    def parse_floats(s: str) -> List[float]:
        return [float(x.strip()) for x in s.split(",") if x.strip()]

    kps = _mk_grid(parse_floats(args.kp))
    kds = _mk_grid(parse_floats(args.kd))
    taus = _mk_grid(parse_floats(args.tau))
    dts = _mk_grid(parse_floats(args.control_dt))
    settles = [int(float(x)) for x in args.settle.split(",") if x.strip()]
    warmups = _mk_grid(parse_floats(args.warmup))
    blends = _mk_grid(parse_floats(args.blend))
    grav_flags = [int(float(x)) for x in args.grav.split(",") if x.strip()]
    grav_scales = _mk_grid(parse_floats(args.grav_scale))
    kd_blends = _mk_grid(parse_floats(args.kd_blend))
    pos_kps = _mk_grid(parse_floats(args.pos_kp)) if args.pos_kp is not None else None
    pos_kds = _mk_grid(parse_floats(args.pos_kd)) if args.pos_kd is not None else None
    stances = [s.strip() for s in str(args.stance).split(",") if s.strip()]
    if not stances:
        stances = ["standing"]
    crouch_knees = _mk_grid(parse_floats(args.crouch_knee)) if args.crouch_knee is not None else None
    crouch_ankles = _mk_grid(parse_floats(args.crouch_ankle)) if args.crouch_ankle is not None else None
    max_rolls = _mk_grid(parse_floats(args.max_roll)) if args.max_roll is not None else None
    max_pitches = _mk_grid(parse_floats(args.max_pitch)) if args.max_pitch is not None else None
    base_heights = _mk_grid(parse_floats(args.base_height)) if args.base_height is not None else None

    def _stable_hash(x: Any) -> str:
        # Stable, deterministic ordering independent of Python's hash randomization.
        b = repr(x).encode("utf-8")
        return hashlib.md5(b).hexdigest()

    def _downsample_grid(all_params: List[Any], k: int) -> List[Any]:
        """
        Downsample a full-factorial grid to k points with less bias than naive step slicing.
        - spread: hash-sort then take evenly spaced indices (deterministic)
        - random: deterministic shuffle by seed then take first k
        """
        if k <= 0:
            return []
        if len(all_params) <= k:
            return all_params

        if args.grid_sample == "random":
            rng = random.Random(args.seed)
            tmp = list(all_params)
            rng.shuffle(tmp)
            return tmp[:k]

        # spread (default): hash-sort then evenly pick indices
        tmp = sorted(all_params, key=_stable_hash)
        if k == 1:
            return [tmp[len(tmp) // 2]]

        n = len(tmp)
        picks = []
        for i in range(k):
            idx = int(round(i * (n - 1) / (k - 1)))
            picks.append(tmp[idx])

        # De-dup in case of rounding collisions; fill from front if needed.
        seen = set()
        out = []
        for p in picks:
            if p not in seen:
                seen.add(p)
                out.append(p)
        for p in tmp:
            if len(out) >= k:
                break
            if p not in seen:
                seen.add(p)
                out.append(p)
        return out

    planned_params: List[Dict[str, Any]] = []

    if args.mode == "grid":
        _pos_kps = pos_kps if pos_kps is not None else [None]
        _pos_kds = pos_kds if pos_kds is not None else [None]
        _ck = crouch_knees if crouch_knees is not None else [None]
        _ca = crouch_ankles if crouch_ankles is not None else [None]
        _mr = max_rolls if max_rolls is not None else [None]
        _mp = max_pitches if max_pitches is not None else [None]
        _bh = base_heights if base_heights is not None else [None]
        all_params = list(
            itertools.product(
                kps,
                kds,
                taus,
                dts,
                settles,
                warmups,
                blends,
                grav_flags,
                grav_scales,
                kd_blends,
                _pos_kps,
                _pos_kds,
                stances,
                _ck,
                _ca,
                _mr,
                _mp,
                _bh,
            )
        )
        max_grid = max(1, args.trials)
        if len(all_params) > max_grid:
            all_params = _downsample_grid(all_params, max_grid)
        for kp, kd, tau, dt, settle, warmup, blend, grav, gscale, kd_blend, pos_kp, pos_kd, stance, ck, ca, mr, mp, bh in all_params:
            planned_params.append(
                {
                    "kp": kp,
                    "kd": kd,
                    "tau_limit": tau,
                    "control_dt": dt,
                    "settle_steps": settle,
                    "warmup_seconds": warmup,
                    "blend_seconds": blend,
                    "use_gravity_comp": bool(int(grav)),
                    "gravity_scale": float(gscale),
                    "kd_blend_factor": float(kd_blend),
                    "pos_kp": float(pos_kp) if pos_kp is not None else None,
                    "pos_kd": float(pos_kd) if pos_kd is not None else None,
                    "stance": str(stance),
                    "crouch_knee": float(ck) if ck is not None else None,
                    "crouch_ankle": float(ca) if ca is not None else None,
                    "max_roll": float(mr) if mr is not None else None,
                    "max_pitch": float(mp) if mp is not None else None,
                    "base_height": float(bh) if bh is not None else None,
                }
            )

        try:
            used = {
                "kp": sorted({p["kp"] for p in planned_params}),
                "kd": sorted({p["kd"] for p in planned_params}),
                "tau_limit": sorted({p["tau_limit"] for p in planned_params}),
                "settle_steps": sorted({p["settle_steps"] for p in planned_params}),
                "warmup_seconds": sorted({p["warmup_seconds"] for p in planned_params}),
                "blend_seconds": sorted({p["blend_seconds"] for p in planned_params}),
                "use_gravity_comp": sorted({p["use_gravity_comp"] for p in planned_params}),
                "gravity_scale": sorted({p["gravity_scale"] for p in planned_params}),
                "kd_blend_factor": sorted({p["kd_blend_factor"] for p in planned_params}),
                "pos_kp": sorted({p["pos_kp"] for p in planned_params}),
                "pos_kd": sorted({p["pos_kd"] for p in planned_params}),
                "stance": sorted({p["stance"] for p in planned_params}),
                "crouch_knee": sorted({p["crouch_knee"] for p in planned_params}),
                "crouch_ankle": sorted({p["crouch_ankle"] for p in planned_params}),
                "max_roll": sorted({p["max_roll"] for p in planned_params}),
                "max_pitch": sorted({p["max_pitch"] for p in planned_params}),
                "base_height": sorted({p["base_height"] for p in planned_params}),
            }
            print("[grid] actual coverage:", used)
        except Exception:
            pass
    else:
        random.seed(args.seed)
        for _ in range(args.trials):
            planned_params.append(
                {
                    "kp": random.choice(kps),
                    "kd": random.choice(kds),
                    "tau_limit": random.choice(taus),
                    "control_dt": random.choice(dts),
                    "settle_steps": random.choice(settles),
                    "warmup_seconds": random.choice(warmups),
                    "blend_seconds": random.choice(blends),
                    "use_gravity_comp": bool(int(random.choice(grav_flags))),
                    "gravity_scale": float(random.choice(grav_scales)),
                    "kd_blend_factor": float(random.choice(kd_blends)),
                    "pos_kp": float(random.choice(pos_kps)) if pos_kps is not None else None,
                    "pos_kd": float(random.choice(pos_kds)) if pos_kds is not None else None,
                    "stance": str(random.choice(stances)) if stances else "standing",
                    "crouch_knee": float(random.choice(crouch_knees)) if crouch_knees is not None else None,
                    "crouch_ankle": float(random.choice(crouch_ankles)) if crouch_ankles is not None else None,
                    "max_roll": float(random.choice(max_rolls)) if max_rolls is not None else None,
                    "max_pitch": float(random.choice(max_pitches)) if max_pitches is not None else None,
                    "base_height": float(random.choice(base_heights)) if base_heights is not None else None,
                }
            )

    if args.dry_run:
        print("Planned trials:")
        for i, p in enumerate(planned_params, 1):
            print(i, p)
        return 0

    best: Optional[TrialResult] = None
    results: List[TrialResult] = []

    for i, p in enumerate(planned_params, 1):
        cfg = dict(base_cfg)
        cfg_runner = dict(runner_cfg)
        cfg_ctrl = dict(ctrl_cfg)
        cfg_safety = dict(cfg.get("safety") or {})

        if p.get("max_roll", None) is not None:
            cfg_safety["max_roll"] = float(p["max_roll"])
        if p.get("max_pitch", None) is not None:
            cfg_safety["max_pitch"] = float(p["max_pitch"])
        cfg["safety"] = cfg_safety

        if p.get("base_height", None) is not None:
            cfg_runner["base_height"] = float(p["base_height"])

        if args.duration is not None:
            cfg_runner["seconds"] = float(args.duration)
        cfg_runner["control_dt"] = float(p["control_dt"])
        cfg_runner["settle_steps"] = int(p["settle_steps"])
        base_seed = int((runner_cfg.get("seed", 0) or 0))
        cfg_runner["seed"] = base_seed

        # Support both controller layouts:
        #  - legacy: controller.{kp,kd,tau_limit}
        #  - two_stage: controller.type == "two_stage" and controller.torque.{kp,kd,tau_limit}
        ctrl_type = str(cfg_ctrl.get("type", "torque_pd"))
        if p.get("stance", None) is not None:
            cfg_ctrl["stance"] = str(p["stance"])
        if p.get("crouch_knee", None) is not None:
            cfg_ctrl["crouch_knee"] = float(p["crouch_knee"])
        if p.get("crouch_ankle", None) is not None:
            cfg_ctrl["crouch_ankle"] = float(p["crouch_ankle"])
        if ctrl_type == "two_stage":
            cfg_ctrl["warmup_seconds"] = float(p.get("warmup_seconds", cfg_ctrl.get("warmup_seconds", 1.0)))
            cfg_ctrl["blend_seconds"] = float(p.get("blend_seconds", cfg_ctrl.get("blend_seconds", 0.0)))

            pos_block = dict(cfg_ctrl.get("position") or {})
            if p.get("pos_kp", None) is not None:
                pos_block["kp"] = float(p["pos_kp"])
            if p.get("pos_kd", None) is not None:
                pos_block["kd"] = float(p["pos_kd"])
            cfg_ctrl["position"] = pos_block

            torque_block = dict(cfg_ctrl.get("torque") or {})
            torque_block["kp"] = float(p["kp"])
            torque_block["kd"] = float(p["kd"])
            torque_block["tau_limit"] = float(p["tau_limit"])
            torque_block["use_gravity_comp"] = bool(
                p.get("use_gravity_comp", torque_block.get("use_gravity_comp", False))
            )
            torque_block["gravity_scale"] = float(p.get("gravity_scale", torque_block.get("gravity_scale", 1.0)))
            torque_block["kd_blend_factor"] = float(p.get("kd_blend_factor", torque_block.get("kd_blend_factor", 1.0)))
            cfg_ctrl["torque"] = torque_block
        else:
            cfg_ctrl["kp"] = float(p["kp"])
            cfg_ctrl["kd"] = float(p["kd"])
            cfg_ctrl["tau_limit"] = float(p["tau_limit"])

        cfg["runner"] = cfg_runner
        cfg["controller"] = cfg_ctrl

        cfg_path = os.path.join(repo_root, args.config)
        _dump_yaml(cfg_path, cfg)

        cmd = [
            sys.executable,
            "main_simulation.py",
            "--mode",
            "standing-pd-ext",
            "--duration",
            str(float(cfg_runner.get("seconds", 10.0))),
        ]
        if args.no_gui:
            cmd.append("--no-gui")

        repeats = max(1, int(args.repeats))
        print(f"\n== Trial {i}/{len(planned_params)} == {p} (repeats={repeats})")

        statuses: List[str] = []
        survivals: List[float] = []
        scores: List[float] = []
        tilts: List[float] = []
        energies: List[float] = []
        abort_reasons: List[str] = []
        log_paths: List[str] = []

        for r in range(repeats):
            before = _find_latest_log(os.path.join(repo_root, args.log_dir), args.prefix)
            before_mtime = os.path.getmtime(before) if before else 0.0

            # Update seed per repeat (if the main loop consumes it, enables reproducibility).
            cfg2 = _load_yaml(cfg_path)
            cfg_runner2 = (cfg2.get("runner") or {})
            cfg_runner2["seed"] = base_seed + r
            cfg2["runner"] = cfg_runner2
            _dump_yaml(cfg_path, cfg2)

            rc = _run(cmd, cwd=src_dir)
            if rc != 0:
                statuses.append(f"EXIT_{rc}")
                survivals.append(0.0)
                scores.append(0.0)
                abort_reasons.append(f"process exited {rc}")
                continue

            latest = _find_latest_log(os.path.join(repo_root, args.log_dir), args.prefix)
            if latest is None:
                statuses.append("NO_LOG")
                survivals.append(0.0)
                scores.append(0.0)
                abort_reasons.append("no log found")
                continue

            latest_mtime = os.path.getmtime(latest)
            status, metrics, survival, tilt, energy, reason = _extract_single(latest, compute_score_fn)
            if latest_mtime <= before_mtime:
                status = status + "_STALELOG"

            statuses.append(status)
            survivals.append(float(survival))
            scores.append(float(compute_score_fn(metrics, status.replace("_STALELOG", ""))))
            if tilt is not None:
                tilts.append(float(tilt))
            if energy is not None:
                energies.append(float(energy))
            if reason is not None:
                abort_reasons.append(str(reason))
            log_paths.append(latest)

        success_count = sum(1 for s in statuses if str(s).upper().startswith("DONE"))
        survival_mean = sum(survivals) / len(survivals) if survivals else 0.0
        survival_min = min(survivals) if survivals else 0.0
        score_mean = sum(scores) / len(scores) if scores else 0.0
        tilt_mean = (sum(tilts) / len(tilts)) if tilts else None
        energy_mean = (sum(energies) / len(energies)) if energies else None

        tr = TrialResult(
            idx=i,
            params=p,
            repeats=repeats,
            success_count=success_count,
            status=f"{'DONE' if success_count==repeats else 'ABORT'}@{success_count}/{repeats}",
            survival_mean=float(survival_mean),
            survival_min=float(survival_min),
            score_mean=float(score_mean),
            tilt_mean=float(tilt_mean) if tilt_mean is not None else None,
            energy_mean=float(energy_mean) if energy_mean is not None else None,
            abort_reasons=abort_reasons,
            log_paths=log_paths,
        )

        results.append(tr)
        print(
            f"  {tr.status}  success_rate={_success_rate(tr):.2f}  "
            f"survival_mean={tr.survival_mean:.3f}  survival_min={tr.survival_min:.3f}  "
            f"score_mean={tr.score_mean:.3f}  tilt_mean={tr.tilt_mean if tr.tilt_mean is not None else '-'}"
        )

        # Best selection: success_rate -> survival_min -> score_mean -> tilt_mean
        if best is None or _best_key(tr) > _best_key(best):
            best = tr

    if best is None:
        print("No successful trials.")
        return 2

    best_cfg = _load_yaml(os.path.join(repo_root, args.config))
    best_cfg_runner = best_cfg.get("runner") or {}
    best_cfg_ctrl = best_cfg.get("controller") or {}
    best_cfg_runner["control_dt"] = float(best.params["control_dt"])
    best_cfg_runner["settle_steps"] = int(best.params["settle_steps"])
    if best.params.get("stance", None) is not None:
        best_cfg_ctrl["stance"] = str(best.params["stance"])
    if best.params.get("crouch_knee", None) is not None:
        best_cfg_ctrl["crouch_knee"] = float(best.params["crouch_knee"])
    if best.params.get("crouch_ankle", None) is not None:
        best_cfg_ctrl["crouch_ankle"] = float(best.params["crouch_ankle"])
    if str(best_cfg_ctrl.get("type", "torque_pd")) == "two_stage":
        best_cfg_ctrl["warmup_seconds"] = float(best.params.get("warmup_seconds", best_cfg_ctrl.get("warmup_seconds", 1.0)))
        best_cfg_ctrl["blend_seconds"] = float(best.params.get("blend_seconds", best_cfg_ctrl.get("blend_seconds", 0.0)))

        pos_block = dict(best_cfg_ctrl.get("position") or {})
        if best.params.get("pos_kp", None) is not None:
            pos_block["kp"] = float(best.params["pos_kp"])
        if best.params.get("pos_kd", None) is not None:
            pos_block["kd"] = float(best.params["pos_kd"])
        best_cfg_ctrl["position"] = pos_block

        torque_block = dict(best_cfg_ctrl.get("torque") or {})
        torque_block["kp"] = float(best.params["kp"])
        torque_block["kd"] = float(best.params["kd"])
        torque_block["tau_limit"] = float(best.params["tau_limit"])
        torque_block["use_gravity_comp"] = bool(
            best.params.get("use_gravity_comp", torque_block.get("use_gravity_comp", False))
        )
        torque_block["gravity_scale"] = float(best.params.get("gravity_scale", torque_block.get("gravity_scale", 1.0)))
        torque_block["kd_blend_factor"] = float(best.params.get("kd_blend_factor", torque_block.get("kd_blend_factor", 1.0)))
        best_cfg_ctrl["torque"] = torque_block
    else:
        best_cfg_ctrl["kp"] = float(best.params["kp"])
        best_cfg_ctrl["kd"] = float(best.params["kd"])
        best_cfg_ctrl["tau_limit"] = float(best.params["tau_limit"])
    best_cfg["runner"] = best_cfg_runner
    best_cfg["controller"] = best_cfg_ctrl

    _dump_yaml(os.path.join(repo_root, args.best_out), best_cfg)

    print("\n== Best ==")
    print("  params:", best.params)
    print("  status:", best.status)
    print(f"  success_rate: {_success_rate(best):.2f} ({best.success_count}/{best.repeats})")
    print("  survival_mean:", best.survival_mean)
    print("  survival_min:", best.survival_min)
    print("  score_mean:", best.score_mean)
    print("  tilt_mean:", best.tilt_mean)
    print("  logs:", best.log_paths[-3:] if best.log_paths else [])
    print("  wrote:", args.best_out)

    summary_path = os.path.join(repo_root, args.log_dir, "sweep_summary.json")
    with open(summary_path, "w", encoding="utf-8") as f:
        json.dump(
            {
                "best": {
                    "params": best.params,
                    "status": best.status,
                    "repeats": best.repeats,
                    "success_count": best.success_count,
                    "success_rate": _success_rate(best),
                    "survival_mean": best.survival_mean,
                    "survival_min": best.survival_min,
                    "score_mean": best.score_mean,
                    "tilt_mean": best.tilt_mean,
                    "log_paths": best.log_paths,
                },
                "results": [
                    {
                        "idx": r.idx,
                        "params": r.params,
                        "status": r.status,
                        "repeats": r.repeats,
                        "success_count": r.success_count,
                        "success_rate": _success_rate(r),
                        "survival_mean": r.survival_mean,
                        "survival_min": r.survival_min,
                        "score_mean": r.score_mean,
                        "tilt_mean": r.tilt_mean,
                        "energy_mean": r.energy_mean,
                        "abort_reasons": r.abort_reasons[:10],
                        "log_paths": r.log_paths[-3:],
                    }
                    for r in results
                ],
            },
            f,
            ensure_ascii=False,
            indent=2,
        )
    print("  wrote:", os.path.join(args.log_dir, "sweep_summary.json"))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
