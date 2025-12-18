#!/usr/bin/env python3
from __future__ import annotations

import argparse
import itertools
import json
import os
import random
import subprocess
import sys
from dataclasses import dataclass
from typing import Any, Dict, List, Optional


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
    log_path: Optional[str]
    status: str
    survival_time: float
    tilt_max_abs: Optional[float]
    base_z_min: Optional[float]
    energy: Optional[float]
    abort_reason: Optional[str]


def _score(tr: TrialResult) -> float:
    """
    Primary objective: maximize survival_time.
    Add big bonus if DONE. Small penalty for tilt.
    """
    s = tr.survival_time
    if tr.status == "DONE":
        s += 100.0
    if tr.tilt_max_abs is not None:
        s -= 0.5 * tr.tilt_max_abs
    return s


def _extract_trial(log_path: str, idx: int, params: Dict[str, Any]) -> TrialResult:
    payload = _load_json(log_path)
    result = payload.get("result", {})
    metrics = result.get("metrics", {}) if isinstance(result, dict) else {}
    abort = result.get("abort", {}) if isinstance(result, dict) else {}

    status = str(result.get("status", "UNKNOWN"))
    survival = float(metrics.get("survival_time", 0.0) or 0.0)
    tilt = metrics.get("tilt_max_abs", None)
    zmin = metrics.get("base_z_min", None)
    energy = metrics.get("energy_abs_tau_dq", None)
    reason = abort.get("reason") if isinstance(abort, dict) else None

    return TrialResult(
        idx=idx,
        params=params,
        log_path=log_path,
        status=status,
        survival_time=survival,
        tilt_max_abs=float(tilt) if tilt is not None else None,
        base_z_min=float(zmin) if zmin is not None else None,
        energy=float(energy) if energy is not None else None,
        abort_reason=str(reason) if reason is not None else None,
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
    ap.add_argument("--mode", choices=["grid", "random"], default="grid", help="Sweep mode")
    ap.add_argument("--seed", type=int, default=0, help="Random seed (random mode)")
    ap.add_argument("--duration", type=float, default=None, help="Override runner.seconds (optional)")
    ap.add_argument("--no-gui", action="store_true", help="Force --no-gui for main_simulation.py")
    ap.add_argument("--dry-run", action="store_true", help="Print planned trials but do not run")

    ap.add_argument("--kp", default="20,40,60,80,100,120", help="Comma-separated kp candidates (grid)")
    ap.add_argument("--kd", default="0.5,1.0,1.5,2.0,3.0,4.0", help="Comma-separated kd candidates (grid)")
    ap.add_argument("--tau", default="30,60,90,120", help="Comma-separated tau_limit candidates (grid)")
    ap.add_argument("--control-dt", default="0.01,0.001", help="Comma-separated control_dt candidates (grid)")
    ap.add_argument("--settle", default="0,300,800", help="Comma-separated settle_steps candidates (grid)")
    args = ap.parse_args()

    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    src_dir = os.path.join(repo_root, "src")

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

    planned_params: List[Dict[str, Any]] = []

    if args.mode == "grid":
        all_params = list(itertools.product(kps, kds, taus, dts, settles))
        max_grid = max(1, args.trials)
        if len(all_params) > max_grid:
            step = max(1, len(all_params) // max_grid)
            all_params = all_params[::step][:max_grid]
        for kp, kd, tau, dt, settle in all_params:
            planned_params.append(
                {"kp": kp, "kd": kd, "tau_limit": tau, "control_dt": dt, "settle_steps": settle}
            )
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

        if args.duration is not None:
            cfg_runner["seconds"] = float(args.duration)
        cfg_runner["control_dt"] = float(p["control_dt"])
        cfg_runner["settle_steps"] = int(p["settle_steps"])

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

        before = _find_latest_log(os.path.join(repo_root, args.log_dir), args.prefix)
        before_mtime = os.path.getmtime(before) if before else 0.0

        print(f"\n== Trial {i}/{len(planned_params)} == {p}")
        rc = _run(cmd, cwd=src_dir)
        if rc != 0:
            tr = TrialResult(
                idx=i,
                params=p,
                log_path=None,
                status=f"EXIT_{rc}",
                survival_time=0.0,
                tilt_max_abs=None,
                base_z_min=None,
                energy=None,
                abort_reason=f"process exited {rc}",
            )
            results.append(tr)
            continue

        latest = _find_latest_log(os.path.join(repo_root, args.log_dir), args.prefix)
        if latest is None:
            tr = TrialResult(
                idx=i,
                params=p,
                log_path=None,
                status="NO_LOG",
                survival_time=0.0,
                tilt_max_abs=None,
                base_z_min=None,
                energy=None,
                abort_reason="no log found",
            )
            results.append(tr)
            continue

        latest_mtime = os.path.getmtime(latest)
        if latest_mtime <= before_mtime:
            tr = _extract_trial(latest, i, p)
            tr.status = tr.status + "_STALELOG"
        else:
            tr = _extract_trial(latest, i, p)

        results.append(tr)

        sc = _score(tr)
        print(
            f"  status={tr.status} survival={tr.survival_time:.3f} "
            f"tilt={tr.tilt_max_abs if tr.tilt_max_abs is not None else '-'} "
            f"score={sc:.3f} log={os.path.basename(latest) if latest else '-'}"
        )
        if best is None or sc > _score(best):
            best = tr

    if best is None:
        print("No successful trials.")
        return 2

    best_cfg = _load_yaml(os.path.join(repo_root, args.config))
    best_cfg_runner = best_cfg.get("runner") or {}
    best_cfg_ctrl = best_cfg.get("controller") or {}
    best_cfg_runner["control_dt"] = float(best.params["control_dt"])
    best_cfg_runner["settle_steps"] = int(best.params["settle_steps"])
    best_cfg_ctrl["kp"] = float(best.params["kp"])
    best_cfg_ctrl["kd"] = float(best.params["kd"])
    best_cfg_ctrl["tau_limit"] = float(best.params["tau_limit"])
    best_cfg["runner"] = best_cfg_runner
    best_cfg["controller"] = best_cfg_ctrl

    _dump_yaml(os.path.join(repo_root, args.best_out), best_cfg)

    print("\n== Best ==")
    print("  params:", best.params)
    print("  status:", best.status)
    print("  survival:", best.survival_time)
    print("  tilt_max:", best.tilt_max_abs)
    print("  log:", best.log_path)
    print("  wrote:", args.best_out)

    summary_path = os.path.join(repo_root, args.log_dir, "sweep_summary.json")
    with open(summary_path, "w", encoding="utf-8") as f:
        json.dump(
            {
                "best": {
                    "params": best.params,
                    "status": best.status,
                    "survival_time": best.survival_time,
                    "tilt_max_abs": best.tilt_max_abs,
                    "log_path": best.log_path,
                },
                "results": [
                    {
                        "idx": r.idx,
                        "params": r.params,
                        "status": r.status,
                        "survival_time": r.survival_time,
                        "tilt_max_abs": r.tilt_max_abs,
                        "base_z_min": r.base_z_min,
                        "energy": r.energy,
                        "abort_reason": r.abort_reason,
                        "log_path": r.log_path,
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

