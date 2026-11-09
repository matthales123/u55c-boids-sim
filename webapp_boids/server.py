#!/usr/bin/env python3
from __future__ import annotations

import html
import math
import os
import random
import re
import shlex
import subprocess
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

try:
    import markdown as markdown_lib
except Exception:  # pragma: no cover
    markdown_lib = None

APP_NAME = "U55C Boids Simulation Lab"
ROOT_DIR = Path(__file__).resolve().parents[1]
STATIC_DIR = Path(__file__).resolve().parent / "static"
PROJECT_ENV = ROOT_DIR / "config" / "project.env"
README_ROOT = ROOT_DIR / "README"
TOP_README = ROOT_DIR / "README.md"
README_ASSETS = README_ROOT / "assets"
BENCH_BIN = ROOT_DIR / "build" / "host" / "u55c_bench"
BUILD_HOST_SCRIPT = ROOT_DIR / "scripts" / "build_host.sh"
XRT_SMI = os.environ.get("WEB_XRT_SMI", "/opt/xilinx/xrt/bin/xrt-smi")
# Centered world coordinates: x in [-320, 320], y in [-240, 240]
BOUND_X = 320
BOUND_Y = 240

VISUAL_RANGE = 75.0
PROTECTED_RANGE = 8.0
CENTERING_FACTOR = 0.0005
MATCHING_FACTOR = 0.05
AVOID_FACTOR = 0.05
TURN_FACTOR = 0.2
MIN_SPEED = 3.0
MAX_SPEED = 6.0

HIT_LEFT = -220.0
HIT_RIGHT = 220.0
HIT_TOP = -140.0
HIT_BOTTOM = 140.0

WORLD_MIN_X = -320.0
WORLD_MAX_X = 320.0
WORLD_MIN_Y = -240.0
WORLD_MAX_Y = 240.0
BOUNCE_NUDGE = 15.0
FPGA_QUANT_STEP = 1.0 / 16.0


class BoidsRequest(BaseModel):
    boids: int = Field(default=512, ge=1, le=50000)
    steps: int = Field(default=50, ge=1, le=2000)
    seed: int = 1
    visual_range: float | None = Field(default=None, gt=0.0, le=1000.0)
    protected_range: float | None = Field(default=None, gt=0.0, le=1000.0)
    centering_factor: float | None = Field(default=None, gt=0.0, le=1.0)
    matching_factor: float | None = Field(default=None, gt=0.0, le=1.0)
    avoid_factor: float | None = Field(default=None, gt=0.0, le=1.0)
    turn_factor: float | None = Field(default=None, gt=0.0, le=10.0)
    min_speed: float | None = Field(default=None, gt=0.0, le=100.0)
    max_speed: float | None = Field(default=None, gt=0.0, le=100.0)


class TraceCompareRequest(BaseModel):
    boids: int = Field(default=512, ge=1, le=10000)
    steps: int = Field(default=50, ge=1, le=2000)
    seed: int = 1
    trace_count: int = Field(default=24, ge=1, le=128)
    visual_range: float | None = Field(default=None, gt=0.0, le=1000.0)
    protected_range: float | None = Field(default=None, gt=0.0, le=1000.0)
    centering_factor: float | None = Field(default=None, gt=0.0, le=1.0)
    matching_factor: float | None = Field(default=None, gt=0.0, le=1.0)
    avoid_factor: float | None = Field(default=None, gt=0.0, le=1.0)
    turn_factor: float | None = Field(default=None, gt=0.0, le=10.0)
    min_speed: float | None = Field(default=None, gt=0.0, le=100.0)
    max_speed: float | None = Field(default=None, gt=0.0, le=100.0)


class BenchmarkRequest(BaseModel):
    mode: str = "hw"
    elements: int = Field(default=16384, ge=1, le=5_000_000)
    iterations: int = Field(default=1, ge=1, le=50_000)
    device: str = "0"
    kernel: str = "boids_step"
    seed: int = 1
    xclbin_path: str = ""
    rebuild_host: bool = False


class CompareRequest(BaseModel):
    boids: int = Field(default=512, ge=1, le=5_000_000)
    steps: int = Field(default=50, ge=1, le=50_000)
    device: str = "0"
    kernel: str = "boids_step"
    seed: int = 1
    xclbin_path: str = ""
    rebuild_host: bool = False


class ProgramRequest(BaseModel):
    xclbin_path: str = ""
    device: str = "0"


class DocRequest(BaseModel):
    key: str


app = FastAPI(title=APP_NAME)
app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")
if README_ASSETS.exists():
    app.mount("/README/assets", StaticFiles(directory=str(README_ASSETS)), name="readme_assets")


def read_project_env() -> Dict[str, str]:
    values: Dict[str, str] = {}
    if not PROJECT_ENV.exists():
        return values
    for raw in PROJECT_ENV.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        values[key.strip()] = value.strip().strip('"').strip("'")
    return values


def startup_gate_state() -> Dict[str, Any]:
    status_raw = os.environ.get("WEBAPP_STARTUP_GATE_STATUS", "ok").strip().lower()
    message = os.environ.get("WEBAPP_STARTUP_GATE_MESSAGE", "").strip()
    rc_raw = os.environ.get("WEBAPP_STARTUP_GATE_RC", "0").strip()

    status = status_raw if status_raw in {"ok", "warn", "skipped"} else "unknown"
    try:
        rc: Optional[int] = int(rc_raw)
    except ValueError:
        rc = None

    return {
        "status": status,
        "message": message,
        "rc": rc,
    }


def resolve_project_path(path_value: str) -> Path:
    expanded = Path(os.path.expandvars(os.path.expanduser(path_value)))
    if expanded.is_absolute():
        return expanded
    return (ROOT_DIR / expanded).resolve()


def default_xclbin_from_env(env_cfg: Dict[str, str]) -> str:
    rtl_path_raw = env_cfg.get("U55C_XCLBIN_RTL", "").strip()
    if rtl_path_raw:
        rtl_path = resolve_project_path(rtl_path_raw)
        if rtl_path.exists():
            return str(rtl_path)

    base_path_raw = env_cfg.get("U55C_XCLBIN", "").strip()
    if base_path_raw:
        return str(resolve_project_path(base_path_raw))
    return ""


def run_command(cmd: List[str], timeout_s: int = 180) -> Dict[str, Any]:
    start = time.perf_counter()
    proc = subprocess.run(
        cmd,
        cwd=str(ROOT_DIR),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=timeout_s,
        check=False,
    )
    elapsed_ms = (time.perf_counter() - start) * 1000.0
    return {
        "ok": proc.returncode == 0,
        "rc": proc.returncode,
        "elapsed_ms": elapsed_ms,
        "cmd": " ".join(shlex.quote(x) for x in cmd),
        "output": proc.stdout,
    }


def ensure_host_binary(rebuild: bool) -> None:
    if rebuild or not BENCH_BIN.exists():
        proc = run_command([str(BUILD_HOST_SCRIPT)], timeout_s=600)
        if not proc["ok"]:
            tail = "\n".join(proc["output"].splitlines()[-40:])
            raise HTTPException(status_code=500, detail=f"host build failed (rc={proc['rc']})\n{tail}")


def parse_bench_summary(text: str) -> Dict[str, Any]:
    lines = [line.strip() for line in text.splitlines() if line.strip().startswith("mode=")]
    if not lines:
        return {"summary_line": "", "metrics": {}}
    summary = lines[-1]
    metrics: Dict[str, Any] = {}
    for token in summary.split():
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        raw = value.strip().strip(",")
        if re.fullmatch(r"[-+]?\d+", raw):
            metrics[key] = int(raw)
        elif re.fullmatch(r"[-+]?\d*\.\d+", raw):
            metrics[key] = float(raw)
        else:
            metrics[key] = raw
    return {"summary_line": summary, "metrics": metrics}


def resolve_program_device(device: str) -> str:
    candidate = device.strip()
    if re.fullmatch(r"[0-9a-fA-F]{4}:[0-9a-fA-F]{2}:[0-9a-fA-F]{2}\\.[0-9a-fA-F]", candidate):
        return candidate
    if candidate.isdigit():
        probe = run_command([XRT_SMI, "examine"], timeout_s=30)
        if probe["ok"]:
            bdfs = re.findall(r"\[([0-9A-Fa-f:.]+)\]", probe["output"])
            idx = int(candidate)
            if idx < len(bdfs):
                return bdfs[idx]
    return candidate


def reflect_with_bound(pos: int, vel: int, bound: int) -> int:
    nxt = pos + vel
    if nxt > bound:
        nxt = bound - (nxt - bound)
    elif nxt < -bound:
        nxt = -bound - (nxt + bound)

    if nxt > bound:
        nxt = bound
    if nxt < -bound:
        nxt = -bound
    return nxt


def _resolve_rules(req: BoidsRequest | TraceCompareRequest) -> Dict[str, float]:
    visual_range = float(req.visual_range if req.visual_range is not None else VISUAL_RANGE)
    protected_range = float(req.protected_range if req.protected_range is not None else PROTECTED_RANGE)
    centering_factor = float(req.centering_factor if req.centering_factor is not None else CENTERING_FACTOR)
    matching_factor = float(req.matching_factor if req.matching_factor is not None else MATCHING_FACTOR)
    avoid_factor = float(req.avoid_factor if req.avoid_factor is not None else AVOID_FACTOR)
    turn_factor = float(req.turn_factor if req.turn_factor is not None else TURN_FACTOR)
    min_speed = float(req.min_speed if req.min_speed is not None else MIN_SPEED)
    max_speed = float(req.max_speed if req.max_speed is not None else MAX_SPEED)

    if visual_range <= 1.0:
        visual_range = 1.0
    if protected_range <= 0.0:
        protected_range = 1.0
    if protected_range >= visual_range:
        protected_range = max(1.0, visual_range - 1.0)
    if min_speed <= 0.0:
        min_speed = 0.1
    if max_speed <= 0.0:
        max_speed = min_speed
    if min_speed > max_speed:
        min_speed, max_speed = max_speed, min_speed

    return {
        "visual_range": visual_range,
        "protected_range": protected_range,
        "centering_factor": centering_factor,
        "matching_factor": matching_factor,
        "avoid_factor": avoid_factor,
        "turn_factor": turn_factor,
        "min_speed": min_speed,
        "max_speed": max_speed,
    }


def _clamp_speed(vx: float, vy: float, min_speed: float, max_speed: float) -> Tuple[float, float]:
    speed = (vx * vx + vy * vy) ** 0.5
    if speed > 1.0e-6:
        if speed < min_speed:
            scale = min_speed / speed
            vx *= scale
            vy *= scale
        elif speed > max_speed:
            scale = max_speed / speed
            vx *= scale
            vy *= scale
    return vx, vy


def _boids_step_model(
    positions: List[List[float]],
    velocities: List[List[float]],
    count: int,
    rules: Dict[str, float],
    quant_step: float = 0.0,
) -> None:
    visual_range = rules["visual_range"]
    protected_range = rules["protected_range"]
    centering_factor = rules["centering_factor"]
    matching_factor = rules["matching_factor"]
    avoid_factor = rules["avoid_factor"]
    turn_factor = rules["turn_factor"]
    min_speed = rules["min_speed"]
    max_speed = rules["max_speed"]

    visual_range2 = visual_range * visual_range
    protected_range2 = protected_range * protected_range

    next_vx = [0.0] * count
    next_vy = [0.0] * count

    for i in range(count):
        x = positions[i][0]
        y = positions[i][1]
        vx = velocities[i][0]
        vy = velocities[i][1]

        close_dx = 0.0
        close_dy = 0.0
        xpos_avg = 0.0
        ypos_avg = 0.0
        xvel_avg = 0.0
        yvel_avg = 0.0
        neighbors = 0

        for j in range(count):
            if i == j:
                continue
            ox = positions[j][0]
            oy = positions[j][1]
            ovx = velocities[j][0]
            ovy = velocities[j][1]

            dx = x - ox
            dy = y - oy
            if abs(dx) < visual_range and abs(dy) < visual_range:
                dist2 = dx * dx + dy * dy
                if dist2 < protected_range2:
                    close_dx += dx
                    close_dy += dy
                elif dist2 < visual_range2:
                    xpos_avg += ox
                    ypos_avg += oy
                    xvel_avg += ovx
                    yvel_avg += ovy
                    neighbors += 1

        if neighbors > 0:
            inv = 1.0 / neighbors
            xpos_avg *= inv
            ypos_avg *= inv
            xvel_avg *= inv
            yvel_avg *= inv
            vx += (xpos_avg - x) * centering_factor + (xvel_avg - vx) * matching_factor
            vy += (ypos_avg - y) * centering_factor + (yvel_avg - vy) * matching_factor

        vx += close_dx * avoid_factor
        vy += close_dy * avoid_factor

        if x < HIT_LEFT:
            vx += turn_factor
        elif x > HIT_RIGHT:
            vx -= turn_factor
        if y < HIT_TOP:
            vy += turn_factor
        elif y > HIT_BOTTOM:
            vy -= turn_factor

        vx, vy = _clamp_speed(vx, vy, min_speed=min_speed, max_speed=max_speed)

        if quant_step > 0.0:
            vx = round(vx / quant_step) * quant_step
            vy = round(vy / quant_step) * quant_step

        next_vx[i] = vx
        next_vy[i] = vy

    for i in range(count):
        x = positions[i][0] + next_vx[i]
        y = positions[i][1] + next_vy[i]
        vx = next_vx[i]
        vy = next_vy[i]

        if x < WORLD_MIN_X:
            x = x + BOUNCE_NUDGE
            vx = -vx
        elif x > WORLD_MAX_X:
            x = x - BOUNCE_NUDGE
            vx = -vx

        if y < WORLD_MIN_Y:
            y = y + BOUNCE_NUDGE
            vy = -vy
        elif y > WORLD_MAX_Y:
            y = y - BOUNCE_NUDGE
            vy = -vy

        if x < WORLD_MIN_X:
            x = WORLD_MIN_X
        elif x > WORLD_MAX_X:
            x = WORLD_MAX_X
        if y < WORLD_MIN_Y:
            y = WORLD_MIN_Y
        elif y > WORLD_MAX_Y:
            y = WORLD_MAX_Y

        if quant_step > 0.0:
            x = round(x / quant_step) * quant_step
            y = round(y / quant_step) * quant_step
            vx = round(vx / quant_step) * quant_step
            vy = round(vy / quant_step) * quant_step

        positions[i][0] = x
        positions[i][1] = y
        velocities[i][0] = vx
        velocities[i][1] = vy


def simulate_boids(req: BoidsRequest) -> Dict[str, Any]:
    boids = req.boids
    rules = _resolve_rules(req)
    positions, velocities, _, _ = _init_states(boids, req.seed, rules["min_speed"], rules["max_speed"])

    initial = [p[:] for p in positions]

    trace_count = min(24, boids)
    trajectories: List[List[List[float]]] = [[[initial[i][0], initial[i][1]]] for i in range(trace_count)]

    start = time.perf_counter()
    for step in range(req.steps):
        _boids_step_model(positions, velocities, boids, rules, quant_step=0.0)

        if step % max(1, req.steps // 48) == 0:
            for i in range(trace_count):
                trajectories[i].append([positions[i][0], positions[i][1]])

    elapsed_ms = (time.perf_counter() - start) * 1000.0

    speeds = [abs(vx) + abs(vy) for vx, vy in velocities]
    avg_speed = sum(speeds) / max(1, len(speeds))

    return {
        "boids": boids,
        "steps": req.steps,
        "rules": rules,
        "elapsed_ms": elapsed_ms,
        "bounds": {"x": BOUND_X, "y": BOUND_Y},
        "avg_l1_speed": avg_speed,
        "initial": initial,
        "final": positions,
        "trajectories": trajectories,
    }


def _init_states(
    boids: int,
    seed: int,
    min_speed: float,
    max_speed: float,
) -> Tuple[List[List[float]], List[List[float]], List[List[float]], List[List[float]]]:
    rng = random.Random(seed)
    cpu_positions: List[List[float]] = []
    cpu_velocities: List[List[float]] = []
    fpga_positions: List[List[float]] = []
    fpga_velocities: List[List[float]] = []

    for _ in range(boids):
        x = rng.uniform(HIT_LEFT, HIT_RIGHT)
        y = rng.uniform(HIT_TOP, HIT_BOTTOM)
        theta = rng.uniform(0.0, 2.0 * 3.141592653589793)
        speed = rng.uniform(min_speed, max_speed)
        vx = speed * math.cos(theta)
        vy = speed * math.sin(theta)
        cpu_positions.append([float(x), float(y)])
        cpu_velocities.append([float(vx), float(vy)])
        fpga_positions.append([float(x), float(y)])
        fpga_velocities.append([float(vx), float(vy)])

    return cpu_positions, cpu_velocities, fpga_positions, fpga_velocities


def simulate_trace_compare(req: TraceCompareRequest) -> Dict[str, Any]:
    boids = req.boids
    steps = req.steps
    trace_count = min(req.trace_count, boids)
    rules = _resolve_rules(req)

    notes: List[str] = []
    if req.protected_range is not None and req.visual_range is not None and req.protected_range >= req.visual_range:
        notes.append("protected_range was clamped to stay below visual_range")
    if req.min_speed is not None and req.max_speed is not None and req.min_speed > req.max_speed:
        notes.append("min_speed/max_speed were swapped for a valid range")

    cpu_positions, cpu_velocities, fpga_positions, fpga_velocities = _init_states(
        boids,
        req.seed,
        rules["min_speed"],
        rules["max_speed"],
    )

    cpu_trace: List[List[List[float]]] = [[[cpu_positions[i][0], cpu_positions[i][1]]] for i in range(trace_count)]
    fpga_trace: List[List[List[float]]] = [[[fpga_positions[i][0], fpga_positions[i][1]]] for i in range(trace_count)]

    t_cpu_start = time.perf_counter()
    for _ in range(steps):
        _boids_step_model(cpu_positions, cpu_velocities, boids, rules, quant_step=0.0)

        for i in range(trace_count):
            cpu_trace[i].append([cpu_positions[i][0], cpu_positions[i][1]])

    cpu_elapsed_ms = (time.perf_counter() - t_cpu_start) * 1000.0

    t_fpga_start = time.perf_counter()
    for _ in range(steps):
        _boids_step_model(fpga_positions, fpga_velocities, boids, rules, quant_step=FPGA_QUANT_STEP)

        for i in range(trace_count):
            fpga_trace[i].append([fpga_positions[i][0], fpga_positions[i][1]])

    fpga_model_elapsed_ms = (time.perf_counter() - t_fpga_start) * 1000.0

    return {
        "boids": boids,
        "steps": steps,
        "trace_count": trace_count,
        "rules": rules,
        "bounds": {"x": BOUND_X, "y": BOUND_Y},
        "notes": notes,
        "cpu_trace": cpu_trace,
        "fpga_trace": fpga_trace,
        "cpu_elapsed_ms": cpu_elapsed_ms,
        "fpga_model_elapsed_ms": fpga_model_elapsed_ms,
    }


def docs_catalog() -> Dict[str, Path]:
    docs: Dict[str, Path] = {}
    hidden_docs = {"RUNBOOK.md", "WEBAPP.md", "TESTS.md", "SETUP.md", "INTERNAL_MILESTONES.md"}
    if TOP_README.exists():
        docs["README.md"] = TOP_README
    if README_ROOT.exists():
        for p in sorted(README_ROOT.glob("*.md")):
            if p.name in hidden_docs:
                continue
            docs[f"README/{p.name}"] = p
    return docs


def markdown_to_html(text: str) -> str:
    if markdown_lib is None:
        return f"<pre>{html.escape(text)}</pre>"
    return markdown_lib.markdown(
        text,
        extensions=["extra", "fenced_code", "tables", "toc", "sane_lists"],
        output_format="html5",
    )


def block_diagram_data() -> Dict[str, Any]:
    return {
        "root_view": "top",
        "views": {
            "top": {
                "id": "top",
                "title": "Boids Accelerator Top-Level",
                "description": "Host-driven flock simulation flow with FPGA acceleration and trace comparison.",
                "notes": [
                    "CPU flock runs continuously for visual baseline behavior.",
                    "FPGA kernel path executes high-volume boid state updates.",
                    "Comparison tooling measures speedup and trajectory divergence.",
                ],
                "nodes": [
                    {
                        "id": "ui",
                        "label": "Web + Host Control",
                        "subtitle": "boids/steps/seed",
                        "x": 40,
                        "y": 70,
                        "w": 240,
                        "h": 110,
                        "group": "software",
                        "details": "Controls simulation parameters and launches CPU/FPGA benchmark paths.",
                    },
                    {
                        "id": "cpu",
                        "label": "CPU Flocking Model",
                        "subtitle": "continuous preview",
                        "x": 320,
                        "y": 70,
                        "w": 240,
                        "h": 110,
                        "group": "software",
                        "details": "Interactive model for visual bird-flight behavior and trace baseline.",
                    },
                    {
                        "id": "xrt",
                        "label": "XRT Runtime",
                        "subtitle": "program + kernel launch",
                        "x": 600,
                        "y": 70,
                        "w": 240,
                        "h": 110,
                        "group": "platform",
                        "details": "Executes boids_step kernel using configured xclbin and runtime controls.",
                    },
                    {
                        "id": "hbm",
                        "label": "HBM State Buffers",
                        "subtitle": "state/velocity/next-state",
                        "x": 880,
                        "y": 50,
                        "w": 270,
                        "h": 150,
                        "group": "memory",
                        "details": "Banked boid state storage enables high-throughput update passes.",
                        "drill_to": "memory_path",
                    },
                    {
                        "id": "kernel",
                        "label": "boids_step Kernel",
                        "subtitle": "state update datapath",
                        "x": 520,
                        "y": 270,
                        "w": 300,
                        "h": 150,
                        "group": "compute",
                        "details": "Applies neighborhood/boundary updates and emits next boid state.",
                        "drill_to": "kernel_core",
                    },
                    {
                        "id": "compare",
                        "label": "Trace + Speedup Analyzer",
                        "subtitle": "CPU vs FPGA metrics",
                        "x": 180,
                        "y": 280,
                        "w": 280,
                        "h": 140,
                        "group": "observability",
                        "details": "Builds side-by-side trajectory traces and computes total/kernel speedups.",
                    },
                ],
                "edges": [
                    {"from": "ui", "to": "cpu", "label": "preview params"},
                    {"from": "ui", "to": "xrt", "label": "runtime params"},
                    {"from": "xrt", "to": "hbm", "label": "buffer setup"},
                    {"from": "hbm", "to": "kernel", "label": "state/vel read"},
                    {"from": "kernel", "to": "hbm", "label": "next-state write"},
                    {"from": "cpu", "to": "compare", "label": "CPU trace"},
                    {"from": "kernel", "to": "compare", "label": "FPGA trace"},
                ],
            },
            "kernel_core": {
                "id": "kernel_core",
                "title": "Boids Kernel Datapath",
                "description": "Per-boid update flow implemented for FPGA acceleration.",
                "notes": [
                    "Position/velocity are loaded from state buffers.",
                    "Neighborhood and boundary logic compute new velocity vector.",
                    "Updated state is reflected back into next-state buffers.",
                ],
                "nodes": [
                    {
                        "id": "state",
                        "label": "State Loader",
                        "subtitle": "pos/vel fetch",
                        "x": 70,
                        "y": 210,
                        "w": 230,
                        "h": 120,
                        "group": "datapath",
                        "details": "Reads current boid position and velocity vectors.",
                    },
                    {
                        "id": "neighbor",
                        "label": "Neighbor Update Logic",
                        "subtitle": "alignment/cohesion/separation",
                        "x": 360,
                        "y": 190,
                        "w": 280,
                        "h": 150,
                        "group": "compute",
                        "details": "Computes velocity steering terms from neighborhood relations.",
                    },
                    {
                        "id": "bound",
                        "label": "Boundary + Clamp",
                        "subtitle": "turn + speed limits",
                        "x": 700,
                        "y": 200,
                        "w": 240,
                        "h": 130,
                        "group": "control",
                        "details": "Applies boundary turns and velocity clamps before state commit.",
                    },
                ],
                "edges": [
                    {"from": "state", "to": "neighbor", "label": "boid vectors"},
                    {"from": "neighbor", "to": "bound", "label": "raw update"},
                ],
            },
            "memory_path": {
                "id": "memory_path",
                "title": "HBM Memory Path",
                "description": "Banked layout for boid state update execution.",
                "notes": [
                    "Current-state and velocity streams are read each iteration.",
                    "Next-state writes are separated to avoid in-place hazards.",
                ],
                "nodes": [
                    {
                        "id": "state",
                        "label": "HBM[0] state_mem",
                        "x": 90,
                        "y": 130,
                        "w": 250,
                        "h": 110,
                        "group": "memory",
                        "details": "Current boid position/state vectors.",
                    },
                    {
                        "id": "vel",
                        "label": "HBM[1] vel_mem",
                        "x": 390,
                        "y": 130,
                        "w": 250,
                        "h": 110,
                        "group": "memory",
                        "details": "Current boid velocity vectors.",
                    },
                    {
                        "id": "next",
                        "label": "HBM[2] next_state_mem",
                        "x": 690,
                        "y": 130,
                        "w": 250,
                        "h": 110,
                        "group": "memory",
                        "details": "Updated boid state vectors written by kernel.",
                    },
                ],
                "edges": [
                    {"from": "state", "to": "next", "label": "update stream"},
                    {"from": "vel", "to": "next", "label": "velocity update"},
                ],
            },
        },
    }


@app.get("/")
def root() -> FileResponse:
    return FileResponse(STATIC_DIR / "index.html")


@app.get("/api/health")
def api_health() -> Dict[str, Any]:
    env = read_project_env()
    gate = startup_gate_state()
    return {
        "ok": True,
        "app": APP_NAME,
        "project_root": str(ROOT_DIR),
        "default_kernel": env.get("U55C_KERNEL_NAME", "boids_step"),
        "default_xclbin": default_xclbin_from_env(env),
        "startup_gate": gate,
    }


@app.get("/api/docs/list")
def api_docs_list() -> Dict[str, Any]:
    docs = docs_catalog()
    return {
        "ok": True,
        "docs": [{"key": key, "title": key} for key in docs],
    }


@app.post("/api/docs/read")
def api_docs_read(req: DocRequest) -> Dict[str, Any]:
    docs = docs_catalog()
    if req.key not in docs:
        raise HTTPException(status_code=404, detail=f"document not found: {req.key}")
    path = docs[req.key]
    content = path.read_text(encoding="utf-8", errors="replace")
    return {
        "ok": True,
        "key": req.key,
        "title": req.key,
        "content": content,
        "content_html": markdown_to_html(content),
    }


@app.get("/api/diagram")
def api_diagram() -> Dict[str, Any]:
    return block_diagram_data()


@app.post("/api/boids/simulate")
def api_boids_simulate(req: BoidsRequest) -> Dict[str, Any]:
    return simulate_boids(req)


@app.post("/api/boids/trace-compare")
def api_boids_trace_compare(req: TraceCompareRequest) -> Dict[str, Any]:
    return simulate_trace_compare(req)


@app.post("/api/benchmark")
def api_benchmark(req: BenchmarkRequest) -> Dict[str, Any]:
    mode = req.mode.strip().lower()
    if mode not in ("dry-run", "hw"):
        raise HTTPException(status_code=400, detail="mode must be dry-run or hw")

    ensure_host_binary(req.rebuild_host)

    env_cfg = read_project_env()
    xclbin_candidate = req.xclbin_path.strip() or default_xclbin_from_env(env_cfg)
    resolved_xclbin = ""

    if mode == "hw":
        if not xclbin_candidate:
            raise HTTPException(status_code=400, detail="xclbin path is required in hw mode")
        xclbin = resolve_project_path(xclbin_candidate)
        if not xclbin.exists():
            raise HTTPException(status_code=404, detail=f"xclbin not found: {xclbin}")
        resolved_xclbin = str(xclbin)

    cmd = [
        str(BENCH_BIN),
        "--mode",
        mode,
        "--kernel",
        req.kernel,
        "--device",
        req.device,
        "--elements",
        str(req.elements),
        "--iterations",
        str(req.iterations),
        "--seed",
        str(req.seed),
    ]
    if mode == "hw":
        cmd.extend(["--xclbin", resolved_xclbin])

    result = run_command(cmd, timeout_s=7200)
    parsed = parse_bench_summary(result["output"])
    return {
        "ok": result["ok"],
        "rc": result["rc"],
        "elapsed_ms": result["elapsed_ms"],
        "cmd": result["cmd"],
        "summary": parsed["summary_line"],
        "metrics": parsed["metrics"],
        "output_tail": "\n".join(result["output"].splitlines()[-40:]),
    }


@app.post("/api/compare")
def api_compare(req: CompareRequest) -> Dict[str, Any]:
    ensure_host_binary(req.rebuild_host)

    base_kwargs = {
        "elements": req.boids,
        "iterations": req.steps,
        "device": req.device,
        "kernel": req.kernel,
        "seed": req.seed,
        "xclbin_path": req.xclbin_path,
        "rebuild_host": False,
    }

    cpu = api_benchmark(BenchmarkRequest(mode="dry-run", **base_kwargs))
    fpga = api_benchmark(BenchmarkRequest(mode="hw", **base_kwargs))

    cpu_metrics = cpu.get("metrics", {})
    fpga_metrics = fpga.get("metrics", {})

    cpu_total = float(cpu_metrics.get("avg_total_ms", 0.0) or 0.0)
    fpga_total = float(fpga_metrics.get("avg_total_ms", 0.0) or 0.0)
    cpu_kernel = float(cpu_metrics.get("avg_kernel_ms", 0.0) or 0.0)
    fpga_kernel = float(fpga_metrics.get("avg_kernel_ms", 0.0) or 0.0)

    return {
        "ok": bool(cpu.get("ok") and fpga.get("ok")),
        "workload": {
            "boids": req.boids,
            "steps": req.steps,
            "kernel": req.kernel,
            "device": req.device,
            "seed": req.seed,
        },
        "cpu": cpu,
        "fpga": fpga,
        "speedup": {
            "total_x": (cpu_total / fpga_total) if fpga_total > 0.0 else 0.0,
            "kernel_x": (cpu_kernel / fpga_kernel) if fpga_kernel > 0.0 else 0.0,
        },
    }


@app.get("/api/fpga/status")
def api_fpga_status() -> Dict[str, Any]:
    if not Path(XRT_SMI).exists():
        raise HTTPException(status_code=500, detail=f"xrt-smi not found at {XRT_SMI}")
    result = run_command([XRT_SMI, "examine"], timeout_s=120)
    return {
        "ok": result["ok"],
        "rc": result["rc"],
        "elapsed_ms": result["elapsed_ms"],
        "output": result["output"],
    }


@app.post("/api/fpga/program")
def api_fpga_program(req: ProgramRequest) -> Dict[str, Any]:
    env_cfg = read_project_env()
    xclbin_candidate = req.xclbin_path.strip() or default_xclbin_from_env(env_cfg)
    if not xclbin_candidate:
        raise HTTPException(status_code=400, detail="xclbin path is required")

    xclbin = resolve_project_path(xclbin_candidate)
    if not xclbin.exists():
        raise HTTPException(status_code=404, detail=f"xclbin not found: {xclbin}")
    if xclbin.suffix != ".xclbin":
        raise HTTPException(status_code=400, detail="program path must end in .xclbin")

    program_device = resolve_program_device(req.device)
    cmd = [XRT_SMI, "--batch", "program", "--device", program_device, "--user", str(xclbin)]
    result = run_command(cmd, timeout_s=600)
    return {
        "ok": result["ok"],
        "rc": result["rc"],
        "elapsed_ms": result["elapsed_ms"],
        "cmd": result["cmd"],
        "resolved_device": program_device,
        "output": result["output"],
    }


if __name__ == "__main__":
    import uvicorn

    port = int(os.environ.get("WEB_PORT", "9893"))
    uvicorn.run(app, host="0.0.0.0", port=port)
