"""
preprocess_trajectory.py
========================
Convert a raw waypoint CSV into a kinematically-consistent reference trajectory
using the kinematic bicycle model for f1tenth.

Output columns: x, y, v, theta, delta, kappa
  x, y     – position [m]
  v        – longitudinal speed [m/s]  (adjusted down when curvature is infeasible)
  theta    – heading angle [rad]
  delta    – reference steering angle [rad]  from bicycle model: atan(L * kappa)
  kappa    – path curvature [1/m]
"""

import csv
import math
import os
import yaml
import argparse
import sys

import rclpy
from .logger_utils import HMLogger
from .params import (
    DEFAULT_WHEELBASE,
    DEFAULT_MAX_STEER,
    DEFAULT_MIN_SPEED,
    DEFAULT_MAX_DELTA_STEP,
    DEFAULT_MAX_SPEED_STEP,
)


def _get_logger():
    """Create a HMLogger-compatible logger for CLI usage."""
    class _LoggerShim:
        def get_logger(self):
            return rclpy.logging.get_logger('horizon_mapper_preprocess')

    return HMLogger(_LoggerShim(), "Preprocess")


# ── geometry helpers ─────────────────────────────────────────────────────────

def _angle_diff(a: float, b: float) -> float:
    """Shortest signed angle from b to a, in (-pi, pi]."""
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d <= -math.pi:
        d += 2 * math.pi
    return d


# ── core processing ──────────────────────────────────────────────────────────

def process_csv(
    input_path,
    output_path,
    wheelbase=DEFAULT_WHEELBASE,
    max_steering=DEFAULT_MAX_STEER,
    min_velocity=DEFAULT_MIN_SPEED,
):
    """Process a raw trajectory CSV and write a kinematically-enriched CSV.

    Input formats accepted (header comment lines '#' are skipped):
      • Named header: columns must include 'x', 'y', 'v'
      • Unnamed:      columns are x, y, v  (in that order)

    Bicycle-model enrichment:
      1. Compute heading theta_i = atan2(y_{i+1}-y_i, x_{i+1}-x_i)
      2. Compute arc-length ds_i between consecutive points
      3. Compute curvature kappa_i = dtheta_i / ds_i
      4. Compute reference steering  delta_i = atan(wheelbase * kappa_i)
         and clamp to ±max_steering
      5. Where |delta| exceeds max_steering, scale velocity down so the
         lateral acceleration stays within the kinematic feasibility limit:
           v_adj = v * (kappa_max / |kappa|)  where kappa_max = tan(max_steer)/L
    """
    log = _get_logger()
    try:
        with open(input_path, 'r') as infile:
            sample = infile.readline()
            infile.seek(0)

            if not sample:
                raise ValueError("Input CSV is empty")

            has_named_header = any(
                tok.strip().lower() == 'x' for tok in sample.split(',')
            )

            if has_named_header:
                reader = csv.DictReader(infile)
                raw = list(reader)
            else:
                reader = csv.reader(infile)
                raw = [row for row in reader if row and not row[0].startswith('#')]

        N = len(raw)
        if N == 0:
            raise ValueError("Input CSV contains no data rows")

        def get_point(row):
            if isinstance(row, dict):
                return float(row['x']), float(row['y']), float(row['v'])
            if len(row) < 3:
                raise ValueError("Each row needs at least x, y, v columns")
            return float(row[0]), float(row[1]), float(row[2])

        # ── Pass 1: extract raw x, y, v ──────────────────────────────────────
        pts = []
        for row in raw:
            x, y, v = get_point(row)
            pts.append({'x': x, 'y': y, 'v': max(v, min_velocity)})

        # ── Pass 2: heading + arc length ─────────────────────────────────────
        for i in range(N):
            x1, y1 = pts[i]['x'], pts[i]['y']
            x2, y2 = pts[(i + 1) % N]['x'], pts[(i + 1) % N]['y']
            dx, dy = x2 - x1, y2 - y1
            pts[i]['theta'] = math.atan2(dy, dx)
            pts[i]['ds'] = math.hypot(dx, dy)

        # ── Pass 3: curvature + bicycle-model steering ───────────────────────
        kappa_max = math.tan(max_steering) / wheelbase  # max feasible curvature

        processed = []
        for i in range(N):
            curr = pts[i]
            nxt = pts[(i + 1) % N]

            ds = curr['ds']
            if ds > 1e-9:
                dtheta = _angle_diff(nxt['theta'], curr['theta'])
                kappa = dtheta / ds
            else:
                kappa = 0.0

            # Bicycle model: delta = atan(L * kappa)
            delta_raw = math.atan(wheelbase * kappa)
            delta = max(-max_steering, min(max_steering, delta_raw))

            # Velocity adjustment for kinematically infeasible sections
            abs_kappa = abs(kappa)
            if abs_kappa > kappa_max and abs_kappa > 1e-9:
                v_adjusted = max(curr['v'] * (kappa_max / abs_kappa), min_velocity)
            else:
                v_adjusted = curr['v']

            processed.append({
                'x':     curr['x'],
                'y':     curr['y'],
                'v':     v_adjusted,
                'theta': curr['theta'],
                'delta': delta,
                'kappa': kappa,
            })

        # ── Write output ──────────────────────────────────────────────────────
        with open(output_path, 'w', newline='') as outfile:
            fieldnames = ['x', 'y', 'v', 'theta', 'delta', 'kappa']
            writer = csv.DictWriter(outfile, fieldnames=fieldnames)
            writer.writeheader()
            for row in processed:
                writer.writerow(row)

        adjusted = sum(1 for p in processed if abs(p['delta']) >= max_steering)
        log.success(
            f"Processed {N} points | "
            f"velocity_adjusted={adjusted} points"
        )
        log.info(f"Output -> {output_path}")
        return True

    except Exception as e:
        log.error("Error processing trajectory", exception=e)
        return False


# ── ROS2 / config helpers ─────────────────────────────────────────────────────

def load_ros2_params(config_path):
    """Load parameters from a ROS2 horizon_mapper.yaml file."""
    log = _get_logger()
    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        params = (
            config.get('horizon_mapper_node', {}).get('ros__parameters', {})
        )
        return {
            'optimal_trajectory_path':  params.get('optimal_trajectory_path'),
            'reference_trajectory_path': params.get('reference_trajectory_path'),
            'wheelbase':           params.get('wheelbase', 0.33),
            'max_steering_angle':  params.get('max_steering_angle', 0.5),
            'min_speed':           params.get('min_speed', 0.1),
            'enable_logging':      params.get('enable_logging', True),
            'horizon_N':           params.get('horizon_N', 10),
        }
    except Exception as e:
        log.error("Error loading ROS2 config", exception=e)
        return {}


def find_config_file():
    """Auto-discover the horizon_mapper.yaml config file."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    candidate = os.path.join(script_dir, '..', 'config', 'horizon_mapper.yaml')
    path = os.path.abspath(candidate)
    return path if os.path.exists(path) else None

def sanity_check(
    file_path="optimal_trajectory.csv",
    wheelbase=DEFAULT_WHEELBASE,
    max_steering=DEFAULT_MAX_STEER,
    min_velocity=DEFAULT_MIN_SPEED,
    max_delta_step=DEFAULT_MAX_DELTA_STEP,
    max_speed_step=DEFAULT_MAX_SPEED_STEP,
):
    """Sanity check a trajectory CSV for kinematic consistency.

    Checks:
      - finite values for x/y/v/theta/delta/kappa
      - speed >= min_velocity
      - |delta| <= max_steering
      - kappa approximately matches delta (if both provided)
      - no large per-step jumps in delta or speed
    """
    log = _get_logger()
    log.info(f"Running sanity check: {file_path}")

    if not os.path.isfile(file_path):
        log.error("Sanity check failed: file not found", exception=FileNotFoundError(file_path))
        return False

    try:
        with open(file_path, 'r') as infile:
            reader = csv.DictReader(infile)
            rows = list(reader)

        if not rows:
            log.error("Sanity check failed: CSV has no data rows")
            return False

        def get_float(row, key, default=None):
            if key not in row or row[key] == "":
                return default
            return float(row[key])

        issues = 0
        prev_delta = None
        prev_v = None

        kappa_max = math.tan(max_steering) / wheelbase

        for i, row in enumerate(rows):
            x = get_float(row, 'x')
            y = get_float(row, 'y')
            v = get_float(row, 'v')
            theta = get_float(row, 'theta', 0.0)
            delta = get_float(row, 'delta')
            kappa = get_float(row, 'kappa')

            if x is None or y is None or v is None:
                issues += 1
                log.warn(f"Row {i}: missing required fields (x/y/v)")
                continue

            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(v) and math.isfinite(theta)):
                issues += 1
                log.warn(f"Row {i}: non-finite values")
                continue

            if v < min_velocity:
                issues += 1
                log.warn(f"Row {i}: v={v:.3f} below min_velocity={min_velocity:.3f}")

            if delta is not None:
                if not math.isfinite(delta):
                    issues += 1
                    log.warn(f"Row {i}: non-finite delta")
                elif abs(delta) > max_steering + 1e-6:
                    issues += 1
                    log.warn(f"Row {i}: |delta|={abs(delta):.3f} exceeds max_steering={max_steering:.3f}")

            if kappa is not None and not math.isfinite(kappa):
                issues += 1
                log.warn(f"Row {i}: non-finite kappa")

            if delta is not None and kappa is not None:
                kappa_from_delta = math.tan(delta) / wheelbase
                if abs(kappa_from_delta - kappa) > max(0.05, 0.25 * abs(kappa)):
                    issues += 1
                    log.warn(
                        f"Row {i}: kappa mismatch (kappa={kappa:.4f}, tan(delta)/L={kappa_from_delta:.4f})"
                    )

            if kappa is not None and abs(kappa) > kappa_max + 1e-6:
                issues += 1
                log.warn(f"Row {i}: |kappa|={abs(kappa):.4f} exceeds kappa_max={kappa_max:.4f}")

            if prev_delta is not None and delta is not None:
                if abs(delta - prev_delta) > max_delta_step:
                    issues += 1
                    log.warn(
                        f"Row {i}: delta jump {abs(delta - prev_delta):.3f} rad exceeds {max_delta_step:.3f}"
                    )
            if prev_v is not None:
                if abs(v - prev_v) > max_speed_step:
                    issues += 1
                    log.warn(
                        f"Row {i}: speed jump {abs(v - prev_v):.3f} m/s exceeds {max_speed_step:.3f}"
                    )

            if delta is not None:
                prev_delta = delta
            prev_v = v

        if issues == 0:
            log.success("Sanity check passed")
            return True

        log.error(f"Sanity check failed with {issues} issues")
        return False

    except Exception as e:
        log.error("Sanity check failed", exception=e)
        return False

def preprocess_trajectory(
        input_path,
        output_path,
        config_path=None,
        wheelbase=None,
        max_steering=None,
        min_velocity=None):
    """Entry point called by the ROS2 node at startup.

    Args:
        input_path:   Path to raw waypoint CSV.
        output_path:  Path where enriched CSV will be written.
        config_path:  Optional ROS2 params YAML (values overridden by explicit args).
        wheelbase:    Vehicle wheelbase [m].
        max_steering: Maximum steering angle [rad].
        min_velocity: Minimum speed [m/s] (floor for the velocity profile).

    Returns:
        bool: True on success.
    """
    L = wheelbase or DEFAULT_WHEELBASE
    max_steer = max_steering or DEFAULT_MAX_STEER
    min_vel = min_velocity or DEFAULT_MIN_SPEED

    log = _get_logger()
    if config_path and os.path.exists(config_path):
        params = load_ros2_params(config_path)
        if not wheelbase:
            L = params.get('wheelbase', DEFAULT_WHEELBASE)
        if not max_steering:
            max_steer = params.get('max_steering_angle', DEFAULT_MAX_STEER)
        if not min_velocity:
            min_vel = params.get('min_speed', DEFAULT_MIN_SPEED)

    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    if not process_csv(input_path, output_path, L, max_steer, min_vel):
        return False

    return sanity_check(
        output_path,
        wheelbase=L,
        max_steering=max_steer,
        min_velocity=min_vel,
    )


# ── CLI ───────────────────────────────────────────────────────────────────────

def create_sample_trajectory(output_dir="trajectory"):
    """Create a figure-8 sample trajectory for testing."""
    log = _get_logger()
    os.makedirs(output_dir, exist_ok=True)
    sample_path = os.path.join(output_dir, "optimal_trajectory.csv")

    with open(sample_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['x', 'y', 'v'])
        writer.writeheader()
        for i in range(100):
            t = 2 * math.pi * i / 100
            writer.writerow({
                'x': 5 * math.sin(t),
                'y': 2.5 * math.sin(2 * t),
                'v': max(0.7, 1.5 + 0.8 * math.cos(t)),
            })

    log.success(f"Sample trajectory: {sample_path}")
    return sample_path


if __name__ == '__main__':
    log = _get_logger()
    parser = argparse.ArgumentParser(
        description='Preprocess trajectory CSV with kinematic bicycle model'
    )
    parser.add_argument('-i', '--input',  help='Input CSV file path')
    parser.add_argument('-o', '--output', help='Output CSV file path')
    parser.add_argument('--create-sample', action='store_true',
                        help='Create a figure-8 sample trajectory and exit')
    parser.add_argument('--sanity-check', nargs='?', const='optimal_trajectory.csv',
                        help='Run sanity check on CSV (default: optimal_trajectory.csv)')
    args = parser.parse_args()

    if args.create_sample:
        create_sample_trajectory()
        sys.exit(0)

    if args.sanity_check is not None:
        sys.exit(0 if sanity_check(args.sanity_check) else 1)

    if args.input and args.output:
        config_path = find_config_file()
        if config_path:
            log.info(f"Config: {config_path}")
        sys.exit(0 if preprocess_trajectory(args.input, args.output, config_path) else 1)

    config_path = find_config_file()
    if config_path:
        log.info(f"Config: {config_path}")
        params = load_ros2_params(config_path)
        inp = params.get('optimal_trajectory_path')
        out = params.get('reference_trajectory_path')
        if inp and out:
            sys.exit(0 if preprocess_trajectory(inp, out, config_path) else 1)
        else:
            log.error("Missing trajectory paths in config")
    else:
        log.warn("No config file found at ../config/horizon_mapper.yaml")
        log.info("Usage: python3 preprocess_trajectory.py -i input.csv -o output.csv")
    sys.exit(1)
