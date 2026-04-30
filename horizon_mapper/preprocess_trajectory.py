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

def process_csv(input_path, output_path, wheelbase=0.33, max_steering=0.5, min_velocity=0.1):
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
        print(f"[✓] Processed {N} points  |  wheelbase={wheelbase} m  "
              f"max_steer={math.degrees(max_steering):.1f}°  "
              f"velocity-adjusted={adjusted} points")
        print(f"[✓] Output → {output_path}")
        return True

    except Exception as e:
        print(f"[✗] Error processing trajectory: {e}")
        return False


# ── ROS2 / config helpers ─────────────────────────────────────────────────────

def load_ros2_params(config_path):
    """Load parameters from a ROS2 horizon_mapper.yaml file."""
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
        print(f"[✗] Error loading ROS2 config: {e}")
        return {}


def find_config_file():
    """Auto-discover the horizon_mapper.yaml config file."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    candidate = os.path.join(script_dir, '..', 'config', 'horizon_mapper.yaml')
    path = os.path.abspath(candidate)
    return path if os.path.exists(path) else None


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
    L = wheelbase or 0.33
    max_steer = max_steering or 0.5
    min_vel = min_velocity or 0.1

    if config_path and os.path.exists(config_path):
        params = load_ros2_params(config_path)
        if not wheelbase:
            L = params.get('wheelbase', 0.33)
        if not max_steering:
            max_steer = params.get('max_steering_angle', 0.5)
        if not min_velocity:
            min_vel = params.get('min_speed', 0.1)

    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    return process_csv(input_path, output_path, L, max_steer, min_vel)


# ── CLI ───────────────────────────────────────────────────────────────────────

def create_sample_trajectory(output_dir="trajectory"):
    """Create a figure-8 sample trajectory for testing."""
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

    print(f"[✓] Sample trajectory: {sample_path}")
    return sample_path


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Preprocess trajectory CSV with kinematic bicycle model'
    )
    parser.add_argument('-i', '--input',  help='Input CSV file path')
    parser.add_argument('-o', '--output', help='Output CSV file path')
    parser.add_argument('--create-sample', action='store_true',
                        help='Create a figure-8 sample trajectory and exit')
    args = parser.parse_args()

    if args.create_sample:
        create_sample_trajectory()
        sys.exit(0)

    if args.input and args.output:
        config_path = find_config_file()
        if config_path:
            print(f"[✓] Config: {config_path}")
        sys.exit(0 if preprocess_trajectory(args.input, args.output, config_path) else 1)

    config_path = find_config_file()
    if config_path:
        print(f"[✓] Config: {config_path}")
        params = load_ros2_params(config_path)
        inp = params.get('optimal_trajectory_path')
        out = params.get('reference_trajectory_path')
        if inp and out:
            sys.exit(0 if preprocess_trajectory(inp, out, config_path) else 1)
        else:
            print("[✗] Missing trajectory paths in config")
    else:
        print("[!] No config file found at ../config/horizon_mapper.yaml")
        print("Usage: python3 preprocess_trajectory.py -i input.csv -o output.csv")
    sys.exit(1)
