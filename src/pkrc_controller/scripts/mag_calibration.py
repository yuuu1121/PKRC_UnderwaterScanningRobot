#!/usr/bin/env python3
"""
Magnetometer Calibration Tool for MicroStrain GV7-INS

3-step workflow:
  1. collect   - Subscribe /imu/mag and save raw data
  2. calibrate - Ellipsoid fitting → hard/soft iron parameters
  3. apply     - Write calibration to sensor flash via MIP protocol

Usage:
  python3 mag_calibration.py collect [--duration 180] [--output mag_cal_data.npy]
  python3 mag_calibration.py calibrate [--input mag_cal_data.npy] [--output mag_cal_result.json]
  python3 mag_calibration.py apply [--input mag_cal_result.json] [--port /dev/ttyACM0] [--baud 115200]
"""

import argparse
import json
import struct
import sys
import time
import signal
from pathlib import Path

import numpy as np


# ──────────────────────────────────────────────
# MIP Protocol (minimal implementation)
# ──────────────────────────────────────────────

class MIPProtocol:
    """Minimal MIP protocol for MicroStrain sensor communication."""

    SYNC1 = 0x75
    SYNC2 = 0x65

    # Descriptor sets
    DESC_SET_BASE = 0x01
    DESC_SET_3DM = 0x0C

    # Base command field descriptors
    FIELD_PING = 0x01
    FIELD_IDLE = 0x02

    # 3DM field descriptors
    FIELD_HARD_IRON = 0x3A
    FIELD_SOFT_IRON = 0x3B

    # Function selectors
    FUNC_WRITE = 0x01
    FUNC_READ = 0x02
    FUNC_SAVE = 0x03

    # ACK/NACK
    ACK_FIELD_DESC = 0xF1

    def __init__(self, port, baudrate=115200, timeout=3.0):
        import serial
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self._set_idle()

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def _drain(self):
        """Drain all pending input data."""
        self.ser.timeout = 0.1
        while self.ser.read(4096):
            pass
        self.ser.timeout = 3.0

    def _set_idle(self):
        """Send Set to Idle command to stop sensor streaming."""
        idle_packet = self.build_packet(self.DESC_SET_BASE, [(self.FIELD_IDLE, b'')])

        for attempt in range(3):
            # Drain any streaming data
            self._drain()
            # Send idle
            self.ser.write(idle_packet)
            self.ser.flush()
            time.sleep(0.5)

            try:
                self._read_response(self.DESC_SET_BASE)
                print("  Sensor set to idle mode.")
                self._drain()
                return
            except TimeoutError:
                waiting = self.ser.in_waiting
                if waiting > 0:
                    raw = self.ser.read(min(waiting, 64))
                    print(f"  [Attempt {attempt+1}] No idle ACK. "
                          f"Buffer has {waiting} bytes: {raw[:32].hex()}")
                else:
                    print(f"  [Attempt {attempt+1}] No idle ACK. Buffer empty.")

        print("  [WARN] Could not confirm idle mode after 3 attempts.")
        self._drain()

    def ping(self):
        """Ping the sensor to verify communication."""
        fields = [(self.FIELD_PING, b'')]
        _, resp_fields = self.send_and_receive(self.DESC_SET_BASE, fields)
        self._check_ack(resp_fields, self.FIELD_PING)
        return True

    @staticmethod
    def fletcher16(data):
        """Compute Fletcher-16 checksum over data bytes."""
        ck_a = 0
        ck_b = 0
        for b in data:
            ck_a = (ck_a + b) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF
        return ck_a, ck_b

    def build_packet(self, desc_set, fields):
        """
        Build a MIP packet.
        fields: list of (field_desc, field_data_bytes)
        """
        payload = b''
        for field_desc, field_data in fields:
            field_len = 2 + len(field_data)  # field_len + field_desc + data
            payload += struct.pack('BB', field_len, field_desc) + field_data

        payload_len = len(payload)
        header = struct.pack('BBBB', self.SYNC1, self.SYNC2, desc_set, payload_len)
        packet_no_cksum = header + payload
        # Checksum is over header (except sync? No — MIP checksum includes all bytes from header)
        # Actually MIP checksum is over: desc_set, payload_len, and all payload bytes
        ck_data = packet_no_cksum[2:]  # skip sync bytes
        ck_a, ck_b = self.fletcher16(ck_data)
        return packet_no_cksum + struct.pack('BB', ck_a, ck_b)

    def send_and_receive(self, desc_set, fields):
        """Send a MIP command and parse the response."""
        packet = self.build_packet(desc_set, fields)
        self.ser.reset_input_buffer()
        self.ser.write(packet)
        self.ser.flush()
        return self._read_response(desc_set)

    def _read_response(self, expected_desc_set):
        """Read and parse a MIP response packet matching expected descriptor set."""
        timeout_end = time.time() + 3.0
        while time.time() < timeout_end:
            b = self.ser.read(1)
            if not b:
                continue
            if b[0] != self.SYNC1:
                continue
            b2 = self.ser.read(1)
            if not b2 or b2[0] != self.SYNC2:
                continue
            # Read desc_set and payload_len
            hdr = self.ser.read(2)
            if len(hdr) < 2:
                continue
            desc_set = hdr[0]
            payload_len = hdr[1]
            if payload_len > 254:
                continue
            payload = self.ser.read(payload_len)
            if len(payload) < payload_len:
                continue
            cksum = self.ser.read(2)
            if len(cksum) < 2:
                continue

            # Verify checksum
            ck_data = hdr + payload
            ck_a, ck_b = self.fletcher16(ck_data)
            if ck_a != cksum[0] or ck_b != cksum[1]:
                continue

            # Skip packets from wrong descriptor set (e.g. streaming data 0x80)
            if desc_set != expected_desc_set:
                continue

            # Parse fields from payload
            fields = []
            idx = 0
            while idx < len(payload):
                flen = payload[idx]
                fdesc = payload[idx + 1]
                fdata = payload[idx + 2:idx + flen]
                fields.append((fdesc, fdata))
                idx += flen

            return desc_set, fields

        raise TimeoutError("No response from sensor within timeout")

    def _check_ack(self, fields, cmd_desc):
        """Check ACK field in response."""
        for fdesc, fdata in fields:
            if fdesc == self.ACK_FIELD_DESC and len(fdata) >= 2:
                acked_cmd = fdata[0]
                error_code = fdata[1]
                if acked_cmd == cmd_desc:
                    if error_code == 0:
                        return True
                    else:
                        raise RuntimeError(
                            f"NACK for command 0x{cmd_desc:02X}, error code: 0x{error_code:02X}"
                        )
        raise RuntimeError(f"No ACK field found for command 0x{cmd_desc:02X}")

    def write_hard_iron(self, offsets):
        """Write hard iron offset (3 floats in Gauss)."""
        data = struct.pack('B', self.FUNC_WRITE)
        data += struct.pack('>3f', *offsets)
        fields = [(self.FIELD_HARD_IRON, data)]
        _, resp_fields = self.send_and_receive(self.DESC_SET_3DM, fields)
        self._check_ack(resp_fields, self.FIELD_HARD_IRON)

    def save_hard_iron(self):
        """Save hard iron offset to flash."""
        data = struct.pack('B', self.FUNC_SAVE)
        fields = [(self.FIELD_HARD_IRON, data)]
        _, resp_fields = self.send_and_receive(self.DESC_SET_3DM, fields)
        self._check_ack(resp_fields, self.FIELD_HARD_IRON)

    def write_soft_iron(self, matrix):
        """Write soft iron matrix (3x3, 9 floats, row-major)."""
        data = struct.pack('B', self.FUNC_WRITE)
        flat = [matrix[i][j] for i in range(3) for j in range(3)]
        data += struct.pack('>9f', *flat)
        fields = [(self.FIELD_SOFT_IRON, data)]
        _, resp_fields = self.send_and_receive(self.DESC_SET_3DM, fields)
        self._check_ack(resp_fields, self.FIELD_SOFT_IRON)

    def save_soft_iron(self):
        """Save soft iron matrix to flash."""
        data = struct.pack('B', self.FUNC_SAVE)
        fields = [(self.FIELD_SOFT_IRON, data)]
        _, resp_fields = self.send_and_receive(self.DESC_SET_3DM, fields)
        self._check_ack(resp_fields, self.FIELD_SOFT_IRON)

    def read_hard_iron(self):
        """Read current hard iron offset from sensor."""
        data = struct.pack('B', self.FUNC_READ)
        fields = [(self.FIELD_HARD_IRON, data)]
        _, resp_fields = self.send_and_receive(self.DESC_SET_3DM, fields)
        self._check_ack(resp_fields, self.FIELD_HARD_IRON)
        for fdesc, fdata in resp_fields:
            if fdesc == self.FIELD_HARD_IRON and len(fdata) >= 12:
                return struct.unpack('>3f', fdata[:12])
        raise RuntimeError("Hard iron data not found in response")

    def read_soft_iron(self):
        """Read current soft iron matrix from sensor."""
        data = struct.pack('B', self.FUNC_READ)
        fields = [(self.FIELD_SOFT_IRON, data)]
        _, resp_fields = self.send_and_receive(self.DESC_SET_3DM, fields)
        self._check_ack(resp_fields, self.FIELD_SOFT_IRON)
        for fdesc, fdata in resp_fields:
            if fdesc == self.FIELD_SOFT_IRON and len(fdata) >= 36:
                flat = struct.unpack('>9f', fdata[:36])
                return [list(flat[i*3:(i+1)*3]) for i in range(3)]
        raise RuntimeError("Soft iron data not found in response")


# ──────────────────────────────────────────────
# Step 1: Data Collection
# ──────────────────────────────────────────────

def cmd_collect(args):
    """Collect magnetometer data from /imu/mag topic."""
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import MagneticField
    except ImportError:
        print("ERROR: rclpy not found. Source your ROS2 workspace first:")
        print("  source /opt/ros/humble/setup.bash")
        print("  source ~/hero_ws/install/setup.bash")
        sys.exit(1)

    print("=" * 60)
    print("  Magnetometer Data Collection")
    print("=" * 60)
    print()
    print("Prerequisites:")
    print("  - gv7_ins.yml must have 'imu_mag_data_rate: 10'")
    print("  - IMU driver must be running")
    print()
    print("Instructions:")
    print("  - Slowly rotate the ROV through all orientations")
    print("  - Try to cover as many directions as possible")
    print("  - Press Ctrl+C to stop collection")
    print()

    rclpy.init()
    samples = []
    start_time = [None]
    running = [True]

    def signal_handler(sig, frame):
        running[0] = False

    signal.signal(signal.SIGINT, signal_handler)

    class MagCollector(Node):
        def __init__(self):
            super().__init__('mag_calibration_collector')
            self.sub = self.create_subscription(
                MagneticField, '/imu/mag', self.mag_callback, 10
            )
            self.get_logger().info("Waiting for /imu/mag messages...")

        def mag_callback(self, msg):
            if start_time[0] is None:
                start_time[0] = time.time()
                print("Receiving data! Rotate the ROV now.\n")

            x = msg.magnetic_field.x
            y = msg.magnetic_field.y
            z = msg.magnetic_field.z
            samples.append([x, y, z])

            elapsed = time.time() - start_time[0]
            n = len(samples)
            # Print status every 10 samples
            if n % 10 == 0:
                print(f"\r  Samples: {n:5d} | Time: {elapsed:6.1f}s", end='', flush=True)

    node = MagCollector()

    while running[0]:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()

    if len(samples) < 100:
        print(f"\n\nWARNING: Only {len(samples)} samples collected.")
        print("  Recommend at least 500 samples with good spatial coverage.")
        if len(samples) == 0:
            print("\n  No data received. Check:")
            print("  1. Is the IMU driver running?")
            print("  2. Is imu_mag_data_rate set to 10 in gv7_ins.yml?")
            print("  3. Run: ros2 topic echo /imu/mag --once")
            sys.exit(1)

    data = np.array(samples)
    output = Path(args.output)
    np.save(output, data)

    elapsed = time.time() - start_time[0] if start_time[0] else 0
    print(f"\n\nCollection complete!")
    print(f"  Samples: {len(samples)}")
    print(f"  Duration: {elapsed:.1f}s")
    print(f"  Saved to: {output}")
    print(f"\nNext step: python3 mag_calibration.py calibrate")


# ──────────────────────────────────────────────
# Step 2: Calibration (Ellipsoid Fitting)
# ──────────────────────────────────────────────

def fit_ellipsoid(data):
    """
    Fit an ellipsoid to 3D magnetometer data using least squares.

    General ellipsoid equation:
      Ax² + By² + Cz² + 2Dxy + 2Exz + 2Fyz + 2Gx + 2Hy + 2Iz = 1

    Returns:
      hard_iron: (3,) offset vector
      soft_iron: (3,3) correction matrix
    """
    x = data[:, 0]
    y = data[:, 1]
    z = data[:, 2]

    # Build design matrix for: Ax² + By² + Cz² + 2Dxy + 2Exz + 2Fyz + 2Gx + 2Hy + 2Iz = 1
    D_mat = np.column_stack([
        x*x, y*y, z*z,
        2*x*y, 2*x*z, 2*y*z,
        2*x, 2*y, 2*z
    ])
    ones = np.ones(len(x))

    # Solve D_mat @ v = ones (least squares)
    v, residuals, rank, sv = np.linalg.lstsq(D_mat, ones, rcond=None)

    # Extract coefficients
    A, B, C, D, E, F, G, H, I = v

    # Build the Q matrix (quadratic form) and linear terms
    Q = np.array([
        [A, D, E],
        [D, B, F],
        [E, F, C]
    ])
    g = np.array([G, H, I])

    # Center (hard iron offset): solve Q @ center = -g
    center = np.linalg.solve(Q, -g)

    # Transform matrix: eigendecompose Q to get soft iron correction
    # The ellipsoid after centering: (x-c)^T Q (x-c) = 1 + c^T Q c - (other terms)
    # We need to normalize so the RHS = 1
    rhs = 1.0 + center @ Q @ center
    Q_normalized = Q / rhs

    # Eigendecomposition
    eigvals, eigvecs = np.linalg.eigh(Q_normalized)

    # The soft iron matrix transforms the ellipsoid to a sphere
    # For each eigenvalue λ, the semi-axis length is 1/√λ
    # The correction matrix scales each axis by √λ (relative to mean)
    radii = 1.0 / np.sqrt(np.abs(eigvals))
    mean_radius = np.mean(radii)

    # Build soft iron correction matrix: S = V @ diag(mean_r/r_i) @ V^T
    scale = mean_radius / radii
    soft_iron = eigvecs @ np.diag(scale) @ eigvecs.T

    return center, soft_iron


def check_spatial_coverage(data):
    """
    Check 3D spatial coverage using PCA singular value ratios.
    Returns (ratio_min, assessment_string).
    Good calibration needs data spread across all 3 axes.
    """
    centered = data - data.mean(axis=0)
    _, sv, _ = np.linalg.svd(centered, full_matrices=False)
    # Ratio of smallest to largest singular value
    ratio = sv[2] / sv[0] if sv[0] > 0 else 0
    return ratio, sv


def cmd_calibrate(args):
    """Compute calibration from collected data."""
    input_path = Path(args.input)
    if not input_path.exists():
        print(f"ERROR: Data file not found: {input_path}")
        print(f"  Run 'python3 mag_calibration.py collect' first.")
        sys.exit(1)

    data_tesla = np.load(input_path)
    print("=" * 60)
    print("  Magnetometer Calibration")
    print("=" * 60)
    print(f"\n  Loaded {len(data_tesla)} samples from {input_path}")

    # Convert Tesla → Gauss (1 T = 10000 Gauss)
    # ROS sensor_msgs/MagneticField uses Tesla
    # MicroStrain MIP hard iron command expects Gauss
    TESLA_TO_GAUSS = 10000.0
    data = data_tesla * TESLA_TO_GAUSS
    print(f"  Converted Tesla → Gauss (x{TESLA_TO_GAUSS:.0f})")

    # Check data quality
    ranges = data.max(axis=0) - data.min(axis=0)
    print(f"\n  Data ranges [Gauss] (X/Y/Z): {ranges[0]:.4f}, {ranges[1]:.4f}, {ranges[2]:.4f}")
    print(f"  Mean magnitude [Gauss]: {np.linalg.norm(data, axis=1).mean():.4f}")
    print(f"    (Earth's field ≈ 0.25-0.65 Gauss)")

    if np.any(ranges < 0.01):
        print("\n  WARNING: Very small range on one or more axes.")
        print("  The ROV may not have been rotated through enough orientations.")

    # Spatial coverage check
    sv_ratio, sv = check_spatial_coverage(data)
    print(f"\n  Spatial coverage (PCA singular values):")
    print(f"    SV: [{sv[0]:.4f}, {sv[1]:.4f}, {sv[2]:.4f}]")
    print(f"    Min/Max ratio: {sv_ratio:.3f}", end='')
    if sv_ratio > 0.3:
        print("  (GOOD - 3D coverage)")
    elif sv_ratio > 0.1:
        print("  (FAIR - consider more rotation)")
    else:
        print("  (POOR - data is nearly planar, calibration unreliable!)")

    # Fit ellipsoid
    print("\n  Fitting ellipsoid...")
    hard_iron, soft_iron = fit_ellipsoid(data)

    print(f"\n  Hard Iron Offset [Gauss]:")
    print(f"    X: {hard_iron[0]:+.6f}")
    print(f"    Y: {hard_iron[1]:+.6f}")
    print(f"    Z: {hard_iron[2]:+.6f}")

    print(f"\n  Soft Iron Matrix:")
    for row in soft_iron:
        print(f"    [{row[0]:+.6f}, {row[1]:+.6f}, {row[2]:+.6f}]")

    # Apply calibration to data for verification
    corrected = (data - hard_iron) @ soft_iron.T

    # Compute quality metrics
    raw_norms = np.linalg.norm(data, axis=1)
    cor_norms = np.linalg.norm(corrected, axis=1)
    improvement = raw_norms.std() / max(cor_norms.std(), 1e-12)
    print(f"\n  Quality metrics:")
    print(f"    Raw field magnitude:  mean={raw_norms.mean():.4f}, std={raw_norms.std():.4f} Gauss")
    print(f"    Corrected magnitude:  mean={cor_norms.mean():.4f}, std={cor_norms.std():.4f} Gauss")
    print(f"    Improvement (std):    {improvement:.1f}x")

    if improvement < 1.0:
        print("\n  *** WARNING: Calibration WORSENED the data! ***")
        print("  Possible causes:")
        print("    - Insufficient 3D rotation coverage")
        print("    - Data collected in only one plane")
        print("    - Not enough distinct orientations")
        print("  Recommendation: Re-collect data with better rotation coverage.")
        print("    Rotate ROV slowly through pitch, roll, AND yaw.")

    # Save result (hard_iron in Gauss, soft_iron dimensionless)
    result = {
        'hard_iron': hard_iron.tolist(),
        'soft_iron': soft_iron.tolist(),
        'unit': 'Gauss',
        'num_samples': len(data),
        'raw_magnitude_mean': float(raw_norms.mean()),
        'raw_magnitude_std': float(raw_norms.std()),
        'corrected_magnitude_mean': float(cor_norms.mean()),
        'corrected_magnitude_std': float(cor_norms.std()),
        'improvement': float(improvement),
        'spatial_coverage_ratio': float(sv_ratio),
    }
    output_path = Path(args.output)
    with open(output_path, 'w') as f:
        json.dump(result, f, indent=2)

    print(f"\n  Calibration saved to: {output_path}")

    # Try plotting
    try:
        plot_calibration(data, corrected, hard_iron)
    except ImportError:
        print("\n  [matplotlib not available, skipping 3D plot]")
    except Exception as e:
        print(f"\n  [Plot error: {e}]")

    if improvement >= 1.5:
        print(f"\nNext step: python3 mag_calibration.py apply")
    else:
        print(f"\nCalibration quality is low. Consider re-collecting data.")
        print(f"If you still want to apply: python3 mag_calibration.py apply")


def plot_calibration(raw, corrected, hard_iron):
    """3D scatter plot of raw vs corrected magnetometer data."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(14, 6))

    # Raw data
    ax1 = fig.add_subplot(121, projection='3d')
    ax1.scatter(raw[:, 0], raw[:, 1], raw[:, 2], s=1, alpha=0.5)
    ax1.scatter(*hard_iron, color='red', s=100, marker='x', label='Center')
    ax1.set_title('Raw Magnetometer Data')
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.legend()

    # Corrected data
    ax2 = fig.add_subplot(122, projection='3d')
    ax2.scatter(corrected[:, 0], corrected[:, 1], corrected[:, 2], s=1, alpha=0.5, color='green')
    ax2.set_title('Corrected Magnetometer Data')
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')

    # Make axes equal
    for ax, d in [(ax1, raw), (ax2, corrected)]:
        max_range = (d.max(axis=0) - d.min(axis=0)).max() / 2
        mid = d.mean(axis=0)
        ax.set_xlim(mid[0] - max_range, mid[0] + max_range)
        ax.set_ylim(mid[1] - max_range, mid[1] + max_range)
        ax.set_zlim(mid[2] - max_range, mid[2] + max_range)

    plt.tight_layout()
    plot_path = 'mag_cal_plot.png'
    plt.savefig(plot_path, dpi=150)
    print(f"\n  Calibration plot saved to: {plot_path}")
    plt.close()


# ──────────────────────────────────────────────
# Step 3: Apply to Sensor
# ──────────────────────────────────────────────

GV7_CONFIG_PATH = Path(__file__).resolve().parent.parent.parent / \
    'microstrain_inertial/microstrain_inertial_driver/config/gv7_ins.yml'


def cmd_apply(args):
    """Apply calibration by writing to gv7_ins.yml config.

    The modified ROS driver reads mag_hard_iron_offset and mag_soft_iron_matrix
    parameters during initialization and writes them to the sensor via MIP SDK.
    """
    input_path = Path(args.input)
    if not input_path.exists():
        print(f"ERROR: Calibration file not found: {input_path}")
        print(f"  Run 'python3 mag_calibration.py calibrate' first.")
        sys.exit(1)

    with open(input_path) as f:
        cal = json.load(f)

    hard_iron = cal['hard_iron']
    soft_iron = cal['soft_iron']
    # Flatten soft iron 3x3 to row-major list
    soft_iron_flat = [soft_iron[i][j] for i in range(3) for j in range(3)]

    print("=" * 60)
    print("  Apply Magnetometer Calibration to Sensor")
    print("=" * 60)

    # Safety check
    improvement = cal.get('improvement', 0)
    if improvement < 1.0:
        print(f"\n  *** WARNING: This calibration WORSENED the data ({improvement:.1f}x) ***")
        print(f"  Applying it may degrade heading accuracy.")
        print(f"  Re-collect data with better 3D rotation coverage.")

    unit = cal.get('unit', 'unknown')
    print(f"\n  Unit: {unit}")

    print(f"\n  Hard Iron Offset [Gauss]:")
    print(f"    X: {hard_iron[0]:+.6f}")
    print(f"    Y: {hard_iron[1]:+.6f}")
    print(f"    Z: {hard_iron[2]:+.6f}")

    print(f"\n  Soft Iron Matrix:")
    for i in range(3):
        row = soft_iron[i]
        print(f"    [{row[0]:+.6f}, {row[1]:+.6f}, {row[2]:+.6f}]")

    config_path = Path(args.config) if args.config else GV7_CONFIG_PATH
    print(f"\n  Config file: {config_path}")

    if not config_path.exists():
        print(f"  ERROR: Config file not found!")
        sys.exit(1)

    try:
        response = input("\n  Write to config and apply? (yes/no): ").strip().lower()
    except EOFError:
        response = 'no'

    if response != 'yes':
        print("  Aborted.")
        return

    # Read existing config
    config_text = config_path.read_text()

    # Format values for YAML
    hi_str = f"[{hard_iron[0]:.6f}, {hard_iron[1]:.6f}, {hard_iron[2]:.6f}]"
    si_str = f"[{', '.join(f'{v:.6f}' for v in soft_iron_flat)}]"

    # Update or add parameters
    import re

    def set_yaml_param(text, param, value):
        pattern = rf'^(\s*){param}:.*$'
        replacement = rf'\g<1>{param}: {value}'
        new_text, count = re.subn(pattern, replacement, text, flags=re.MULTILINE)
        if count == 0:
            # Add before the last line (before closing)
            # Find the last parameter line and add after it
            lines = new_text.rstrip().split('\n')
            indent = '    '  # Match existing indentation
            lines.append(f'{indent}{param}: {value}')
            new_text = '\n'.join(lines) + '\n'
        return new_text

    config_text = set_yaml_param(config_text, 'mag_hard_iron_offset', hi_str)
    config_text = set_yaml_param(config_text, 'mag_soft_iron_matrix', si_str)

    config_path.write_text(config_text)

    print(f"\n  Config updated:")
    print(f"    mag_hard_iron_offset: {hi_str}")
    print(f"    mag_soft_iron_matrix: {si_str}")

    print(f"\n  Next steps:")
    print(f"    1. Rebuild the driver:  colcon build --packages-select microstrain_inertial_driver --symlink-install")
    print(f"    2. Restart the IMU driver:  ros2 launch microstrain_inertial_driver microstrain_launch.py")
    print(f"       → Driver will write hard/soft iron to sensor flash on startup")
    print(f"    3. Check /imu/mag data — corrected values should form a sphere")
    print(f"    4. Verify heading stability")


# ──────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Magnetometer Calibration Tool for MicroStrain GV7-INS',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s collect                  # Collect mag data (Ctrl+C to stop)
  %(prog)s calibrate                # Compute calibration
  %(prog)s apply                    # Write calibration to driver config
        """
    )
    sub = parser.add_subparsers(dest='command')

    # collect
    p_collect = sub.add_parser('collect', help='Collect magnetometer data from /imu/mag')
    p_collect.add_argument('--output', default='mag_cal_data.npy',
                           help='Output file (default: mag_cal_data.npy)')

    # calibrate
    p_cal = sub.add_parser('calibrate', help='Compute calibration from collected data')
    p_cal.add_argument('--input', default='mag_cal_data.npy',
                        help='Input data file (default: mag_cal_data.npy)')
    p_cal.add_argument('--output', default='mag_cal_result.json',
                        help='Output calibration file (default: mag_cal_result.json)')

    # apply
    p_apply = sub.add_parser('apply', help='Write calibration to driver config')
    p_apply.add_argument('--input', default='mag_cal_result.json',
                          help='Input calibration file (default: mag_cal_result.json)')
    p_apply.add_argument('--config', default=None,
                          help='Config file path (default: auto-detect gv7_ins.yml)')

    args = parser.parse_args()

    if args.command is None:
        parser.print_help()
        sys.exit(1)

    if args.command == 'collect':
        cmd_collect(args)
    elif args.command == 'calibrate':
        cmd_calibrate(args)
    elif args.command == 'apply':
        cmd_apply(args)


if __name__ == '__main__':
    main()
