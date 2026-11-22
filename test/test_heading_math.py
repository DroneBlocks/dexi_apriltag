"""Math checks for apriltag_odometry.py heading fusion (no ROS needed: ROS modules are stubbed).

    python test/test_heading_math.py
"""
import importlib.util
import math
import random
import sys
import types
from pathlib import Path

for name in ("rclpy", "rclpy.node", "rclpy.qos", "tf2_ros", "px4_msgs", "px4_msgs.msg", "apriltag_msgs", "apriltag_msgs.msg"):
    m = types.ModuleType(name)
    sys.modules[name] = m
sys.modules["rclpy.node"].Node = object
for n in ("QoSProfile", "ReliabilityPolicy", "DurabilityPolicy"):
    setattr(sys.modules["rclpy.qos"], n, object)
for n in ("VehicleOdometry", "VehicleLocalPosition", "VehicleAttitude"):
    setattr(sys.modules["px4_msgs.msg"], n, object)
sys.modules["apriltag_msgs.msg"].AprilTagDetectionArray = object
spec = importlib.util.spec_from_file_location("odom", Path(__file__).resolve().parent.parent / "scripts" / "apriltag_odometry.py")
odom = importlib.util.module_from_spec(spec)
spec.loader.exec_module(odom)


def quat_from_matrix(R):
    tr = R[0][0] + R[1][1] + R[2][2]
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2
        return (0.25 * S, (R[2][1] - R[1][2]) / S, (R[0][2] - R[2][0]) / S, (R[1][0] - R[0][1]) / S)
    if R[0][0] > R[1][1] and R[0][0] > R[2][2]:
        S = math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2
        return ((R[2][1] - R[1][2]) / S, 0.25 * S, (R[0][1] + R[1][0]) / S, (R[0][2] + R[2][0]) / S)
    if R[1][1] > R[2][2]:
        S = math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2
        return ((R[0][2] - R[2][0]) / S, (R[0][1] + R[1][0]) / S, 0.25 * S, (R[1][2] + R[2][1]) / S)
    S = math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2
    return ((R[1][0] - R[0][1]) / S, (R[0][2] + R[2][0]) / S, (R[1][2] + R[2][1]) / S, 0.25 * S)


def tag_quat_in_base_link(psi, tag_yaw):
    """Tag lying flat, printed top edge pointing tag_yaw (map frame, 0 = north); drone level, heading psi.
    AprilTag frame: x = printed right, y = printed down, z = into the tag. base_link axes (from the node's
    translation code): x = body down, y = body forward, z = body right."""
    def ned_to_bl(v):                  # NED vector -> body FRD -> base_link (down, fwd, right)
        n, e, d = v
        f = n * math.cos(psi) + e * math.sin(psi)
        r = -n * math.sin(psi) + e * math.cos(psi)
        return (d, f, r)
    top = (math.cos(tag_yaw), math.sin(tag_yaw), 0.0)
    x = (math.cos(tag_yaw + math.pi / 2), math.sin(tag_yaw + math.pi / 2), 0.0)    # printed right = top turned +90
    y = (-top[0], -top[1], 0.0)                                                      # printed down
    z = (0.0, 0.0, 1.0)                                                              # into the floor
    cols = [ned_to_bl(x), ned_to_bl(y), ned_to_bl(z)]
    R = [[cols[j][i] for j in range(3)] for i in range(3)]
    return quat_from_matrix(R)


def test_heading():
    rng = random.Random(0)
    worst = 0.0
    for _ in range(2000):
        psi, tyaw = rng.uniform(-math.pi, math.pi), rng.uniform(-math.pi, math.pi)
        q = tag_quat_in_base_link(psi, tyaw)
        h = odom.heading_from_tag(*q, tag_map_yaw=tyaw, yaw_offset=math.radians(90.0))
        worst = max(worst, abs(odom.wrap_angle(h - psi)))
    assert worst < 1e-9, worst
    return worst


def test_quat():
    for r, p, y in ((0.1, -0.05, 2.0), (0.0, 0.0, -3.0), (-0.2, 0.15, 0.5)):
        w, x, yq, z = odom.quat_from_euler(r, p, y)
        yaw = math.atan2(2 * (w * z + x * yq), 1 - 2 * (yq * yq + z * z))
        assert abs(odom.wrap_angle(yaw - y)) < 1e-9
        assert abs(w * w + x * x + yq * yq + z * z - 1) < 1e-12


def test_circular_mean():
    assert abs(odom.circular_mean([math.radians(179), math.radians(-179)]) - math.pi) < 1e-9 or \
        abs(odom.circular_mean([math.radians(179), math.radians(-179)]) + math.pi) < 1e-9


if __name__ == "__main__":
    print("heading worst error (rad):", test_heading())
    test_quat()
    test_circular_mean()
    print("all ok")
