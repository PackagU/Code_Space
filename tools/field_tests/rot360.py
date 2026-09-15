import math, time, rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

W = 0.25
SEP_CFG = 0.4323
rclpy.init()
n = rclpy.create_node("rot360_test")
pub = n.create_publisher(Twist, "/cmd_vel", 10)
st = {"yaw": None, "scans": [], "ready": None, "gate": None}

def on_odom(m):
    q = m.pose.pose.orientation
    st["yaw"] = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
n.create_subscription(Odometry, "/odom", on_odom, 50)
n.create_subscription(LaserScan, "/scan", lambda m: st["scans"].append(m), 10)
n.create_subscription(Bool, "/drive/ready", lambda m: st.__setitem__("ready", m.data), 10)
from rclpy.qos import qos_profile_sensor_data

def spin_for(sec, msg=None):
    end = time.monotonic() + sec; nxt = 0.0
    while time.monotonic() < end:
        if msg is not None and time.monotonic() >= nxt:
            pub.publish(msg); nxt = time.monotonic() + 0.05
        rclpy.spin_once(n, timeout_sec=0.01)

def mean_scan(k=6):
    st["scans"].clear()
    spin_for(0.1, Twist())
    while len(st["scans"]) < k:
        pub.publish(Twist()); rclpy.spin_once(n, timeout_sec=0.05)
    sc = st["scans"][-k:]
    raw_dump.append([[r if math.isfinite(r) else None for r in s.ranges] for s in sc])
    print("per-scan valid:", [sum(1 for r in s.ranges if math.isfinite(r) and s.range_min < r < s.range_max) for s in sc], "range_min/max", sc[0].range_min, sc[0].range_max)
    N = len(sc[0].ranges); out = []
    for i in range(N):
        vals = [s.ranges[i] for s in sc if len(s.ranges) == N and math.isfinite(s.ranges[i]) and s.range_min < s.ranges[i] < s.range_max]
        out.append(sorted(vals)[len(vals) // 2] if len(vals) >= 2 else float("nan"))
    return out, sc[0].angle_increment, sc[0].angle_min

raw_dump = []

spin_for(2.0, Twist())
if st["yaw"] is None or st["ready"] is not True:
    print(f"ABORT: odom={st['yaw'] is not None} drive_ready={st['ready']}"); raise SystemExit(1)
s0, inc, amin = mean_scan()
near = [(round(math.degrees(amin + i * inc)), round(r, 2)) for i, r in enumerate(s0) if math.isfinite(r) and 0.35 <= r <= 0.55]
print(f"beams={len(s0)} inc={math.degrees(inc):.3f}deg valid={sum(1 for r in s0 if math.isfinite(r))} near(0.35-0.55m)={len(near)} {near[:10]}")
if len(near) > 3:
    print("ABORT: obstacle within rotation clearance"); raise SystemExit(2)

prev = st["yaw"]; total = 0.0
cmd = Twist(); cmd.angular.z = W
t0 = time.monotonic(); nxt = 0.0; ready_false = 0
while total < 2 * math.pi - 0.02:
    if time.monotonic() - t0 > 40:
        print("ABORT: timeout"); break
    if time.monotonic() >= nxt:
        pub.publish(cmd); nxt = time.monotonic() + 0.05
    rclpy.spin_once(n, timeout_sec=0.005)
    y = st["yaw"]; d = math.atan2(math.sin(y - prev), math.cos(y - prev)); prev = y; total += d
    if st["ready"] is False: ready_false += 1
t_rot = time.monotonic() - t0
spin_for(2.0, Twist())
y = st["yaw"]; total += math.atan2(math.sin(y - prev), math.cos(y - prev))
s1, _, _ = mean_scan()
print(f"rotation time={t_rot:.1f}s odom_total={math.degrees(total):.2f}deg drive_ready_false_samples={ready_false}")
import json
with open("/tmp/rot360_scans.json", "w") as f:
    json.dump({"raw": raw_dump, "inc": inc, "angle_min": amin, "odom_total": total, "sep_cfg": SEP_CFG}, f)
print("SAVED /tmp/rot360_scans.json")

N = len(s0); best = []
for sh in range(-int(math.radians(120) / inc), int(math.radians(120) / inc) + 1):
    err = [];
    for i in range(N):
        a, b = s1[i], s0[(i + sh) % N]
        if math.isfinite(a) and math.isfinite(b): err.append(abs(a - b))
    if len(err) >= 40:
        err.sort(); trimmed = err[: int(len(err) * 0.8)]
        best.append((sum(trimmed) / len(trimmed), sh, len(err)))
best.sort()
if not best:
    print("NO MATCH CANDIDATES (scans saved; analyze offline)"); raise SystemExit(3)
e0, sh0, cnt = best[0]
side = {sh: e for e, sh, _ in best}
em, ep = side.get(sh0 - 1), side.get(sh0 + 1)
frac = 0.0
if em is not None and ep is not None and (em - 2 * e0 + ep) > 1e-9:
    frac = 0.5 * (em - ep) / (em - 2 * e0 + ep)
delta = (sh0 + frac) * inc
second = next((e for e, sh, _ in best if abs(sh - sh0) > 5), None)
real = total + delta
print(f"scan match: shift={math.degrees(delta):+.2f}deg err={e0:.3f}m (2nd-best other shift err={second if second is None else round(second,3)}) overlap={cnt}")
print(f"REAL rotation={math.degrees(real):.2f}deg  odom={math.degrees(total):.2f}deg  real/odom={real/total:.4f}")
print(f"suggested wheel_separation = {SEP_CFG} * odom/real = {SEP_CFG * total / real:.4f} m")
n.destroy_node(); rclpy.shutdown()
