import time, rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

V, W, DUR = 0.10, 0.25, 3.0
rclpy.init()
n = rclpy.create_node("stand_rpm48_test")
pub = n.create_publisher(Twist, "/cmd_vel", 10)
odom, ready = [], []
n.create_subscription(Odometry, "/odom", lambda m: odom.append((time.monotonic(), m.twist.twist.linear.x, m.twist.twist.angular.z)), 50)
n.create_subscription(Bool, "/drive/ready", lambda m: ready.append((time.monotonic(), m.data)), 50)

def spin_for(sec, msg=None):
    end = time.monotonic() + sec
    nxt = 0.0
    while time.monotonic() < end:
        if msg is not None and time.monotonic() >= nxt:
            pub.publish(msg); nxt = time.monotonic() + 0.05
        rclpy.spin_once(n, timeout_sec=0.01)

spin_for(2.0, Twist())
if not ready or not ready[-1][1]:
    print("ABORT: drive ready not true"); raise SystemExit(1)
cmd = Twist(); cmd.linear.x = V; cmd.angular.z = W
t0 = time.monotonic()
spin_for(DUR, cmd)
t1 = time.monotonic()
spin_for(1.5, Twist())
seg = [o for o in odom if t1 - 1.5 <= o[0] <= t1]
if seg:
    vs = [o[1] for o in seg]; ws = [o[2] for o in seg]
    print(f"steady(last 1.5s) n={len(seg)} v min/mean/max={min(vs):.3f}/{sum(vs)/len(vs):.3f}/{max(vs):.3f}  w min/mean/max={min(ws):.3f}/{sum(ws)/len(ws):.3f}/{max(ws):.3f}")
else:
    print("no odom during command")
falses = sum(1 for t, d in ready if t0 <= t <= t1 and not d)
print(f"drive_ready false samples during command: {falses}")
last = odom[-1] if odom else None
print("odom after zero:", last and (round(last[1], 3), round(last[2], 3)))
n.destroy_node(); rclpy.shutdown()
