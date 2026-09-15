cd /home/hsm/Code_Space
L=/home/hsm/field_rpm48_base3.log
X() { docker exec -w /ros2_ws ros2_humble bash -lc "source /opt/ros/humble/setup.bash && source install/setup.bash && $*" < /dev/null; }
R=$(X "timeout 8 ros2 topic echo --once /drive/ready" | grep -c "data: true")
O=$(X "timeout 8 ros2 topic echo --once /odom --field twist.twist" | grep -c "linear")
./scripts/fieldctl status < /dev/null 2>&1 | sed -n 1,8p
if [ "$R" != 1 ] || [ "$O" != 1 ]; then echo "ABORT: drive=$R odom=$O"; exit 1; fi
M=$(wc -l < $L)
./scripts/fieldctl resume < /dev/null 2>&1 | head -1
X "(timeout 6 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.10}, angular: {z: 0.25}}' > /dev/null &) ; sleep 4; timeout 1.2 ros2 topic echo /odom --field twist.twist | grep -A1 -E '^linear|^angular' | grep -E ' x:| z:' | head -8; sleep 1.5"
X "timeout 1.5 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{}' > /dev/null"
./scripts/fieldctl stop < /dev/null 2>&1 | head -1
sleep 1
echo "--- odom after stop"; X "timeout 5 ros2 topic echo --once /odom --field twist.twist"
echo "--- bridge/gate log during test"
tail -n +$M $L | grep -E "opencr_bridge|nav_safety_gate" | cut -c1-170 | tail -15
