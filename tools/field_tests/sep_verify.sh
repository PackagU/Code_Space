set -u
cd /home/hsm/Code_Space
F=src/drive_pkg/config/drive_calib.yaml
cp -n $F $F.before-sep4323-20260914
./scripts/fieldctl stop < /dev/null 2>&1 | head -1
./scripts/fieldctl base stop < /dev/null 2>&1 | tail -1
for i in $(seq 1 15); do docker exec ros2_humble pgrep -f "field_base.launch|opencr_bridge" >/dev/null || break; sleep 2; done
docker exec ros2_humble pgrep -f "field_base.launch|opencr_bridge" >/dev/null && { echo "ABORT: base still up"; exit 1; }
grep -q "^    wheel_separation: 0.4194$" $F || { echo "ABORT: unexpected wheel_separation line"; grep -n wheel_separation $F; exit 1; }
sed -i 's|^    wheel_separation: 0.4194$|    wheel_separation: 0.4323  # 2026-09-14 바닥 360° 시험: odom 362.46° vs 스캔매칭 실제 351.64° → 0.4194×1.0308 (실효 트랙폭, URDF 기하는 ±0.2097 유지)|' $F
grep -n "wheel_separation:" $F
sed -i 's|^SEP_CFG = 0.4194$|SEP_CFG = 0.4323|' /home/hsm/rot360.py
docker cp /home/hsm/rot360.py ros2_humble:/tmp/rot360.py
L=/home/hsm/field_base_sep4323.log
(nohup ./scripts/fieldctl base start --drive > $L 2>&1 < /dev/null &)
for i in $(seq 1 40); do grep -q "drive ready=True" $L && break; sleep 2; done
sleep 5
X() { docker exec -w /ros2_ws ros2_humble bash -lc "source /opt/ros/humble/setup.bash && source install/setup.bash && $*" < /dev/null; }
X "ros2 param get /packagu_opencr_bridge wheel_separation"
X "ros2 param get /packagu_opencr_bridge wheel_separation" | grep -q "0.4323" || { echo "ABORT: param not applied"; exit 1; }
ok=0
for i in 1 2 3 4; do r=$(./scripts/fieldctl resume < /dev/null 2>&1 | head -1); echo "$r"; echo "$r" | grep -q "software_stop=false (gate acknowledgement observed)" && { ok=1; break; }; sleep 3; done
if [ $ok = 1 ]; then X "python3 /tmp/rot360.py" 2>&1; else echo "SKIP: resume not acknowledged"; fi
./scripts/fieldctl stop < /dev/null 2>&1 | head -1
