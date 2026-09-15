cd /home/hsm/Code_Space
./scripts/fieldctl stop < /dev/null 2>&1 | head -1
./scripts/fieldctl base stop < /dev/null 2>&1 | tail -1
for i in $(seq 1 15); do docker exec ros2_humble pgrep -f "field_base.launch|opencr_bridge" >/dev/null || break; sleep 2; done
docker exec ros2_humble pgrep -af "field_base.launch|opencr_bridge" || echo BASE_DOWN
F=src/drive_pkg/config/drive_calib.yaml
grep -q "feedback_max_abs_rpm: 0.0" $F && sed -i 's|^    feedback_max_abs_rpm: 0.0$|    feedback_max_abs_rpm: 56.0  # 2026-09-13 받침대 44 rpm 명령에서 F 48.548 오버슈트로 drive ready 끊김 → 명령 상한 48 유지, 피드백 타당성만 56|' $F
grep -n "feedback_max_abs_rpm:" $F
L=/home/hsm/field_rpm48_base4.log
(nohup ./scripts/fieldctl base start --drive > $L 2>&1 < /dev/null &)
for i in $(seq 1 30); do grep -q "drive ready=True" $L && break; sleep 2; done
sleep 4
docker cp /home/hsm/stand_test.py ros2_humble:/tmp/stand_test.py
./scripts/fieldctl status < /dev/null 2>&1 | sed -n 1,7p
