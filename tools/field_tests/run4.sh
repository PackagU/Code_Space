cd /home/hsm/Code_Space
N=/home/hsm/field_f3_nav4.log
./scripts/fieldctl resume < /dev/null 2>&1 | head -1
(nohup ./scripts/fieldctl nav start F3 /ros2_ws/maps/field/f3/f3_c192.yaml > $N 2>&1 < /dev/null &)
for i in $(seq 1 45); do grep -q "Managed nodes are active" $N && break; grep -q "^error" $N && break; sleep 2; done
grep -E "^error|Managed nodes are active" $N | tail -2
grep -q "Managed nodes are active" $N || { echo "ABORT: nav not active"; ./scripts/fieldctl stop < /dev/null 2>&1 | head -1; exit 1; }
if ! ./scripts/fieldctl pose set 0.38 0.29 -0.41 < /dev/null 2>&1 | tee /tmp/pose4.txt | tail -2 || ! grep -q "AMCL accepted" /tmp/pose4.txt; then
  echo "ABORT: pose not accepted"; ./scripts/fieldctl stop < /dev/null 2>&1 | head -1; exit 1
fi
P=/home/hsm/probe_f3_run4.log
(nohup ./scripts/fieldctl probe f3_run4_v010 90 > $P 2>&1 < /dev/null &)
sleep 3
grep "Begin navigating" $N | tail -1 | cut -c1-40 > /dev/null
timeout -s INT 120 ./scripts/fieldctl goal f3_blue F3 < /dev/null 2>&1 | grep -vE "^feedback" | tail -3
echo "=== AUTO STOP"
./scripts/fieldctl stop < /dev/null 2>&1 | head -1
grep -E "Begin navigating|Reached the goal|Goal succeeded|Goal failed|aborted" $N | tail -3 | cut -c1-160
