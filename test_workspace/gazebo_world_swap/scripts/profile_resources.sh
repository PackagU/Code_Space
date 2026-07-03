#!/usr/bin/env bash
# 호스트/컨테이너/Jetson 공용 CPU/메모리/런타임 샘플러.
#
# 용도:
#   - smoke 실행 동안(WITH_PROFILE=1) 백그라운드로 띄워 부하를 기록한다.
#   - Jetson Xavier NX 실기에서 동일 명령으로 돌려 sim 대비 실기 부하를 비교한다.
#
# 단독 실행:
#   bash profile_resources.sh --out /tmp/prof --interval 2 --duration 60 --label sim
#   # duration 생략 시 SIGTERM(또는 Ctrl-C) 받을 때까지 샘플 후 요약을 쓴다.
#
# 산출물:
#   <out>/resource_samples.csv   : 한 줄 = 한 샘플
#   <out>/resource_summary.txt   : avg/peak CPU%, peak mem, 샘플 수, Jetson 여부
#
# 주의: Jetson(aarch64)에서는 tegrastats 가 있으면 GPU/EMC/온도까지 같은 CSV 마지막 칼럼(raw)에 덧붙인다.
set -euo pipefail

OUT="./prof"
INTERVAL=2
DURATION=0           # 0 = 무한(시그널까지)
LABEL="run"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --out) OUT="$2"; shift 2;;
    --interval) INTERVAL="$2"; shift 2;;
    --duration) DURATION="$2"; shift 2;;
    --label) LABEL="$2"; shift 2;;
    *) echo "unknown arg: $1" >&2; exit 2;;
  esac
done

mkdir -p "$OUT"
CSV="$OUT/resource_samples.csv"
SUMMARY="$OUT/resource_summary.txt"

# Jetson 감지: tegrastats 존재 여부.
JETSON=0
if command -v tegrastats >/dev/null 2>&1; then JETSON=1; fi

NCPU="$(nproc 2>/dev/null || echo 1)"

read_cpu_idle_total() {
  # /proc/stat 첫 줄에서 idle, total jiffies 반환.
  local cpu user nice system idle iowait irq softirq steal rest
  read -r cpu user nice system idle iowait irq softirq steal rest < /proc/stat
  local total=$((user+nice+system+idle+iowait+irq+softirq+steal))
  echo "$idle $total"
}

echo "epoch,label,cpu_pct,mem_used_mb,mem_total_mb,load1,nproc,tegrastats_raw" > "$CSV"

# CPU% 계산용 직전 값.
read -r prev_idle prev_total < <(read_cpu_idle_total)
SAMPLES=0
SUM_CPU=0
PEAK_CPU=0
PEAK_MEM=0

finish() {
  local avg_cpu=0
  if (( SAMPLES > 0 )); then
    avg_cpu=$(awk "BEGIN{printf \"%.1f\", $SUM_CPU/$SAMPLES}")
  fi
  {
    echo "label: $LABEL"
    echo "jetson: $JETSON"
    echo "nproc: $NCPU"
    echo "samples: $SAMPLES"
    echo "interval_sec: $INTERVAL"
    echo "cpu_pct_avg: $avg_cpu"
    echo "cpu_pct_peak: $PEAK_CPU"
    echo "mem_used_peak_mb: $PEAK_MEM"
    echo "csv: $CSV"
  } > "$SUMMARY"
  cat "$SUMMARY"
  exit 0
}
trap finish TERM INT

END=0
if (( DURATION > 0 )); then END=$((SECONDS + DURATION)); fi

while true; do
  sleep "$INTERVAL"

  read -r cur_idle cur_total < <(read_cpu_idle_total)
  local_didle=$((cur_idle - prev_idle))
  local_dtotal=$((cur_total - prev_total))
  prev_idle=$cur_idle
  prev_total=$cur_total
  cpu_pct=0
  if (( local_dtotal > 0 )); then
    cpu_pct=$(awk "BEGIN{printf \"%.1f\", (1 - $local_didle/$local_dtotal)*100}")
  fi

  # 메모리(MB): /proc/meminfo MemTotal/MemAvailable.
  mem_total_kb=$(awk '/MemTotal/{print $2}' /proc/meminfo)
  mem_avail_kb=$(awk '/MemAvailable/{print $2}' /proc/meminfo)
  mem_used_mb=$(( (mem_total_kb - mem_avail_kb) / 1024 ))
  mem_total_mb=$(( mem_total_kb / 1024 ))

  load1=$(awk '{print $1}' /proc/loadavg)

  tegra_raw=""
  if (( JETSON == 1 )); then
    tegra_raw=$(timeout 1 tegrastats --interval 200 2>/dev/null | head -1 | tr ',' ';' || true)
  fi

  epoch=$(date +%s)
  echo "$epoch,$LABEL,$cpu_pct,$mem_used_mb,$mem_total_mb,$load1,$NCPU,$tegra_raw" >> "$CSV"

  SAMPLES=$((SAMPLES + 1))
  SUM_CPU=$(awk "BEGIN{print $SUM_CPU + $cpu_pct}")
  if awk "BEGIN{exit !($cpu_pct > $PEAK_CPU)}"; then PEAK_CPU=$cpu_pct; fi
  if (( mem_used_mb > PEAK_MEM )); then PEAK_MEM=$mem_used_mb; fi

  if (( DURATION > 0 && SECONDS >= END )); then finish; fi
done
