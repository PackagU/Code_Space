# roundtrip repeat summary

- started: 20260817_105618
- target: 10 consecutive PASS (WITH_RETURN=1 WITH_PEDESTRIAN=1 WITH_ARM=1)

| run | result | duration_s | cpu_pct_peak | mem_used_peak_mb | missed_rate |
|-----|--------|------------|--------------|------------------|-------------|
| 1 | PASS | 654 | 6.9 | 8717 | 3 |
| 2 | FAIL(rc=1) | 533 | n/a | n/a | n/a |

consecutive PASS broken at run_02 (rc=1) — see run_02.log
