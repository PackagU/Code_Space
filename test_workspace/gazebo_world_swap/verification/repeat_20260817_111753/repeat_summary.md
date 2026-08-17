# roundtrip repeat summary

- started: 20260817_111753
- target: 10 consecutive PASS (WITH_RETURN=1 WITH_PEDESTRIAN=1 WITH_ARM=1)

| run | result | duration_s | cpu_pct_peak | mem_used_peak_mb | missed_rate |
|-----|--------|------------|--------------|------------------|-------------|
| 1 | PASS | 502 | 6.8 | 8684 | 10 |
| 2 | PASS | 276 | 4.6 | 8656 | 2 |
| 3 | PASS | 252 | 4.6 | 8656 | 1 |
| 4 | PASS | 258 | 5.7 | 8708 | 2 |
| 5 | FAIL(rc=1) | 769 | n/a | n/a | n/a |

consecutive PASS broken at run_05 (rc=1) — see run_05.log
