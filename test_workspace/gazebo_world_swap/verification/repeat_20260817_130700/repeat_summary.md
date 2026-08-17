# roundtrip repeat summary

- started: 20260817_130700
- target: 10 consecutive PASS (WITH_RETURN=1 WITH_PEDESTRIAN=1 WITH_ARM=1)

| run | result | duration_s | cpu_pct_peak | mem_used_peak_mb | missed_rate |
|-----|--------|------------|--------------|------------------|-------------|
| 1 | PASS | 263 | 7.4 | 8693 | 2 |
| 2 | PASS | 445 | 7.4 | 8693 | 2 |
| 3 | PASS | 374 | 7.4 | 8693 | 6 |
| 4 | FAIL(rc=1) | 633 | n/a | n/a | n/a |

consecutive PASS broken at run_04 (rc=1) — see run_04.log
