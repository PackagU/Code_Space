# roundtrip repeat summary

- started: 20260817_115630
- target: 10 consecutive PASS (WITH_RETURN=1 WITH_PEDESTRIAN=1 WITH_ARM=1)

| run | result | duration_s | cpu_pct_peak | mem_used_peak_mb | missed_rate |
|-----|--------|------------|--------------|------------------|-------------|
| 1 | PASS | 616 | 5.7 | 8708 | 3 |
| 2 | PASS | 251 | 5.3 | 8623 | 5 |
| 3 | PASS | 604 | 5.3 | 8623 | 4 |
| 4 | PASS | 323 | 7.1 | 8698 | 1 |
| 5 | PASS | 276 | 7.1 | 8698 | 4 |
| 6 | PASS | 494 | 7.1 | 8698 | 3 |
| 7 | PASS | 563 | 6.3 | 8698 | 13 |
| 8 | FAIL(rc=1) | 892 | n/a | n/a | n/a |

consecutive PASS broken at run_08 (rc=1) — see run_08.log
