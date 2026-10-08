#!/usr/bin/env python3
"""Collect pre-test (run_pretest.py) and E2E (run_e2e.py) results into small shareable tables.

  python3 sim/real_maps/summarize_runs.py --pretest 'pre2_*' --e2e 'e2e2_*' --out results/20261008
Reads logs/real_map_sim/<run>/result.json (gitignored); writes sim/real_maps/<out>_pretest.csv,
<out>_e2e_stages.csv and <out>_summary.md (committable). Runs host-side; no ROS needed.
All numbers are simulation evidence with provisional dimensions.
"""
import argparse
import csv
import json
import statistics
from pathlib import Path

HERE = Path(__file__).resolve().parent
LOG = HERE.parents[1]/'logs/real_map_sim'
FIELD = {'G1': 'FAILED 96.9 s (9/16 F1 시작→택배함)', 'G2': 'FAILED 49.4 s·171.2 s, collision-ahead 18·0 (9/16)'}


def fmt(v, digits=3):
    if v is None or v == '':
        return '—'
    if isinstance(v, float):
        return f'{v:.{digits}f}'
    return str(v)


def pretest(patterns):
    rows = []
    for pattern in patterns:
        for path in sorted(LOG.glob(f'{pattern}/result.json')):
            data = json.loads(path.read_text())
            if 'summary' in data:
                rows.append(data['summary'])
    return rows


def group(rows):
    out = {}
    for r in rows:
        out.setdefault((r['case'], r['params'], r.get('odom'), r.get('lidar_noise')), []).append(r)
    table = []
    for (case, params, odom, noise), items in sorted(out.items(), key=lambda kv: [str(k) for k in kv[0]]):
        def vals(key):
            return [r[key] for r in items if isinstance(r.get(key), (int, float)) and not isinstance(r.get(key), bool)]
        sims = vals('sim_s')
        table.append({
            'case': case, 'params': params, 'odom': odom, 'lidar_noise': noise, 'n': len(items),
            'succeeded': sum(r.get('status') == 'SUCCEEDED' for r in items),
            'pass': sum(bool(r.get('pass')) for r in items),
            'statuses': ' '.join(str(r.get('status') or r.get('error', '')[:20]) for r in items),
            'min_wall_m': min(vals('min_wall_m'), default=None),
            'corner_min_wall_m': min(vals('corner_min_wall_m'), default=None),
            'sim_s_median': statistics.median(sims) if sims else None,
            'rtf_mean': statistics.mean(vals('rtf')) if vals('rtf') else None,
            'collision_ahead': sum(vals('collision_ahead')), 'lethal_start': sum(vals('lethal_start')),
            'recoveries': sum(vals('recoveries')), 'rpm_blocks': sum(vals('rpm_blocks')),
            'ready_drops': sum(vals('ready_drops')),
            'arrival_err_max_m': max(vals('arrival_pos_err_m'), default=None),
            'amcl_last10_max_m': max(vals('amcl_last10_max_m'), default=None),
            'in_place_rotations': sum(vals('in_place_rotations')),
            'in_place_rotation_deg': sum(vals('in_place_rotation_deg')),
            'angular_sign_changes': sum(vals('angular_sign_changes')),
            'path_ratio_max': max(vals('path_ratio'), default=None),
            'stop_restarts': sum(vals('stop_restarts')),
        })
    return table


def e2e(patterns):
    runs = []
    for pattern in patterns:
        for path in sorted(LOG.glob(f'{pattern}/result.json')):
            data = json.loads(path.read_text())
            if 'e2e' in data:
                runs.append(data['e2e'])
    return runs


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--pretest', action='append', default=[])
    parser.add_argument('--e2e', action='append', default=[])
    parser.add_argument('--out', default='results/20261008')
    args = parser.parse_args()
    out = HERE/args.out
    out.parent.mkdir(parents=True, exist_ok=True)
    md = ['# 시뮬 결과 요약 (simulation only, 임시 치수)', '']
    rows = pretest(args.pretest)
    if rows:
        with open(f'{out}_pretest.csv', 'w', newline='') as handle:
            writer = csv.DictWriter(handle, list(rows[0].keys()), extrasaction='ignore')
            writer.writeheader()
            writer.writerows(rows)
        md += ['## 사전 시험', '', '| 시험 | params | odom | n | SUCCEEDED | 기준 통과 | 최소 벽 m | 코너 m | 시간 중앙 s | RTF | '
               'collision-ahead | recovery | 48 rpm 차단 | ready 이탈 | 도착 오차 최대 m | AMCL 마지막10 최대 m | '
               '제자리 회전 회·° | 부호 전환 | 경로비 최대 | 정지·재출발 | 9/16 현장 |',
               '|'+'---|'*21]
        for g in group(rows):
            md.append('| '+' | '.join([g['case'], g['params'], str(g['odom']), str(g['n']), str(g['succeeded']),
                                        str(g['pass']), fmt(g['min_wall_m']), fmt(g['corner_min_wall_m']),
                                        fmt(g['sim_s_median'], 1), fmt(g['rtf_mean'], 2), str(g['collision_ahead']),
                                        str(g['recoveries']), str(g['rpm_blocks']), str(g['ready_drops']),
                                        fmt(g['arrival_err_max_m']), fmt(g['amcl_last10_max_m']),
                                        f"{g['in_place_rotations']}·{g['in_place_rotation_deg']:.0f}",
                                        str(g['angular_sign_changes']), fmt(g['path_ratio_max'], 2),
                                        str(g['stop_restarts']), FIELD.get(g['case'], '—')])+' |')
        md.append('')
    runs = e2e(args.e2e)
    if runs:
        stage_rows = []
        md += ['## End-to-End 왕복', '', '| 실행 | 캐빈 방식 | 완주 | 실패 단계 | 시뮬 s | 실제 s | 최소 벽 m | 오류 |', '|---|---|---|---|---|---|---|---|']
        for run in runs:
            md.append(f"| {run['name']} | {run['cabin_mode']} | {run['completed']} | {run.get('failed_stage', '—')} | "
                      f"{fmt(run.get('sim_s'), 1)} | {fmt(run.get('wall_s'), 1)} | {fmt(run.get('min_wall_m'))} | "
                      f"{(run.get('error') or '—')[:120]} |")
            for s in run.get('stages', []):
                stage_rows.append({'run': run['name'], **s})
        if stage_rows:
            keys = ['run']+[k for k in stage_rows[0] if k != 'run']
            for s in stage_rows:
                for k in s:
                    if k not in keys:
                        keys.append(k)
            with open(f'{out}_e2e_stages.csv', 'w', newline='') as handle:
                writer = csv.DictWriter(handle, keys, extrasaction='ignore')
                writer.writeheader()
                writer.writerows(stage_rows)
        md.append('')
    Path(f'{out}_summary.md').write_text('\n'.join(md)+'\n')
    print('\n'.join(md))


if __name__ == '__main__':
    main()
