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
            if (path.parent/'INVALID.txt').exists():   # e.g. pre1/pre2 G2 offset bug — never aggregated
                print(f'excluded (INVALID.txt): {path.parent.name}')
                continue
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
            run_dir = path.parent
            data = json.loads(path.read_text())
            if 'e2e' not in data:
                continue
            run = dict(data['e2e'])
            run['tag'] = ('INVALID' if (run_dir/'INVALID.txt').exists() else
                          'EXPLORATORY' if (run_dir/'EXPLORATORY.txt').exists() else 'FINAL')
            run['tag_reason'] = next(((run_dir/f).read_text().strip() for f in ('INVALID.txt', 'EXPLORATORY.txt')
                                      if (run_dir/f).exists()), '')
            run['bag_status'] = data.get('bag', {}).get('status') or (
                'OK' if data.get('bag_finalized') and data.get('bag_contract_exit') == 0 else 'see result.json')
            run['git_head'] = data.get('provenance', {}).get('git_head') or '실행 당시 미기록(provenance 도입 전)'
            run['criteria'] = run.get('criteria') or criteria(run)
            runs.append(run)
    return runs


def criteria(result, min_wall=.15):
    """Same rule as run_e2e.criteria, recomputed for runs recorded before it existed."""
    failed = [] if result.get('completed') else ['not completed']
    for s in result.get('stages', []):
        if s.get('min_wall_m') is not None and s['min_wall_m'] < min_wall:
            failed.append(f"{s['stage']} min_wall {s['min_wall_m']:.3f} < {min_wall}")
        for key in ('lethal_start', 'rpm_blocks', 'ready_drops'):
            if s.get(key):
                failed.append(f"{s['stage']} {key}={s[key]}")
        if s.get('kind') == 'relocalization' and s.get('status') != 'CONVERGED':
            failed.append(f"{s['stage']} not converged")
    return {'pass': not failed, 'failed': failed, 'min_wall_threshold_m': min_wall}


def e2e_tables(runs):
    md = []
    for tag, title in (('FINAL', 'End-to-End 왕복 — 최신 모델(V5) 최종 결과'),
                       ('EXPLORATORY', 'End-to-End 왕복 — 이전 모델 탐색 실행(통계 제외)'),
                       ('INVALID', 'End-to-End 왕복 — 무효(통계 제외)')):
        group = [r for r in runs if r['tag'] == tag]
        if not group:
            continue
        md += [f'## {title}', '', '| 실행 | 캐빈 방식 | 팔·리프트 | 완주 | 실패 단계 | 기준 통과 | 기준 미달 | 시뮬 s | 실제 s | RTF | '
               '최소 벽 m | recovery | 48 rpm 차단 | ready 이탈 | bag |', '|'+'---|'*15]
        for r in group:
            st = r.get('stages', [])
            crit = r['criteria']
            md.append('| '+' | '.join([r['name'], r['cabin_mode'], '예' if r.get('arm_sim') else '아니오',
                                        '완주' if r['completed'] else '미완주', r.get('failed_stage') or '—',
                                        '통과' if crit['pass'] else '미달',
                                        '; '.join(x for x in crit['failed'] if x != 'not completed')[:160] or '—',
                                        fmt(r.get('sim_s'), 1), fmt(r.get('wall_s'), 1),
                                        fmt((r.get('sim_s') or 0)/r['wall_s'] if r.get('wall_s') else None, 2),
                                        fmt(r.get('min_wall_m')), str(sum(s.get('recoveries') or 0 for s in st)),
                                        str(sum(s.get('rpm_blocks') or 0 for s in st)),
                                        str(sum(s.get('ready_drops') or 0 for s in st)), str(r['bag_status'])])+' |')
        if tag != 'FINAL':
            md += ['', '사유: '+'; '.join(sorted({r['tag_reason'] for r in group}))]
        md.append('')
    final = [r for r in runs if r['tag'] == 'FINAL']
    rides = [(r['name'], s) for r in final for s in r.get('stages', []) if s.get('kind') == 'elevator_ride']
    relocs = {(r['name'], s['stage'][-2:]): s for r in final for s in r.get('stages', []) if s.get('kind') == 'relocalization'}
    if rides:
        md += ['## 층 전환 (최신 모델)', '', '| 실행 | 구간 | 도착 | odom 변화 m·rad | 캐빈 상대 오차 m·rad | 재정위 | 수렴 s | 새 AMCL 표본 | '
               '위치·방향 오차 m·rad | 공분산 xx·yy·yaw | TF 흔들림 m | initialpose 오차 m·rad |', '|'+'---|'*12]
        for name, s in rides:
            rl = relocs.get((name, s['stage'][-2:]), {})
            md.append('| '+' | '.join([name, s['stage'].replace('ride_', ''), s['status'],
                                        f"{fmt(s.get('odom_jump_m'), 4)}·{fmt(s.get('odom_jump_rad'), 4)}",
                                        f"{fmt(s.get('cabin_rel_err_m'), 4)}·{fmt(s.get('cabin_rel_err_rad'), 4)}",
                                        rl.get('status', '—'), fmt(rl.get('reloc_converge_s'), 1),
                                        str(rl.get('reloc_new_samples', '—')),
                                        f"{fmt(rl.get('reloc_pos_err_m'))}·{fmt(rl.get('reloc_yaw_err_rad'))}",
                                        f"{fmt(rl.get('reloc_cov_xx'))}·{fmt(rl.get('reloc_cov_yy'))}·{fmt(rl.get('reloc_cov_yaw'))}",
                                        fmt(rl.get('reloc_tf_jump_m')),
                                        f"{fmt(rl.get('initialpose_err_m'))}·{fmt(rl.get('initialpose_err_rad'))}"])+' |')
        md.append('')
    arms = [(r['name'], s) for r in final for s in r.get('stages', []) if s.get('kind') in ('arm', 'lift_parcel')]
    if arms:
        md += ['## 팔·리프트·상자 (시뮬 전용 옵션, 가정값)', '', '| 실행 | 단계 | 결과 | 시뮬 s | 내용 |', '|---|---|---|---|---|']
        for name, s in arms:
            md.append(f"| {name} | {s['stage']} | {s['status']} | {fmt(s.get('sim_s'), 1)} | {(s.get('note') or '')[:300]} |")
        md.append('')
    return md


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
            writer = csv.DictWriter(handle, list(rows[0].keys()), extrasaction='ignore', lineterminator='\n')
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
        md += e2e_tables(runs)
        stage_rows = [{'run': r['name'], 'tag': r['tag'], **st} for r in runs for st in r.get('stages', [])]
        keys = []
        for row in stage_rows:
            keys += [k for k in row if k not in keys]
        with open(f'{out}_e2e_stages.csv', 'w', newline='') as handle:
            writer = csv.DictWriter(handle, keys, extrasaction='ignore', lineterminator='\n')
            writer.writeheader()
            writer.writerows(stage_rows)
        with open(f'{out}_e2e_runs.csv', 'w', newline='') as handle:
            fields = ['name', 'tag', 'cabin_mode', 'arm_sim', 'completed', 'failed_stage', 'sim_s', 'wall_s', 'min_wall_m',
                      'bag_status', 'git_head', 'criteria_pass', 'criteria_failed', 'error']
            writer = csv.DictWriter(handle, fields, extrasaction='ignore', lineterminator='\n')
            writer.writeheader()
            for r in runs:
                writer.writerow({**r, 'criteria_pass': r['criteria']['pass'],
                                 'criteria_failed': '; '.join(r['criteria']['failed'])})
    Path(f'{out}_summary.md').write_text('\n'.join(md)+'\n')
    print('\n'.join(md))


if __name__ == '__main__':
    main()
