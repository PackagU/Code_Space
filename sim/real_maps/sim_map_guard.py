"""Identity guard for explicitly generated simulation maps; field-map guard stays unchanged."""
import hashlib
import importlib.util
import json
from pathlib import Path
import time


def pin_map(map_yaml):
    import yaml
    path = Path(map_yaml).resolve()
    image = Path(yaml.safe_load(path.read_text())['image'])
    image = image if image.is_absolute() else path.parent/image
    return {'yaml': str(path), 'image': str(image.resolve()),
            'yaml_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
            'image_sha256': hashlib.sha256(image.read_bytes()).hexdigest()}


def check_map(pin, stage, directory, source):
    """Check hashes and, at goal time, the only live map_server's actual loaded map."""
    directory = Path(directory)
    checks = {}
    try:
        checks['files_match_run_pin'] = pin_map(pin['yaml']) == pin
        if stage == 'goal':
            pids = source.live_map_servers('/proc')
            checks['exactly_one_live_map_server'] = len(pids) == 1
            if len(pids) == 1:
                outputs = list(directory.glob('nav2*.log'))
                checks['launch_log_present'] = bool(outputs)
                if outputs:
                    logdir = directory/'guard_stdout'
                    logdir.mkdir(exist_ok=True)
                    output = max(outputs, key=lambda p: p.stat().st_mtime_ns)
                    (logdir/f'map_server_{pids[0]}_stdout.log').write_text(output.read_text())
                    log, loaded, read = source.map_server_loaded(pids[0], str(logdir))
                    checks['map_server_log_found'] = log is not None
                    checks['loaded_yaml_matches_run_pin'] = loaded == pin['yaml']
                    checks['read_image_matches_run_pin'] = read == pin['image']
    except (OSError, ValueError, KeyError) as exc:
        checks['readable_map_and_log'] = False
        error = str(exc)
    else:
        error = None
    ok = bool(checks) and all(checks.values())
    record = {'validation_scope': 'simulation', 'stage': stage, 'pin': pin,
              'checks': checks, 'error': error, 'result': 'PASS' if ok else 'FAIL'}
    out = directory/'guard'
    out.mkdir(exist_ok=True)
    (out/f'sim_map_{stage}_{time.time_ns()}.json').write_text(json.dumps(record, indent=2))
    return record


def guard(pin, stage, directory):
    root = Path(__file__).resolve().parents[2]
    spec = importlib.util.spec_from_file_location('field_guard_helpers', root/'scripts/field_map_guard.py')
    source = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(source)
    record = check_map(pin, stage, directory, source)
    print('SIM_MAP_GUARD '+json.dumps(record['checks'])+' '+record['result'], flush=True)
    if record['result'] != 'PASS':
        raise RuntimeError(f'simulation map guard {stage} FAIL: {record["checks"]}')
