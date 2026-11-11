"""Collect native conservation qualification without changing its tolerances.

Inputs are completed Unreal runs, not generated browser/demo values. Windows
file times associate each native CSV with its unique case log; ambiguity fails.
"""
import argparse
import csv
import hashlib
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('runs', nargs='+')
parser.add_argument('--out', default='Docs/References/MecanumConservation-20261029.json')
args = parser.parse_args()

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def numbers(line):
    return {k: float(v) for k, v in re.findall(r'(\w+)=([-+\d.eE]+)', line)}

report = {'date': '2026-10-29', 'scope': 'Native numerical qualification; not real-vehicle calibration or complete loss accounting.',
          'full_energy_closure': False, 'cases': [], 'runs': []}
seen = set()
for name in args.runs:
    run = ROOT / name
    manifest_path = run / 'manifest.json'
    manifest = json.loads(manifest_path.read_text(encoding='utf-8-sig'))
    results = json.loads((run / 'results.json').read_text(encoding='utf-8-sig'))
    if isinstance(results, dict):
        results = [results]
    assert manifest['dll_sha256'].lower() == digest(ROOT / 'Binaries/Win64/UnrealEditor-ForkliftSim.dll')
    for entry in manifest['sources'] + manifest['configs']:
        assert digest(Path(entry['Path'])) == entry['Hash'].lower(), entry['Path']
    report['runs'].append({'path': name, 'dll_sha256': manifest['dll_sha256'].lower(),
                           'manifest_sha256': digest(manifest_path), 'requested_physics_hz': manifest['physics_hz'],
                           'position_iterations': manifest['position_iterations'], 'velocity_iterations': manifest['velocity_iterations']})
    for result in results:
        case = result['case']
        assert case not in seen, case
        seen.add(case)
        log = Path(result['log'])
        text = log.read_text(encoding='utf-8-sig', errors='replace')
        check = next(line for line in text.splitlines() if f'MECANUM_CHECK mecanum_{case} ' in line)
        numeric = next(line for line in text.splitlines() if 'Display: MECANUM_NUMERICS ' in line)
        metrics = numbers(check)
        clocks = numbers(numeric)
        assert clocks['inner_clock_error_s'] < 1e-7
        assert clocks['smallest_observed_s'] <= 1 / 1920 + 1e-9
        candidates = [p for p in (ROOT / 'Saved/Telemetry').glob(f'Stability-mecanum_{case}-*.csv')
                      if log.stat().st_ctime - 2 <= p.stat().st_mtime <= log.stat().st_mtime + 2]
        assert len(candidates) == 1, (case, [p.name for p in candidates])
        path = candidates[0]
        rows = list(csv.DictReader(path.open(encoding='utf-8-sig')))
        assert rows and all(None not in row and None not in row.values() for row in rows)
        assert 'energy_bearing_transport_work_j' in rows[-1]
        item = {'case': case, 'pass': result['pass'], 'metrics': metrics, 'numerics': clocks,
                'log': str(log.relative_to(ROOT)), 'log_sha256': digest(log),
                'csv': str(path.relative_to(ROOT)), 'csv_sha256': digest(path), 'rows': len(rows)}
        if case == 'reaction':
            line = next(line for line in text.splitlines() if 'MECANUM_REACTION root' in line)
            bench = numbers(line)
            bench['coast_relative_energy_change'] = bench['coast_delta_j'] / bench['coast_start_j']
            assert bench['delta_l_nms'] < .02 * bench['input_impulse_nms']
            assert abs(bench['coast_relative_energy_change']) < .02
            item['bench'] = bench
        if case == 'coast':
            item['coast'] = numbers(next(line for line in text.splitlines() if 'MECANUM_COAST start' in line))
            assert item['coast']['peak_gain_j'] < .02 * item['coast']['initial_translational_j']
        if case in {'brake', 'ramp_brake'}:
            item['brake_energy'] = numbers(next(line for line in text.splitlines() if 'MECANUM_BRAKE_ENERGY ' in line))
            assert item['brake_energy']['passive'] == 1
        if case == 'ramp_brake':
            item['final_energy_observer'] = {key: float(value) for key, value in rows[-1].items() if key.startswith('energy_')}
        report['cases'].append(item)

assert len(seen) == 28, f'Expected 28 unique cases, collected {len(seen)}'
report['passed'] = sum(item['pass'] for item in report['cases'])
report['total'] = len(report['cases'])
report['behaviour_qualification_pass'] = report['passed'] == report['total']
report['source_files'] = [{'path': str(p.relative_to(ROOT)), 'sha256': digest(p)}
                          for p in sorted((ROOT / 'Source/ForkliftSim').iterdir()) if p.is_file()]
out = ROOT / args.out
out.parent.mkdir(parents=True, exist_ok=True)
out.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
print(json.dumps({'pass': report['behaviour_qualification_pass'], 'passed': report['passed'], 'total': report['total'], 'report': str(out)}))
raise SystemExit(0 if report['behaviour_qualification_pass'] else 1)
