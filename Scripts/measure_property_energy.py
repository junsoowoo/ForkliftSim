"""Collect completed native observer regressions with raw CSV provenance."""
import argparse
import csv
import hashlib
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('run')
    args = parser.parse_args()
    run = ROOT / args.run
    manifest = json.loads((run / 'manifest.json').read_text(encoding='utf-8-sig'))
    assert manifest['dll_sha256'].lower() == sha(ROOT / 'Binaries/Win64/UnrealEditor-ForkliftSim.dll')
    results = json.loads((run / 'results.json').read_text(encoding='utf-8-sig'))
    if isinstance(results, dict):
        results = [results]
    report = dict(date='2026-10-29', run=args.run, dll_sha256=manifest['dll_sha256'],
                  full_energy_closure=False, measured_calibration_complete=False, cases=[])
    for entry in manifest['sources']:
        assert sha(Path(entry['Path'])) == entry['Hash'].lower(), entry['Path']
    for entry in manifest['configs']:
        if Path(entry['Path']).name != 'RealComponents.json':
            assert sha(Path(entry['Path'])) == entry['Hash'].lower(), entry['Path']
    report['config_metadata_note'] = 'RealComponents.json historical reference description was reformatted and annotated during the run. It is not loaded by the dynamics. All runtime config and compiled source hashes match the manifest.'
    for result in results:
        assert result['pass'], result
        log = Path(result['log'])
        case = result['case']
        files = [p for p in (ROOT / 'Saved/Telemetry').glob(f'Stability-mecanum_{case}-*.csv')
                 if log.stat().st_ctime - 2 <= p.stat().st_mtime <= log.stat().st_mtime + 2]
        assert len(files) == 1, (case, files)
        path = files[0]
        with path.open(encoding='utf-8-sig', newline='') as f:
            reader = csv.DictReader(f)
            rows = list(reader)
            assert all(None not in r and all(v is not None for v in r.values()) for r in rows)
        valid = [r for r in rows if r['energy_reference_valid'] == '1']
        assert valid
        values = [float(r['energy_chain_damping_j']) for r in valid]
        assert min(values) >= -1e-7
        assert all(b >= a - 1e-7 for a, b in zip(values, values[1:]))
        for r in valid:
            number = lambda k: float(r[k])
            assert abs(number('energy_unresolved_j') - number('energy_stored_delta_j') + number('energy_known_input_j') - number('energy_known_loss_j')) < 1e-5
            expected = (number('energy_shaft_loss_j') + number('energy_hydraulic_heat_j') + number('energy_chain_damping_j')
                        + number('energy_tyre_damping_j') - number('energy_rolling_work_j') + number('energy_roller_bearing_loss_j'))
            assert abs(expected - number('energy_known_loss_j')) < 1e-5
            assert r['energy_full_closure'] == '0'
        final = valid[-1]
        report['cases'].append(dict(case=case, pass_=True, csv=str(path.relative_to(ROOT)),
                                    csv_sha256=sha(path), log=str(log.relative_to(ROOT)), log_sha256=sha(log),
                                    rows=len(rows), columns=len(reader.fieldnames),
                                    energy={k: float(v) for k, v in final.items() if k.startswith('energy_')},
                                    check=result['result']))
    unit = ROOT / 'Saved/Validation/Mecanum-PropertyEnergy-Units-20261029.log'
    text = unit.read_text(encoding='utf-8-sig', errors='replace')
    assert len(re.findall(r'Result=\{Success\}', text)) == 32
    assert 'Result={Fail}' not in text
    report['unit_tests'] = dict(count=32, log=str(unit.relative_to(ROOT)), sha256=sha(unit))
    export = ROOT / 'Saved/Validation/Mecanum-PropertyEnergy-WebExport.json'
    capture = ROOT / 'Saved/Validation/Mecanum-PropertyEnergy-LiveCapture.json'
    if export.exists():
        report['actual_web_export'] = json.loads(export.read_text(encoding='utf-8'))
    if capture.exists():
        report['observed_peak_capture'] = json.loads(capture.read_text(encoding='utf-8'))
    report['primary_sources_manifest'] = 'Docs/References/MecanumSources-20261029/manifest.json'
    report['final_source_files'] = [{'path': str(p.relative_to(ROOT)), 'sha256': sha(p)} for p in sorted((ROOT / 'Source/ForkliftSim').iterdir()) if p.is_file()]
    report['final_config_files'] = [{'path': str(p.relative_to(ROOT)), 'sha256': sha(p)} for p in sorted((ROOT / 'Config').iterdir()) if p.is_file()]
    report['scope'] = '5 native regressions and chain constitutive energy accounting; catalogue provenance and observer tests are not real-vehicle calibration or complete loss closure.'
    report['pass'] = len(results) == 5
    assert report['pass']
    (ROOT / 'Docs/References/MecanumPropertyEnergy-20261029.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({'pass': report['pass'], 'cases': [(r['case'], r['energy']['energy_chain_damping_j'], r['energy']['energy_unresolved_j']) for r in report['cases']]}))


if __name__ == '__main__':
    main()
