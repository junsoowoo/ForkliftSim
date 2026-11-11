"""Compare hash-verified native wheel loads with a catalogue reference.

No fitted parameter is inferred from another simulation. The custom wheel is
not granted the comparison product's rating, inertia or material properties.
"""
import csv
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    config = json.loads((ROOT / 'Config/Mecanum.json').read_text(encoding='utf-8'))
    vehicle = json.loads((ROOT / 'Config/Vehicle3D.json').read_text(encoding='utf-8'))
    reference = config['catalogue_comparison']
    old = json.loads((ROOT / 'Docs/References/MecanumConservation-20261029.json').read_text(encoding='utf-8'))
    results = []
    for case in old['cases']:
        path = ROOT / case['csv']
        assert sha(path) == case['csv_sha256'], path
        with path.open(encoding='utf-8-sig', newline='') as f:
            rows = list(csv.DictReader(f))
        assert rows and None not in rows[0], path
        # Ignore startup settling. Keep both frame averages and inner peaks;
        # neither is substituted for a manufacturer's missing duty/speed test.
        samples = [r for r in rows if float(r['time_s']) >= 1]
        peak = max((float(r[f'{w}_normal_n']), w, float(r['time_s']))
                   for r in samples for w in ('FL', 'FR', 'RL', 'RR'))
        # This historical field is populated only for legacy integration tests.
        # A zero in mecanum CSVs means NOT OBSERVED, not zero impact force.
        inner = None
        results.append(dict(case=case['case'], csv=case['csv'], sha256=case['csv_sha256'],
                            peak_frame_load_n=peak[0], wheel=peak[1], time_s=peak[2],
                            peak_inner_load_n=inner,
                            catalogue_frame_ratio=None,
                            catalogue_inner_ratio=None))
    report = dict(date='2026-10-31', catalogue=reference, cases=results,
                  scope='Actual historical v0.18 native loads; catalogue comparison only. Not certification, calibration or fatigue life.',
                  actual_wheel_rated=False, measured_calibration_complete=False,
                  current=dict(diameter_m=2 * vehicle['wheel_radius_m'], width_m=config['wheel_width_m'],
                               mass_kg=vehicle['wheel_mass_kg'], roller_count=config['roller_count'],
                               hub_mass_kg=vehicle['wheel_mass_kg'] - config['roller_count'] * config['roller_mass_kg'],
                               roller_mass_kg=config['roller_mass_kg']),
                  unknown_parameters=reference['missing'],
                  config_sha256={name: sha(ROOT / 'Config' / name) for name in ('Mecanum.json', 'Vehicle3D.json')})
    report['largest_frame_comparison'] = max(results, key=lambda r: r['peak_frame_load_n'])
    report['largest_inner_comparison'] = None
    report['inner_peak_status'] = 'Not recorded by historical mecanum fixture; frame averages cannot qualify impact loads.'
    configs = {name: json.loads((ROOT / 'Config' / name).read_text(encoding='utf-8'))
               for name in ('Mecanum.json', 'Vehicle3D.json', 'Drivetrain.json')}
    groups = [
        ('Vehicle3D.json', ['wheel_mass_kg', 'wheel_radius_m', 'wheel_width_m', 'tyre_friction', 'road_friction',
                            'rolling_resistance_coefficient', 'bearing_coulomb_nm', 'bearing_viscous_nm_per_rad_s',
                            'drive_torque_nm', 'brake_torque_nm']),
        ('Mecanum.json', ['roller_count', 'wheel_width_m', 'hub_radius_m', 'hub_width_m', 'roller_max_radius_m',
                          'roller_half_length_m', 'roller_mass_kg', 'roller_bearing_coulomb_nm', 'roller_bearing_viscous_nms']),
        ('Drivetrain.json', [key for key in configs['Drivetrain.json'] if key.startswith('drive_')])]
    report['parameter_ledger'] = [dict(config=cfg, key=key, value=configs[cfg][key], status='design_assumption_not_fitted_to_hardware')
                                  for cfg, keys in groups for key in keys]
    report['legacy_width_note'] = 'Vehicle3D.json width is the regular-wheel reference. Active mecanum width is Mecanum.json wheel_width_m.'
    report['primary_source_archive'] = 'Docs/References/MecanumSources-20261029/manifest.json'
    out = ROOT / 'Docs/References/MecanumPropertyAudit-20261031.json'
    out.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    print(json.dumps({key: report[key] for key in ('largest_frame_comparison', 'largest_inner_comparison', 'current')}, ensure_ascii=False))


if __name__ == '__main__':
    main()
