"""Audit real Unreal CSVs. A one-sided creation bound is NOT full closure."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path


def measure(path):
    path=Path(path)
    if any(tag in path.name for tag in ('swerve_components_plastic','swerve_components_fracture','mecanum_reaction')):
        raise ValueError('External destructive-bench work is outside this robot-actuator budget.')
    with path.open(encoding='utf-8-sig',newline='') as file:
        source=list(csv.DictReader(file))
    if not source: raise ValueError('Empty actual CSV')
    keys=['energy_stored_delta_j','energy_drive_work_j','energy_steering_joint_work_j','energy_rolling_work_j',
          'energy_hydraulic_dc_j','energy_hydraulic_heat_j','energy_tyre_damping_j','energy_known_input_j',
          'energy_known_loss_j','energy_unresolved_j','energy_positive_peak_j']
    rows=[]
    for row in source:
        if len(row)!=len(source[0]) or None in row: raise ValueError('CSV row width mismatch')
        if row.get('energy_reference_valid')=='1':
            if row.get('energy_full_closure')!='0': raise ValueError('Diagnostic must not claim full closure')
            item={key:float(row[key]) for key in keys}; item['time_s']=float(row['time_s'])
            if not all(math.isfinite(x) for x in item.values()): raise ValueError('Nonfinite energy observation')
            rows.append(item)
    if len(rows)<20 or rows[-1]['time_s']-rows[0]['time_s']<2: raise ValueError('Insufficient actual physical samples')
    closure_error=max(abs(r['energy_unresolved_j']-(r['energy_stored_delta_j']-r['energy_known_input_j']+r['energy_known_loss_j'])) for r in rows)
    # Normalize by delivered work, never by gravitational energy's arbitrary zero.
    scale=max(100.,max(abs(r['energy_known_input_j']) for r in rows))
    limit=20.+.05*scale
    # Preserve peaks measured between CSV writes rather than sampling them away.
    peak=max(0.,max(r['energy_positive_peak_j'] for r in rows))
    hold=[r for r in rows if r['time_s']>=20]
    drift=hold[-1]['energy_unresolved_j']-hold[0]['energy_unresolved_j'] if len(hold)>1 else None
    return dict(csv=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),samples=len(rows),
                observed_s=rows[-1]['time_s']-rows[0]['time_s'],full_closure=False,
                scope='Observed work plus constitutive loss estimates; contact, brake, bearing, structure/body losses and numerical error remain in signed residual.',
                algebra_error_j=closure_error,positive_unresolved_j=peak,positive_bound_j=limit,
                one_sided_bound_pass=peak<=limit,observer_pass=closure_error<1e-5,
                late_window_residual_change_j=drift,final=rows[-1])


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv',nargs='+'); parser.add_argument('--output',required=True); parser.add_argument('--require-pass',action='store_true')
    args=parser.parse_args(); reports=[]
    for path in args.csv:
        try: reports.append(measure(path))
        except (ValueError,KeyError,TypeError,OSError) as error: reports.append(dict(csv=path,observer_pass=False,one_sided_bound_pass=False,error=str(error),full_closure=False))
    result=dict(schema=1,full_closure=False,pass_bound_and_observer=all(r['observer_pass'] and r['one_sided_bound_pass'] for r in reports),reports=reports)
    Path(args.output).parent.mkdir(parents=True,exist_ok=True)
    Path(args.output).write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False))
    raise SystemExit(1 if args.require_pass and not result['pass_bound_and_observer'] else 0)
