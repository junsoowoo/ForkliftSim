"""Fit an equivalent foundation ONLY to identified wheel-rig measurements.

No catalogue-load, Shore-hardness or material-coupon data are converted to wheel
stiffness. This command writes a candidate report; it never edits Config.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def validate_measurements(data, vehicle, tyre):
    if data.get('schema') != 1 or data.get('source_kind') != 'measured_wheel_rig':
        raise ValueError('Identified wheel-rig measurements are required, not coupon/catalogue data.')
    for key in ('source_document', 'equipment_id', 'test_method', 'measured_at'):
        if not isinstance(data.get(key), str) or not data[key].strip():
            raise ValueError('Missing provenance: '+key)
    geometry=data.get('wheel', {})
    for key, expected in [('radius_m',vehicle['wheel_radius_m']),('width_m',vehicle['wheel_width_m']),('tread_depth_m',tyre['tread_depth_m'])]:
        value=geometry.get(key)
        if not isinstance(value,(int,float)) or not math.isfinite(value) or abs(value-expected)>1e-6:
            raise ValueError('Measurement geometry differs: '+key)
    if data.get('normal_unit') != 'N' or data.get('position_unit') != 'm':
        raise ValueError('Explicit SI units N and m are required.')
    temperature=data.get('temperature_c')
    if not isinstance(temperature,(int,float)) or not math.isfinite(temperature):
        raise ValueError('Measurement temperature is required.')
    points=data.get('points', [])
    counts={'train':0,'holdout':0}; heights=set()
    for point in points:
        h, force, uncertainty=point.get('hub_height_m'),point.get('normal_n'),point.get('uncertainty_n')
        if not all(isinstance(x,(int,float)) and math.isfinite(x) for x in (h,force,uncertainty)):
            raise ValueError('Nonfinite or missing measured values.')
        if not .002<=vehicle['wheel_radius_m']-h<=.02 or force<=0 or uncertainty<=0 or uncertainty>=force:
            raise ValueError('Measurement outside supported indentation/uncertainty range.')
        split=point.get('split')
        if split not in counts:
            raise ValueError('Split must be train or holdout.')
        counts[split]+=1; heights.add(h)
    if counts['train']<3 or counts['holdout']<2 or len(heights)<3:
        raise ValueError('At least three training and two held-out wheel measurements at varied indentation are required.')
    if len({p['hub_height_m'] for p in points if p['split']=='train'})<3:
        raise ValueError('Training needs three distinct indentation levels.')
    return points


def calibrate(args):
    root=Path(args.config_root); vehicle=read(root/'Vehicle3D.json'); tyre=read(root/'Tyres.json')
    report={'schema':1,'status':'awaiting_wheel_measurements','applied':False,
            'active_foundation_n_per_m3':tyre['tread_foundation_n_per_m3'],
            'scope':'One equivalent foundation coefficient; damping/Poisson/friction not identified.',
            'config_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(root.glob('*.json'))}}
    if not args.measurements:
        report['reason']='No actual wheel load-deflection measurements supplied. Published capacity/hardness are insufficient.'
        return report
    data=read(args.measurements)
    points=validate_measurements(data,vehicle,tyre)
    report['measurement_sha256']=hashlib.sha256(Path(args.measurements).read_bytes()).hexdigest()
    if not args.numerics:
        report.update(status='blocked_numerical_qualification',reason='Supply a passing, hash-matched native numerical report first.')
        return report
    qualification=read(args.numerics)
    report['numerics_sha256']=hashlib.sha256(Path(args.numerics).read_bytes()).hexdigest()
    # Verify native artifacts instead of trusting an isolated pass=true flag.
    native_path=Path(qualification.get('native_report',''))
    if not native_path.is_file() or hashlib.sha256(native_path.read_bytes()).hexdigest().lower()!=qualification.get('native_sha256','').lower():
        raise ValueError('Native qualification artifact missing or hash mismatch.')
    native=read(native_path); manifest=read(native_path.parent/'manifest.json')
    oracle=Path(__file__).with_name('tread_ring_reference.py')
    if hashlib.sha256(oracle.read_bytes()).hexdigest().lower()!=manifest.get('oracle_sha256','').lower():
        raise ValueError('Qualification used a different independent constitutive oracle.')
    dll=root.parent/'Binaries/Win64/UnrealEditor-ForkliftSim.dll'
    if not dll.is_file() or hashlib.sha256(dll.read_bytes()).hexdigest().lower()!=manifest.get('dll_sha256','').lower():
        raise ValueError('Native executable differs from the qualified build.')
    for config in manifest['configs']:
        p=root/Path(config['Path']).name
        if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest().lower()!=config['Hash'].lower():
            raise ValueError('Native qualification used different config: '+p.name)
    if not qualification.get('pass') or len(native)!=4 or not all(p.get('pass') and p.get('numerics_observed') for p in native):
        report.update(status='blocked_numerical_qualification',reason='Native force/step/resolution qualification has not passed. Do not absorb solver error into material coefficients.')
        return report
    low=min(p['height_m'] for p in native); high=max(p['height_m'] for p in native)
    if any(not low<=p['hub_height_m']<=high for p in points):
        report.update(status='blocked_outside_qualified_range',reason='Extend native numerical qualification to the measured indentation range before fitting.')
        return report
    report['qualification_scope']=qualification.get('scope')
    report['measurement_conditions']={k:data[k] for k in ('source_document','equipment_id','test_method','measured_at','temperature_c','wheel')}
    from scipy.optimize import minimize_scalar
    from tread_ring_reference import solve
    count=native[0]['count']; cache={}
    def predict(k,point):
        key=(k,point['hub_height_m'])
        if key not in cache:
            result=solve(count,point['hub_height_m'],0.,radius=vehicle['wheel_radius_m'],width=vehicle['wheel_width_m'],
                         depth=tyre['tread_depth_m'],foundation=k,mass=tyre['tread_mass_kg'],poisson=tyre.get('tread_poisson_ratio',.49))
            if not result['success']: raise ValueError('Independent mechanical solve failed.')
            cache[key]=result['normal_n']
        return cache[key]
    training=[p for p in points if p['split']=='train']
    def objective(logk):
        k=math.exp(logk)
        return sum(((predict(k,p)-p['normal_n'])/p['uncertainty_n'])**2 for p in training)
    fit=minimize_scalar(objective,bounds=(math.log(1e7),math.log(1e9)),method='bounded',options={'xatol':1e-5})
    k=math.exp(fit.x)
    comparisons=[dict(p,predicted_n=predict(k,p),relative_error=abs(predict(k,p)/p['normal_n']-1)) for p in points]
    holdout=all(p['relative_error']<=.03 and abs(p['predicted_n']-p['normal_n'])<=2*p['uncertainty_n'] for p in comparisons if p['split']=='holdout')
    report.update(status='candidate_requires_native_revalidation' if fit.success and holdout else 'rejected_fit',
                  candidate_foundation_n_per_m3=k,holdout_pass=holdout,points=comparisons,
                  reason='Candidate changes the constitutive law; repeat native numerical/loaded-terrain tests before applying. No runtime file was changed.')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config-root',default='Config')
    parser.add_argument('--measurements')
    parser.add_argument('--numerics')
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    try:
        result=calibrate(args)
    except (ValueError,KeyError,TypeError,OSError) as error:
        result={'schema':1,'status':'rejected_input','applied':False,'reason':str(error)}
    Path(args.output).parent.mkdir(parents=True,exist_ok=True)
    Path(args.output).write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k!='points'},ensure_ascii=False))
    raise SystemExit(0 if result['status']=='candidate_requires_native_revalidation' else 2)
