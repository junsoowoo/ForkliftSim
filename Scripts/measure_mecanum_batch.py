"""Compare actual mecanum CSVs against static equilibrium and friction A/B.
Contact velocities are inner-step impulse-weighted RMS; no sensor model.
"""
import argparse,csv,hashlib,json,math,statistics
from pathlib import Path
P=argparse.ArgumentParser();P.add_argument('--out',default='Saved/Validation/Mecanum-Batch1234-Checks.json');P.add_argument('--clock-a');P.add_argument('--clock-b');args=P.parse_args()
root=Path(__file__).resolve().parents[1]
spec=json.loads((root/'Config/Vehicle3D.json').read_text(encoding='utf-8-sig'))
def read(p):return list(csv.DictReader(p.open(encoding='utf-8-sig')))
def latest(case):return max((root/'Saved/Telemetry').glob(f'Stability-mecanum_{case}-*.csv'),key=lambda p:p.stat().st_mtime)
def mean(rows,key):return statistics.mean(float(r[key]) for r in rows)
def static(case):
 p=latest(case);r=[r for r in read(p) if 10<=float(r['time_s'])<13]
 assert len(r)>20 and mean(r,'fork_actual_m')>.35,case
 return p,r
base,br=static('lift');ecc,er=static('eccentric_lift');shallow,sr=static('shallow_lift')
def axle(r,which):return mean(r,which[0]+'_normal_n')+mean(r,which[1]+'_normal_n')
mass=mean(br,'payload_mass_kg');g=9.81
front=axle(sr,['FL','FR'])-axle(br,['FL','FR'])
expected_front=mass*g*(mean(sr,'payload_com_heel_x_m')-mean(br,'payload_com_heel_x_m'))/spec['wheelbase_m']
right=axle(er,['FR','RR'])-axle(br,['FR','RR'])
expected_right=mass*g*(mean(er,'payload_com_y_m')-mean(br,'payload_com_y_m'))/spec['track_m']
def check(actual,expected):return dict(actual_n=actual,expected_n=expected,relative_error=abs(actual-expected)/abs(expected),pass_check=abs(actual-expected)<.15*abs(expected))
result={'scope':'Actual CSV static load transfer versus moment balance; friction A/B; not real-component calibration.','static_window_s':[10,13],'shallow_front_load_transfer':check(front,expected_front),'eccentric_right_load_transfer':check(right,expected_right)}
paths=[base,ecc,shallow]
for case in ['grip_normal','grip_low']:
 try:p=latest(case)
 except ValueError:continue
 rows=[r for r in read(p) if 6<=float(r['time_s'])<7.5]
 result[case]={'contact_slip_rms_mps':math.sqrt(statistics.mean(sum(float(r[w+'_long_slip_mps'])**2+float(r[w+'_lateral_slip_mps'])**2 for w in ['FL','FR','RL','RR'])/4 for r in rows)),'effective_mu':mean(rows,'effective_mu')}
 paths.append(p)
if 'grip_normal' in result and 'grip_low' in result:
 result['friction_response']={'pass_check':result['grip_low']['contact_slip_rms_mps']>2*result['grip_normal']['contact_slip_rms_mps']}
if args.clock_a and args.clock_b:
 a,b=[read(root/Path(p)) for p in [args.clock_a,args.clock_b]]
 result['clock_comparison']={'endpoint_speed_difference_mps':abs(float(a[-1]['speed_mps'])-float(b[-1]['speed_mps'])),'endpoint_distance_difference_m':abs(float(a[-1]['chassis_x_m'])-float(b[-1]['chassis_x_m'])),'max_clock_lag_s':max(abs(float(r['physics_clock_lag_s'])) for r in a+b)}
 result['clock_comparison']['pass_check']=result['clock_comparison']['endpoint_speed_difference_mps']<.05 and result['clock_comparison']['endpoint_distance_difference_m']<.1 and result['clock_comparison']['max_clock_lag_s']<.034
 paths.extend(root/Path(p) for p in [args.clock_a,args.clock_b])
result['artifacts']=[{'path':str(p.relative_to(root)),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} for p in dict.fromkeys(paths)]
result['pass_check']=all(v['pass_check'] for v in result.values() if isinstance(v,dict) and 'pass_check' in v)
out=root/args.out;out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print(json.dumps(result))
raise SystemExit(0 if result['pass_check'] else 1)
