import argparse,csv,json,hashlib
from pathlib import Path
def readj(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def rows(p):
 with open(p,encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 p=argparse.ArgumentParser();p.add_argument('reports',nargs='+');p.add_argument('--output',required=True);a=p.parse_args();root=Path(__file__).resolve().parents[1];dll=sha(root/'Binaries/Win64/UnrealEditor-ForkliftSim.dll');out=[]
 expected={'camera_loss':'pallet_track_or_camera_lost','sensor_loss':'required_sensor_or_controller_fault','estop':'required_sensor_or_controller_fault','cancel':'operator_cancel','obstacle':'non_target_obstacle'}
 for folder in map(Path,a.reports):
  m=readj(folder/'manifest.json');assert m['dll_sha256'].lower()==dll
  for item in m['sources']+m['configs']:assert sha(item['Path'])==item['Hash'].lower(),item['Path']
  for c in readj(folder/'results.json'):
   assert c['pass'],c['case']
   if not c['case'].startswith('handling_'):out.append({'case':c['case'],'regression':True,'pass':True,'csv':c['csv'],'csv_sha256':sha(c['csv'])});continue
   assert m['rendered'];log=Path(c['log']).read_text(encoding='utf-8-sig');assert '-nullrhi' not in next(s for s in log.splitlines() if 'Command Line:' in s)
   r=rows(c['handling_csv']);assert r and all(len(x)==33 and None not in x for x in r)
   for x in r:
    assert abs(float(x['vx_command_mps']))<=.12+1e-6 and abs(float(x['vy_command_mps']))<=.06+1e-6 and abs(float(x['yaw_command_rad_s']))<=.08+1e-6
    if x['fault']=='1':assert x['brake']=='1' and all(float(x[k])==0 for k in ['vx_command_mps','vy_command_mps','yaw_command_rad_s'])
    if x['state']=='insert' and x['measurements']=='0':assert x['brake']=='1' and float(x['vx_command_mps'])==0
   fault=next((v for k,v in expected.items() if c['case'].endswith(k)),None);states=[x['state'] for x in r];physical=rows(c['csv']);metrics={}
   if fault:
    assert r[-1]['fault']=='1' and r[-1]['reason']==fault
    first=next(x for x in r if x['fault']=='1');end=float(physical[-1]['chassis_x_m']);at=min(physical,key=lambda t:abs(float(t['time_s'])-float(first['sim_s'])));metrics={'fault_s':float(first['sim_s']),'post_fault_x_drift_m':abs(end-float(at['chassis_x_m']))}
   else:
    assert r[-1]['state']=='complete' and r[-1]['transfer']=='1'
    if 'place' in c['case']:
     lower=next(i for i,x in enumerate(r) if x['state']=='place_withdraw');assert any(float(x['left_load_n'])+float(x['right_load_n'])<130 for x in r[max(0,lower-8):lower+1]);assert 'place_lower' in states
    else:
     lift=next(x for x in r if x['state']=='pick_lift');t=float(lift['sim_s']);assert any(x['full']=='1' for x in r if t-.3<float(x['sim_s'])<=t)
     actual=min(physical,key=lambda x:abs(float(x['time_s'])-t));minimum=min(float(actual['left_depth_m']),float(actual['right_depth_m']));assert minimum>=.99
     metrics={'actual_depth_before_lift_m':minimum};assert 'pick_withdraw' in states
   out.append({'case':c['case'],'pass':True,'csv':c['handling_csv'],'csv_sha256':sha(c['handling_csv']),'physics_csv':c['csv'],'log':c['log'],'states':list(dict.fromkeys(states)),'metrics':metrics})
 assert len(out)==14 and len({x['case'] for x in out})==14 and sum(x.get('regression',False) for x in out)==3
 result={'date':'2026-11-06','pass':True,'dll_sha256':dll,'cases':out,'scope':'Measured local handling, known pallet family and surveyed rack; independent physical ground truth only in test verdicts. Manual carrier preparation for place fixtures. No global route planning, universal recognition or certified target masking.'};Path(a.output).write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8');print(json.dumps({'pass':True,'cases':len(out)}))
if __name__=='__main__':main()
