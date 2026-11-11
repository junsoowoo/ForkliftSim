import argparse,json,csv,hashlib,re
from pathlib import Path
def j(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def rows(p):
 with open(p,encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 p=argparse.ArgumentParser();p.add_argument('reports',nargs='+');p.add_argument('--output',required=True);a=p.parse_args();root=Path(__file__).resolve().parents[1];dll=sha(root/'Binaries/Win64/UnrealEditor-ForkliftSim.dll');cases=[]
 for folder in map(Path,a.reports):
  m=j(folder/'manifest.json');assert m['dll_sha256'].lower()==dll
  for x in m['sources']+m['configs']:assert sha(x['Path'])==x['Hash'].lower(),x['Path']
  for c in j(folder/'results.json'):
   assert c['pass'],c['case']
   if not c['case'].startswith('mission_'):cases.append({'case':c['case'],'regression':True,'pass':True,'csv':c['csv'],'csv_sha256':sha(c['csv']),'log':c['log']});continue
   assert m['rendered'];log=Path(c['log']).read_text(encoding='utf-8-sig');assert '-nullrhi' not in next(s for s in log.splitlines() if 'Command Line:' in s)
   r=rows(c['mission_csv']);assert r and all(len(x)==36 and None not in x for x in r)
   for x in r:
    cap=.18 if x['carrying']=='1' else .25;assert (float(x['vx_command_mps'])**2+float(x['vy_command_mps'])**2)**.5<=cap+1e-6
    if x['state'] in ['paused','fault_latched','blocked','cancelled']:assert x['brake']=='1' and all(float(x[k])==0 for k in ['vx_command_mps','vy_command_mps','yaw_command_rad_s'])
   name=c['case'];last=r[-1];metrics={};states=list(dict.fromkeys(x['state'] for x in r))
   if name.endswith(('low','high','avoid_pause')):
    assert last['state']=='complete' and last['pickup_receipt']==last['dropoff_receipt']=='1'
    assert all(s in states for s in ['to_pickup','pickup','to_dropoff','dropoff'])
    h=rows(c['handling_csv']);lift=next(x for x in h if x['state']=='pick_lift');t=float(lift['sim_s']);physical=rows(c['csv']);actual=min(physical,key=lambda x:abs(float(x['time_s'])-t));depth=min(float(actual['left_depth_m']),float(actual['right_depth_m']));assert depth>=.99;metrics['actual_depth_before_lift_m']=depth
   elif name.endswith('sensor_recovery'):
    assert 'fault_latched' in states and last['state']=='to_dropoff' and last['carrying']=='1' and last['localized']=='1'
   elif name.endswith('retry'):
    assert last['retries']=='1' and int(last['handling_sequence'])>=2 and last['dropoff_receipt']=='0'
    events=[(float(t),s) for t,s in re.findall(r'HANDLING_STAGE mecanum_mission_retry time=([0-9.]+) state=([a-z_]+)',log)]
    assert sum(s=='pick_height' for _,s in events)==2 and events[-1][1]=='insert'
    # Mission CSV samples the handling input before that tick's local update.
    # An end-on-transition fixture can therefore end with cached "align".
    assert 0<=events[-1][0]-float(last['sim_s'])+.001<=.101
    metrics.update(retry_insert_event_s=events[-1][0],last_sampled_handling_state=last['handling_state'])
   else:
    expected={'mission_blocked':'blocked','mission_cancel':'cancelled','mission_watchdog':'fault_latched'}[name];assert last['state']==expected and last['dropoff_receipt']=='0'
    if name.endswith('watchdog'):assert last['reason']=='mission_command_expired'
   if name.endswith('avoid_pause'):
    assert 'paused' in states;cl=re.search(r'MISSION_CHECK .* clearance_m=([0-9.]+)',log);assert cl and float(cl.group(1))>.04;metrics['minimum_independent_obstacle_clearance_m']=float(cl.group(1));metrics['clearance_method']='Actual chassis pose and fixed-heading rectangular body/fork envelope; not a mesh signed distance.'
   check=re.search(r'MISSION_CHECK .* map_error_m=([0-9.]+)',log);assert check and float(check.group(1))<.15;metrics['final_independent_map_error_m']=float(check.group(1))
   cases.append({'case':name,'pass':True,'csv':c['mission_csv'],'csv_sha256':sha(c['mission_csv']),'physics_csv':c['csv'],'handling_csv':c['handling_csv'],'log':c['log'],'states':states,'metrics':metrics})
 assert len(cases)==11 and len({c['case'] for c in cases})==11
 out={'date':'2026-11-07','pass':True,'dll_sha256':dll,'cases':cases,'scope':'Known mapped 2D workspace, fixed-heading full-envelope mecanum planning, measured localization/LiDAR and local handling receipts. Low/high/avoidance are full cycles; retry and sensor-recovery fixtures verify specific checkpoints, not full cycles. Ground truth only in independent verdicts. No SLAM, dynamic-human safety certification or universal localization guarantee.'};Path(a.output).write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n',encoding='utf-8');print(json.dumps({'pass':True,'conditions':len(cases)}))
if __name__=='__main__':main()
