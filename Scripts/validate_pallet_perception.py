import argparse,json,csv,math,hashlib,re
from pathlib import Path
def j(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def rows(p):
 with open(p,encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))
def main():
 p=argparse.ArgumentParser();p.add_argument('reports',nargs='+');p.add_argument('--output',required=True);a=p.parse_args();root=Path(__file__).resolve().parents[1];dll=sha(root/'Binaries/Win64/UnrealEditor-ForkliftSim.dll');out=[];regress=[]
 for folder in map(Path,a.reports):
  m=j(folder/'manifest.json');assert m['dll_sha256'].lower()==dll
  for item in m['sources']+m['configs']:assert sha(item['Path'])==item['Hash'].lower(),item['Path']
  for c in j(folder/'results.json'):
   assert c['pass'],c['case']
   if not c['case'].startswith('pallet_vision_'):regress.append(c);continue
   assert m['rendered'];log=Path(c['log']).read_text(encoding='utf-8-sig');assert '-nullrhi' not in next(l for l in log.splitlines() if 'Command Line:' in l)
   r=rows(c['pallet_perception_csv']);assert r and all(len(s)==24 and None not in s for s in r);name=c['case'];fault=m.get('proximity_sensor_fault')=='all';metrics={}
   valid=[s for s in r if s['vision_valid']=='1'];full=[s for s in r if s['state']=='fully_inserted']
   if name.endswith('box'):assert not valid and not full
   else:assert len(valid)>3 and all(float(s['confidence'])>=.65 and .25<float(s['left_width_m'])<.43 and .25<float(s['right_width_m'])<.43 for s in valid)
   if name.endswith(('yaw','shift')):assert r[-1]['state']=='misaligned' and not full
   if name.endswith('disabled'):assert all(s['state']=='unknown' and s['vision_valid']=='0' for s in r if float(s['sim_s'])>5.2)
   if fault:assert all(s['state']=='unknown' and s['signals_fresh']=='0' for s in r if 6.5<float(s['sim_s'])<7.9)
   if name.endswith('insert'):
    assert any(s['state']=='partial' for s in r) and full
    physical=rows(c['csv']);minimum=100;max_error=0
    for s in full:
     assert s['signals_fresh']==s['aligned']=='1'
     assert min(float(s['depth_left_m']),float(s['depth_right_m']))-float(s['uncertainty_m'])>=1.02-1e-6
     truth=min(physical,key=lambda t:abs(float(t['time_s'])-float(s['sim_s'])));actual=min(float(truth['left_depth_m']),float(truth['right_depth_m']));minimum=min(minimum,actual);assert actual>=.99
     max_error=max(max_error,abs(float(s['depth_left_m'])-float(truth['left_depth_m'])),abs(float(s['depth_right_m'])-float(truth['right_depth_m'])))
    metrics={'full_snapshots':len(full),'minimum_actual_full_depth_m':minimum,'maximum_full_depth_estimate_error_m':max_error}
   mcheck=re.search(r'PALLET_PERCEPTION_CHECK .*?max_distance_error_m=([0-9.]+)',log);assert mcheck
   metrics['distance_check_max_error_m']=float(mcheck.group(1));out.append({'case':name,'proximity_fault':fault,'pass':True,'valid_vision_snapshots':len(valid),'csv':c['pallet_perception_csv'],'csv_sha256':sha(c['pallet_perception_csv']),'physics_csv':c['csv'],'proximity_csv':c['proximity_sensor_csv'],'log':c['log'],'frame':c['pallet_perception_csv']+'.frame.json','metrics':metrics})
 assert len(out)==9 and len(regress)==3
 result={'date':'2026-11-06','pass':True,'dll_sha256':dll,'gpu_conditions':out,'regression':regress,'scope':'Actual GPU depth on a known pallet family + proximity/presence/load/height fusion. Test ground truth is diagnostics only. Timed manual fixture input is not autonomous insertion. No universal object recognition or certified insertion guarantee.'};Path(a.output).write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8');print(json.dumps({'pass':True,'gpu_conditions':len(out),'regression_conditions':len(regress)}))
if __name__=='__main__':main()
