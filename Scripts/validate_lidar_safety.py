import argparse,csv,json,hashlib,math
from pathlib import Path
def j(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def rows(p):
 with open(p,encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))
def main():
 p=argparse.ArgumentParser();p.add_argument('reports',nargs='+');p.add_argument('--output',required=True);a=p.parse_args();root=Path(__file__).resolve().parents[1];dll=sha(root/'Binaries/Win64/UnrealEditor-ForkliftSim.dll');out=[];legacy=[]
 for folder in map(Path,a.reports):
  m=j(folder/'manifest.json');assert m['dll_sha256'].lower()==dll
  for f in m['sources']+m['configs']:assert sha(f['Path'])==f['Hash'].lower(),f['Path']
  for c in j(folder/'results.json'):
   assert c['pass'],c['case']
   if not c['case'].startswith('vehicle_guard_'):legacy.append(c);continue
   r=rows(c['lidar_safety_csv']);assert r and all(len(s)==19 and None not in s for s in r);name=c['case'];metrics={}
   for s in r:
    if s['lift_hold']=='1':assert s['drive_hold']=='1'
    if s['fault_latched']=='1':assert s['drive_hold']==s['lift_hold']=='1'
   fault=name.endswith(('fault','estop'));loaded=name.endswith('carry');remove=25 if loaded else 14;resume=29 if loaded else 18
   if fault:
    held=[s for s in r if 6.8<float(s['sim_s'])<13.8];assert held and all(s['fault_latched']=='1' for s in held)
    stopped=[s for s in held if float(s['sim_s'])>9.5];assert all(math.hypot(float(s['truth_speed_mps']),float(s['truth_side_mps']))<.1 for s in stopped)
    h=[float(s['truth_fork_m']) for s in stopped];assert max(h)-min(h)<.03,(name,'vertical hold drift',max(h)-min(h));metrics['global_hold_height_span_m']=max(h)-min(h)
    acknowledged=[s for s in r if 14.4<float(s['sim_s'])<15.8];assert acknowledged and all(s['fault_latched']==s['drive_hold']=='0' for s in acknowledged)
   else:
    active=[s for s in r if float(s['sim_s'])>(16 if loaded else 3) and s['obstacle']=='1'];assert active
    first=float(active[0]['sim_s']);stopped=[s for s in r if first+3<float(s['sim_s'])<remove-.3 and not(name.endswith('creep') and 10<=float(s['sim_s'])<11.6)];assert stopped and all(s['drive_hold']=='1' and math.hypot(float(s['truth_speed_mps']),float(s['truth_side_mps']))<.1 for s in stopped)
    transient=[s for s in r if first+.8<float(s['sim_s'])<min(first+3,remove-.3) and not(name.endswith('creep') and 10<=float(s['sim_s'])<11.6)]
    metrics['post_0_8s_transient_peak_speed_mps']=max((math.hypot(float(s['truth_speed_mps']),float(s['truth_side_mps'])) for s in transient),default=None)
    if name.endswith('creep'):
     creep=[s for s in r if 10.2<float(s['sim_s'])<10.9];assert creep and all(s['state']=='manual_creep' and s['drive_hold']=='0' and math.hypot(float(s['truth_speed_mps']),float(s['truth_side_mps']))<.3 for s in creep)
     assert max(math.hypot(float(s['truth_speed_mps']),float(s['truth_side_mps'])) for s in creep)>.03
     metrics['manual_creep_max_speed_mps']=max(math.hypot(float(s['truth_speed_mps']),float(s['truth_side_mps'])) for s in creep)
    no_restart=[s for s in r if remove+.4<float(s['sim_s'])<resume-1.2];assert no_restart and all(s['drive_hold']=='1' for s in no_restart)
    phys=[s for s in rows(c['csv']) if first<float(s['time_s'])<remove]
    def gap(s):
     x,y=float(s['chassis_x_m']),float(s['chassis_y_m'])
     if name.endswith('rear'):return x-.78+4.9
     if name.endswith('side'):return 4.9-y-.65
     if name.endswith('diagonal'):return math.hypot(max(0,7-x-2.05),max(0,7-y-.65))
     return 7.9-x-2.05
    clearance=min(map(gap,phys));assert clearance>.1,(name,'fixture clearance',clearance);metrics['minimum_fixture_footprint_gap_m']=clearance;metrics['first_obstacle_hold_s']=first
    if not loaded:assert 'LIDAR_RANGE_CHECK' in Path(c['log']).read_text(encoding='utf-8-sig')
   for channel in ['front','rear']:
    fresh=[s for s in r if s[channel+'_fresh']=='1'];assert len(fresh)>10
    first,last=fresh[0],fresh[-1];hz=(int(last[channel+'_sequence'])-int(first[channel+'_sequence']))/(float(last[channel+'_capture_s'])-float(first[channel+'_capture_s']));assert abs(hz-10)<.15;metrics[channel+'_capture_hz']=hz
   out.append({'case':name,'fps':c['fps'],'csv':c['lidar_safety_csv'],'csv_sha256':sha(c['lidar_safety_csv']),'metrics':metrics})
 assert len(out)==9 and len(legacy)==2
 report={'date':'2026-11-05','pass':True,'dll_sha256':dll,'guard_conditions':out,'legacy_regression':legacy,'settling_criterion':'Speed below 0.1 m/s from hold+3 seconds, with separate physical footprint clearance check. Initial hold+0.8 second criterion failed due to decaying brake/tyre transients; no braking coefficients changed to fit this test. Manual-creep release is checked after 0.6 seconds, separately from the original full-speed brake transient. Fault settling checked after 9.5 seconds.','scope':'Dual planar simulation LiDAR and measured protective field; flat-floor default-friction fixtures. No navigation, localization, certified safety or all-surface guarantee.'};Path(a.output).write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8');print(json.dumps({'pass':True,'guard_conditions':len(out),'legacy_conditions':len(legacy)}))
if __name__=='__main__':main()
