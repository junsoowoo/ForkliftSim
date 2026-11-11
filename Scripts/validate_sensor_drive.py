import argparse,csv,json,hashlib,math
from pathlib import Path
def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def rows(p):
 with open(p,encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 p=argparse.ArgumentParser();p.add_argument('reports',nargs='+');p.add_argument('--output',required=True);a=p.parse_args();root=Path(__file__).resolve().parents[1];dll=sha(root/'Binaries/Win64/UnrealEditor-ForkliftSim.dll');out=[];legacy=[]
 for folder in map(Path,a.reports):
  m=read(folder/'manifest.json');assert m['dll_sha256'].lower()==dll
  for f in m['sources']+m['configs']:assert sha(f['Path'])==f['Hash'].lower(),f['Path']
  for c in read(folder/'results.json'):
   assert c['pass'],c['case']
   if not c['case'].startswith('drive_sensor_'):legacy.append(c);continue
   r=rows(c['sensor_drive_csv']);assert r and all(len(s)==31 and None not in s for s in r);name=c['case'];fault=name.endswith(('fault','watchdog'))
   assert all(max(abs(float(s[k])) for k in ['fl_nm','fr_nm','rl_nm','rr_nm'])<=450.00001 for s in r)
   assert all(all(abs(float(s[k]))<1e-8 for k in ['fl_nm','fr_nm','rl_nm','rr_nm']) for s in r if s['fault']=='1')
   located=[s for s in r if float(s['sim_s'])>2 and s['localized']=='1'];assert len(located)>50
   final=[s for s in r if float(s['sim_s'])>28];assert final and all(s['state']=='ready' and s['localized']=='1' for s in final)
   error=lambda s:math.hypot(float(s['map_x_m'])-float(s['truth_x_m']),float(s['map_y_m'])-float(s['truth_y_m']))
   odom=lambda s:math.hypot(float(s['odom_x_m'])-float(s['truth_x_m']),float(s['odom_y_m'])-float(s['truth_y_m']))
   assert max(map(error,final))<.15
   moving=[s for s in r if 3<float(s['sim_s'])<5];speed=[math.hypot(float(s['truth_vx_mps']),float(s['truth_vy_mps'])) for s in moving];assert max(speed)>.1
   tracking=[s for s in r if 5<float(s['sim_s'])<5.7];assert tracking
   axis='vy_mps' if name.endswith('strafe') else 'vx_mps';target=.4 if name.endswith('strafe') else .5
   tracking_error=max(abs(float(s[axis])-target) for s in tracking);assert tracking_error<.1,(name,'measured velocity tracking',tracking_error)
   if name.endswith('curve'):assert max(abs(float(s['gyro_rad_s'])-.24) for s in tracking)<.08
   if fault:
    held=[s for s in r if 6.8<float(s['sim_s'])<13.8];assert held and all(s['fault']=='1' for s in held)
    stopped=[s for s in held if float(s['sim_s'])>9.5];assert max(math.hypot(float(s['truth_vx_mps']),float(s['truth_vy_mps'])) for s in stopped)<.1
    guard=rows(c['lidar_safety_csv']);assert all(s['drive_hold']==s['lift_hold']=='1' for s in guard if 6.8<float(s['sim_s'])<13.8)
    assert all(s['fault_latched']=='0' for s in guard if 14.5<float(s['sim_s'])<15.8)
   out.append({'case':name,'fps':c['fps'],'measured_velocity_tracking_error_max_mps':tracking_error,'final_position_error_max_m':max(map(error,final)),'localized_position_error_peak_m':max(map(error,located)),'final_raw_odometry_error_max_m':max(map(odom,final)),'localized_snapshots':len(located),'csv':c['sensor_drive_csv'],'csv_sha256':sha(c['sensor_drive_csv'])})
   if name.endswith('slip'):assert max(map(odom,final))>max(map(error,final))+.05,'Slip fixture must demonstrate actual odometry drift and measured map correction'
 report={'date':'2026-11-05','pass':True,'dll_sha256':dll,'sensor_drive_conditions':out,'legacy_regression':legacy,'scope':'Measured encoder/IMU drive torque and planar odometry + fixed surveyed map LiDAR registration. Known starting bay; no mapping, global localization, navigation or certified stop guarantee. GT used only in validation.'};Path(a.output).write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8');print(json.dumps({'pass':True,'sensor_drive_conditions':len(out),'legacy_conditions':len(legacy)}))
if __name__=='__main__':main()
