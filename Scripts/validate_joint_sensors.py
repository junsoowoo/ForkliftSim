"""Validate actual native sensor snapshots, with truth used only as a test oracle."""
import argparse,csv,hashlib,json,math,re
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('directories',nargs='+');ap.add_argument('--output',default='Docs/References/JointSensorsValidation-20261031.json');args=ap.parse_args()
 spec=json.loads((ROOT/'Config/JointSensors.json').read_text(encoding='utf-8'));reports=[]
 for name in args.directories:
  d=Path(name);d=d if d.is_absolute() else ROOT/d
  manifest=json.loads((d/'manifest.json').read_text(encoding='utf-8-sig'))
  results=json.loads((d/'results.json').read_text(encoding='utf-8-sig'))
  for result in results:
   assert result['pass'],(name,result['case'],'native scenario failed')
   p=Path(result['joint_sensor_csv']);assert p.is_file(),p
   rows=list(csv.DictReader(p.open(encoding='utf-8-sig',newline='')));assert rows and all(None not in r for r in rows)
   ids=['encoder_fl','encoder_fr','encoder_rl','encoder_rr','fork_height','fork_limits'];assert {r['id'] for r in rows}==set(ids)
   valid=[r for r in rows if r['valid']=='1'];assert valid
   latency=min(float(r['delivery_sim_s'])-float(r['capture_sim_s']) for r in valid);assert latency+1e-8>=spec['latency_s']
   assert all(float(r['capture_sim_s'])<=float(r['delivery_sim_s'])+1e-9<=float(r['snapshot_sim_s'])+1e-9 for r in valid)
   assert all(float(r['age_s'])<=spec['stale_s']+1e-8 for r in valid)
   errors=[];rates=[]
   for r in rows:
    if r['valid']=='0':assert not any(r[k] for k in ['counts','position_rad','angular_velocity_rad_s','height_m','lower_active','upper_active'])
    if r['valid']!='1':continue
    if r['id'].startswith('encoder_'):
     error=abs(float(r['position_rad'])-float(r['truth_angle_rad']));errors.append(error)
     assert error<=(.5+6*spec['encoder_phase_noise_counts'])*2*math.pi/spec['encoder_counts_per_turn']+1e-8
     assert abs(float(r['position_rad'])-int(r['counts'])*2*math.pi/spec['encoder_counts_per_turn'])<1e-8
     if r['angular_velocity_rad_s']:rates.append(float(r['angular_velocity_rad_s']))
    elif r['id']=='fork_height':
     assert abs(float(r['height_m'])-float(r['truth_height_m']))<=abs(spec['height_bias_m'])+6*spec['height_sigma_m']+spec['height_quantum_m']*.5+1e-8
    else:assert not (r['lower_active']=='1' and r['upper_active']=='1')
   coverage={i:sum(r['id']==i and r['valid']=='1' for r in rows) for i in ids};assert min(coverage.values())>10
   sample_rates={}
   for ident in ids:
    last=max((r for r in valid if r['id']==ident),key=lambda r:int(r['sequence']))
    hz=spec['encoder_hz'] if ident.startswith('encoder') else spec['height_hz'] if ident=='fork_height' else spec['limit_hz']
    assert abs(int(last['sequence'])-float(last['capture_sim_s'])*hz-1)<3,(ident,'Acquisition clock skipped physical samples')
    sample_rates[ident]=(int(last['sequence'])-1)/float(last['capture_sim_s'])
   metrics={'snapshot_rows':len(rows),'valid_by_channel':coverage,'min_delivery_latency_s':latency,'max_encoder_angle_error_rad':max(errors),'observed_capture_hz':sample_rates,'min_measured_shaft_rad_s':min(rates),'max_measured_shaft_rad_s':max(rates)}
   if result['case']=='zero_grip':
    physical=list(csv.DictReader(open(result['csv'],encoding='utf-8-sig')))
    assert max(abs(float(r['speed_mps'])) for r in physical)<.1 and max(abs(v) for v in rates)>5,'Encoder must see wheel spin even when the chassis does not move'
    metrics['wheel_spin_without_vehicle_motion_verified']=True
   if result['case']=='joint_sensors':
    assert any(r['lower_active']=='1' for r in valid if r['id']=='fork_limits'),'Lower switch not exercised'
    assert any(r['upper_active']=='1' for r in valid if r['id']=='fork_limits'),'Upper switch not exercised'
    for ident in ids[:4]:
     omega=[float(r['angular_velocity_rad_s']) for r in valid if r['id']==ident and r['angular_velocity_rad_s']]
     assert min(omega)<-1 and max(omega)>1,(ident,'Physical forward/reverse shaft motion not exercised')
    if manifest.get('joint_sensor_fault')=='all':
     for ident in ids:
      outage=[r for r in rows if r['id']==ident and 6.4<float(r['snapshot_sim_s'])<8]
      assert outage and all(r['status']=='stale' and r['valid']=='0' for r in outage),(ident,'Missing stale outage')
      recovery=[r for r in rows if r['id']==ident and 8.15<float(r['snapshot_sim_s'])<8.5]
      assert recovery and all(r['valid']=='1' for r in recovery),(ident,'Recovery not fresh')
      assert int(recovery[-1]['dropped'])>50,(ident,'Missing packet-loss counter')
     metrics['outage_and_recovery_verified']=True
   reports.append({'case':result['case'],'fps':result['fps'],'physics_hz':result['physics_hz'],'transport_fault':manifest.get('joint_sensor_fault',''),'csv':str(p),'sha256':digest(p),'physics_csv':result['csv'],'dll_sha256':manifest['dll_sha256'],'metrics':metrics,'pass':True})
 comparisons=[]
 for nominal in reports:
  if nominal['case']!='joint_sensors' or nominal['transport_fault']:continue
  fault=next((r for r in reports if r['case']=='joint_sensors' and r['transport_fault']=='all' and (r['fps'],r['physics_hz'])==(nominal['fps'],nominal['physics_hz'])),None)
  if not fault:continue
  a,b=[list(csv.DictReader(open(r['physics_csv'],encoding='utf-8-sig'))) for r in [nominal,fault]];assert len(a)==len(b)
  keys=['time_s','chassis_x_m','chassis_y_m','speed_mps','lateral_mps','yaw_deg_s','fork_actual_m','normal_sum_n','energy_unresolved_j']
  differences={k:max(abs(float(x[k])-float(y[k])) for x,y in zip(a,b)) for k in keys};assert max(differences.values())<1e-6,'Sensor blackout changed physical commands/state'
  comparisons.append({'fps':nominal['fps'],'physics_hz':nominal['physics_hz'],'max_absolute_differences':differences,'pass':True})
 report={'date':'2026-10-31','scope':'Read-only virtual physical-joint sensors. Truth columns are independent diagnostics, not controller inputs. No navigation, IMU, load sensor or safety interlock claim.','native_cases':reports,'fault_physics_comparisons':comparisons,'pass':bool(reports),'hardware_calibrated':False,'controller_connected':False}
 out=ROOT/args.output;out.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8');print(json.dumps({'pass':True,'cases':len(reports),'output':str(out)},ensure_ascii=False))
if __name__=='__main__':main()
