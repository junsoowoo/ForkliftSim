"""Qualify the current native compact/reduced rig; never generate fake measurements."""
import argparse,csv,hashlib,json,math,re,statistics
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def num(r,k):
 v=float(r[k]);assert math.isfinite(v),(k,v);return v
def main():
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('directories',nargs='+');ap.add_argument('--output',default='Docs/References/CurrentPhysics-20261031.json');a=ap.parse_args()
 limits=json.loads((ROOT/'Config/PublicCalibration.json').read_text(encoding='utf-8'))['acceptance'];cases=[]
 for directory in a.directories:
  d=Path(directory);d=d if d.is_absolute() else ROOT/d
  assert not (d/'INVALID-REPORT.txt').exists(),'Invalid collided report directory'
  manifest=json.loads((d/'manifest.json').read_text(encoding='utf-8-sig'))
  assert manifest['reduced_mecanum'] and manifest['reference_model'],'Wrong rig'
  for result in json.loads((d/'results.json').read_text(encoding='utf-8-sig')):
   log=Path(result['log']);text=log.read_text(encoding='utf-8-sig',errors='replace')
   assert result.get('metadata_pass') is True,'Unverified run metadata'
   assert 'REDUCED_SUPPORT profile=analytic_finite_cylinder_box_v2' in text,'Wrong support profile'
   solver=re.search(r'TREAD_SOLVER quality=[^\r\n]*position_iterations=(\d+) velocity_iterations=(\d+)',text)
   assert solver and tuple(map(int,solver.groups()))==(manifest['position_iterations'],manifest['velocity_iterations']),'Wrong actual solver iterations'
   p=Path(result.get('csv') or re.search(r'STABILITY_CSV (.+)',text).group(1).strip());p=p if p.is_absolute() else ROOT/p
   rows=list(csv.DictReader(p.open(encoding='utf-8-sig',newline='')));assert rows and all(None not in r for r in rows)
   referenced=[r for r in rows if r['energy_reference_valid']=='1'];last=rows[-1]
   algebra=max(abs(num(r,'energy_unresolved_j')-(num(r,'energy_stored_delta_j')-num(r,'energy_known_input_j')+num(r,'energy_known_loss_j'))) for r in referenced)
   work=max(100,max(abs(num(r,'energy_known_input_j')) for r in referenced));peak=max(num(r,'energy_positive_peak_j') for r in referenced)
   checks={'native_behaviour':result['pass'],'energy_observer':algebra<1e-5,'no_full_closure_claim':all(r['energy_full_closure']=='0' for r in rows),'positive_energy_bound':peak<=20+.05*work}
   metrics={k:num(last,k) for k in ['time_s','chassis_x_m','chassis_y_m','speed_mps','lateral_mps','fork_actual_m','energy_unresolved_j','energy_known_input_j','energy_reduced_contact_loss_j','energy_contact_normal_work_estimate_j','energy_contact_tangent_work_estimate_j','energy_contact_correction_pseudo_j','energy_fork_damping_estimate_j']}
   metrics.update(energy_algebra_error_j=algebra,positive_energy_peak_j=peak,positive_energy_limit_j=20+.05*work)
   if result['case']=='settle':
    window=[r for r in rows if num(r,'time_s')>3];err=abs(statistics.mean(num(r,'normal_sum_n') for r in window)/(3140*9.81)-1)
    metrics['static_weight_relative_error']=err;checks['static_weight']=err<limits['static_weight_relative']
   if 'rack_' in result['case']:
    metrics['pallet_physics_mass_kg']=num(last,'pallet_physics_mass_kg');metrics['cargo_physics_mass_kg']=num(last,'cargo_physics_mass_kg')
    checks['pallet_mass']=abs(metrics['pallet_physics_mass_kg']-25)<.001;checks['cargo_mass']=abs(metrics['cargo_physics_mass_kg']-400)<.001
    if 'place' in result['case']:
     err=abs(num(last,'rack_support_n')/(425*9.81)-1);metrics['rack_weight_relative_error']=err;checks['rack_weight']=err<limits['rack_weight_relative']
   coeff=manifest.get('reference_rolling_coefficient',-1)
   if result['case']=='coast' and coeff>=0:
    window=[r for r in rows if 5.5<num(r,'time_s')<8.0];ts=[num(r,'time_s') for r in window];vs=[num(r,'speed_mps') for r in window]
    tbar=statistics.mean(ts);vbar=statistics.mean(vs);slope=sum((t-tbar)*(v-vbar) for t,v in zip(ts,vs))/sum((t-tbar)**2 for t in ts)
    # Engine-reported locked-wheel axial inertia, never inferred from a vendor's total mass.
    inertia=[float(x) for x in re.findall(r'MECANUM_INERTIA scale_z=[\d.]+ shaft_i_kgm2=([\d.]+)',text)];assert len(inertia)==4
    meff=3140+sum(inertia)/.15**2;normal=statistics.mean(num(r,'normal_sum_n') for r in window);estimated=-slope*meff/normal
    metrics.update(reference_crr=coeff,identified_simulation_crr=estimated,coast_acceleration_mps2=slope,effective_mass_kg=meff,mean_normal_n=normal,crr_relative_error=abs(estimated/coeff-1));checks['analytic_coast']=metrics['crr_relative_error']<limits['coast_reference_speed_relative']
   cases.append(dict(case=result['case'],fps=result['fps'],physics_hz=result['physics_hz'],position_iterations=manifest['position_iterations'],velocity_iterations=manifest['velocity_iterations'],reference_crr=coeff,checks=checks,pass_check=all(checks.values()),metrics=metrics,csv=str(p.relative_to(ROOT)),csv_sha256=sha(p),log=str(log.relative_to(ROOT)),log_sha256=sha(log),manifest=str((d/'manifest.json').relative_to(ROOT)),dll_sha256=manifest['dll_sha256']))
 comparisons=[]
 for name,pos,vel,dll in sorted({(r['case'],r['position_iterations'],r['velocity_iterations'],r['dll_sha256']) for r in cases}):
  # A convergence comparison must not silently mix different solvers or builds.
  group=[r for r in cases if (r['case'],r['position_iterations'],r['velocity_iterations'],r['dll_sha256'])==(name,pos,vel,dll) and r['reference_crr']<0]
  base=next((r for r in group if r['fps']==60 and r['physics_hz']==960),None)
  if not base:continue
  for r in group:
   if r is base or (r['fps'],r['physics_hz'])==(60,960):continue
   x=base['metrics'];y=r['metrics'];distance=math.hypot(x['chassis_x_m']-y['chassis_x_m'],x['chassis_y_m']-y['chassis_y_m']);speed=math.hypot(x['speed_mps']-y['speed_mps'],x['lateral_mps']-y['lateral_mps']);height=abs(x['fork_actual_m']-y['fork_actual_m']);energy=abs(x['energy_unresolved_j']-y['energy_unresolved_j']);energy_limit=50+.05*max(abs(x['energy_known_input_j']),abs(y['energy_known_input_j']))
   comparisons.append(dict(case=name,position_iterations=pos,velocity_iterations=vel,dll_sha256=dll,a=[60,960],b=[r['fps'],r['physics_hz']],distance_difference_m=distance,speed_difference_mps=speed,fork_height_difference_m=height,signed_residual_difference_j=energy,residual_difference_limit_j=energy_limit,pass_check=distance<.1 and speed<.05 and height<.02 and energy<energy_limit))
 report=dict(date='2026-10-31',scope='Actual native tests and analytic/numerical qualification. Public proxy tests are not hardware identification. Signed contact estimates are excluded from loss sum.',qualification_script_sha256=sha(Path(__file__)),calibration_config_sha256=sha(ROOT/'Config/PublicCalibration.json'),hardware_calibration_complete=False,full_energy_closure=False,limits=limits,cases=cases,comparisons=comparisons,pass_check=bool(cases) and all(r['pass_check'] for r in cases+comparisons))
 out=ROOT/a.output;out.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8');print(json.dumps({'cases':len(cases),'comparisons':len(comparisons),'failed_cases':[{k:r[k] for k in ['case','fps','physics_hz','checks']} for r in cases if not r['pass_check']],'failed_comparisons':[r for r in comparisons if not r['pass_check']],'output':str(out)},ensure_ascii=False))
 return 0 if report['pass_check'] else 1
if __name__=='__main__':raise SystemExit(main())
