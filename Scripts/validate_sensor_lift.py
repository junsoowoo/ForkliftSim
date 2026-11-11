import argparse,csv,hashlib,json,math,statistics
from pathlib import Path
def j(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def rows(p):
    with Path(p).open(encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))
def main():
    p=argparse.ArgumentParser();p.add_argument('reports',nargs='+');p.add_argument('--output',required=True);a=p.parse_args();root=Path(__file__).resolve().parents[1];spec=j(root/'Config/SensorLiftControl.json');dll=sha(root/'Binaries/Win64/UnrealEditor-ForkliftSim.dll');cases=[];legacy=[]
    for folder in map(Path,a.reports):
        m=j(folder/'manifest.json');assert m['dll_sha256'].lower()==dll
        for f in m['sources']+m['configs']:assert sha(f['Path'])==f['Hash'].lower(),f['Path']
        for c in j(folder/'results.json'):
            assert c['pass'],c['case']
            if not c['case'].startswith('lift_sensor_'):legacy.append({'case':c['case'],'csv':c['csv'],'dll_sha256':dll});continue
            r=rows(c['sensor_lift_csv']);assert r and all(len(s)==26 and None not in s for s in r)
            for s in r:
                cmd=float(s['command_spool']);integral=float(s['integral_spool']);assert math.isfinite(cmd) and -.220001<=cmd<=.650001 and abs(integral)<=.200001
                if s['height_valid']=='0':assert s['height_sensor_m']==''
                if s['fault_latched']=='1':assert cmd==0 and s['pump_enabled']=='0'
                if s['state']=='pressure_limited':assert cmd==0 and s['pump_enabled']=='0'
                assert 0<=float(s['target_m'])<=2.060001
            metrics={};name=c['case']
            if name in ['lift_sensor_empty','lift_sensor_loaded']:
                errors=[]
                for lo,hi,target in [(17,19,.9),(37,39.8,.25),(57,60,.6)]:
                    rr=[s for s in r if lo<float(s['sim_s'])<hi];assert rr and all(s['armed']=='1' and s['fault_latched']=='0' for s in rr)
                    err=max(abs(float(s['truth_height_m'])-target) for s in rr);assert err<.025,(name,lo,err);errors.append(err)
                metrics['settled_max_height_errors_m']=errors
            if 'fault_' in name or name.endswith('watchdog'):
                down=name.endswith('down');watch=name.endswith('watchdog');start=22 if down else 6;ack=32 if down else 14;newgoal=36 if down else 18
                fault=[s for s in r if start+.4<float(s['sim_s'])<ack-.1];assert fault and all(s['fault_latched']=='1' for s in fault)
                first=next(s for s in r if float(s['sim_s'])>=start+.4 and s['fault_latched']=='1');stopheight=float(first['truth_height_m']);drift=max(abs(float(s['truth_height_m'])-stopheight) for s in fault);assert drift<.03,(name,'closed valve drift',drift)
                before=next(s for s in r if float(s['sim_s'])>=start);totalstop=max(abs(float(s['truth_height_m'])-float(before['truth_height_m'])) for s in fault);assert totalstop<.09,(name,'movement after outage',totalstop)
                held=[s for s in r if ack+.3<float(s['sim_s'])<newgoal-.2];assert held and all(s['fault_latched']=='0' for s in held)
                recovered_target=statistics.mean(float(s['target_m']) for s in held);assert abs(recovered_target-float(held[0]['height_sensor_m']))<.02 and abs(recovered_target-.9)>.1
                assert all(abs(float(s['truth_height_m'])-recovered_target)<.025 for s in held)
                assert abs(float(r[-1]['truth_height_m'])-.6)<.025
                if watch:assert any(s['actuator_expired']=='1' for s in r)
                metrics.update({'fault_stop_drift_m':drift,'travel_after_outage_m':totalstop,'recovery_captured_target_m':recovered_target})
            if name.endswith('pressure'):
                inhibited=[s for s in r if 5<float(s['sim_s'])<19 and s['state']=='pressure_limited'];assert len(inhibited)>20
                assert any(float(s['command_spool'])<-.01 and float(s['pressure_sensor_pa'] or 0)>5.5e6 for s in r if float(s['sim_s'])>20)
                assert float(r[-1]['truth_height_m'])<.06
                metrics['pressure_inhibit_snapshots']=len(inhibited)
            cases.append({'case':name,'fps':c['fps'],'dll_sha256':dll,'controller_csv':c['sensor_lift_csv'],'csv_sha256':sha(c['sensor_lift_csv']),'metrics':metrics})
    assert len(cases)==7 and len(legacy)==2
    # Controller input plumbing must not copy diagnostic/plant state into feedback.
    source=(root/'Source/ForkliftSim/SensorLiftIntegration.cpp').read_text(encoding='utf-8');adapter=source.split('void AStabilityLab::UpdateSensorLiftControl')[1].split('// Plant values below')[0]
    assert '.Truth' not in adapter and 'ForkHeightM' not in adapter and 'HydraulicState' not in adapter and 'LoadMass' not in adapter
    report={'date':'2026-11-05','pass':True,'scope':'Sensor-only lift valve feedback and simulated interlocks. Manual vehicle driving; no pallet recognition, insertion decision, navigation or hardware safety certification.','sensor_control_cases':cases,'legacy_regression':legacy,'feedback_plumbing_no_truth_copy':True,'dll_sha256':dll}
    Path(a.output).parent.mkdir(parents=True,exist_ok=True);Path(a.output).write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8');print(json.dumps({'pass':True,'sensor_cases':len(cases),'legacy_cases':len(legacy)}))
if __name__=='__main__':main()
