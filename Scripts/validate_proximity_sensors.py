"""Independent validation of actual scene-query sensor recordings."""
import argparse,bisect,csv,hashlib,json,math
from collections import Counter
from pathlib import Path
def j(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def rows(p):
    with Path(p).open(encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))
def main():
    parser=argparse.ArgumentParser();parser.add_argument('reports',nargs='+');parser.add_argument('--output',required=True);args=parser.parse_args()
    root=Path(__file__).resolve().parents[1];spec=j(root/'Config/ProximitySensors.json');dll=sha(root/'Binaries/Win64/UnrealEditor-ForkliftSim.dll');out=[];bycase={};scene=False
    ids=['heel_left','heel_right','tip_left','tip_right','presence_left_near','presence_right_near','presence_left_far','presence_right_far']
    for folder in map(Path,args.reports):
        manifest=j(folder/'manifest.json');assert manifest['dll_sha256'].lower()==dll
        for file in manifest['sources']+manifest['configs']:assert sha(file['Path'])==file['Hash'].lower(),file['Path']
        for case in j(folder/'results.json'):
            assert case['pass'],case['case']
            if manifest.get('proximity_scene_validation'):
                log=Path(case['log']).read_text(encoding='utf-8-sig');assert 'PROXIMITY_SCENE_VALIDATION checks=7 PASS' in log;scene=True
            rr=rows(case['proximity_sensor_csv']);assert rr and all(len(r)==22 and None not in r for r in rr)
            channels={i:[r for r in rr if r['id']==i] for i in ids};metrics={}
            for i,cr in channels.items():
                assert cr;fresh=[r for r in cr if r['transport_valid']=='1'];assert len(fresh)>10
                unique=[];previous=-1;maxerr=0
                for r in cr:
                    transport=r['transport_valid']=='1';valid=r['measurement_valid']=='1';assert not valid or transport
                    if not valid:assert r['range_m']==r['detected']=='','Invalid measurement must be blank'
                    if not transport:assert r['reading_status']=='unavailable'
                    if not transport:continue
                    t,delivery,now,age=map(float,[r['capture_sim_s'],r['delivery_sim_s'],r['snapshot_sim_s'],r['age_s']]);assert delivery-t>=spec['latency_s']-1e-8 and 0<=age<=spec['stale_s']+1e-8 and abs(now-t-age)<2e-8
                    seq=int(r['sequence']);assert seq>=previous
                    if seq!=previous:unique.append(r)
                    previous=seq
                    if valid and i.startswith(('heel','tip')):
                        v,truth=float(r['range_m']),float(r['truth_range_m']);assert r['truth_hit']=='1' and spec['min_range_m']<=v<=spec['max_range_m'];err=abs(v-truth-spec['bias_m']);maxerr=max(maxerr,err);assert err<=6*spec['sigma_m']+spec['quantum_m']*.5+1e-8
                        assert abs(v/spec['quantum_m']-round(v/spec['quantum_m']))<1e-6
                    if valid and i.startswith('presence'):assert r['detected'] in ('0','1') and r['range_m']==''
                    direction=[float(r['direction_'+a]) for a in ['x','y','z']];assert abs(sum(v*v for v in direction)-1)<1e-7
                assert len(unique)>10
                rate=(int(unique[-1]['sequence'])-int(unique[0]['sequence']))/(float(unique[-1]['capture_sim_s'])-float(unique[0]['capture_sim_s']));assert abs(rate-spec['hz'])/spec['hz']<.02,(i,rate)
                metrics[i]={'capture_hz':rate,'max_range_bias_removed_error_m':maxerr,'reading_states':dict(Counter(r['reading_status'] for r in cr)),'on':sum(r['detected']=='1' for r in cr),'off':sum(r['detected']=='0' for r in cr)}
                if manifest.get('proximity_sensor_fault')=='all':
                    stale=[r for r in cr if 6.4<float(r['snapshot_sim_s'])<8];assert len(stale)>5 and all(r['status']=='stale' for r in stale)
                    restored=[r for r in cr if 8.3<float(r['snapshot_sim_s'])<8.7];assert restored and all(r['transport_valid']=='1' for r in restored)
                    assert int(cr[-1]['dropped'])>=38
            key=(case['case'],manifest.get('proximity_sensor_fault',''),case['fps']);bycase[key]=(channels,case)
            out.append({'case':case['case'],'fps':case['fps'],'fault':key[1],'dll_sha256':dll,'csv':case['proximity_sensor_csv'],'csv_sha256':sha(case['proximity_sensor_csv']),'metrics':metrics})
    assert scene
    racks=[]
    for tier in ['low','high']:
        ch,c=bycase[(f'rack_pick_{tier}','',60)];pr=rows(c['csv']);times=[float(p['time_s']) for p in pr];mount_errors=[];maxz=0;high_on=0;on=off=0
        for r in ch['presence_left_near']:
            if r['transport_valid']!='1':continue
            t=float(r['capture_sim_s']);idx=min(max(1,bisect.bisect_left(times,t)),len(times)-1);p=min(pr[idx-1:idx+1],key=lambda p:abs(float(p['time_s'])-t));z=float(r['origin_z_m']);maxz=max(maxz,z);mount_errors.append(z-float(p['fork_actual_m']))
            on+=r['detected']=='1';off+=r['detected']=='0';high_on+=r['detected']=='1' and z>1.95
        # Pose follows solved fork geometry rather than a fixed floor-mounted ray.
        assert max(mount_errors)-min(mount_errors)<.05
        assert on>5 and off>5,(tier,'presence does not change with real insertion')
        if tier=='high':assert maxz>2 and high_on>5,(maxz,high_on)
        racks.append({'tier':tier,'max_sensor_world_z_m':maxz,'mount_minus_lift_span_m':max(mount_errors)-min(mount_errors),'on':on,'off':off,'above_upper_tier_on':high_on})
    a=rows(bycase[('carry','',60)][1]['csv']);b=rows(bycase[('carry','all',60)][1]['csv']);assert len(a)==len(b)
    fields=['time_s','chassis_x_m','chassis_y_m','speed_mps','lateral_mps','yaw_deg_s','fork_actual_m','normal_sum_n','energy_unresolved_j'];parity={f:max(abs(float(x[f])-float(y[f])) for x,y in zip(a,b)) for f in fields};assert max(parity.values())==0
    report={'date':'2026-11-05','pass':True,'scope':'Virtual single-ray range and binary surface presence; no semantic pallet recognition, full insertion claim or automatic control.','native_cases':out,'scene_geometry_checks':7,'rack_observations':racks,'fault_physics_max_differences':parity}
    Path(args.output).parent.mkdir(parents=True,exist_ok=True);Path(args.output).write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8');print(json.dumps({'pass':True,'cases':len(out),'rack_observations':racks}))
if __name__=='__main__':main()
