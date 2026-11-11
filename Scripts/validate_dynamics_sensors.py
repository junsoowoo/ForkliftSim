"""Validate received native sensor snapshots against separately named diagnostics."""
import argparse, bisect, csv, hashlib, json, math, statistics
from pathlib import Path

def read_json(p): return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def digest(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
    parser=argparse.ArgumentParser();parser.add_argument('reports',nargs='+');parser.add_argument('--output',required=True);args=parser.parse_args()
    spec=read_json(Path(__file__).resolve().parents[1]/'Config/DynamicsSensors.json');results=[];bycase={}
    ids=['imu','lift_pressure','fork_load_left','fork_load_right']
    for folder in map(Path,args.reports):
        manifest=read_json(folder/'manifest.json')
        for case in read_json(folder/'results.json'):
            assert case['pass'],(folder,case['case'],'native failed')
            path=Path(case['dynamics_sensor_csv']);rows=list(csv.DictReader(path.open(encoding='utf-8-sig',newline='')))
            assert rows and all(len(r)==23 and None not in r for r in rows)
            channels={i:[r for r in rows if r['id']==i] for i in ids};metrics={};clipped=0
            for i,rr in channels.items():
                valid=[r for r in rr if r['valid']=='1'];assert len(valid)>10,(i,'no samples')
                for r in rr:
                    if r['valid']!='1':assert all(r[f'value_{a}']=='' for a in range(6)),'invalid data not blank'
                last_seq=-1;unique=[];maxerr=0
                for r in valid:
                    capture,delivery,now,age=map(float,[r['capture_sim_s'],r['delivery_sim_s'],r['snapshot_sim_s'],r['age_s']])
                    assert delivery-capture>=spec['latency_s']-1e-8
                    assert 0<=age<=spec['stale_s']+1e-8 and abs(now-capture-age)<2e-8
                    seq=int(r['sequence']);assert seq>=last_seq
                    if seq!=last_seq:unique.append(r)
                    last_seq=seq;mask=int(r['clipped_mask']);clipped+=bool(mask)
                    width=6 if i=='imu' else 1
                    for a in range(width):
                        v,t=map(float,[r[f'value_{a}'],r[f'truth_{a}']]);assert math.isfinite(v) and math.isfinite(t)
                        prefix='acc' if i=='imu' and a<3 else 'gyro' if i=='imu' else 'pressure' if i=='lift_pressure' else 'load'
                        unit={'acc':'mps2','gyro':'rad_s','pressure':'pa','load':'n'}[prefix]
                        sigma,bias,quantum,limit=[spec[f'{prefix}_{k}_{unit}'] for k in ['sigma','bias','quantum','range']]
                        assert (-limit if i!='lift_pressure' else 0)-1e-8<=v<=limit+1e-8
                        if not(mask&(1<<a)):
                            err=abs(v-t-bias);maxerr=max(maxerr,err)
                            assert err<=6*sigma+quantum*.5+1e-7,(i,a,err,'noise envelope')
                            assert abs(v/quantum-round(v/quantum))<1e-5,(i,'quantization')
                first,last=unique[0],unique[-1];rate=(int(last['sequence'])-int(first['sequence']))/(float(last['capture_sim_s'])-float(first['capture_sim_s']))
                assert abs(rate-spec['hz'])/spec['hz']<.005,(i,rate)
                metrics[i]={'valid_snapshots':len(valid),'observed_capture_hz':rate,'max_unclipped_bias_removed_error':maxerr}
            if manifest.get('dynamics_sensor_fault')=='all':
                for i,rr in channels.items():
                    stale=[r for r in rr if 6.4<float(r['snapshot_sim_s'])<8];assert len(stale)>5 and all(r['status']=='stale' for r in stale)
                    recovered=[r for r in rr if 8.2<float(r['snapshot_sim_s'])<8.6];assert recovered and all(r['valid']=='1' for r in recovered)
                    assert int(rr[-1]['dropped'])>=190
            bycase[(case['case'],manifest.get('dynamics_sensor_fault',''),case['fps'],case['physics_hz'])]=(channels,case)
            results.append({'case':case['case'],'fps':case['fps'],'physics_hz':case['physics_hz'],'fault':manifest.get('dynamics_sensor_fault',''),'csv':str(path),'csv_sha256':digest(path),'dll_sha256':manifest['dll_sha256'],'metrics':metrics,'clipped_snapshot_count':clipped})
    stationary=bycase[('settle','',60,960)][0]
    imu=[r for r in stationary['imu'] if r['valid']=='1' and float(r['capture_sim_s'])>3]
    means=[statistics.mean(float(r[f'value_{a}']) for r in imu) for a in range(6)]
    assert abs(means[0])<.05 and abs(means[1])<.05 and abs(means[2]-9.81)<.05 and max(abs(v) for v in means[3:])<.005,means
    carry=bycase[('carry','',60,960)][0]
    def mean(rr,a=0):
        good=[r for r in rr if r['valid']=='1' and 12<float(r['capture_sim_s'])<13.5];assert good
        return statistics.mean(float(r[f'value_{a}']) for r in good)
    left,right=mean(carry['fork_load_left']),mean(carry['fork_load_right']);assert abs(left+right-425*9.81)/(425*9.81)<.05,(left,right)
    loaded_pressure=mean(carry['lift_pressure']);empty_pressure=statistics.mean(float(r['value_0']) for r in stationary['lift_pressure'] if r['valid']=='1' and float(r['capture_sim_s'])>3)
    assert loaded_pressure>empty_pressure+1e5,(loaded_pressure,empty_pressure)
    eccentric=bycase[('eccentric_lift','',60,960)][0];ecc_left,ecc_right=mean(eccentric['fork_load_left']),mean(eccentric['fork_load_right']);assert abs(ecc_left-ecc_right)>100,(ecc_left,ecc_right)
    nominal_case=bycase[('carry','',60,960)][1];fault_case=bycase[('carry','all',60,960)][1]
    def physics(p):return list(csv.DictReader(Path(p).open(encoding='utf-8-sig')))
    a,b=physics(nominal_case['csv']),physics(fault_case['csv']);assert len(a)==len(b)
    fields=['time_s','chassis_x_m','chassis_y_m','speed_mps','lateral_mps','yaw_deg_s','fork_actual_m','normal_sum_n','energy_unresolved_j']
    # Use actual stable column names, reject missing comparison fields.
    for field in fields:assert field in a[0],field
    parity={field:max(abs(float(x[field])-float(y[field])) for x,y in zip(a,b)) for field in fields};assert max(parity.values())==0,parity
    grids=[]
    for fps,hz in [(30,960),(60,1920)]:
        c=bycase[('carry','',fps,hz)][0];forces=[mean(c['fork_load_left']),mean(c['fork_load_right'])];pressure=mean(c['lift_pressure'])
        assert abs(sum(forces)/(left+right)-1)<.03 and abs(pressure/loaded_pressure-1)<.03
        grids.append({'fps':fps,'hz':hz,'fork_forces_n':forces,'pressure_pa':pressure})
    spin_channels,spin_case=bycase[('joint_sensors','',60,960)];pp=physics(spin_case['csv']);times=[float(p['time_s']) for p in pp];errors=[];peak=0
    for r in spin_channels['imu']:
        if r['valid']!='1' or not 64.2<float(r['capture_sim_s'])<65.8:continue
        t=float(r['capture_sim_s']);idx=bisect.bisect_left(times,t);idx=min(max(1,idx),len(times)-1)
        p=min(pp[idx-1:idx+1],key=lambda p:abs(float(p['time_s'])-t));measured=float(r['value_5'])
        errors.append(abs(measured-math.radians(float(p['yaw_deg_s']))));peak=max(peak,abs(measured))
    assert len(errors)>10 and peak>.08 and max(errors)<.05,(len(errors),peak,max(errors,default=-1))
    report={'date':'2026-11-05','pass':True,'scope':'Virtual IMU, cylinder pressure and ideal fork contact instrument pads; not calibrated hardware, payload scale, safety interlocks or automation.','native_cases':results,'stationary_imu_means':means,'loaded_fork_forces_n':[left,right],'eccentric_fork_forces_n':[ecc_left,ecc_right],'pressure_empty_loaded_pa':[empty_pressure,loaded_pressure],'fault_physics_max_difference':parity,'static_time_grid_comparisons':grids,'spin_gyro_max_nearest_physics_error_rad_s':max(errors),'spin_gyro_peak_rad_s':peak}
    Path(args.output).parent.mkdir(parents=True,exist_ok=True);Path(args.output).write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8');print(json.dumps({'pass':True,'cases':len(results),'stationary_imu':means,'loaded_forks_n':[left,right]},ensure_ascii=False))
if __name__=='__main__':main()
