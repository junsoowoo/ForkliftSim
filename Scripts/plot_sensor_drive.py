import argparse,json,csv,math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('report');p.add_argument('output');a=p.parse_args()
r=json.loads(Path(a.report).read_text(encoding='utf-8-sig'))
def data(name):
 c=next(c for c in r['sensor_drive_conditions'] if c['case']==name)
 with open(c['csv'],encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))
def values(rows,k):return [float(s[k]) for s in rows]
slip=data('drive_sensor_slip');curve=data('drive_sensor_curve');fault=data('drive_sensor_encoder_fault')
fig,axes=plt.subplots(2,2,figsize=(12,7.5),layout='constrained');fig.suptitle('Sensor drive + surveyed-map localization | actual Unreal records',fontsize=15)
ax=axes[0,0]
for x,y,label in [('truth_x_m','truth_y_m','Physics reference'),('odom_x_m','odom_y_m','Encoder + IMU odometry'),('map_x_m','map_y_m','LiDAR map correction')]:ax.plot([float(s[x]) if x!='map_x_m' or s['localized']=='1' else math.nan for s in curve],[float(s[y]) if y!='map_y_m' or s['localized']=='1' else math.nan for s in curve],label=label,lw=1.6)
ax.set(title='Curve: estimated trajectory',xlabel='X (m)',ylabel='Y (m)',aspect='equal');ax.legend(fontsize=7,loc='lower left')
ax=axes[0,1];t=values(slip,'sim_s')
for prefix,label in [('odom','Raw odometry'),('map','Map-corrected')]:
 e=[math.hypot(float(s[prefix+'_x_m'])-float(s['truth_x_m']),float(s[prefix+'_y_m'])-float(s['truth_y_m'])) if prefix=='odom' or s['localized']=='1' else math.nan for s in slip];ax.plot(t,e,label=label)
ax.axvspan(8,10,color='orange',alpha=.15,label='Friction 0.01 + acceleration');ax.set(title='Slip: position error vs independent physics reference',xlabel='Simulation time (s)',ylabel='Position error (m)');ax.legend(fontsize=8)
ax=axes[1,0];t=values(fault,'sim_s');ax.plot(t,[math.hypot(float(s['truth_vx_mps']),float(s['truth_vy_mps'])) for s in fault],label='Physics speed');ax.plot(t,[math.hypot(float(s['vx_mps']),float(s['vy_mps'])) if s['state']=='ready' else math.nan for s in fault],label='Measured kinematic speed');ax.axvspan(6,8,color='red',alpha=.12,label='Encoder transmission outage');ax.axvline(14,color='green',ls='--',label='Neutral + Y acknowledgement');ax.set(title='Sensor loss: stop, latch and explicit recovery',xlabel='Simulation time (s)',ylabel='Speed (m/s)');ax.legend(fontsize=8)
ax=axes[1,1]
for k,label in [('fl_nm','FL'),('fr_nm','FR'),('rl_nm','RL'),('rr_nm','RR')]:ax.plot(values(slip,'sim_s'),values(slip,k),label=label,lw=1)
ax.set(title='Measured-feedback motor torque requests (slip fixture)',xlabel='Simulation time (s)',ylabel='Torque request (Nm)');ax.legend(ncol=4,fontsize=8)
for ax in axes.flat:ax.grid(alpha=.2)
Path(a.output).parent.mkdir(parents=True,exist_ok=True);fig.savefig(a.output,dpi=155);plt.close(fig)
print('SENSOR_DRIVE_PLOT',a.output)
