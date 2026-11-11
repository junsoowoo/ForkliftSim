import argparse,json,csv,math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
p=argparse.ArgumentParser();p.add_argument('report');p.add_argument('output');a=p.parse_args();report=json.loads(Path(a.report).read_text(encoding='utf-8-sig'))
def rows(p):return list(csv.DictReader(open(p,encoding='utf-8-sig')))
aligned=next(c for c in report['gpu_conditions'] if c['case']=='pallet_vision_aligned' and not c['proximity_fault']);insert=next(c for c in report['gpu_conditions'] if c['case']=='pallet_vision_insert')
meta=json.loads(Path(aligned['frame']).read_text(encoding='utf-8-sig'));w,h=meta['width'],meta['height'];depth=np.fromfile(aligned['frame']+'.depth.bin',dtype='<f4').reshape(h,w);valid=np.fromfile(aligned['frame']+'.validity.bin',dtype='u1').reshape(h,w)
obs=next(s for s in rows(aligned['csv']) if s['vision_valid']=='1' and int(s['sequence'])==meta['sequence'])
fig=plt.figure(figsize=(13,8),layout='constrained');gs=fig.add_gridspec(2,2,width_ratios=[1,1.35]);ax=fig.add_subplot(gs[0,0]);image=ax.imshow(np.where(valid==0,depth,np.nan),vmin=.3,vmax=3.5,cmap='viridis');fig.colorbar(image,ax=ax,label='Measured optical depth (m)',fraction=.035)
fx=w/(2*np.tan(np.deg2rad(meta['fov_deg'])/2));pitch=np.deg2rad(meta['pitch_deg']);yaw=float(obs['yaw_rad']);n=np.array([np.cos(yaw),np.sin(yaw),0.]);t=np.array([-n[1],n[0],0.]);d=float(obs['plane_distance_m']);cy=float(obs['center_y_m']);cz=float(obs['center_z_m'])
def project(q):
 ox=np.cos(pitch)*q[0]+np.sin(pitch)*q[2];oz=-np.sin(pitch)*q[0]+np.cos(pitch)*q[2];return [(w-1)/2+fx*q[1]/ox,(h-1)/2-fx*oz/ox]
for side,key in [(-1,'left_width_m'),(1,'right_width_m')]:
 center=cy+side*.225;hw=float(obs[key])*.5;hh=.093*.5;corners=[project(n*d+t*u+np.array([0,0,z])) for u,z in [(center-hw,cz-hh),(center+hw,cz-hh),(center+hw,cz+hh),(center-hw,cz+hh)]];ax.add_patch(Polygon(corners,fill=False,edgecolor='lime',lw=1.4))
ax.set(title='Actual GPU depth + fitted known-family openings\n(height/spacing are dimensional priors)',xlabel='Pixel X',ylabel='Pixel Y')
data=rows(insert['csv']);physical=rows(insert['physics_csv']);time=np.array([float(s['sim_s']) for s in data]);truth_time=np.array([float(s['time_s']) for s in physical]);actual=np.array([min(float(s['left_depth_m']),float(s['right_depth_m'])) for s in physical]);estimate=np.array([min(float(s['depth_left_m']),float(s['depth_right_m'])) if s['state'] in ['partial','fully_inserted'] else np.nan for s in data]);unc=np.array([float(s['uncertainty_m']) for s in data]);full=np.array([s['state']=='fully_inserted' for s in data])
ax=fig.add_subplot(gs[0,1]);ax.plot(truth_time,actual,label='Independent physics overlap',color='#364c59');ax.plot(time,estimate,label='Sensor-derived depth',color='#137d8d');ax.fill_between(time,estimate-unc,estimate+unc,alpha=.18,color='#137d8d',label='Decision margin (assumed bound)');ax.axhline(1.02,color='#b56925',ls='--',label='Full threshold after margin');ax.scatter(time[full],estimate[full],s=8,color='#20814c',label='Full evidence confirmed');ax.set(title='Timed manual input: insertion depth and margin',xlabel='Simulation time (s)',ylabel='Depth / physical overlap (m)');ax.grid(alpha=.2);ax.legend(fontsize=7)
ax=fig.add_subplot(gs[1,:]);names=['unknown','observing','aligned','misaligned','partial','fully_inserted','supported','uneven_support'];index={s:i for i,s in enumerate(names)};ax.step(time,[index[s['state']] for s in data],where='post',color='#186e75');ax.set_yticks(range(len(names)),names);ax.set_ylim(-.4,len(names)-.6);ax.set(xlabel='Simulation time (s)',title='Inference states from actual measured depth, presence and range data');ax.grid(alpha=.2)
fig.suptitle('Pallet perception + insertion evidence | actual Unreal records',fontsize=15);Path(a.output).parent.mkdir(parents=True,exist_ok=True);fig.savefig(a.output,dpi=150);plt.close(fig);print('PALLET_PERCEPTION_PLOT',a.output)
