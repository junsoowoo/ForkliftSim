"""Actual CSV figure; failed conservation stays red and visible."""
import csv,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parents[1]
r=json.loads((root/'Docs/References/MecanumBatch1234-20261028.json').read_text(encoding='utf-8'))
checks=json.loads((root/'Saved/Validation/Mecanum-Batch1234-Checks.txt').read_text(encoding='utf-8-sig'))
def load(case):
 p=root/next(c['csv'] for c in r['cases'] if c['case']==case)
 return list(csv.DictReader(p.open(encoding='utf-8-sig')))
def a(rows,k):return np.array([float(x[k]) for x in rows])
fig,axs=plt.subplots(2,2,figsize=(12,8),layout='constrained');fig.suptitle('ForkliftSim 0.17 — actual Unreal physics verification',fontsize=16)
for case,label,color in [('grip_normal','mu = 0.80','#168c91'),('grip_low','mu = 0.05','#e19732')]:
 d=load(case);t=a(d,'time_s');slip=np.sqrt(sum(a(d,w+'_long_slip_mps')**2+a(d,w+'_lateral_slip_mps')**2 for w in ['FL','FR','RL','RR'])/4)
 axs[0,0].plot(t,slip,label=label,color=color,lw=1.5)
axs[0,0].set(xlim=(5.5,8),ylabel='Wheel-average contact slip RMS (m/s)',xlabel='Simulation time (s)',title='Same reversal command; only road friction changes');axs[0,0].legend()
d=load('ramp_brake');t=a(d,'time_s');x=a(d,'chassis_x_m');brake=a(d,'brake');start=np.flatnonzero((t>14)&(brake>.5))[0]
axs[0,1].plot(t,x,color='#168c91');axs[0,1].axvline(t[start],ls='--',color='#555',label='Brake applied');axs[0,1].set(xlim=(t[start]-1,60),ylim=(7.6,8.3),xlabel='Simulation time (s)',ylabel='Chassis world X (m)',title='Loaded 6 degree uphill braking and hold');axs[0,1].legend()
case=next(c for c in r['cases'] if c['case']=='ramp_brake');axs[0,1].text(.98,.05,f"Max X excursion: {case['metrics']['stop_distance_m']:.3f} m\nHold drift: {case['metrics']['hold_drift_m']*1000:.2f} mm",transform=axs[0,1].transAxes,ha='right')
for i,(case,label,color) in enumerate([('lift','Centered / 0.90 m','#168c91'),('eccentric_lift','Cargo Y +0.12 m','#e19732'),('shallow_lift','Insertion 0.65 m','#6979a5')]):
 d=load(case);d=[x for x in d if 10<=float(x['time_s'])<13]
 vals=[np.mean(a(d,w+'_normal_n'))/1000 for w in ['FL','FR','RL','RR']]
 axs[1,0].bar(np.arange(4)+(i-1)*.25,vals,width=.24,label=label,color=color)
axs[1,0].set(xticks=np.arange(4),xticklabels=['FL','FR','RL','RR'],ylabel='Mean wheel normal load (kN)',title='Loaded static equilibrium; actual contact forces');axs[1,0].legend(fontsize=8)
error=r['reaction_detail']['delta_l_nms'];axs[1,1].bar(['Observed max error'],[error],color='#c75a52',width=.4);axs[1,1].axhline(1,color='#333',ls='--',label='Required: < 1 N m s (2%)');axs[1,1].set(xlim=(-.7,.7),ylim=(0,52),ylabel='Total angular-momentum error (N m s)',title='Free-floating coupled wheel — FAIL');axs[1,1].legend();axs[1,1].text(0,error+.8,f'{error:.2f}',ha='center');axs[1,1].text(.5,.08,'Hub +50 / chassis -50 N m for 1 s\nConservation is NOT qualified',transform=axs[1,1].transAxes,ha='center',color='#7a2824',bbox=dict(facecolor='white',alpha=.94,edgecolor='none',pad=4))
for ax in axs.flat:ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
fig.savefig(root/'Docs/MecanumBatch1234Validation.png',dpi=160)
print('ACTUAL_FIGURE_SAVED')
