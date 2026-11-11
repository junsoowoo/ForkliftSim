"""Plot actual Unreal CSVs; never synthesize physics or replace live telemetry."""
import argparse,csv
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

parser=argparse.ArgumentParser()
for key in ('straight','strafe','diagonal','lift','ramp'):parser.add_argument('--'+key,required=True)
parser.add_argument('--output',required=True);args=parser.parse_args()
def read(path):
    with Path(path).open(encoding='utf-8-sig',newline='') as f:
        rows=[{k:float(v) for k,v in r.items() if v and k in ('time_s','speed_mps','lateral_mps','target_vx_mps','target_vy_mps','brake','chassis_x_m','FL_normal_n','FR_normal_n','RL_normal_n','RR_normal_n','fork_actual_m')} for r in csv.DictReader(f)]
    if len(rows)<20:raise ValueError('Actual native samples required')
    return rows
data={k:read(getattr(args,k)) for k in ('straight','strafe','diagonal','lift','ramp')}
plt.rcParams.update({'font.family':'Malgun Gothic','axes.unicode_minus':False,'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
fig,axs=plt.subplots(2,2,figsize=(13,8),layout='constrained')
for key,field,name in [('straight','speed_mps','직진'),('strafe','lateral_mps','횡이동')]:
    r=data[key];axs[0,0].plot([v['time_s'] for v in r],[v[field] for v in r],label=name)
axs[0,0].axhline(1,color='#999',ls='--',lw=1,label='명령 상한 1m/s');axs[0,0].set(title='메카넘 직진·횡이동의 실제 속도',ylabel='m/s');axs[0,0].legend()
r=data['diagonal'];axs[0,1].plot([v['time_s'] for v in r],[v['speed_mps'] for v in r],label='전방');axs[0,1].plot([v['time_s'] for v in r],[v['lateral_mps'] for v in r],label='오른쪽');axs[0,1].set(title='대각선 주행: 실제 두 속도 성분',ylabel='m/s');axs[0,1].legend()
r=data['lift'];t=[v['time_s'] for v in r]
for name in ('FL','FR','RL','RR'):axs[1,0].plot(t,[v[name+'_normal_n']/1000 for v in r],label=name,lw=1)
axs[1,0].set(title='425kg 적재 양중: 바퀴별 실제 하중',ylabel='kN');axs[1,0].legend(ncols=4)
r=data['ramp'];t=[v['time_s'] for v in r];axs[1,1].plot(t,[v['chassis_x_m'] for v in r],color='#24776a',label='실제 X 위치');ax2=axs[1,1].twinx();ax2.plot(t,[v['speed_mps'] for v in r],color='#b98530',alpha=.8,label='실제 전후 속도');ax2.set_ylabel('m/s');brake=[v['time_s'] for v in r if v['brake']>0 and v['time_s']>15]
if brake:axs[1,1].axvline(brake[0],color='#999',ls='--',lw=1,label='브레이크 입력')
axs[1,1].set(title='적재 경사 제동: 위치·속도 원본',ylabel='위치 m');axs[1,1].legend(loc='lower left');ax2.legend(loc='upper right')
for ax in axs.flat:ax.set_xlabel('시뮬레이션 시간 s');ax.grid(alpha=.15)
fig.suptitle('Unreal 실제 메카넘 물리 계측\n시험용 가상 장비 · 실차 물성·전체 에너지 폐합 미보정',fontsize=15,fontweight='bold')
Path(args.output).parent.mkdir(parents=True,exist_ok=True);fig.savefig(args.output,dpi=150);plt.close(fig);print('ACTUAL_MECANUM_PLOT '+args.output)
