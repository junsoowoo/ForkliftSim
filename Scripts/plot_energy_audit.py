"""Plot actual Unreal energy-audit CSVs; never generates simulation samples."""
import argparse
import csv
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def read(path):
    with Path(path).open(encoding='utf-8-sig',newline='') as file:
        return [{k:float(v) for k,v in row.items() if k in {'time_s','fork_actual_m','energy_stored_delta_j','energy_known_input_j','energy_known_loss_j','energy_unresolved_j'}}
                for row in csv.DictReader(file) if row.get('energy_reference_valid')=='1']


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ('carry480','carry960','lift60','lift30'): parser.add_argument('--'+key,required=True)
    parser.add_argument('--output',required=True); args=parser.parse_args()
    datasets={key:read(getattr(args,key)) for key in ('carry480','carry960','lift60','lift30')}
    plt.rcParams.update({'font.family':'Malgun Gothic','axes.unicode_minus':False,'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
    fig,axes=plt.subplots(2,2,figsize=(13,8),layout='constrained')
    colors={'energy_known_input_j':'#21676c','energy_stored_delta_j':'#b37b21','energy_known_loss_j':'#668855'}
    names={'energy_known_input_j':'관측 입력','energy_stored_delta_j':'저장 변화','energy_known_loss_j':'모델 손실'}
    for key,style in [('carry480','-'),('carry960','--')]:
        rows=datasets[key]; t=[r['time_s'] for r in rows]; hz=key.removeprefix('carry')
        for field,color in colors.items(): axes[0,0].plot(t,[r[field]/1000 for r in rows],style,color=color,label=names[field]+' '+hz+'Hz',linewidth=1.5)
        axes[0,1].plot(t,[r['energy_unresolved_j'] for r in rows],style,label=hz+'Hz')
    axes[0,0].set(title='적재 운반: 같은 물성, 다른 계산 간격',ylabel='에너지 [kJ]'); axes[0,0].legend(ncols=2,fontsize=8)
    axes[0,1].set(title='운반 잔차: 미계상 손실 + 수치 오차',ylabel='잔차 [J]'); axes[0,1].axhline(0,color='#aaa',lw=.7); axes[0,1].legend()
    for key,style in [('lift60','-'),('lift30','--')]:
        rows=datasets[key]; t=[r['time_s'] for r in rows]; label=key.removeprefix('lift')+' FPS / 480Hz'
        axes[1,0].plot(t,[r['fork_actual_m'] for r in rows],style,label=label)
        axes[1,1].plot(t,[r['energy_unresolved_j'] for r in rows],style,label=label)
    axes[1,0].set(title='양중: 실제 포크 높이',ylabel='높이 [m]'); axes[1,0].legend()
    axes[1,1].set(title='양중 잔차: 0으로 보정하지 않은 측정',ylabel='잔차 [J]'); axes[1,1].legend()
    for ax in axes.flat: ax.set_xlabel('실제 시뮬레이션 시간 [s]'); ax.grid(alpha=.15)
    fig.suptitle('Unreal 실제 계측 — 에너지 관측·간격 비교\n전체 에너지 폐합 미완료 · 수치 수렴/실차 보정과 구분',fontsize=15,fontweight='bold')
    Path(args.output).parent.mkdir(parents=True,exist_ok=True); fig.savefig(args.output,dpi=150); plt.close(fig)
    print('ACTUAL_ENERGY_PLOT '+args.output)
