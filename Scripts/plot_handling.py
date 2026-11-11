import argparse,csv,json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
def rows(p):
 with open(p,encoding='utf-8-sig') as f:return list(csv.DictReader(f))
def main():
 p=argparse.ArgumentParser();p.add_argument('report');p.add_argument('--output',required=True);a=p.parse_args();report=json.loads(Path(a.report).read_text(encoding='utf-8-sig'));by={c['case']:c for c in report['cases']}
 fig,axes=plt.subplots(2,2,figsize=(16,9),layout='constrained');fig.suptitle('Measured local handling — actual Unreal trials / 2026-11-06',fontsize=16)
 titles={'handling_pick':'Ground pickup','handling_pick_high':'Tier 2 pickup','handling_place_low':'Tier 1 unload','handling_place_high':'Tier 2 unload'}
 for ax,(name,title) in zip(axes.flat,titles.items()):
  c=by[name];r=rows(c['csv']);physical=rows(c['physics_csv']);t=[float(x['sim_s']) for x in r]
  ax.plot(t,[float(x['measured_height_m']) for x in r],color='#176a9a',label='Height sensor',lw=1.8);ax.plot(t,[float(x['target_height_m']) for x in r],color='#176a9a',ls=':',alpha=.65,label='Task height target')
  ax.plot([float(x['time_s']) for x in physical],[float(x['fork_actual_m']) for x in physical],color='#59798a',ls='--',lw=.7,label='Physical height (diagnostic)');ax.set_title(title);ax.set_xlabel('Simulation time (s)');ax.set_ylabel('Height (m)');ax.set_ylim(-.1,2.3);ax.grid(alpha=.18)
  right=ax.twinx();right.plot(t,[(float(x['left_load_n'])+float(x['right_load_n']))/1000 for x in r],color='#b96f2d',lw=1.2,label='Measured total fork load');right.plot([float(x['time_s']) for x in physical],[float(x['rack_support_n'])/1000 for x in physical],color='#54985c',lw=1,ls='--',label='Physical rack support (diagnostic)');right.set_ylabel('Load (kN)');right.set_ylim(-.2,5.8)
  old=None
  for x in r:
   state=x['state']
   if state==old:continue
   old=state
   if state in ['idle','acquire','align']:continue
   time=float(x['sim_s']);ax.axvline(time,color='#9aa6a3',lw=.6,alpha=.45);ax.text(time,2.22,state.replace('_',' '),rotation=80,ha='right',va='top',fontsize=7,color='#39484a')
  if name=='handling_pick':ax.legend(loc='upper left',bbox_to_anchor=(0,.69),fontsize=7);right.legend(loc='upper left',bbox_to_anchor=(0,.45),fontsize=7)
 fig.supxlabel('Known pallet family and surveyed station. Sensor-driven commands; physical truth is used only for independent validation. No FPS or hardware safety qualification.',fontsize=9)
 Path(a.output).parent.mkdir(parents=True,exist_ok=True);fig.savefig(a.output,dpi=160);print(a.output)
if __name__=='__main__':main()
