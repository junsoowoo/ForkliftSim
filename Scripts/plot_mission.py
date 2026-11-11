"""Plot recorded Unreal mission measurements; never synthesize physics values."""
import argparse,csv,json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

def rows(path):
 with open(path,encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))
def xy(r,x,y):return [float(v[x]) for v in r],[float(v[y]) for v in r]
def main():
 p=argparse.ArgumentParser();p.add_argument('report');p.add_argument('output');a=p.parse_args();report=json.loads(Path(a.report).read_text(encoding='utf-8-sig'));cases={x['case']:x for x in report['cases']};fig,axes=plt.subplots(2,2,figsize=(12,8),constrained_layout=True)
 for ax,key,title in [(axes[0,0],'mission_low','Tier 1 complete mission'),(axes[0,1],'mission_avoid_pause','New obstacle + explicit pause/resume')]:
  c=cases[key];m=[r for r in rows(c['csv']) if r['localized']=='1'];physical=rows(c['physics_csv']);ax.plot(*xy(m,'map_x_m','map_y_m'),color='#007e87',lw=2,label='Sensor map estimate');ax.plot(*xy(physical,'chassis_x_m','chassis_y_m'),'--',color='#df973e',lw=1,label='Independent physical truth');ax.scatter([2,3.24],[0,-2.5],color='#343d45',s=25);ax.annotate('PICK',(2,0),xytext=(5,8),textcoords='offset points');ax.annotate('DROP',(3.24,-2.5),xytext=(5,8),textcoords='offset points')
  if key.endswith('pause'):ax.add_patch(Rectangle((5.4,-.8),.4,.4,fc='#b34e48',label='Injected obstacle'))
  ax.set(title=title,xlabel='Map X [m]',ylabel='Map Y [m]');ax.set_aspect('equal',adjustable='datalim');ax.grid(alpha=.2);ax.legend(fontsize=8)
 high=[r for r in rows(cases['mission_high']['csv']) if r['healthy']=='1'];ax=axes[1,0];ax.plot(*xy(high,'sim_s','height_m'),color='#007e87',label='Measured height');ax.set(title='Tier 2: measured carriage height',xlabel='Simulation time [s]',ylabel='Height [m]');ax.grid(alpha=.2)
 ax=axes[1,1];ax.plot(*xy(high,'sim_s','left_load_n'),label='Left measured fork load',color='#007e87');ax.plot(*xy(high,'sim_s','right_load_n'),label='Right measured fork load',color='#b37d36');ax.set(title='Tier 2: pickup, transport and transfer',xlabel='Simulation time [s]',ylabel='Measured force [N]');ax.legend(fontsize=8);ax.grid(alpha=.2)
 fig.suptitle('ForkliftSim batch 5 | recorded Unreal data | known-map, fixed-heading tests',fontsize=13);fig.savefig(a.output,dpi=160);plt.close(fig)
if __name__=='__main__':main()
