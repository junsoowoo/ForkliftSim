"""Actual controller recordings, with plant truth explicitly diagnostic."""
import argparse,csv
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
def read(p):
    with Path(p).open(encoding='utf-8-sig') as f:return list(csv.DictReader(f))
def main():
    p=argparse.ArgumentParser();p.add_argument('loaded_csv');p.add_argument('fault_csv');p.add_argument('--output',required=True);a=p.parse_args()
    fig,ax=plt.subplots(2,2,figsize=(12,6.5),sharex='col',constrained_layout=True)
    for col,(path,title) in enumerate([(a.loaded_csv,'425 kg: sensor height control'),(a.fault_csv,'Height sensor outage / explicit acknowledgement')]):
        r=read(path);t=[float(s['sim_s']) for s in r]
        values=lambda key:[float(s[key]) if s[key] else float('nan') for s in r]
        ax[0,col].plot(t,values('target_m'),label='Controller target',color='#bd7620',ls='--',lw=1.5)
        ax[0,col].plot(t,values('truth_height_m'),label='Plant height (diagnostic)',color='#9a9a9a',lw=2.8)
        ax[0,col].plot(t,values('height_sensor_m'),label='Delivered height sensor',color='#007f86',lw=1)
        ax[1,col].plot(t,values('command_spool'),label='Controller command',color='#007f86',lw=1.2)
        ax[1,col].plot(t,values('truth_valve'),label='Physical valve (diagnostic)',color='#a95443',lw=.8,alpha=.8)
        if col==1:
            for row in range(2):
                ax[row,col].axvspan(6,8,color='#c14343',alpha=.15,label='Transport outage' if row==0 else None)
                ax[row,col].axvline(14,color='#606080',ls=':',label='Acknowledge / rebase' if row==0 else None)
        ax[0,col].set_title(title);ax[1,col].set_xlabel('Simulation time (s)')
        for row in range(2):ax[row,col].grid(alpha=.2);ax[row,col].legend(fontsize=8,loc='best')
    ax[0,0].set_ylabel('Height (m)');ax[1,0].set_ylabel('Spool command / response');fig.suptitle('ForkliftSim | measured feedback; no plant-height feedback in controller',fontsize=13)
    Path(a.output).parent.mkdir(parents=True,exist_ok=True);fig.savefig(a.output,dpi=160);plt.close(fig)
if __name__=='__main__':main()
