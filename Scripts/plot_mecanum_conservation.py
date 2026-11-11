"""Plot recorded Unreal conservation checks; no computed/demo vehicle motion."""
import csv
import hashlib
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parents[1]
report = json.loads((root/'Docs/References/MecanumConservation-20261029.json').read_text(encoding='utf-8'))
cases = {item['case']: item for item in report['cases']}
old_path = root/'Saved/Telemetry/Stability-mecanum_coast-20261029-004611-D7D25EB0.csv'
new_path = root/cases['coast']['csv']

def coast(path):
    rows = list(csv.DictReader(path.open(encoding='utf-8-sig')))
    rows = [row for row in rows if float(row['time_s']) >= 5]
    origin = float(rows[0]['energy_stored_delta_j'])
    return {'time_s': [float(row['time_s'])-5 for row in rows],
            'speed_mps': [float(row['speed_mps']) for row in rows],
            'energy_delta_j': [float(row['energy_stored_delta_j'])-origin for row in rows]}

old, new = coast(old_path), coast(new_path)
bench = cases['reaction']['bench']
energy = cases['ramp_brake']['final_energy_observer']
fig, axes = plt.subplots(2,2,figsize=(12,8),layout='constrained')
red, green = '#c65353', '#168276'
ax = axes[0,0]
values = [45.86107548, bench['delta_l_nms']]
ax.bar(['Previous joint solve', 'Current 1920 Hz'], values, color=[red,green],width=.5)
ax.set_yscale('log'); ax.axhline(1,color='#a17418',linestyle='--',label='Unchanged limit: 1 N m s')
for x,y in enumerate(values): ax.text(x,y*1.2,f'{y:.3f}',ha='center')
ax.set_ylim(.01,100);ax.set_ylabel('Max angular momentum error [N m s]');ax.set_title('Free-floating motor / reaction pair');ax.legend(fontsize=8)
for label,data,color in [('480 Hz contact (rejected)',old,red),('1920 Hz contact (current)',new,green)]:
    axes[0,1].plot(data['time_s'],data['speed_mps'],label=label,color=color)
    axes[1,0].plot(data['time_s'],data['energy_delta_j'],label=label,color=color)
axes[0,1].set_title('Neutral coast: motor OFF, rolling resistance ON')
axes[0,1].set_ylabel('Chassis speed [m/s]');axes[0,1].set_xlabel('Time after release [s]');axes[0,1].legend(fontsize=8)
axes[1,0].axhline(0,color='gray',linewidth=.7);axes[1,0].set_title('Same actual coast recordings: stored energy')
axes[1,0].set_ylabel('Stored energy change [J]');axes[1,0].set_xlabel('Time after release [s]')
ax = axes[1,1]
keys = ['energy_stored_delta_j','energy_known_input_j','energy_known_loss_j','energy_unresolved_j']
bars = [energy[key]/1000 for key in keys]
ax.bar(['Stored delta','Input','Known loss','Residual'],bars,color=['#4f7399','#687b5e','#b39b62','#888888'])
for x,y in enumerate(bars):ax.text(x,y+(0.35 if y>=0 else -0.8),f'{y:+.2f}',ha='center',fontsize=9)
ax.set_ylim(min(-3,min(bars)-1),max(bars)*1.25);ax.axhline(0,color='gray',linewidth=.7);ax.set_ylabel('Energy [kJ]');ax.set_title('Loaded 6 degree uphill brake: observer budget')
ax.text(.02,.97,'Full loss accounting remains incomplete',transform=ax.transAxes,va='top',fontsize=9,
        bbox={'facecolor':'white','alpha':.85,'edgecolor':'none'})
for ax in axes.flat:ax.grid(axis='y',alpha=.18);ax.set_axisbelow(True)
fig.suptitle('ForkliftSim v0.18 | Recorded Unreal numerical qualification | 2026-10-29',fontsize=14)
fig.savefig(root/'Docs/MecanumConservationValidation.png',dpi=150)
data={'scope':'Measured native Unreal CSV samples and log metrics; 480 Hz coast is an intermediate rejected contact profile.',
      'old_coast':old,'new_coast':new,'reaction_bench':bench,'loaded_energy_observer':energy,
      'sources':[{'path':str(path.relative_to(root)), 'sha256':hashlib.sha256(path.read_bytes()).hexdigest()} for path in [old_path,new_path]]}
(root/'Docs/References/MecanumConservation-PlotData.json').write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8')
print('Recorded native plot saved: Docs/MecanumConservationValidation.png')
