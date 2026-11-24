"""Authoring floorplan from the actual map's shared metre-space layout."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
ROOT=Path(__file__).resolve().parents[1];d=json.loads((ROOT/'Config/WarehouseCampus.json').read_text());fig,ax=plt.subplots(figsize=(15,8),constrained_layout=True)
ax.add_patch(Rectangle((-40,-12),40,62,color='#d8dfdf'));ax.add_patch(Rectangle((0,0),100,50,facecolor='#f1f2ed',edgecolor='#283d42',lw=2));ax.add_patch(Rectangle((14,22),86,6,color='#dee5cf'));ax.add_patch(Rectangle((14,3),8,44,color='#dee5cf'))
colors={'Blue':'#9dc6d6','Green':'#a9cfb8','Orange':'#e4b487','Purple':'#bdb0d3'}
for z in d['zones']:
 x0,y0,x1,y1=z['bounds_m'];ax.add_patch(Rectangle((x0,y0),x1-x0,y1-y0,facecolor=colors[z['color']],edgecolor='white'));ax.text((x0+x1)/2,(y0+y1)/2,{'LIFT':'FUTURE\nLIFT','STAIR':'FUTURE\nSTAIR','RESERVE':'SERVICE\nRESERVE'}.get(z['id'],z['name'].replace(' / ','\n')),ha='center',va='center',fontsize=6.5)
for p in d['parking']:
 x0,y0,x1,y1=p['bounds_m'];ax.add_patch(Rectangle((x0,y0),x1-x0,y1-y0,facecolor='#488a9e' if p['initial_display'] else '#b6c6ca',edgecolor='white'));ax.text((x0+x1)/2,(y0+y1)/2,p['id']+('  AGV' if p['initial_display'] else '  spare'),ha='center',va='center',fontsize=7)
for r in d['racks']:
 x0,y0,x1,y1=r['bounds_m'];ax.add_patch(Rectangle((x0,y0),x1-x0,y1-y0,color='#4c6b79'));ax.text((x0+x1)/2,y1+1.3,r['id']+' / 32 slots',ha='center',fontsize=8)
 for i in range(1,4):ax.plot([x0+3*i]*2,[y0,y1],color='white',lw=.6)
for y,name in [(12,'IN'),(38,'OUT')]:ax.plot([0,0],[y-4,y+4],color='#edf0e9',lw=5);ax.annotate(name,xy=(0,y),xytext=(-10,y),arrowprops={'arrowstyle':'->' if name=='IN' else '<-','color':'#356b7c'},va='center',ha='center',fontsize=9)
ax.text(56,25,'6 m MAIN TRANSFER SPINE',ha='center',va='center',fontsize=11,color='#425b48');ax.text(-19,25,'YARD\n2,000 m²\n+ access road',ha='center',va='center',fontsize=9,color='#344b52');ax.text(50,54,'F1 INTERIOR  100 × 50 m = 5,000 m²',ha='center',fontsize=15,weight='bold');ax.text(50,-8,'GREEN PERIMETER: PEDESTRIAN ROUTE | 8 rack islands × 32 slots = 256 slots',ha='center',fontsize=9)
ax.set(xlim=(-42,103),ylim=(-14,57),xlabel='Metres / X',ylabel='Metres / Y');ax.set_aspect('equal');ax.grid(alpha=.12);fig.suptitle('ForkliftSim | initial 3 AGVs, 8 reserved bays | one floor, upper-floor cores reserved',fontsize=13);(ROOT/'Docs').mkdir(exist_ok=True);fig.savefig(ROOT/'Docs/WarehouseCampusPlan.png',dpi=170);plt.close(fig)
