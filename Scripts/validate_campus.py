"""Layout invariants and the assumed four-way visual prototype. Not fleet qualification."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def load(p):return json.loads((ROOT/p).read_text(encoding='utf-8-sig'))
def overlap(a,b):return min(a[2],b[2])-max(a[0],b[0])>1e-8 and min(a[3],b[3])-max(a[1],b[1])>1e-8
def inside(a,b):return b[0]<=a[0]<=a[2]<=b[2] and b[1]<=a[1]<=a[3]<=b[3]
def main():
 d=load('Config/WarehouseCampus.json');p=load('ContentSource/Campus/PalletFourWay1100.json');checks=[]
 def check(name,value):assert value,name;checks.append(name)
 b=d['indoor_bounds_m'];check('Clear indoor area is 5000 m2',(b[2]-b[0])*(b[3]-b[1])==5000)
 check('One floor and 3 initial displays, 8 parking bays',d['floors_built']==1 and d['initial_vehicles']==3 and len(d['parking'])==8 and sum(x['initial_display'] for x in d['parking'])==3)
 check('Parking contained and separated',all(inside(x['bounds_m'],d['outdoor_bounds_m']) for x in d['parking']) and all(not overlap(x['bounds_m'],y['bounds_m']) for i,x in enumerate(d['parking']) for y in d['parking'][i+1:]))
 check('Eight separated rack islands',len(d['racks'])==8 and all(inside(x['bounds_m'],b) for x in d['racks']) and all(not overlap(x['bounds_m'],y['bounds_m']) for i,x in enumerate(d['racks']) for y in d['racks'][i+1:]))
 check('256 unique two-tier slot IDs',len(d['slots'])==len({s['id'] for s in d['slots']})==256 and {s['tier'] for s in d['slots']}=={1,2})
 racks={r['id']:r for r in d['racks']}
 check('Slot pallet footprint contained',all(inside([s['center_m'][0]-.55,s['center_m'][1]-.55,s['center_m'][0]+.55,s['center_m'][1]+.55],racks[s['rack_id']]['bounds_m']) for s in d['slots']))
 check('Approaches outside rack bodies',all(not any(r['bounds_m'][0]<=s['approach_m'][0]<=r['bounds_m'][2] and r['bounds_m'][1]<=s['approach_m'][1]<=r['bounds_m'][3] for r in d['racks']) for s in d['slots']))
 # Pure authored connectivity, not actual vehicle paths/traffic capacity.
 ids={n['id'] for n in d['route_nodes']};seen=set();todo=['GATE-IN'];adj={k:[] for k in ids}
 for e in d['route_edges']:assert e['a'] in ids and e['b'] in ids;adj[e['a']].append(e['b']);adj[e['b']].append(e['a'])
 while todo:
  n=todo.pop()
  if n in seen:continue
  seen.add(n);todo.extend(adj[n])
 check('Reserved circulation graph connected',seen==ids)
 check('Image overall pallet dimensions',p['dimensions_m']==[1.1,1.1,.12])
 # A 120 x 50 mm tine at the assumed channel centre traverses both axes.
 clear=True
 for axis in [0,1]:
  side=1-axis
  for centre in [-.23,.23]:
   for part in p['parts']:
    c,s=part['center_m'],part['size_m'];intersects_z=c[2]-s[2]/2<.07 and c[2]+s[2]/2>.02;intersects_side=c[side]-s[side]/2<centre+.06 and c[side]+s[side]/2>centre-.06
    clear &= not(intersects_z and intersects_side)
 check('Assumed box geometry has four-direction tine corridors',clear)
 report={'date':'2026-11-12','pass':True,'checks':checks,'indoor_m2':5000,'parking_bays':8,'rack_slots':256,'minimum_rack_end_gap_m':4.88,'scope':'Authored layout, assumed pallet prism geometry and reserved graph only. Not forklift motion, multi-vehicle throughput, load rating, charging or four-way sensing qualification.'}
 out=ROOT/'Saved/Validation/Campus/LayoutChecks.json';out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(report))
if __name__=='__main__':main()
