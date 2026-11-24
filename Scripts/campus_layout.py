"""Authored metre-space layout, shared by the Unreal builder and geometric review."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def make():
 d={'schema':1,'map':'/Game/Maps/WarehouseCampus','stage':'authored layout; runtime fleet physics, perception and dispatch are owned by CampusOperations','indoor_bounds_m':[0,0,100,50],'indoor_area_m2':5000,'outdoor_bounds_m':[-40,0,0,50],'outdoor_area_m2':2000,'access_road_bounds_m':[-40,-12,0,0],'access_road_area_m2':480,'wall_height_m':6,'minimum_overhead_clearance_m':5.79,'future_floor_pitch_m':7,'floors_built':1,'initial_vehicles':3,'parking_capacity':8,'pallet_dimensions_m':[1.1,1.1,.12],'rack_tiers_m':[.35,1.95],'zones':[], 'parking':[], 'racks':[], 'slots':[], 'stations':[], 'route_nodes':[], 'route_edges':[]}
 def zone(id,name,box,color):d['zones'].append({'id':id,'name':name,'bounds_m':box,'color':color})
 zone('IN','INBOUND / CHECK',[2,4,12,8],'Blue');zone('OUT','OUTBOUND / DISPATCH',[2,28.5,12,33.5],'Green');zone('HOLD','HOLD / DAMAGED',[24,2.5,36,6.5],'Orange');zone('EMPTY','EMPTY PALLETS',[40,2.5,52,6.5],'Blue')
 zone('SERVICE','SERVICE / RECOVERY',[-13,43,-2,49],'Orange');zone('LIFT','FUTURE GOODS LIFT',[2,42,8,49],'Purple');zone('STAIR','FUTURE STAIR / LOBBY',[9,42,13,49],'Purple');zone('RESERVE','FUTURE FLOOR SERVICES',[16,46,22,49],'Purple')
 for i,y in enumerate([5,11,17,23,29,35,41,47],1):d['parking'].append({'id':f'P{i:02}','center_m':[-32,y],'bounds_m':[-36,y-2,-28,y+2],'initial_display':i<=3,'charging_reserved':True})
 for row,y in [('S',12),('N',38)]:
  for col,x in enumerate([26,43,60,77],1):
   rid=f'{row}{col:02}';d['racks'].append({'id':rid,'bounds_m':[x,y-1.4,x+12,y+1.4],'bays':4,'tier_heights_m':d['rack_tiers_m'],'back_to_back':True})
   for bay in range(4):
    for side in [-1,1]:
     for tier,h in enumerate(d['rack_tiers_m'],1):
      for sub,dx in enumerate([.75,2.25],1):
       sid=f'{rid}-B{bay+1}-{"S" if side<0 else "N"}-L{tier}-{sub}';p=[x+3*bay+dx,y+side*.73,h]
       d['slots'].append({'id':sid,'rack_id':rid,'center_m':p,'approach_m':[p[0],y+side*4.,0],'approach_yaw_deg':90 if side<0 else -90,'tier':tier,'initial_visual_stock':tier==1 and bay in [0,2] and sub==1})
 for i,y in enumerate([4.9,6.1,7.3],1):d['stations'].append({'id':f'PICK-{i:02}','kind':'surveyed_floor_pickup','center_m':[7,y,0],'entry_directions_deg':[0,90,180,270]})
 for id,p in [('GATE-IN',[0,12]),('GATE-OUT',[0,38]),('PARK-ROAD',[-20,25]),('PARK-IN',[-20,12]),('PARK-OUT',[-20,38]),('ENTRY',[18,12]),('EXIT',[18,38])]:d['route_nodes'].append({'id':id,'xy_m':p})
 for x in [18,40.5,57.5,74.5,96]:
  for y in [7,17,25,33,43]:d['route_nodes'].append({'id':f'N{x:g}_{y:g}','xy_m':[x,y]})
 def edge(a,b,width):d['route_edges'].append({'a':a,'b':b,'nominal_width_m':width,'source':'layout reservation; rack-end member extents reduce 5 m nominal crossing to 4.88 m; fleet route validation is next stage'})
 for x in [18,40.5,57.5,74.5,96]:
  for a,b in zip([7,17,25,33],[17,25,33,43]):edge(f'N{x:g}_{a}',f'N{x:g}_{b}',8 if x in [18,96] else 5)
 for y in [7,17,25,33,43]:
  for a,b in zip([18,40.5,57.5,74.5],[40.5,57.5,74.5,96]):edge(f'N{a:g}_{y}',f'N{b:g}_{y}',6 if y==25 else 4)
 edge('GATE-IN','ENTRY',8);edge('GATE-OUT','EXIT',8)
 for a,b in [('PARK-ROAD','PARK-IN'),('PARK-ROAD','PARK-OUT'),('PARK-IN','GATE-IN'),('GATE-OUT','PARK-OUT'),('ENTRY','N18_7'),('ENTRY','N18_17'),('EXIT','N18_33'),('EXIT','N18_43')]:edge(a,b,8)
 return d
if __name__=='__main__':
 d=make();p=ROOT/'Config/WarehouseCampus.json';p.write_text(json.dumps(d,indent=2)+'\n',encoding='utf-8');print(f'CAMPUS_LAYOUT indoor={d["indoor_area_m2"]}m2 outdoor={d["outdoor_area_m2"]}m2 racks={len(d["racks"])} slots={len(d["slots"])} parking={len(d["parking"])}')
