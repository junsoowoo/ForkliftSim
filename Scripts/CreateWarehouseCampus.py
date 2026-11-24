"""Build the 5,000 m2 map. Full editor Python, after compiling WarehouseCampus classes."""
import unreal,json,math
from pathlib import Path
root=Path(unreal.Paths.project_dir());data=json.loads((root/'Config/WarehouseCampus.json').read_text(encoding='utf-8'));out=root/'Saved/Validation/Campus';out.mkdir(parents=True,exist_ok=True)
actors=unreal.get_editor_subsystem(unreal.EditorActorSubsystem);levels=unreal.get_editor_subsystem(unreal.LevelEditorSubsystem);tools=unreal.AssetToolsHelpers.get_asset_tools()
palette={'Context':(.14,.19,.17),'Asphalt':(.075,.095,.11),'Concrete':(.28,.31,.31),'White':(.82,.88,.88),'Yellow':(.95,.62,.08),'Blue':(.09,.34,.49),'Green':(.08,.42,.32),'Orange':(.64,.25,.08),'Purple':(.31,.22,.48),'Dark':(.07,.09,.11),'Steel':(.35,.4,.43),'Polymer':(.035,.043,.05)}
materials={}
for name,color in palette.items():
 path='/Game/Campus/Materials/M_'+name
 if unreal.EditorAssetLibrary.does_asset_exist(path):
  m=unreal.load_asset(path);unreal.MaterialEditingLibrary.set_material_usage(m,unreal.MaterialUsage.MATUSAGE_INSTANCED_STATIC_MESHES);unreal.MaterialEditingLibrary.recompile_material(m);unreal.EditorAssetLibrary.save_loaded_asset(m);materials[name]=m;continue
 m=unreal.load_asset(path) if unreal.EditorAssetLibrary.does_asset_exist(path) else tools.create_asset('M_'+name,'/Game/Campus/Materials',unreal.Material,unreal.MaterialFactoryNew())
 unreal.MaterialEditingLibrary.delete_all_material_expressions(m)
 e=unreal.MaterialEditingLibrary.create_material_expression(m,unreal.MaterialExpressionConstant3Vector,-180,0);e.set_editor_property('constant',unreal.LinearColor(*color));unreal.MaterialEditingLibrary.connect_material_property(e,'',unreal.MaterialProperty.MP_BASE_COLOR)
 r=unreal.MaterialEditingLibrary.create_material_expression(m,unreal.MaterialExpressionConstant,-180,100);r.set_editor_property('r',.75);unreal.MaterialEditingLibrary.connect_material_property(r,'',unreal.MaterialProperty.MP_ROUGHNESS);unreal.MaterialEditingLibrary.set_material_usage(m,unreal.MaterialUsage.MATUSAGE_INSTANCED_STATIC_MESHES);unreal.MaterialEditingLibrary.recompile_material(m);unreal.EditorAssetLibrary.save_loaded_asset(m);materials[name]=m
task=unreal.AssetImportTask();task.set_editor_property('filename',str(root/'ContentSource/Campus/SM_PalletFourWay1100.fbx'));task.set_editor_property('destination_path','/Game/Campus');task.set_editor_property('destination_name','SM_PalletFourWay1100');task.set_editor_property('automated',True);task.set_editor_property('save',True);task.set_editor_property('replace_existing',True)
opt=unreal.FbxImportUI();opt.set_editor_property('import_mesh',True);opt.set_editor_property('import_as_skeletal',False);opt.set_editor_property('import_materials',False);opt.set_editor_property('import_textures',False);opt.set_editor_property('mesh_type_to_import',unreal.FBXImportType.FBXIT_STATIC_MESH);imp=opt.get_editor_property('static_mesh_import_data');imp.set_editor_property('combine_meshes',True);imp.set_editor_property('auto_generate_collision',False);imp.set_editor_property('convert_scene',True);imp.set_editor_property('convert_scene_unit',True);task.set_editor_property('options',opt);tools.import_asset_tasks([task])
pallet='/Game/Campus/SM_PalletFourWay1100';pm=unreal.load_asset(pallet);pm.set_material(0,materials['Polymer']);unreal.EditorAssetLibrary.save_loaded_asset(pm);pb=pm.get_bounding_box();ps=(pb.max-pb.min)/100;assert abs(ps.x-1.1)<.005 and abs(ps.y-1.1)<.005 and abs(ps.z-.12)<.005,(ps.x,ps.y,ps.z)
map_path=data['map']
if unreal.EditorAssetLibrary.does_asset_exist(map_path):
 assert levels.load_level(map_path)
 for a in actors.get_all_level_actors():
  if 'CampusOwned' in [str(t) for t in a.tags]:actors.destroy_actor(a)
else:assert levels.new_level(map_path)
world=unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world();settings=world.get_world_settings();settings.set_editor_property('force_no_precomputed_lighting',True);settings.set_editor_property('default_game_mode',unreal.load_class(None,'/Script/ForkliftSim.WarehouseMapGameMode'))
groups={};record_ids={};meshes={};records=[];checked_materials=set()
cube='/Engine/BasicShapes/Cube.Cube';rack_prefix='/Game/Warehouse/Assets/Props/WM00219-Warehouse_Scaffolding/'
upright=rack_prefix+'SM_WM00219-Vertical_Beam_3m';beam=rack_prefix+'SM_WM00219-Horizontal_Beam_3m';boxmesh='/Game/Warehouse/Assets/Props/WM00221-Scatter_Boxes/SM_WM00221-Scatter_Boxes_2';wall='/Game/Warehouse/Assets/Structural/SM_corrugated_Wall_4x4';roof='/Game/Warehouse/Assets/Structural/Exterior/SM_4x4_Roof_Piece'
def mesh(path):
 if path not in meshes:
  m=unreal.load_asset(path)
  if not isinstance(m,unreal.StaticMesh):raise RuntimeError('Missing local Fab / project mesh '+path)
  for sm in m.get_editor_property('static_materials'):
   material=sm.get_editor_property('material_interface')
   while material and not isinstance(material,unreal.Material):material=material.get_editor_property('parent')
   if material and str(material.get_path_name()).startswith('/Game/') and str(material.get_path_name()) not in checked_materials:
    unreal.MaterialEditingLibrary.set_material_usage(material,unreal.MaterialUsage.MATUSAGE_INSTANCED_STATIC_MESHES);unreal.MaterialEditingLibrary.recompile_material(material);unreal.EditorAssetLibrary.save_loaded_asset(material);checked_materials.add(str(material.get_path_name()))
  meshes[path]=m
 return meshes[path]
def fit(path,center,size,color=None,yaw=0,blocking=False,invisible=False,roof_group=False,record_id=""):
 m=mesh(path);b=m.get_bounding_box();s=b.max-b.min;c=(b.max+b.min)*.5;scale=[size[i]*100/[s.x,s.y,s.z][i] for i in range(3)];a=math.radians(yaw);cx,cy,cz=c.x*scale[0],c.y*scale[1],c.z*scale[2];loc=unreal.Vector(center[0]*100-(math.cos(a)*cx-math.sin(a)*cy),center[1]*100-(math.sin(a)*cx+math.cos(a)*cy),center[2]*100-cz)
 t=unreal.Transform(location=loc,rotation=unreal.Rotator(pitch=0,yaw=yaw,roll=0),scale=unreal.Vector(*scale));key=(path,color,blocking,invisible,roof_group);groups.setdefault(key,[]).append(t);record_ids.setdefault(key,[]).append(record_id)
 if blocking:records.append({'center_m':center,'size_m':size,'yaw_deg':yaw,'kind':'static_structure','record_id':record_id})
def solid(center,size,color='Steel',visible=True,roof_group=False,record_id=''):fit(cube,center,size,color,blocking=True,invisible=not visible,roof_group=roof_group,record_id=record_id)
def paint(bounds,color):
 x0,y0,x1,y1=bounds;fit(cube,[(x0+x1)/2,(y0+y1)/2,.008],[x1-x0,y1-y0,.008],color)
def line(a,b,color='Yellow',width=.08):
 dx,dy=b[0]-a[0],b[1]-a[1];fit(cube,[(a[0]+b[0])/2,(a[1]+b[1])/2,.021],[math.hypot(dx,dy),width,.012],color,yaw=math.degrees(math.atan2(dy,dx)))
def rect(bounds,color='Yellow',width=.08):
 x0,y0,x1,y1=bounds;pts=[(x0,y0),(x1,y0),(x1,y1),(x0,y1)];[line(pts[i],pts[(i+1)%4],color,width) for i in range(4)]
def text(label,p,size=.45,yaw=0,floor=False,color='White'):
 a=actors.spawn_actor_from_class(unreal.TextRenderActor,unreal.Vector(*[v*100 for v in p]),unreal.Rotator(pitch=90 if floor else 0,yaw=yaw+90 if floor else yaw,roll=0));a.set_actor_label(label);a.set_editor_property('tags',['CampusOwned']);c=a.text_render;c.set_text(label);c.set_world_size(size*100);rgb=palette[color];c.set_text_render_color(unreal.Color(r=int(rgb[0]*255),g=int(rgb[1]*255),b=int(rgb[2]*255),a=255))
 if floor:assert abs(c.get_forward_vector().z)>.999, "Floor label must be horizontal: "+label
 return a
def arrow(p,yaw,color='White'):
 a=math.radians(yaw);c,s=math.cos(a),math.sin(a)
 def w(x,y):return (p[0]+c*x-s*y,p[1]+s*x+c*y)
 line(w(-1,0),w(1,0),color,.12);line(w(.35,.5),w(1,0),color,.12);line(w(.35,-.5),w(1,0),color,.12)
# The clear interior dimensions, measured between inner wall faces, are exactly 100 x 50 m.
def campus_floor(c,size,color):
 a=actors.spawn_actor_from_class(unreal.StaticMeshActor,unreal.Vector(*[v*100 for v in c]));a.set_actor_label('Campus shared road '+color);a.set_editor_property('tags',['CampusOwned','CampusRoad']);b=a.static_mesh_component;b.set_static_mesh(mesh(cube));b.set_material(0,materials[color]);b.set_world_scale3d(unreal.Vector(*size));b.set_collision_profile_name('BlockAll');b.set_cast_shadow(False)
fit(cube,[25,20,-.3],[480,300,.2],'Context');campus_floor([50,25,-.1],[100,50,.2],'Concrete');campus_floor([-20,25,-.1],[40,50,.2],'Asphalt');campus_floor([-20,-6,-.1],[40,12,.2],'Asphalt');rect([-40,-12,0,50],'White',.12)
for x in [-26,-16]:line((x,-12),(x,0),'White',.12)
arrow([-23,-6],90);arrow([-18,-6],-90)
paint([-39,1,-1,3],'Green');paint([.4,.6,99.6,2.2],'Green');paint([.4,47.8,99.6,49.4],'Green')
for z in data['zones']:
 paint(z['bounds_m'],z['color']);rect(z['bounds_m'],'White');x0,y0,x1,y1=z['bounds_m'];text(z['name'],[x0+.3,y0+.35,.035],.35,floor=True)
for x in [-26,-16]:line((x,0),(x,50),'White',.12)
for y in range(2,49,6):line((-21,y),(-21,y+2),'White',.1)
for y in [8,20,32,44]:arrow([-23,y],90);arrow([-18,y],-90)
for bay in data['parking']:
 rect(bay['bounds_m'],'Yellow');x,y=bay['center_m'];paint([x-2.7,y-1.7,x+2.7,y+1.7],'Blue' if bay['initial_display'] else 'Dark');text(bay['id']+(' / AGV' if bay['initial_display'] else ' / RESERVE'),[x-2.5,y-1.3,.035],.36,floor=True)
 solid([-37,y,.65],[.6,1.0,1.3],'Blue');fit(cube,[-36.68,y,.9],[.03,.65,.5],'Green');solid([-36.0,y-1,.35],[.12,.12,.7],'Yellow');solid([-36.0,y+1,.35],[.12,.12,.7],'Yellow')
# Reserved painted circulation, with 6 m main spine and 5 m rack-end crossings.
for y in [22,28]:line((14,y),(99,y),'Yellow',.12)
for x in [14,22,92]:line((x,3),(x,47),'Yellow',.12)
for x in [40.5,57.5,74.5]:
 for y0,y1 in [(3,22),(28,47)]:line((x-2.5,y0),(x-2.5,y1),'White');line((x+2.5,y0),(x+2.5,y1),'White')
for x in [28,48,68,88]:arrow([x,23.5],0);arrow([x,26.5],180)
for x in [-12,-6,4,10]:arrow([x,12],0);arrow([x,38],180)
for y in [11,12,13]:paint([-3,y-.14,3,y+.14],'White')
for y in [37,38,39]:paint([-3,y-.14,3,y+.14],'White')
# Perimeter walls are split at two 8 m vehicle portals and pedestrian exits.
def wallbox(c,size):
 if size[0]<size[1]:fit(wall,c,[size[1],size[0],size[2]],yaw=90)
 else:fit(wall,c,size)
 solid(c,size,visible=False)
for y0,y1 in [(0,2),(3.2,8),(16,34),(42,50)]:wallbox([-.1,(y0+y1)/2,3],[.2,y1-y0,6])
for y0,y1 in [(0,2),(3.2,46.8),(48,50)]:wallbox([100.1,(y0+y1)/2,3],[.2,y1-y0,6])
for x in range(0,100,4):
 wallbox([x+2,-.1,3],[4,.2,6]);wallbox([x+2,50.1,3],[4,.2,6])
for y0,y1 in [(8,16),(34,42)]:wallbox([-.1,(y0+y1)/2,5.1],[.2,y1-y0,1.8]);text('ENTRY' if y0==8 else 'EXIT',[-.24,y1-1,4.55],.7,180,color='Yellow')
for x in [0,100]:
 for y in [2,46.8] if x==100 else [2]:wallbox([x,(y+.6),5],[.2,1.2,2])
for y in [0,50]:
 for x in [0,20,40,60,80,100]:solid([x,y,3],[.32,.32,6],'Steel')
for x in range(0,100,4):
 for y in range(0,50,5):fit(roof,[x+2,y+2.5,6.12],[4,5,.08],roof_group=True)
for x in [0,20,40,60,80,100]:fit(cube,[x,25,5.9],[.25,50,.22],'Steel',roof_group=True)
# Back-to-back, 4-bay rack islands. Separate structural proxy members preserve real openings.
for rack in data['racks']:
 x0,y0,x1,y1=rack['bounds_m'];yc=(y0+y1)/2;rect([x0-.2,y0-.3,x1+.2,y1+.3],'Yellow')
 for x in [x0+3*i for i in range(5)]:
  for y in [y0,yc,y1]:fit(upright,[x,y,1.5],[.12,.075,3]);solid([x,y,1.5],[.12,.075,3],visible=False)
 for h in rack['tier_heights_m']:
  for bay in range(4):
   x=x0+bay*3+1.5
   for y in [y0,yc,y1]:fit(beam,[x,y,h-.1],[2.9,.065,.2]);solid([x,y,h-.1],[2.9,.065,.2],visible=False)
   for side in [-1,1]:solid([x,yc+side*.7,h-.035],[2.85,1.3,.07],'Steel')
 text(rack['id'],[(x0+x1)/2,y1+.02,3.1],.45,90 if yc<25 else -90,color='Yellow')
for slot in data['slots']:
 if slot['initial_visual_stock']:
  x,y,z=slot['center_m'];rid='LOAD-'+slot['id'];fit(pallet,[x,y,z+.06],[1.1,1.1,.12],record_id=rid);fit(boxmesh,[x,y,z+.12+.38],[.85,.85,.76],record_id=rid);solid([x,y,z+.44],[1.1,1.1,.88],visible=False,record_id=rid)
for station in data['stations']:
 x,y,z=station['center_m'];fit(pallet,[x,y,.06],[1.1,1.1,.12],record_id=station['id']);rect([x-.65,y-.55,x+.65,y+.55],'White')
for x in [43.5,46,48.5]:
 for i in range(4):fit(pallet,[x,4.5,.06+i*.12],[1.1,1.1,.12])
# Visible reserved vertical cores. These are not working lifts or an engineered upper floor.
for bounds in [[2,42,8,49],[9,42,13,49]]:
 x0,y0,x1,y1=bounds
 for x,y in [(x0,y0),(x1,y0),(x0,y1),(x1,y1)]:solid([x,y,1.2],[.1,.1,2.4],'Purple')
 for y in [y0,y1]:fit(cube,[(x0+x1)/2,y,1.2],[x1-x0,.06,.08],'Purple')
# Three non-physical parked appearance displays; no AStabilityLab or fake fleet telemetry.
model_profile=json.loads((root/'Config/Modeling.json').read_text(encoding='utf-8'))
forkman_preview=model_profile.get('source_folder')=='AllturnForkman' and unreal.EditorAssetLibrary.does_asset_exist('/Game/AllturnForkman/SM_ForkmanAssembly')
for bay in data['parking'][:3]:
 x,y=bay['center_m'];text('AGV-'+bay['id'][1:]+' / BAY',[x-1.7,y+1.55,.035],.3,floor=True)
 if forkman_preview:
  path='/Game/AllturnForkman/SM_ForkmanAssembly';b=mesh(path).get_bounding_box();s=(b.max-b.min)/100;c=(b.max+b.min)/200
  fit(path,[x+c.x,y+c.y,c.z],[s.x,s.y,s.z],record_id='DISPLAY-AGV-'+bay['id'][1:]);continue
 for name,center,size in [('SM_AGVBody',[x,y,.70],[1.405,.985,1.2]),('SM_FixedMast',[x+.55,y,1.55],[.16,.85,2.5]),('SM_Carriage',[x+.75,y,.45],[.08,.88,.8])]:fit('/Game/ForkliftModel/'+name,center,size,record_id='DISPLAY-AGV-'+bay['id'][1:])
 for sy in [-.3,.3]:fit('/Game/ForkliftModel/SM_ForkSegment',[x+1.3,y+sy,.12],[1.2,.12,.05],record_id='DISPLAY-AGV-'+bay['id'][1:])
 for dx in [-.469,.469]:
  for dy in [-.35,.35]:fit('/Engine/BasicShapes/Sphere',[x+dx,y+dy,.15],[.30,.16,.30],'Dark',yaw=0,record_id='DISPLAY-AGV-'+bay['id'][1:])
# Authored localization references outside the parking exit lines.
for y in [2,8,14,20,26,32,38,44]:solid([-27,y,.6],[.35,.35,1.2],'Orange')
# Roadside bumpers and recovery bay markers remain clear of portals.
for y in [7.4,16.6,33.4,42.6]:solid([.5,y,.5],[.18,.18,1],'Yellow')
text('5,000 m2 / F1',[-.25,26,4.5],.7,180,color='White');text('8 BAYS / 3 INITIAL AGVs',[-27,49,.035],.5,floor=True);text('PEDESTRIAN',[28,1.2,.035],.4,floor=True)
summary=[]
for n,(key,transforms) in enumerate(groups.items()):
 path,color,blocking,invisible,isroof=key;a=actors.spawn_actor_from_class(unreal.WarehouseMapGroup,unreal.Vector());a.set_actor_label('Campus group '+str(n)+' '+Path(path).name);a.set_editor_property('tags',['CampusOwned']+(['CampusRoof'] if isroof else []));a.set_editor_property('layout_mesh',mesh(path));a.set_editor_property('layout_material',materials[color] if color else None);a.set_editor_property('layout_transforms',transforms);a.set_editor_property('layout_record_ids',record_ids[key]);a.set_editor_property('blocking',blocking);a.set_editor_property('invisible_proxy',invisible);a.set_editor_property('cast_layout_shadow',False);a.rebuild();summary.append({'mesh':path,'material':color,'instances':len(transforms),'blocking':blocking,'invisible_proxy':invisible,'roof':isroof})
sun=actors.spawn_actor_from_class(unreal.DirectionalLight,unreal.Vector(0,0,900),unreal.Rotator(pitch=-55,yaw=-25,roll=0));sun.set_actor_label('Campus sunlight');sun.set_editor_property('tags',['CampusOwned']);c=sun.get_component_by_class(unreal.DirectionalLightComponent);c.set_mobility(unreal.ComponentMobility.MOVABLE);c.set_intensity(3.0);c.set_cast_shadows(False);c.set_editor_property("atmosphere_sun_light",True)
atmosphere=actors.spawn_actor_from_class(unreal.SkyAtmosphere,unreal.Vector());atmosphere.set_editor_property("tags",["CampusOwned"])
sky=actors.spawn_actor_from_class(unreal.SkyLight,unreal.Vector(0,0,800));sky.set_editor_property('tags',['CampusOwned']);c=sky.get_component_by_class(unreal.SkyLightComponent);c.set_mobility(unreal.ComponentMobility.MOVABLE);c.set_editor_property('source_type',unreal.SkyLightSourceType.SLS_SPECIFIED_CUBEMAP);c.set_cubemap(unreal.load_asset('/Engine/MapTemplates/Sky/DaylightAmbientCubemap'));c.set_intensity(.65);c.set_cast_shadows(False)
start=actors.spawn_actor_from_class(unreal.PlayerStart,unreal.Vector(1800,2500,500));start.set_editor_property('tags',['CampusOwned'])
unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).set_level_viewport_camera_info(unreal.Vector(1300,2400,550),unreal.Rotator(pitch=-3,yaw=0,roll=0))
assert levels.save_current_level();report={'map':map_path,'stage':data['stage'],'indoor_area_m2':5000,'outdoor_area_m2':2000,'groups':summary,'instances':sum(x['instances'] for x in summary),'static_collision_boxes':records,'pallet_dimensions_m':[ps.x,ps.y,ps.z],'rack_slots':len(data['slots']),'parked_displays':3,'parking_capacity':8};(out/'BuiltCampus.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
# Survey only authored static geometry intersecting the nominal scanner plane.
# Load envelopes are excluded: inventory movement never rewrites a localization fix.
segments=[]
for r in records:
 if r.get('record_id'):continue
 x,y,z=r['center_m'];sx,sy,sz=r['size_m']
 if not z-sz/2<.22<z+sz/2:continue
 a=math.radians(r['yaw_deg']);c,s=math.cos(a),math.sin(a);pts=[(x+c*dx-s*dy,y+s*dx+c*dy) for dx,dy in [(-sx/2,-sy/2),(sx/2,-sy/2),(sx/2,sy/2),(-sx/2,sy/2)]]
 segments.extend({'a':pts[i],'b':pts[(i+1)%4]} for i in range(4))
(root/'Config/CampusSurvey.json').write_text(json.dumps({'id':'warehouse-campus-f1-authored-static-survey','source':'Authored static member geometry at nominal scanner plane; excludes stock, vehicles and any runtime actor pose. Known initial bay poses are priors, not sensor fixes.','segments':segments},indent=2)+'\n',encoding='utf-8')
unreal.log('CAMPUS_BUILD PASS groups='+str(len(groups))+' instances='+str(report['instances'])+' slots='+str(len(data['slots'])))
unreal.SystemLibrary.quit_editor()
