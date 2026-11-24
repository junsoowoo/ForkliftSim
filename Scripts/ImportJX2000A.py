"""Import authored parts and create an inspectable native Unreal model review.

Run on a closed editor with UnrealEditor-Cmd -ExecutePythonScript, not a cook.
The campus receives a labelled, non-physical exterior exhibit. Active fleet
physics is deliberately not hidden behind an incompatible low-lift truck shell.
"""
import unreal, json, math
from pathlib import Path

ROOT=Path(unreal.Paths.project_dir())
manifest=json.loads((ROOT/'ContentSource/JX2000A/manifest.json').read_text(encoding='utf-8'))
tools=unreal.AssetToolsHelpers.get_asset_tools()
actors=unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
levels=unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
editor=unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
materials={}
for name,spec in manifest['palette'].items():
 path='/Game/JX2000A/Materials/M_'+name
 exists=unreal.EditorAssetLibrary.does_asset_exist(path)
 material=unreal.load_asset(path) if exists else tools.create_asset('M_'+name,'/Game/JX2000A/Materials',unreal.Material,unreal.MaterialFactoryNew())
 if exists and '-JXRebuildMaterials' not in unreal.SystemLibrary.get_command_line():
  materials[name]=material
  continue
 unreal.MaterialEditingLibrary.delete_all_material_expressions(material)
 color=unreal.MaterialEditingLibrary.create_material_expression(material,unreal.MaterialExpressionConstant3Vector,-240,0)
 color.set_editor_property('constant',unreal.LinearColor(*spec['color']))
 unreal.MaterialEditingLibrary.connect_material_property(color,'',unreal.MaterialProperty.MP_BASE_COLOR)
 for prop,value in [(unreal.MaterialProperty.MP_METALLIC,spec['metallic']),(unreal.MaterialProperty.MP_ROUGHNESS,spec['roughness'])]:
  node=unreal.MaterialEditingLibrary.create_material_expression(material,unreal.MaterialExpressionConstant,-240,160)
  node.set_editor_property('r',value);unreal.MaterialEditingLibrary.connect_material_property(node,'',prop)
 if spec['emissive']:
  glow=unreal.MaterialEditingLibrary.create_material_expression(material,unreal.MaterialExpressionConstant3Vector,-240,260)
  glow.set_editor_property('constant',unreal.LinearColor(*[v*spec['emissive'] for v in spec['color'][:3]],1))
  unreal.MaterialEditingLibrary.connect_material_property(glow,'',unreal.MaterialProperty.MP_EMISSIVE_COLOR)
 unreal.MaterialEditingLibrary.recompile_material(material)
 assert unreal.EditorAssetLibrary.save_loaded_asset(material)
 materials[name]=material

results=[]
for part in manifest['parts']:
 name=part['name'];task=unreal.AssetImportTask()
 for k,v in {'filename':str(ROOT/'ContentSource/JX2000A'/(name+'.fbx')),'destination_path':'/Game/JX2000A','destination_name':name,'automated':True,'save':True,'replace_existing':True}.items():task.set_editor_property(k,v)
 options=unreal.FbxImportUI()
 for k,v in {'import_mesh':True,'import_as_skeletal':False,'import_materials':False,'import_textures':False,'mesh_type_to_import':unreal.FBXImportType.FBXIT_STATIC_MESH}.items():options.set_editor_property(k,v)
 data=options.get_editor_property('static_mesh_import_data')
 for k,v in {'combine_meshes':True,'auto_generate_collision':False,'convert_scene':True,'convert_scene_unit':True}.items():data.set_editor_property(k,v)
 task.set_editor_property('options',options);tools.import_asset_tasks([task])
 mesh=unreal.load_asset('/Game/JX2000A/'+name)
 assert isinstance(mesh,unreal.StaticMesh),name
 for i,slot in enumerate(mesh.get_editor_property('static_materials')):
  key=str(slot.get_editor_property('material_slot_name')).split('.')[0]
  if key not in materials:key=part['material_slots'][i]
  mesh.set_material(i,materials[key])
 assert unreal.EditorAssetLibrary.save_loaded_asset(mesh)
 bounds=mesh.get_bounding_box();actual=[(bounds.max-bounds.min).x/100,(bounds.max-bounds.min).y/100,(bounds.max-bounds.min).z/100]
 assert all(abs(a-b)<.001 for a,b in zip(actual,part['dimensions_m'])),(name,actual,part['dimensions_m'])
 results.append({'name':name,'dimensions_m':actual,'materials':len(mesh.get_editor_property('static_materials'))})

MAP='/Game/Maps/JX2000AReview'
if unreal.EditorAssetLibrary.does_asset_exist(MAP):
 assert levels.load_level(MAP)
 for a in actors.get_all_level_actors():
  if a.actor_has_tag('JXReviewOwned'):actors.destroy_actor(a)
else:assert levels.new_level(MAP)
world=editor.get_editor_world();settings=world.get_world_settings()
settings.set_editor_property('force_no_precomputed_lighting',True)
settings.set_editor_property('default_game_mode',unreal.GameModeBase)

def own(actor,label):
 actor.set_actor_label(label);actor.set_editor_property('tags',['JXReviewOwned']);return actor
def cube(label,center,size,material='Panel'):
 a=own(actors.spawn_actor_from_class(unreal.StaticMeshActor,unreal.Vector(*center)),label)
 c=a.static_mesh_component;c.set_static_mesh(unreal.load_asset('/Engine/BasicShapes/Cube'));c.set_material(0,materials[material]);c.set_collision_enabled(unreal.CollisionEnabled.NO_COLLISION)
 a.set_actor_scale3d(unreal.Vector(*[v/100 for v in size]));return a
def label(body,pos,rotation=unreal.Rotator(pitch=0,yaw=0,roll=0),size=6):
 a=own(actors.spawn_actor_from_class(unreal.TextRenderActor,unreal.Vector(*pos),rotation),body)
 c=a.get_component_by_class(unreal.TextRenderComponent);c.set_text(body);c.set_world_size(size);c.set_text_render_color(unreal.Color(215,230,238,255));return a
floor=cube('Review ground',(40,0,-4),(1100,1000,8),'Graphite')
floor.static_mesh_component.set_collision_enabled(unreal.CollisionEnabled.QUERY_AND_PHYSICS)
floor.static_mesh_component.set_collision_profile_name('BlockAll')
model=own(actors.spawn_actor_from_class(unreal.JX2000AExterior,unreal.Vector()),'JX-2000A CN / 1940mm / EXTERIOR ONLY')
origin,extent=model.get_actor_bounds(False)
native_size=[extent.x*.02,extent.y*.02,extent.z*.02]
assert all(abs(a-b)<.001 for a,b in zip(native_size,[1.613,.735,1.94])),('Assembled UE bounds',native_size)
assert len(model.get_components_by_class(unreal.StaticMeshComponent))==10
for c in model.get_components_by_class(unreal.StaticMeshComponent):
 assert c.get_collision_enabled()==unreal.CollisionEnabled.NO_COLLISION
 assert c.static_mesh is not None
# Metre/centimetre rulers outside the complete model silhouette.
for k in range(18):cube('Length tick '+str(k),(-40+k*10,-53,.5),(.35,3 if k%5 else 6,.5),'Cyan')
cube('Length dimension',(47.65,-53,.5),(161.3,.5,.5),'Cyan')
for x in [-33,128.3]:cube('Length extension',(x,-48,.5),(.5,18,.5),'Cyan')
cube('Width dimension',(-47,0,.5),(.5,73.5,.5),'Cyan')
for y in [-36.75,36.75]:cube('Width extension',(-40,y,.5),(20,.5,.5),'Cyan')
cube('Height dimension',(-47,52,97),(.5,.5,194),'Cyan')
for z in [0,194]:cube('Height end',(-47,52,z),(8,.5,.5),'Cyan')
label('1613 mm',(-15,-64,1),unreal.Rotator(pitch=90,yaw=0,roll=0),7)
label('735 mm',(-59,-12,1),unreal.Rotator(pitch=90,yaw=90,roll=0),7)
label('1940 mm',(-47,52,105),unreal.Rotator(pitch=0,yaw=180,roll=0),7)
label('JX-2000A / CN-95',(-90,72,25),unreal.Rotator(pitch=0,yaw=180,roll=0),9)
label('DIMENSIONED EXTERIOR\n460 kg / 2000 kg rated payload\nNo lift-stroke specification published',(-90,72,11),unreal.Rotator(pitch=0,yaw=180,roll=0),4)
sun=own(actors.spawn_actor_from_class(unreal.DirectionalLight,unreal.Vector(0,0,400),unreal.Rotator(pitch=-50,yaw=-40,roll=0)),'Review key')
c=sun.get_component_by_class(unreal.DirectionalLightComponent);c.set_mobility(unreal.ComponentMobility.MOVABLE);c.set_intensity(4)
c.set_forward_shading_priority(1)
fill=own(actors.spawn_actor_from_class(unreal.DirectionalLight,unreal.Vector(0,0,350),unreal.Rotator(pitch=-30,yaw=130,roll=0)),'Review fill')
c=fill.get_component_by_class(unreal.DirectionalLightComponent);c.set_mobility(unreal.ComponentMobility.MOVABLE);c.set_intensity(1.3);c.set_cast_shadows(False)
sky=own(actors.spawn_actor_from_class(unreal.SkyLight,unreal.Vector(0,0,300)),'Review sky')
c=sky.get_component_by_class(unreal.SkyLightComponent);c.set_mobility(unreal.ComponentMobility.MOVABLE);c.set_editor_property('source_type',unreal.SkyLightSourceType.SLS_SPECIFIED_CUBEMAP);c.set_cubemap(unreal.load_asset('/Engine/MapTemplates/Sky/DaylightAmbientCubemap'));c.set_intensity(.7)
start=own(actors.spawn_actor_from_class(unreal.PlayerStart,unreal.Vector(-240,-320,200),unreal.Rotator(pitch=-14,yaw=48,roll=0)),'Review fly camera')
editor.set_level_viewport_camera_info(unreal.Vector(-240,-320,200),unreal.Rotator(pitch=-14,yaw=48,roll=0))
assert levels.save_current_level()

# Default integrated map exposes the same assembled actor near the parking bays.
# It is an exterior exhibit, not a replacement physics fleet or a telemetry source.
campus='/Game/Maps/WarehouseCampus'
exhibit=False
if unreal.EditorAssetLibrary.does_asset_exist(campus):
 assert levels.load_level(campus)
 for a in actors.get_all_level_actors():
  if a.actor_has_tag('JXCampusExhibit'):actors.destroy_actor(a)
 a=actors.spawn_actor_from_class(unreal.JX2000AExterior,unreal.Vector(-2950,2600,0))
 a.set_actor_label('JX-2000A CN / EXTERIOR EXHIBIT');a.set_editor_property('tags',['JXCampusExhibit','JX2000AExteriorOnly'])
 t=actors.spawn_actor_from_class(unreal.TextRenderActor,unreal.Vector(-2990,2675,150),unreal.Rotator(pitch=0,yaw=180,roll=0));t.set_actor_label('JX exterior status');t.set_editor_property('tags',['JXCampusExhibit'])
 c=t.get_component_by_class(unreal.TextRenderComponent);c.set_text('JX-2000A CN / EXTERIOR ONLY\n1613 x 735 x 1940 mm\nLive fleet: custom Mecanum rig');c.set_world_size(9)
 assert levels.save_current_level();exhibit=True
assert levels.load_level(MAP)
editor.set_level_viewport_camera_info(unreal.Vector(-240,-320,200),unreal.Rotator(pitch=-14,yaw=48,roll=0))
camera_position,camera_rotation=editor.get_level_viewport_camera_info()
# Python's positional Rotator order differs from C++ FRotator. Check the native
# camera axes as well as mesh dimensions to catch an upward-facing review view.
assert abs(camera_rotation.pitch+14)<.01 and abs(camera_rotation.yaw-48)<.01 and abs(camera_rotation.roll)<.01
report={'source_url':'https://www.jxagv.com/products/detail/95.html','parts':results,'assembled_dimensions_m':native_size,
 'no_collision_exterior_components':10,'review_map':MAP,'integrated_campus_exhibit':exhibit,
 'review_camera':{'location_cm':[camera_position.x,camera_position.y,camera_position.z],'pitch_deg':camera_rotation.pitch,'yaw_deg':camera_rotation.yaw,'roll_deg':camera_rotation.roll},
 'active_fleet_changed':False,'unknown_lift_stroke_m':None,'oem_cad':False}
out=ROOT/'Saved/Validation/JX2000A';out.mkdir(parents=True,exist_ok=True)
(out/'NativeImport.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
unreal.log('JX_NATIVE_IMPORT PASS '+json.dumps(report))
unreal.SystemLibrary.quit_editor()
