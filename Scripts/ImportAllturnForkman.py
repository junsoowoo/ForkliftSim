"""Headless native import and default campus presentation update. No GUI inputs."""
import unreal,json
from pathlib import Path
ROOT=Path(unreal.Paths.project_dir());FOLDER=ROOT/'ContentSource/AllturnForkman';DEST='/Game/AllturnForkman'
manifest=json.loads((FOLDER/'manifest.json').read_text(encoding='utf-8'))
tools=unreal.AssetToolsHelpers.get_asset_tools();materials={};results=[]
for name,color in manifest['palette'].items():
 path=DEST+'/Materials/M_'+name;exists=unreal.EditorAssetLibrary.does_asset_exist(path)
 material=unreal.load_asset(path) if exists else tools.create_asset('M_'+name,DEST+'/Materials',unreal.Material,unreal.MaterialFactoryNew())
 if not exists:
  tint=unreal.MaterialEditingLibrary.create_material_expression(material,unreal.MaterialExpressionConstant3Vector,-250,0)
  tint.set_editor_property('constant',unreal.LinearColor(*color));unreal.MaterialEditingLibrary.connect_material_property(tint,'',unreal.MaterialProperty.MP_BASE_COLOR)
  for prop,value in [(unreal.MaterialProperty.MP_ROUGHNESS,.78 if name=='Rubber' else .34),(unreal.MaterialProperty.MP_METALLIC,.75 if name in ['Steel','Silver'] else .04)]:
   e=unreal.MaterialEditingLibrary.create_material_expression(material,unreal.MaterialExpressionConstant,-250,160);e.set_editor_property('r',value);unreal.MaterialEditingLibrary.connect_material_property(e,'',prop)
 unreal.MaterialEditingLibrary.set_material_usage(material,unreal.MaterialUsage.MATUSAGE_INSTANCED_STATIC_MESHES)
 unreal.MaterialEditingLibrary.recompile_material(material);assert unreal.EditorAssetLibrary.save_loaded_asset(material);materials[name]=material
for part in manifest['parts']:
 name=part['name'];task=unreal.AssetImportTask()
 for k,v in {'filename':str(FOLDER/(name+'.fbx')),'destination_path':DEST,'destination_name':name,'automated':True,'save':True,'replace_existing':True}.items():task.set_editor_property(k,v)
 opt=unreal.FbxImportUI()
 for k,v in {'import_mesh':True,'import_as_skeletal':False,'import_materials':False,'import_textures':False,'mesh_type_to_import':unreal.FBXImportType.FBXIT_STATIC_MESH}.items():opt.set_editor_property(k,v)
 data=opt.get_editor_property('static_mesh_import_data')
 for k,v in {'combine_meshes':True,'auto_generate_collision':False,'convert_scene':True,'convert_scene_unit':True}.items():data.set_editor_property(k,v)
 task.set_editor_property('options',opt);tools.import_asset_tasks([task]);mesh=unreal.load_asset(DEST+'/'+name);assert isinstance(mesh,unreal.StaticMesh),name
 for i,slot in enumerate(mesh.get_editor_property('static_materials')):
  key=str(slot.get_editor_property('material_slot_name')).split('.')[0]
  if key not in materials:key=part['material_slots'][i]
  mesh.set_material(i,materials[key])
 assert unreal.EditorAssetLibrary.save_loaded_asset(mesh)
 b=mesh.get_bounding_box();size=b.max-b.min;actual=[size.x/100,size.y/100,size.z/100]
 assert all(abs(a-b)<.001 for a,b in zip(actual,part['dimensions_m'])),(name,actual,part['dimensions_m'])
 results.append({'name':name,'dimensions_m':actual,'materials':len(mesh.get_editor_property('static_materials'))})
# Replace only authored editor parking displays; runtime removes these IDs when
# real physics vehicles spawn, so no duplicate wheel images or fake telemetry.
levels=unreal.get_editor_subsystem(unreal.LevelEditorSubsystem);actors=unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
campus='/Game/Maps/WarehouseCampus';updated=False;preview_ids=[]
if unreal.EditorAssetLibrary.does_asset_exist(campus):
 assert levels.load_level(campus)
 for a in actors.get_all_level_actors():
  if a.actor_has_tag('JXCampusExhibit') or a.actor_has_tag('ForkmanCampusPreview'):
   actors.destroy_actor(a);continue
  if isinstance(a,unreal.WarehouseMapGroup):
   ids=list(a.get_editor_property('layout_record_ids'));trans=list(a.get_editor_property('layout_transforms'))
   if any(str(i).startswith('DISPLAY-AGV-') for i in ids):
    keep=[i for i,r in enumerate(ids) if not str(r).startswith('DISPLAY-AGV-')]
    a.set_editor_property('layout_record_ids',[ids[i] for i in keep]);a.set_editor_property('layout_transforms',[trans[i] for i in keep]);a.rebuild()
 layout=json.loads((ROOT/'Config/WarehouseCampus.json').read_text(encoding='utf-8'))
 group=actors.spawn_actor_from_class(unreal.WarehouseMapGroup,unreal.Vector())
 group.set_actor_label('FORKMAN virtual AGV / parked previews');group.set_editor_property('tags',['ForkmanCampusPreview','CampusOwned'])
 trans=[]
 for bay in layout['parking'][:3]:
  x,y=bay['center_m'];trans.append(unreal.Transform(location=unreal.Vector(x*100,y*100,0),rotation=unreal.Rotator(pitch=0,yaw=0,roll=0)))
  preview_ids.append('DISPLAY-AGV-'+bay['id'][1:])
 group.set_editor_property('layout_mesh',unreal.load_asset(DEST+'/SM_ForkmanAssembly'));group.set_editor_property('layout_transforms',trans);group.set_editor_property('layout_record_ids',preview_ids);group.set_editor_property('blocking',False);group.rebuild()
 assert levels.save_current_level();updated=True
lab_updated=False
if unreal.EditorAssetLibrary.does_asset_exist('/Game/Maps/WarehouseLab'):
 assert levels.load_level('/Game/Maps/WarehouseLab')
 for a in actors.get_all_level_actors():
  if a.get_actor_label().startswith('Preview '):actors.destroy_actor(a)
 a=actors.spawn_actor_from_class(unreal.StaticMeshActor,unreal.Vector())
 a.set_actor_label('Preview FORKMAN virtual AGV');a.static_mesh_component.set_static_mesh(unreal.load_asset(DEST+'/SM_ForkmanAssembly'))
 a.static_mesh_component.set_collision_profile_name('NoCollision');a.static_mesh_component.set_collision_enabled(unreal.CollisionEnabled.NO_COLLISION)
 a.set_actor_enable_collision(False);a.set_actor_hidden_in_game(True);a.set_editor_property('is_editor_only_actor',True);a.set_editor_property('tags',['ForkliftWarehouseOwned'])
 assert levels.save_current_level();lab_updated=True
report={'model':'FORKMAN-inspired virtual AGV','oem_replica':False,'parts':results,'runtime_selected_by':'Config/Modeling.json',
 'campus_previews_updated':updated,'lab_preview_updated':lab_updated,'preview_ids':preview_ids,'physics_parameters_changed':False,'gui_automation_used':False}
out=ROOT/'Saved/Validation/Forkman';out.mkdir(parents=True,exist_ok=True);(out/'NativeImport.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
unreal.log('FORKMAN_MODEL_IMPORT PASS '+json.dumps(report))
if not globals().get('MODELING_PIPELINE',False):unreal.SystemLibrary.quit_editor()
