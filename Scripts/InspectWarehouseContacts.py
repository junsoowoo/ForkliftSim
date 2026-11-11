import unreal,json
from pathlib import Path
levels=unreal.get_editor_subsystem(unreal.LevelEditorSubsystem);levels.load_level('/Game/Maps/WarehouseLab')
actors=unreal.get_editor_subsystem(unreal.EditorActorSubsystem);rows=[]
for a in actors.get_all_level_actors():
 for c in a.get_components_by_class(unreal.StaticMeshComponent):
  origin,extent=a.get_actor_bounds(False)
  if abs(origin.x)<230 and abs(origin.y)<150 or a.actor_has_tag('StabilityRoad'):
   rows.append({'actor':a.get_actor_label(),'mesh':str(c.static_mesh),'collision':str(c.get_collision_enabled()),'hidden':a.get_editor_property('hidden'),'location':str(a.get_actor_location()),'scale':str(a.get_actor_scale3d()),'bounds':str(origin)+' '+str(extent)})
Path(unreal.Paths.project_dir(),'Saved/Validation/Modeling/MapContacts.json').write_text(json.dumps(rows,indent=2),encoding='utf-8')
unreal.log('MAP_CONTACTS '+json.dumps(rows))
