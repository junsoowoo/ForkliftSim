"""Build a compact handling bay from the user's owned Warehouse pack."""
import unreal,json,math
from pathlib import Path
root=Path(unreal.Paths.project_dir());out=root/'Saved/Validation/Modeling'
actors=unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
levels=unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
if not actors or not levels:raise RuntimeError('Use -ExecutePythonScript with the full editor')
if unreal.EditorAssetLibrary.does_asset_exist('/Game/Maps/WarehouseLab'):
    if not levels.load_level('/Game/Maps/WarehouseLab'):raise RuntimeError('Cannot load WarehouseLab')
    for existing in actors.get_all_level_actors():
        if 'ForkliftWarehouseOwned' in [str(t) for t in existing.tags] or existing.get_actor_label().startswith(('Bay light ','Warehouse key','Warehouse fill')):actors.destroy_actor(existing)
else:
    if not levels.new_level('/Game/Maps/WarehouseLab'):raise RuntimeError('Cannot create WarehouseLab')
unreal.log('WAREHOUSE_BUILD new level created')
world=unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
world.get_world_settings().set_editor_property('force_no_precomputed_lighting',True)
cube=unreal.load_asset('/Engine/BasicShapes/Cube')
floor_mat=unreal.load_asset('/Game/Warehouse/Materials/Instances/MI_Concrete_Floor_Dark_WS')
records=[]
def put(mesh,name,location,rotation=0,scale=(1,1,1),collide=True,preview=False):
    asset=unreal.load_asset(mesh) if isinstance(mesh,str) else mesh
    if not isinstance(asset,unreal.StaticMesh):raise RuntimeError('Missing warehouse mesh '+str(mesh))
    rot=rotation if isinstance(rotation,unreal.Rotator) else unreal.Rotator(0,rotation,0)
    actor=actors.spawn_actor_from_class(unreal.StaticMeshActor,unreal.Vector(*location),rot)
    actor.set_actor_label(name);actor.set_actor_scale3d(unreal.Vector(*scale))
    actor.set_editor_property('tags',['ForkliftWarehouseOwned'])
    component=actor.static_mesh_component;component.set_static_mesh(asset)
    component.set_collision_profile_name('WarehouseVisual' if collide else 'NoCollision')
    component.set_collision_enabled(unreal.CollisionEnabled.QUERY_ONLY if collide else unreal.CollisionEnabled.NO_COLLISION)
    if preview:
        actor.set_actor_hidden_in_game(True);actor.set_actor_enable_collision(False);actor.set_editor_property('is_editor_only_actor',True)
    if collide:
        # Efficient static simple proxies; visible render geometry remains queryable.
        b=asset.get_bounding_box();center=(b.min+b.max)*.5;size=b.max-b.min
        proxy=actors.spawn_actor_from_class(unreal.StaticMeshActor,unreal.Vector(),rot)
        proxy.set_actor_label(name+'_Collision');proxy.static_mesh_component.set_static_mesh(cube)
        proxy.set_editor_property('tags',['ForkliftWarehouseOwned'])
        proxy.set_actor_transform(actor.get_actor_transform(),False,True)
        proxy.set_actor_location(actor.get_actor_transform().transform_location(center),False,False)
        proxy.set_actor_scale3d(unreal.Vector(max(.005,size.x/100*scale[0]),max(.005,size.y/100*scale[1]),max(.005,size.z/100*scale[2])))
        proxy.static_mesh_component.set_collision_profile_name('WarehouseSolid')
        proxy.static_mesh_component.set_collision_enabled(unreal.CollisionEnabled.PHYSICS_ONLY)
        proxy.static_mesh_component.set_visibility(False,False)
        proxy.set_actor_hidden_in_game(True);proxy.static_mesh_component.set_cast_shadow(False)
    records.append({'label':name,'asset':asset.get_path_name(),'location_cm':location})
    if len(records)%20==0:unreal.log('WAREHOUSE_BUILD placed '+str(len(records)))
    return actor

floor=put(cube,'Warehouse floor',(600,0,-5),scale=(22,16,.1),collide=False)
floor.static_mesh_component.set_material(0,floor_mat)
floor.static_mesh_component.set_collision_profile_name('BlockAll')
floor.static_mesh_component.set_collision_enabled(unreal.CollisionEnabled.QUERY_AND_PHYSICS)
floor.set_editor_property('tags',['StabilityRoad','WarehouseScene','ForkliftWarehouseOwned'])

wall='/Game/Warehouse/Assets/Structural/SM_corrugated_Wall_4x4'
for x in [-300,100,500,900,1300]:
    put(wall,'North wall '+str(x),(x,800,0),0)
    put(wall,'South wall '+str(x),(x,-800,0),180)
for y in [-600,-200,200,600]:put(wall,'Rear wall '+str(y),(-500,y,0),90)
# The camera side remains open; ceiling strips leave direct light in the bay.
roof='/Game/Warehouse/Assets/Structural/Exterior/SM_4x4_Roof_Piece'
for x in [-300,100,500,900,1300]:
    for y in [-600,600]:put(roof,'Roof '+str((x,y)),(x,y,400))

prefix='/Game/Warehouse/Assets/Props/WM00219-Warehouse_Scaffolding/'
vertical=prefix+'SM_WM00219-Vertical_Beam_3m'
horizontal=prefix+'SM_WM00219-Horizontal_Beam_3m'
endbeam=prefix+'SM_WM00219-Horizontal_Beam_1_5m'
platform=prefix+'SM_WM00219-Platform_Beam_Cluster_02'
pallet='/Game/Warehouse/Assets/Props/WM00217_Pallet/SM_WM00217-Pallet_Standard'
box='/Game/Warehouse/Assets/Props/WM00221-Scatter_Boxes/SM_WM00221-Scatter_Boxes_2'
for cx in [350,750,1150]:
    for cy in [-500,500]:
        label='Rack '+str((cx,cy))
        for dx in [-150,150]:
            for dy in [-75,75]:put(vertical,label+' upright '+str((dx,dy)),(cx+dx,cy+dy,0))
        for deck_top in [35,195]:
            # Match actual mesh top bounds to the two physical deck levels.
            z=deck_top-unreal.load_asset(horizontal).get_bounding_box().max.z
            for dy in [-75,75]:put(horizontal,label+' beam '+str((z,dy)),(cx-147,cy+dy,z))
            for dx in [-150,150]:put(endbeam,label+' side '+str((z,dx)),(cx+dx,cy-72,z),90)
            for dx in [-75,75]:put(platform,label+' platform '+str((z,dx)),(cx+dx,cy,deck_top-unreal.load_asset(platform).get_bounding_box().max.z))
        # Static visual stock has a simple whole-load proxy; the active test
        # pallet and its loose cargo are spawned separately by the simulator.
        for dx in [-75,75]:
            put(pallet,label+' stored pallet '+str(dx),(cx+dx,cy,35-unreal.load_asset(pallet).get_bounding_box().min.z))
            for dz in [0,27]:
                put(box,label+' carton '+str((dx,dz)),(cx+dx,cy,35+unreal.load_asset(pallet).get_bounding_box().max.z-unreal.load_asset(pallet).get_bounding_box().min.z-unreal.load_asset(box).get_bounding_box().min.z+dz))

def light(kind,name,location,rotation=None):
    a=actors.spawn_actor_from_class(kind,unreal.Vector(*location),rotation or unreal.Rotator())
    a.set_actor_label(name);a.set_editor_property('tags',['ForkliftWarehouseOwned']);return a
for x in [-100,500,1100]:
    for y in [-250,250]:
        a=light(unreal.PointLight,'Bay light '+str((x,y)),(x,y,365))
        c=a.point_light_component;c.set_mobility(unreal.ComponentMobility.MOVABLE)
        c.set_intensity(12);c.set_attenuation_radius(900);c.set_cast_shadows(False)
sun=light(unreal.DirectionalLight,'Warehouse key',(-100,-100,500),unreal.Rotator(-50,-35,0))
sun_component=sun.get_component_by_class(unreal.DirectionalLightComponent)
sun_component.set_editor_property("forward_shading_priority",1);sun_component.set_mobility(unreal.ComponentMobility.MOVABLE);sun_component.set_intensity(2.5)
fill=light(unreal.DirectionalLight,'Warehouse fill',(0,0,450),unreal.Rotator(-35,145,0))
fill_component=fill.get_component_by_class(unreal.DirectionalLightComponent)
fill_component.set_mobility(unreal.ComponentMobility.MOVABLE);fill_component.set_intensity(1);fill_component.set_cast_shadows(False)

# A small fixed environment fill works without Lumen or a captured black sky.
cubemap=unreal.load_asset('/Engine/MapTemplates/Sky/DaylightAmbientCubemap')
if isinstance(cubemap,unreal.TextureCube):
    ambient=light(unreal.SkyLight,'Warehouse ambient',(0,0,350))
    ambient_component=ambient.get_component_by_class(unreal.SkyLightComponent)
    ambient_component.set_mobility(unreal.ComponentMobility.MOVABLE)
    ambient_component.set_editor_property('source_type',unreal.SkyLightSourceType.SLS_SPECIFIED_CUBEMAP)
    ambient_component.set_cubemap(cubemap);ambient_component.set_intensity(.35)
    ambient_component.set_cast_shadows(False)

# Visible in Editor before Play, hidden during the actual articulated simulation.
for name,location in [('SM_AGVBody',(0,0,49)),('SM_FixedMast',(0,0,49)),('SM_MovingMast',(55.25,0,68.5)),('SM_Carriage',(67.25,0,18.5))]:
    put('/Game/ForkliftModel/'+name,'Preview '+name,location,collide=False,preview=True)
for y in [-30,30]:
    for i in range(4):put('/Game/ForkliftModel/SM_ForkSegment','Preview fork '+str((y,i)),(85.25+30*i,y,3.5),collide=False,preview=True)
for i,(x,y) in enumerate([(46.9,-35),(46.9,35),(-46.9,-35),(-46.9,35)]):
    hub=put('/Engine/BasicShapes/Cylinder','Preview hub '+str(i),(x,y,15),unreal.Rotator(0,0,90),(.21,.21,.14),False,True)
    hub.static_mesh_component.set_material(0,unreal.load_asset('/Game/ForkliftModel/Materials/M_Graphite'))
    for j in range(12):
        angle=2*math.pi*j/12;hand=1 if i in (0,3) else -1
        axis=(-math.sin(angle)/math.sqrt(2),hand/math.sqrt(2),math.cos(angle)/math.sqrt(2))
        rot=unreal.Rotator(math.degrees(math.atan2(axis[2],math.hypot(axis[0],axis[1]))),math.degrees(math.atan2(axis[1],axis[0])),0)
        roller=put('/Engine/BasicShapes/Sphere','Preview roller '+str((i,j)),(x+13*math.cos(angle),y,15+13*math.sin(angle)),rot,(.15,.04,.04),False,True)
        roller.static_mesh_component.set_material(0,unreal.load_asset('/Game/ForkliftModel/Materials/M_Rubber'))

# StaticMeshActor reloads its named collision profile. Verify saved previews
# through an explicit NoCollision profile, not only a transient enabled flag.
for owned in actors.get_all_level_actors():
    if owned.get_actor_label().startswith('Preview '):
        assert not owned.get_actor_enable_collision()
        assert owned.static_mesh_component.get_collision_enabled()==unreal.CollisionEnabled.NO_COLLISION
unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).set_level_viewport_camera_info(unreal.Vector(-105,-675,360),unreal.Rotator(-19,70,0))
levels.save_current_level()
(out/'WarehouseLayout.json').write_text(json.dumps({'map':'/Game/Maps/WarehouseLab','actors':records},indent=2),encoding='utf-8')
# Geometry inspection remains local in ignored Saved; no licensed mesh is uploaded.
task=unreal.AssetExportTask();task.object=unreal.load_asset(pallet);task.filename=str(out/'Pallet_Standard.obj')
task.automated=True;task.prompt=False;task.replace_identical=True;task.exporter=unreal.StaticMeshExporterOBJ()
unreal.Exporter.run_asset_export_task(task)
unreal.log('WAREHOUSE_LAYOUT_SAVED actors='+str(len(records)))
