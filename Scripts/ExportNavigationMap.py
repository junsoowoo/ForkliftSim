"""Offline surveyed map from the authored static warehouse collision proxies.
Never reads forklift, pallet or cargo poses. Run in the editor commandlet.
"""
import unreal,json,hashlib
from pathlib import Path
root=Path(unreal.Paths.project_dir())
levels=unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
if not levels.load_level('/Game/Maps/WarehouseLab'):raise RuntimeError('Warehouse map missing')
segments=[];plane=.22
for actor in unreal.get_editor_subsystem(unreal.EditorActorSubsystem).get_all_level_actors():
    if 'ForkliftWarehouseOwned' not in [str(t) for t in actor.tags] or not actor.get_actor_label().endswith('_Collision'):continue
    if abs(actor.get_actor_rotation().pitch)>.01 or abs(actor.get_actor_rotation().roll)>.01:continue
    origin,extent=actor.get_actor_bounds(False)
    if not(origin.z-extent.z<plane*100<origin.z+extent.z):continue
    # The collision proxies in this bay are axis-aligned (0/90/180 degrees).
    yaw=actor.get_actor_rotation().yaw
    if abs(yaw/90-round(yaw/90))>.001:raise RuntimeError('Map requires oriented polygon exporter for '+actor.get_actor_label())
    x0,x1=(origin.x-extent.x)/100,(origin.x+extent.x)/100
    y0,y1=(origin.y-extent.y)/100,(origin.y+extent.y)/100
    points=[[x0,y0],[x1,y0],[x1,y1],[x0,y1]]
    for a,b in zip(points,points[1:]+points[:1]):
        if (a[0]-b[0])**2+(a[1]-b[1])**2>1e-6:segments.append({'a':a,'b':b})
if len(segments)<12:raise RuntimeError('Insufficient surveyed geometry')
report={'id':'warehouse-survey-v1','source':'Offline authoring export of owned static axis-aligned collision proxies at z=0.22 m. Frozen known map; not live ground truth, mapping or SLAM. Initial bay pose is a known deployment assumption.','initial_pose_m_rad':[0,0,0],'segments':segments}
(root/'Config/NavigationMap.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
unreal.log('NAV_MAP_EXPORTED segments='+str(len(segments)))
unreal.SystemLibrary.quit_editor()
