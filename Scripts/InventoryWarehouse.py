"""Inspect the locally owned Fab pack; never creates substitute warehouse assets."""
import unreal,json
from pathlib import Path
root=Path(unreal.Paths.project_dir());out=root/'Saved/Validation/Modeling';out.mkdir(parents=True,exist_ok=True)
registry=unreal.AssetRegistryHelpers.get_asset_registry()
registry.scan_paths_synchronous(['/Game/Warehouse'],True)
assets=registry.get_assets_by_path('/Game/Warehouse',True)
entries=[];candidates=[]
for asset in assets:
    name=str(asset.asset_name);kind=str(asset.asset_class_path.asset_name);path=str(asset.package_name)
    entries.append({'name':name,'class':kind,'path':path})
    selected=(name.startswith('SM_WM00217') or name.startswith('SM_WM00219') or
              name in ['SM_WM00218-Unwrapped_Box_Square_2_','SM_WM00221-Scatter_Boxes_2',
                       'SM_WM00236-Unexpanded_Boxes_Square_Single','SM_Floor_Module_01',
                       'SM_corrugated_Wall_4x4','SM_Interior_Wall_With_Door_And_Window',
                       'SM_Truck_Door_Open','SM_4x4_Roof_Piece','SM_Roof_Support','SM_Large_Center_Pillar'])
    if kind=='StaticMesh' and selected:candidates.append(path)
(out/'WarehouseAssets.json').write_text(json.dumps(entries,indent=2),encoding='utf-8')
meshes=[];sub=unreal.get_editor_subsystem(unreal.StaticMeshEditorSubsystem)
for path in candidates:
    mesh=unreal.load_asset(path)
    if not isinstance(mesh,unreal.StaticMesh):continue
    b=mesh.get_bounding_box();size=b.max-b.min
    meshes.append({'path':path,'min_cm':[b.min.x,b.min.y,b.min.z],'max_cm':[b.max.x,b.max.y,b.max.z],
                   'size_cm':[size.x,size.y,size.z],'materials':[str(m.get_editor_property('material_slot_name')) for m in mesh.get_editor_property('static_materials')],
                   'simple_collision_count':sub.get_simple_collision_count(mesh) if sub else None})
    (out/'WarehouseMeshes.json').write_text(json.dumps(meshes,indent=2),encoding='utf-8')
    unreal.log('WAREHOUSE_MESH '+json.dumps(meshes[-1]))
(out/'WarehouseMeshes.json').write_text(json.dumps(meshes,indent=2),encoding='utf-8')
unreal.log('WAREHOUSE_INVENTORY PASS assets='+str(len(entries))+' candidate_meshes='+str(len(meshes)))
