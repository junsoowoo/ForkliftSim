"""Run with UnrealEditor-Cmd -run=pythonscript; only project-owned FBX files."""
import unreal
import json
from pathlib import Path

ROOT=Path(unreal.Paths.project_dir())
manifest=json.loads((ROOT/'ContentSource/ForkliftModel/manifest.json').read_text(encoding='utf-8'))
tools=unreal.AssetToolsHelpers.get_asset_tools()
materials={}
for name,color in manifest['palette'].items():
    path='/Game/ForkliftModel/Materials/M_'+name
    exists=unreal.EditorAssetLibrary.does_asset_exist(path)
    material=unreal.load_asset(path) if exists else tools.create_asset('M_'+name,'/Game/ForkliftModel/Materials',unreal.Material,unreal.MaterialFactoryNew())
    if exists:
        materials[name]=material
        continue
    unreal.MaterialEditingLibrary.delete_all_material_expressions(material)
    tint=unreal.MaterialEditingLibrary.create_material_expression(material,unreal.MaterialExpressionConstant3Vector,-250,0)
    tint.set_editor_property('constant',unreal.LinearColor(*color))
    unreal.MaterialEditingLibrary.connect_material_property(tint,'',unreal.MaterialProperty.MP_BASE_COLOR)
    for key,value in [(unreal.MaterialProperty.MP_ROUGHNESS,.32 if name=='Steel' else .48),(unreal.MaterialProperty.MP_METALLIC,.7 if name in ('Steel','Silver') else .08)]:
        expr=unreal.MaterialEditingLibrary.create_material_expression(material,unreal.MaterialExpressionConstant,-250,160)
        expr.set_editor_property('r',value);unreal.MaterialEditingLibrary.connect_material_property(expr,'',key)
    unreal.MaterialEditingLibrary.recompile_material(material)
    unreal.EditorAssetLibrary.save_loaded_asset(material)
    materials[name]=material

results=[]
for part in manifest['parts']:
    name=part['name'];task=unreal.AssetImportTask()
    task.set_editor_property('filename',str(ROOT/'ContentSource/ForkliftModel'/(name+'.fbx')))
    task.set_editor_property('destination_path','/Game/ForkliftModel')
    task.set_editor_property('destination_name',name)
    task.set_editor_property('automated',True);task.set_editor_property('save',True);task.set_editor_property('replace_existing',True)
    options=unreal.FbxImportUI();options.set_editor_property('import_mesh',True)
    options.set_editor_property('import_as_skeletal',False)
    options.set_editor_property('import_materials',False);options.set_editor_property('import_textures',False)
    options.set_editor_property('mesh_type_to_import',unreal.FBXImportType.FBXIT_STATIC_MESH)
    data=options.get_editor_property('static_mesh_import_data')
    data.set_editor_property('combine_meshes',True);data.set_editor_property('auto_generate_collision',False)
    data.set_editor_property('convert_scene',True);data.set_editor_property('convert_scene_unit',True)
    task.set_editor_property('options',options);tools.import_asset_tasks([task])
    mesh=unreal.load_asset('/Game/ForkliftModel/'+name)
    if not isinstance(mesh,unreal.StaticMesh):raise RuntimeError('Missing imported mesh '+name)
    for i,slot in enumerate(mesh.get_editor_property('static_materials')):
        slot_name=str(slot.get_editor_property('material_slot_name'))
        match=next((key for key in materials if key.lower() in slot_name.lower()),None)
        if not match and i<len(part['material_slots']):match=part['material_slots'][i]
        if match:mesh.set_material(i,materials[match])
    if not unreal.EditorAssetLibrary.save_loaded_asset(mesh):
        raise RuntimeError("Could not save imported mesh; close project editor and tests first: "+name)
    box=mesh.get_bounding_box();dimensions=(box.max-box.min)/100
    expected=part['dimensions_m']
    actual=[dimensions.x,dimensions.y,dimensions.z]
    if any(abs(a-b)>.01 for a,b in zip(actual,expected)):
        raise RuntimeError('FBX units or axes mismatch '+name+' '+str(actual)+' expected '+str(expected))
    results.append({'name':name,'bounds_m':actual,'material_slots':len(mesh.get_editor_property('static_materials'))})

out=ROOT/'Saved/Validation/Modeling/ImportedModels.json';out.parent.mkdir(parents=True,exist_ok=True)
out.write_text(json.dumps(results,indent=2),encoding='utf-8')
unreal.log('FORKLIFT_MODEL_IMPORT PASS '+json.dumps(results))
