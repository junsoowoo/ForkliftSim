"""Blender: project-owned four-way visual prototype from the user's dimensioned image."""
import bpy,json
from pathlib import Path
root=Path(__file__).resolve().parents[1];out=root/'ContentSource/Campus';out.mkdir(parents=True,exist_ok=True)
bpy.ops.object.select_all(action='SELECT');bpy.ops.object.delete(use_global=False)
bpy.context.scene.unit_settings.system='METRIC';bpy.context.scene.unit_settings.scale_length=1
mat=bpy.data.materials.new('CampusPolymer');mat.diffuse_color=(.035,.043,.05,1);mat.use_nodes=True
bsdf=mat.node_tree.nodes.get('Principled BSDF');bsdf.inputs['Base Color'].default_value=mat.diffuse_color;bsdf.inputs['Roughness'].default_value=.72
parts=[]
def box(name,center,size):
 bpy.ops.mesh.primitive_cube_add(size=1,location=center);o=bpy.context.object;o.name=name;o.dimensions=size;bpy.ops.object.transform_apply(location=False,rotation=False,scale=True);o.data.materials.append(mat);parts.append({'name':name,'center_m':center,'size_m':size})
for x in [-.46,0,.46]:
 for y in [-.46,0,.46]:box('Foot', (x,y,.045),(.18,.18,.09))
for y in [-.535,.535]:box('Deck perimeter X',(0,y,.105),(1.1,.03,.03))
for x in [-.535,.535]:box('Deck perimeter Y',(x,0,.105),(.03,1.04,.03))
for i in range(15):
 p=-.49+i*.07;box('Grid X',(0,p,.111),(1.04,.012,.018));box('Grid Y',(p,0,.111),(.012,1.04,.018))
for x in [-.46,0,.46]:box('Underside support Y',(x,0,.097),(.035,1.04,.014))
for y in [-.46,0,.46]:box('Underside support X',(0,y,.097),(1.04,.035,.014))
bpy.ops.object.select_all(action='SELECT');bpy.context.view_layer.objects.active=bpy.context.selected_objects[0];bpy.ops.object.join();obj=bpy.context.object;obj.name='SM_PalletFourWay1100';bpy.context.scene.cursor.location=(0,0,0);bpy.ops.object.origin_set(type='ORIGIN_CURSOR')
bpy.ops.export_scene.fbx(filepath=str(out/'SM_PalletFourWay1100.fbx'),use_selection=True,global_scale=1,apply_unit_scale=True,apply_scale_options='FBX_SCALE_UNITS',object_types={'MESH'},axis_forward='-Y',axis_up='Z',use_mesh_modifiers=True,add_leaf_bones=False,bake_anim=False)
bpy.ops.wm.save_as_mainfile(filepath=str(out/'PalletFourWay1100.blend'))
(out/'PalletFourWay1100.json').write_text(json.dumps({'source':'User image overall dimensions; project-owned visual prototype. Foot/grid/opening details are design assumptions, not OEM CAD or load rating. Runtime manipulable collision/sensor model is a subsequent task.','dimensions_m':[1.1,1.1,.12],'entry_directions_deg':[0,90,180,270],'assumed_clear_opening_m':{'width':.28,'height':.09},'parts':parts},indent=2)+'\n',encoding='utf-8')
print('CAMPUS_PALLET_SOURCE PASS')
