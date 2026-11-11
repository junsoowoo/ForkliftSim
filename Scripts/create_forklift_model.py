"""Project-owned articulated AGV appearance; run with Blender --background.

SI modelling coordinates: X toward forks, Z up. EKS215a 2019 headlength,
width, forks, wheelbase and lowered mast option are referenced, not replicated.
The four mecanum chassis, panels and component mass split are custom.
"""
import bpy
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'ContentSource/ForkliftModel'
OUT.mkdir(parents=True, exist_ok=True)
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)
bpy.context.scene.unit_settings.system = 'METRIC'
bpy.context.scene.unit_settings.scale_length = 1
COLORS = {'Graphite': (.12,.15,.18,1), 'Yellow': (.95,.57,.04,1),
          'Steel': (.32,.38,.43,1), 'Rubber': (.025,.035,.045,1),
          'Silver': (.65,.71,.74,1), 'Red': (.75,.035,.02,1),
          'White': (.85,.89,.9,1), 'Amber': (1,.32,.015,1)}
MATS = {}
for name,color in COLORS.items():
    mat = bpy.data.materials.new(name)
    mat.diffuse_color = color
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get('Principled BSDF')
    bsdf.inputs['Base Color'].default_value = color
    bsdf.inputs['Roughness'].default_value = .48 if name != 'Steel' else .32
    bsdf.inputs['Metallic'].default_value = .7 if name in ('Steel','Silver') else .08
    MATS[name] = mat

def box(name,center,size,material='Graphite',bevel=.004):
    bpy.ops.mesh.primitive_cube_add(size=1, location=center)
    obj = bpy.context.object; obj.name=name
    obj.dimensions=size
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    obj.data.materials.append(MATS[material])
    if bevel:
        mod=obj.modifiers.new('Machined edges','BEVEL');mod.width=min(bevel,min(size)*.22);mod.segments=3
        bpy.ops.object.modifier_apply(modifier=mod.name)
        mod=obj.modifiers.new('Weighted normals','WEIGHTED_NORMAL');bpy.ops.object.modifier_apply(modifier=mod.name)
    return obj

def cylinder(name,center,radius,depth,material='Steel',rotation=(0,0,0)):
    bpy.ops.mesh.primitive_cylinder_add(vertices=24,radius=radius,depth=depth,location=center,rotation=rotation)
    obj=bpy.context.object;obj.name=name;obj.data.materials.append(MATS[material]);return obj

def text(name,string,location,size,rotation=(math.pi/2,0,math.pi/2),material='White'):
    curve=bpy.data.curves.new(name,'FONT');curve.body=string;curve.size=size;curve.extrude=.0006
    obj=bpy.data.objects.new(name,curve);bpy.context.collection.objects.link(obj);obj.location=location;obj.rotation_euler=rotation;obj.data.materials.append(MATS[material])
    bpy.context.view_layer.objects.active=obj;obj.select_set(True);bpy.ops.object.convert(target='MESH');obj.select_set(False);return obj

manifest={'coordinate_units':'m','import_units':'cm via FBX unit metadata','palette':COLORS,'parts':[]}
def export(name,build):
    before=set(bpy.data.objects);build();objects=[o for o in bpy.data.objects if o not in before]
    bpy.ops.object.select_all(action='DESELECT')
    for obj in objects:obj.select_set(True)
    bpy.context.view_layer.objects.active=objects[0];bpy.ops.object.join();obj=bpy.context.object;obj.name=name
    bpy.context.scene.cursor.location=(0,0,0);bpy.ops.object.origin_set(type='ORIGIN_CURSOR')
    bpy.ops.object.transform_apply(location=False,rotation=True,scale=True)
    bpy.ops.export_scene.fbx(filepath=str(OUT/(name+'.fbx')),use_selection=True,object_types={'MESH'},axis_forward='X',axis_up='Z',add_leaf_bones=False,bake_anim=False,use_mesh_modifiers=True,apply_unit_scale=True)
    manifest['parts'].append({'name':name,'dimensions_m':list(obj.dimensions),'vertices':len(obj.data.vertices),'material_slots':[m.name for m in obj.data.materials]})

def body():
    box('Deck',(-.06,0,0),(1.285,.985,.22),'Graphite',.018)
    box('Battery housing',(-.12,0,.36),(1.06,.94,.52),'Graphite',.025)
    for y in [-.476,.476]:
        box('Side service panel',(-.12,y,.36),(.96,.012,.40),'Yellow',.008)
        for x in [-.51,.25]:
            for z in [.21,.51]:cylinder('Panel fastener',(x,y,z),.006,.005,'Steel',(math.pi/2,0,0))
        box('Service handle',(-.40,y*1.016,.48),(.10,.015,.018),'Graphite')
    box('Rubber rear bumper',(-.685,0,-.025),(.035,.94,.095),'Rubber',.012)
    box('Rear access panel',(-.656,0,.34),(.018,.76,.41),'Steel',.012)
    for y in [-.28,.28]:box('Reflector',(-.67,y,.35),(.005,.09,.025),'White',.002)
    for i in range(7):box('Cooling vent',(-.67,-.14+i*.045,.47),(.006,.018,.07),'Rubber',.001)
    box('Top cover',(-.12,0,.626),(1.00,.91,.016),'Graphite',.006)
    cylinder('E-stop collar',(-.43,-.31,.647),.030,.015,'Yellow')
    cylinder('E-stop button',(-.43,-.31,.664),.021,.025,'Red')
    cylinder('Status lamp base',(-.45,.31,.649),.035,.018,'Rubber')
    cylinder('Inactive lamp lens',(-.45,.31,.681),.029,.05,'Amber')
    text('Asset ID','MEC-01',(-.16,-.485,.30),.055)
    text('Autonomous label','AGV',(.19,-.485,.44),.035)

def fixed():
    x=.5325;rest_z=.49;base=.38;height=2.475-base
    for y in [-.335,.335]:
        box('Fixed rail',(x,y,base+height/2-rest_z),(.08,.07,height),'Graphite',.006)
        box('Rail running face',(x+.041,y,base+height/2-rest_z),(.004,.048,height-.10),'Silver',.001)
        cylinder('Chain guide',(x-.035,y*.75,1.40-rest_z),.009,1.85,'Steel')
    box('Mast top bridge',(x,0,2.435-rest_z),(.14,.74,.08),'Graphite')
    box('Mast ID plate',(x+.074,0,2.43-rest_z),(.008,.22,.05),'Yellow',.002)
    cylinder('Fixed lift cylinder',(.4975,0,1.25-rest_z),.030,1.2,'Graphite')

def moving():
    box('Moving crossmember',(0,0,0),(.08,.62,.08),'Steel')
    for y in [-.27,.27]:
        box('Inner rail',(0,y,.60),(.08,.05,1.82),'Steel')
        box('Inner runner',(.043,y,.60),(.006,.027,1.74),'Silver',.001)
    cylinder('Moving piston rod',(-.055,0,.665),.015,1.2,'Silver')

def carriage():
    box('Carriage',(0,0,0),(.06,.88,.35),'Graphite',.006)
    for y in [-.4,.4]:box('Load guard',(-.01,y,.45),(.05,.04,.65),'Graphite')
    box('Guard bridge',(-.01,0,.76),(.05,.84,.04),'Graphite')
    for y in [-.2,0,.2]:box('Guard infill',(-.01,y,.45),(.015,.012,.59),'Steel',.001)
    for y in [-.3,.3]:box('Fork hook',(.036,y,-.075),(.018,.145,.18),'Steel')

export('SM_AGVBody',body)
export('SM_FixedMast',fixed)
export('SM_MovingMast',moving)
export('SM_Carriage',carriage)
export('SM_ForkSegment',lambda:box('Fork steel segment',(0,0,0),(.3,.12,.05),'Steel',.001))
# FBX parts retain local origins; the editable Blender file shows the assembly.
for name,location in {'SM_AGVBody':(0,0,.49),'SM_FixedMast':(0,0,.49),'SM_MovingMast':(.5525,0,.685),'SM_Carriage':(.6725,0,.185)}.items():
    bpy.data.objects[name].location=location
segment=bpy.data.objects['SM_ForkSegment']
for side,y in enumerate([-.3,.3]):
    for i in range(4):
        obj=segment if side==0 and i==0 else segment.copy()
        if obj is not segment:bpy.context.collection.objects.link(obj)
        obj.location=(.8525+.3*i,y,.035)
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'AGV_Reference.blend'))
(OUT/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
print('FORKLIFT_MODEL_SOURCE_READY '+str(OUT))
