"""FORKMAN-inspired virtual AGV, using the existing tested articulation frames.

Headless Blender authoring; no desktop automation. Public example dimensions
remain in reference.json; this model intentionally keeps the project's rig.
"""
import bpy,json,math,sys
from pathlib import Path
from mathutils import Vector
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'ContentSource/AllturnForkman';OUT.mkdir(parents=True,exist_ok=True)
IMAGES=ROOT/'Docs/Images';IMAGES.mkdir(parents=True,exist_ok=True)
bpy.ops.object.select_all(action='SELECT');bpy.ops.object.delete(use_global=False)
scene=bpy.context.scene;scene.unit_settings.system='METRIC';scene.unit_settings.scale_length=1
COLORS={'Orange':(1,.16,.012,1),'Graphite':(.035,.044,.052,1),'Steel':(.23,.27,.30,1),
 'Rubber':(.015,.019,.021,1),'Silver':(.55,.60,.64,1),'White':(.86,.90,.93,1),
 'Red':(.67,.008,.015,1),'Cyan':(.005,.50,.70,1),'Amber':(.8,.32,.008,1)}
MATS={}
for name,color in COLORS.items():
 m=bpy.data.materials.new(name);m.diffuse_color=color;m.use_nodes=True
 bs=m.node_tree.nodes.get('Principled BSDF');bs.inputs['Base Color'].default_value=color
 bs.inputs['Metallic'].default_value=.75 if name in ['Silver','Steel'] else .04
 bs.inputs['Roughness'].default_value=.34 if name not in ['Rubber'] else .78;MATS[name]=m
FONT=bpy.data.fonts.load('C:/Windows/Fonts/arialbd.ttf')
def finish(o,name,mat,bevel=0):
 o.name=name;o.data.materials.append(MATS[mat])
 if bevel:
  mod=o.modifiers.new('Radiused edges','BEVEL');mod.width=bevel;mod.segments=3
  mod=o.modifiers.new('Face normals','WEIGHTED_NORMAL');mod.keep_sharp=True
 return o
def box(name,p,size,mat='Graphite',bevel=.003):
 bpy.ops.mesh.primitive_cube_add(size=1,location=p);o=bpy.context.object;o.dimensions=size
 bpy.ops.object.transform_apply(location=False,rotation=False,scale=True);return finish(o,name,mat,bevel)
def cyl(name,p,r,d,mat='Steel',axis='Z',vertices=20,bevel=.001):
 bpy.ops.mesh.primitive_cylinder_add(vertices=vertices,radius=r,depth=d,location=p);o=bpy.context.object
 if axis=='Y':o.rotation_euler[0]=math.pi/2
 if axis=='X':o.rotation_euler[1]=math.pi/2
 bpy.ops.object.transform_apply(location=False,rotation=True,scale=True)
 for f in o.data.polygons:f.use_smooth=len(f.vertices)==4
 return finish(o,name,mat,bevel)
def profile(name,xz,w,y,mat='Orange',bevel=.003):
 n=len(xz);v=[(x,y+s*w/2,z) for s in [-1,1] for x,z in xz]
 faces=[tuple(reversed(range(n))),tuple(range(n,2*n))]+[(i,(i+1)%n,(i+1)%n+n,i+n) for i in range(n)]
 mesh=bpy.data.meshes.new(name);mesh.from_pydata(v,[],faces);mesh.update()
 o=bpy.data.objects.new(name,mesh);scene.collection.objects.link(o);return finish(o,name,mat,bevel)
def tube(name,points,r,mat='Graphite'):
 c=bpy.data.curves.new(name,'CURVE');c.dimensions='3D';c.resolution_u=1;c.bevel_depth=r;c.bevel_resolution=2
 sp=c.splines.new('POLY');sp.points.add(len(points)-1)
 for a,p in zip(sp.points,points):a.co=(*p,1)
 o=bpy.data.objects.new(name,c);scene.collection.objects.link(o);return finish(o,name,mat)
def text(body,p,size,rotation=(math.pi/2,0,0),mat='White'):
 c=bpy.data.curves.new(body,'FONT');c.body=body;c.font=FONT;c.align_x='CENTER';c.align_y='CENTER';c.size=size;c.extrude=.00015
 o=bpy.data.objects.new(body,c);scene.collection.objects.link(o);o.location=p;o.rotation_euler=rotation;return finish(o,body,mat)
manifest={'model':'FORKMAN-inspired virtual autonomous forklift','oem_replica':False,'reference':'reference.json',
 'palette':COLORS,'coordinate_units':'m','parts':[],
 'rig_geometry':'Existing tested custom rig: body headlength 1.405m, width .985m, fixed mast 2.475m, forks 1.2m, 2.1m travel; not the public FORKMAN example dimensions.',
 'collision':'Existing simplified physics boxes retained; visible panel bevels and sloped covers are not engineering collision surfaces.'}
models={}
def export(name,build):
 before=set(bpy.data.objects);build();objects=[o for o in bpy.data.objects if o not in before]
 bpy.ops.object.select_all(action='DESELECT')
 for o in objects:o.select_set(True)
 bpy.context.view_layer.objects.active=objects[0];bpy.ops.object.convert(target='MESH');bpy.ops.object.join();o=bpy.context.object;o.name=name
 scene.cursor.location=(0,0,0);bpy.ops.object.origin_set(type='ORIGIN_CURSOR');bpy.ops.object.transform_apply(location=False,rotation=True,scale=True)
 bpy.ops.object.mode_set(mode='EDIT');bpy.ops.mesh.select_all(action='SELECT');bpy.ops.uv.smart_project(angle_limit=math.radians(66),island_margin=.01);bpy.ops.object.mode_set(mode='OBJECT')
 pts=[o.matrix_world@v.co for v in o.data.vertices];low=[min(v[i] for v in pts) for i in range(3)];high=[max(v[i] for v in pts) for i in range(3)]
 o.data.calc_loop_triangles();manifest['parts'].append({'name':name,'dimensions_m':[high[i]-low[i] for i in range(3)],'bounds_min_m':low,'bounds_max_m':high,'triangles':len(o.data.loop_triangles),'material_slots':[m.name for m in o.data.materials]})
 bpy.ops.export_scene.fbx(filepath=str(OUT/(name+'.fbx')),use_selection=True,object_types={'MESH'},axis_forward='X',axis_up='Z',add_leaf_bones=False,bake_anim=False,apply_unit_scale=True)
 models[name]=o;return o
def body():
 box('Underframe',(-.06,0,0),(1.285,.985,.22),'Graphite',.015)
 box('Battery lower enclosure',(-.12,0,.17),(1.06,.94,.16),'Graphite',.014)
 hood=[(-.65,.20),(-.49,.16),(.405,.16),(.405,.628),(.24,.650),(-.13,.54),(-.52,.31)]
 profile('Sloped orange hood',hood,.90,0,'Orange',.012)
 profile('Central graphite stripe',[(x,z+.004) for x,z in hood],.095,0,'Graphite',.004)
 for side in [-1,1]:
  y=side*.467
  profile('Triangular service panel',[(-.53,.205),(.355,.205),(.355,.585),(.215,.605),(-.115,.505)],.014,y,'Orange',.008)
  tube('Diagonal frame seam',[(-.53,y+side*.012,.205),(-.115,y+side*.012,.505),(.215,y+side*.012,.605),(.355,y+side*.012,.205)],.010,'Graphite')
  box('Side sill',(-.09,side*.482,-.052),(1.14,.020,.085),'Graphite',.006)
  box('Side sill inlay',(-.07,side*.494,-.044),(.77,.003,.016),'Silver',.001)
  text('FORKMAN',(.075,side*.480,.335),.052,(math.pi/2 if side<0 else -math.pi/2,0,0 if side<0 else math.pi),'Graphite')
  text('VIRTUAL AGV',(.075,side*.481,.275),.020,(math.pi/2 if side<0 else -math.pi/2,0,0 if side<0 else math.pi),'Graphite')
  for x,z in [(-.44,.23),(.31,.23),(.30,.55)]:cyl('Panel bolt',(x,side*.481,z),.004,.003,'Steel','Y',6,.0005)
  # Static fenders follow the chassis, while the real drive rollers remain visible.
  for x in [-.469,.469]:
   box('Module mounting bracket',(x,side*.26,-.19),(.105,.070,.19),'Graphite',.005)
   cyl('Inner axle bearing',(x,side*.28,-.34),.052,.036,'Steel','Y',24,.003)
   cyl('Octagonal wheel guard',(x,side*.451,-.29),.106,.018,'Graphite','Y',8,.006)
   cyl('Wheel guard cap',(x,side*.462,-.29),.072,.005,'Steel','Y',8,.002)
 box('Rear wraparound bumper',(-.684,0,-.045),(.036,.94,.12),'Rubber',.012)
 for y in [-.30,.30]:box('Rear reflector',(-.706,y,-.015),(.003,.095,.027),'White',.002)
 box('Controller tray',(.27,0,.658),(.23,.78,.024),'Graphite',.006)
 for y in [-.29,-.22,-.15]:box('Controller ventilation',(.265,y,.672),(.13,.027,.003),'Rubber',.002)
 cyl('Emergency stop collar',(.27,.27,.684),.026,.022,'Amber')
 cyl('Emergency stop',(.27,.27,.705),.018,.025,'Red',bevel=.003)
 box('Service display bezel',(.285,.02,.680),(.14,.19,.018),'Graphite',.003)
 box('Service display glass',(.285,.02,.691),(.111,.151,.003),'Steel',.002)
 for i in range(3):box('Status display row',(.285,.02+(i-1)*.036,.693),(.076,.007,.001),'Cyan',.0003)
 text('SIM / CUSTOM',(-.10,-.498,.018),.022)
def fixed():
 rest=.49;x=.5325;base=.38;h=2.475-base
 for side in [-1,1]:
  y=side*.335
  box('Fixed mast channel',(x,y,base+h/2-rest),(.08,.07,h),'Graphite',.005)
  box('Polished mast race',(x+.041,y,base+h/2-rest),(.004,.041,h-.09),'Silver',.0008)
  for z in [.42,1.4,2.30]:cyl('Mast fixing',(x+.045,y,z-rest),.008,.006,'Steel','X',6)
  cyl('Lift chain guide',(x-.035,side*.245,1.37-rest),.007,1.75,'Steel')
  for z in [2.33,2.36]:cyl('Chain pulley',(x-.015,side*.245,z-rest),.055,.025,'Graphite','Y',24,.002)
 box('Top crossmember',(x,0,2.435-rest),(.14,.74,.08),'Graphite',.005)
 text('FORKMAN / SIM',(x+.074,0,2.435-rest),.028,(math.pi/2,0,math.pi/2))
 cyl('Lift cylinder tube',(.4975,0,1.25-rest),.030,1.2,'Steel')
 tube('Hydraulic return hose',[(.41,.13,.2-rest),(.43,.15,.55-rest),(.45,.13,1.20-rest),(.45,.10,1.8-rest)],.008,'Rubber')
def moving():
 box('Moving crossmember',(0,0,0),(.08,.62,.08),'Steel',.004)
 for side in [-1,1]:
  box('Inner mast channel',(0,side*.27,.6),(.08,.05,1.82),'Steel',.003)
  box('Inner running face',(.043,side*.27,.6),(.006,.027,1.74),'Silver',.0008)
  for z in [-.18,1.35]:cyl('Guide roller',(.012,side*.30,z),.022,.018,'Graphite','Y',20)
 cyl('Hydraulic piston rod',(-.055,0,.665),.015,1.2,'Silver')
def carriage():
 box('Fork carriage',(0,0,0),(.06,.88,.35),'Graphite',.005)
 for side in [-1,1]:
  box('Load backrest post',(-.01,side*.40,.45),(.05,.04,.65),'Graphite',.003)
  box('Fork heel hook',(.036,side*.30,-.075),(.018,.145,.18),'Steel',.002)
 box('Backrest upper beam',(-.01,0,.76),(.05,.84,.04),'Graphite',.003)
 for y in [-.28,-.14,0,.14,.28]:box('Load backrest grille',(-.01,y,.45),(.015,.014,.59),'Steel',.001)
 for y in [-.35,.35]:cyl('Fork lock pin',(.04,y,.15),.009,.020,'Silver','X',12)
def segment():
 box('Fork blade segment',(0,0,0),(.30,.12,.05),'Steel',.001)
 for y in [-.05,.05]:box('Blade edge wear',(0,y,.0251),(.295,.004,.0002),'Silver',0)
export('SM_AGVBody',body);export('SM_FixedMast',fixed);export('SM_MovingMast',moving);export('SM_Carriage',carriage);export('SM_ForkSegment',segment)
# Source .blend and assembled preview use the SAME origins as runtime attachment.
for name,p in {'SM_AGVBody':(0,0,.49),'SM_FixedMast':(0,0,.49),'SM_MovingMast':(.5525,0,.685),'SM_Carriage':(.6725,0,.185)}.items():models[name].location=p
seg=models['SM_ForkSegment']
for side,y in enumerate([-.3,.3]):
 for i in range(4):
  o=seg if side==0 and i==0 else seg.copy()
  if o is not seg:scene.collection.objects.link(o)
  o.location=(.8525+.3*i,y,.035)
# Preview wheels only. The live rig continues to render its actual physical wheels.
wheel_objects=[]
for xi,x in enumerate([.469,-.469]):
 for yi,y in enumerate([-.35,.35]):
  before=set(bpy.data.objects)
  cyl('Wheel hub',(x,y,.15),.105,.14,'Steel','Y',24,.002)
  for i in range(12):
   a=i*math.tau/12;radial=Vector((math.sin(a),0,math.cos(a)));tangent=Vector((math.cos(a),0,-math.sin(a)))
   axis=(Vector((0,1,0))+tangent*(1 if xi==yi else -1)).normalized()
   p=Vector((x,y,.15))+radial*.130
   bpy.ops.mesh.primitive_uv_sphere_add(segments=12,ring_count=8,location=p)
   o=bpy.context.object;o.scale=(.020,.020,.075);o.rotation_euler=axis.to_track_quat('Z','Y').to_euler()
   bpy.ops.object.transform_apply(location=False,rotation=True,scale=True);finish(o,'45 degree passive roller','Rubber')
   for f in o.data.polygons:f.use_smooth=True
  wheel_objects.extend(set(bpy.data.objects)-before)
# Export one non-articulated inspection/display asset without changing live rig meshes.
bpy.ops.object.select_all(action='SELECT');bpy.context.view_layer.objects.active=models['SM_AGVBody']
bpy.ops.object.duplicate();bpy.ops.object.join();assembly=bpy.context.object;assembly.name='SM_ForkmanAssembly'
scene.cursor.location=(0,0,0);bpy.ops.object.origin_set(type='ORIGIN_CURSOR');bpy.ops.object.transform_apply(location=False,rotation=True,scale=True)
assembly.data.calc_loop_triangles();pts=[assembly.matrix_world@v.co for v in assembly.data.vertices]
lo=[min(v[i] for v in pts) for i in range(3)];hi=[max(v[i] for v in pts) for i in range(3)]
manifest['parts'].append({'name':assembly.name,'dimensions_m':[hi[i]-lo[i] for i in range(3)],'bounds_min_m':lo,'bounds_max_m':hi,'triangles':len(assembly.data.loop_triangles),'material_slots':[m.name for m in assembly.data.materials],'display_only':True})
bpy.ops.export_scene.fbx(filepath=str(OUT/'SM_ForkmanAssembly.fbx'),use_selection=True,object_types={'MESH'},axis_forward='X',axis_up='Z',add_leaf_bones=False,bake_anim=False,apply_unit_scale=True)
bpy.ops.export_scene.gltf(filepath=str(OUT/'FORKMAN-VirtualAGV.glb'),export_format='GLB',use_selection=True)
bpy.data.objects.remove(assembly,do_unlink=True)
manifest['runtime_triangles']=sum(p['triangles'] for p in manifest['parts'] if not p.get('display_only'))
(OUT/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
# Neutral studio renders are generated from the native authored meshes.
ground=box('Studio ground',(0,0,-.024),(200,200,.04),'Graphite',0)
scene.world.use_nodes=True;scene.world.node_tree.nodes['Background'].inputs[0].default_value=(.32,.39,.48,1);scene.world.node_tree.nodes['Background'].inputs[1].default_value=.6
def aim(o,p):o.rotation_euler=(Vector(p)-o.location).to_track_quat('-Z','Y').to_euler()
for name,p,power,size in [('Key',(-3,-4,5),700,4),('Rim',(2,3,4),1000,3),('Fill',(-1,3,3),350,3)]:
 bpy.ops.object.light_add(type='AREA',location=p);o=bpy.context.object;o.name=name;o.data.energy=power;o.data.shape='DISK';o.data.size=size;aim(o,(.4,0,1.1))
bpy.ops.object.camera_add(location=(-3.8,-5.5,3.2));cam=bpy.context.object;cam.data.type='ORTHO';cam.data.ortho_scale=3.5;aim(cam,(.55,0,1.15));scene.camera=cam
scene.render.engine='CYCLES';scene.cycles.samples=24;scene.cycles.use_denoising=True
scene.render.resolution_x=1200;scene.render.resolution_y=1200;scene.render.resolution_percentage=100
scene.render.image_settings.file_format='PNG';scene.view_settings.view_transform='AgX'
bpy.context.preferences.filepaths.save_version=0;bpy.context.preferences.filepaths.file_preview_type='NONE'
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'FORKMAN-VirtualAGV.blend'))
if '--no-render' not in sys.argv:
 scene.render.filepath=str(IMAGES/'Forkman-VirtualAGV-Rear.png');bpy.ops.render.render(write_still=True)
 cam.location=(4,-5.5,3);aim(cam,(.55,0,1.15));scene.render.filepath=str(IMAGES/'Forkman-VirtualAGV-Forks.png');bpy.ops.render.render(write_still=True)
print('FORKMAN_MODEL_SOURCE PASS runtime_triangles='+str(manifest['runtime_triangles']),flush=True)
