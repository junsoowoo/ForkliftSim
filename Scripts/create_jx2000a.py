"""Author the Chinese JX-2000A exterior from published dimensions and photos.

Blender 5.2: blender --background --python Scripts/create_jx2000a.py -- <root>
No OEM CAD, copied mesh, texture, or undocumented engineering dimension is used.
X points towards fork tips; Z is up; all dimensions are metres.
"""
import bpy, json, math, sys
from pathlib import Path
from mathutils import Vector

ROOT=Path(sys.argv[sys.argv.index('--')+1])
OUT=ROOT/'ContentSource/JX2000A'; OUT.mkdir(parents=True,exist_ok=True)
IMAGES=ROOT/'Docs/Images'; IMAGES.mkdir(parents=True,exist_ok=True)
bpy.ops.object.select_all(action='SELECT'); bpy.ops.object.delete(use_global=False)
scene=bpy.context.scene; scene.unit_settings.system='METRIC'; scene.unit_settings.scale_length=1
PALETTE={
 'Orange':([1.0,.105,.022,1],.0,.30),
 'Graphite':([.045,.052,.058,1],.25,.36),
 'Panel':([.090,.097,.102,1],.15,.43),
 'Rubber':([.014,.018,.022,1],.0,.76),
 'Steel':([.29,.32,.34,1],.78,.34),
 'Silver':([.54,.58,.60,1],.85,.29),
 'White':([.88,.91,.90,1],.1,.38),
 'Cyan':([.015,.8,.9,1],.0,.27),
 'Red':([.65,.012,.012,1],.1,.26),
 'Lime':([.50,.8,.01,1],.1,.35),
 'Glass':([.008,.025,.035,1],.3,.17),
}
MATS={}
for name,(color,metal,rough) in PALETTE.items():
 m=bpy.data.materials.new(name);m.diffuse_color=color;m.use_nodes=True
 bs=m.node_tree.nodes.get('Principled BSDF');bs.inputs['Base Color'].default_value=color
 bs.inputs['Metallic'].default_value=metal;bs.inputs['Roughness'].default_value=rough
 if name=='Cyan':bs.inputs['Emission Color'].default_value=color;bs.inputs['Emission Strength'].default_value=.65
 MATS[name]=m
parts={};current=None
def finish(o,name,material,bevel=0):
 o.name=name; o.data.materials.append(MATS[material])
 if bevel:
  mod=o.modifiers.new('Machined edge radii','BEVEL');mod.width=bevel;mod.segments=3
  mod=o.modifiers.new('Face weighted normals','WEIGHTED_NORMAL');mod.keep_sharp=True;mod.weight=50
 parts[current].append(o);return o
def box(name,pos,size,mat='Graphite',bevel=.002):
 bpy.ops.mesh.primitive_cube_add(size=1,location=pos);o=bpy.context.object
 o.dimensions=size;bpy.ops.object.transform_apply(location=False,rotation=False,scale=True)
 return finish(o,name,mat,bevel)
def cyl(name,pos,radius,depth,mat='Steel',axis='Z',vertices=24,bevel=.001):
 bpy.ops.mesh.primitive_cylinder_add(vertices=vertices,radius=radius,depth=depth,location=pos)
 o=bpy.context.object
 if axis=='Y':o.rotation_euler[0]=math.pi/2
 if axis=='X':o.rotation_euler[1]=math.pi/2
 bpy.ops.object.transform_apply(location=False,rotation=False,scale=True)
 for p in o.data.polygons:p.use_smooth=len(p.vertices)==4
 return finish(o,name,mat,bevel)
def profile(name,xz,width,y=0,mat='Steel',bevel=.002):
 n=len(xz);verts=[(x,y+side*width/2,z) for side in [-1,1] for x,z in xz]
 faces=[tuple(reversed(range(n))),tuple(range(n,2*n))]
 faces += [(i,(i+1)%n,(i+1)%n+n,i+n) for i in range(n)]
 mesh=bpy.data.meshes.new(name);mesh.from_pydata(verts,[],faces);mesh.update()
 o=bpy.data.objects.new(name,mesh);scene.collection.objects.link(o);return finish(o,name,mat,bevel)
def tube(name,points,radius,mat):
 curve=bpy.data.curves.new(name,'CURVE');curve.dimensions='3D';curve.resolution_u=1
 curve.bevel_depth=radius;curve.bevel_resolution=2
 spline=curve.splines.new('POLY');spline.points.add(len(points)-1)
 for p,v in zip(spline.points,points):p.co=(*v,1)
 o=bpy.data.objects.new(name,curve);scene.collection.objects.link(o);finish(o,name,mat)
 return o
FONT=bpy.data.fonts.load('C:/Windows/Fonts/msyhbd.ttc')
def text(name,body,pos,size,rot,mat='White'):
 c=bpy.data.curves.new(name,'FONT');c.body=body;c.font=FONT;c.size=size;c.extrude=.00015;c.align_x='CENTER';c.align_y='CENTER'
 o=bpy.data.objects.new(name,c);scene.collection.objects.link(o);o.location=pos;o.rotation_euler=rot;finish(o,name,mat)
 return o
def bolt(pos,axis='X',r=.003):
 return cyl('Service fastener',pos,r,.0018,'Silver',axis,6,.0003)
def part(name):
 global current
 current=name;parts[name]=[]

# Casing: full-width lower bumper, radiused orange shell and split service doors.
part('SM_JX_Body')
box('Rubber perimeter bumper',(-.002,0,.068),(.656,.735,.070),'Rubber',.025)
box('Bumper steel backing',(-.002,0,.105),(.644,.718,.022),'Steel',.009)
box('Bottom graphite chassis',(-.008,0,.147),(.626,.694,.102),'Graphite',.019)
box('Orange main enclosure',(.004,0,.365),(.626,.700,.436),'Orange',.037)
profile('Sloped top enclosure',[(-.289,.540),(.288,.540),(.288,.601),(.177,.640),(-.195,.640),(-.289,.594)],.656,mat='Orange',bevel=.010)
for side in [-1,1]:
 y=side*.343
 box('Side door gasket',(-.012,y,.349),(.440,.008,.340),'Rubber',.015)
 box('Orange service door',(-.012,y+side*.004,.349),(.427,.009,.326),'Orange',.012)
 # Deep slit backs and individually separated vent louvers.
 for xx in [-.226,.210]:
  box('Recessed side vent',(xx,side*.354,.409),(.050,.004,.112),'Panel',.003)
  for i in range(8):box('Orange vent louver',(xx,side*.357,.365+i*.012),(.048,.003,.005),'Orange',.001)
 for xx in [-.208,.184]:
  for zz in [.214,.487]:bolt((xx,side*.356,zz),'Y')
 box('Service lift recess',(.08,side*.357,.238),(.055,.003,.018),'Panel',.004)
 # Front rounded shoulder and control button.
 box('Corner shoulder',(-.214,side*.295,.547),(.168,.125,.064),'Orange',.015)
 cyl('Emergency stop yellow collar',(-.206,side*.343,.584),.019,.010,'Lime')
 cyl('Emergency mushroom',(-.206,side*.343,.595),.013,.016,'Red',vertices=24,bevel=.003)
 # Front side seals and access panel.
 box('Front upright shroud',(.272,side*.312,.338),(.064,.075,.380),'Orange',.012)
 for zz in [.203,.477]:bolt((.307,side*.310,zz),'X')
box('Rear inset gasket',(-.313,0,.399),(.006,.474,.252),'Rubber',.015)
box('Rear grey fascia',(-.318,0,.399),(.005,.454,.236),'Panel',.013)
for sy in [-.143,.143]:
 points=[(-.322,sy+.046,.411),(-.322,sy+.046,.438)]
 points += [(-.322,sy+.046*math.cos(a),.438+.046*math.sin(a)) for a in [i*math.pi/14 for i in range(15)]]
 points += [(-.322,sy-.046,.411)]
 tube('Cyan status arch',points,.004,'Cyan')
text('Rear manufacturer wordmark','见行AGV',(-.315,0,.234),.055,(math.pi/2,0,-math.pi/2))
box('Rear charge socket recess',(-.324,0,.133),(.005,.105,.045),'Rubber',.004)
box('Rear charge connector',(-.328,0,.133),(.004,.052,.031),'Steel',.003)
for yy in [-.012,.012]:cyl('Charge contact',(-.330+.001,yy,.133),.004,.001,'Graphite','X',16,.0002)
for yy in [-.046,.046]:bolt((-.328,yy,.133))
box('Low front protective plate',(.311,0,.186),(.022,.565,.164),'Graphite',.008)
for yy in [-.23,.23]:box('Fork guide flange',(.302,yy,.143),(.045,.157,.070),'Steel',.004)
text('Asset identification','JX-2000A',(-.015,-.361,.270),.023,(math.pi/2,0,0))

# Low lifting frame: tapered toes and separate load-wheel cassettes.
part('SM_JX_ForkFrame')
box('Fork cross member',(.322,0,.070),(.100,.583,.030),'Steel',.005)
for side in [-1,1]:
 y=side*.23
 profile('Load fork shaft',[(.26,.052),(1.035,.052),(1.035,.085),(.26,.085)],.13,y,'Steel',.003)
 profile('Tapered load fork toe',[(1.205,.052),(1.232,.052),(1.283,.061),(1.283,.071),(1.235,.085),(1.205,.085)],.13,y,'Steel',.003)
 for sy in [-1,1]:box('Load roller window rail',(1.120,y+sy*.056,.0685),(.170,.018,.033),'Steel',.002)
 for sy in [-1,1]:box('Fork vertical skirt',(.751,y+sy*.058,.049),(.918,.009,.030),'Steel',.002)
 # Load roller window bounded by steel lips, yellow bearing guards.
 for yy in [-.043,.043]:
  box('Roller carrier bearing',(1.120,y+yy,.042),(.135,.012,.032),'Lime',.003)
 for xx in [1.08,1.16]:
  cyl('Roller axle head',(xx,y-.054,.035),.006,.005,'Silver','Y',12)
 for xx in [.38,.60,.89]:
  cyl('Fork top rivet',(xx,y,.086),.004,.002,'Silver',vertices=12,bevel=.0004)

part('SM_JX_Tower')
box('Tower base shoe',(.150,0,.640),(.154,.142,.018),'Steel',.005)
box('Navigation tower',(.150,0,1.215),(.083,.078,1.150),'Graphite',.007)
for yy in [-.047,.047]:box('Tower mounting stiffener',(.150,yy,.714),(.092,.006,.140),'Silver',.001)
box('Cable channel',(.197,0,1.128),(.014,.040,.925),'Rubber',.003)
# HMI faces the rear operator/service side, separate bevelled bezel + glass.
box('HMI housing',(.089,0,1.342),(.080,.177,.217),'Panel',.009)
box('HMI bezel',(.046,0,1.353),(.008,.127,.161),'Graphite',.004)
box('HMI display glass',(.041,0,1.353),(.003,.105,.139),'Glass',.002)
for i in range(3):
 box('HMI screen row',(.0388,-.011,1.400-i*.028),(.0007,.067,.006),'Silver',.0005)
 box('HMI touch tile',(.0388,-.027+i*.027,1.313),(.0007,.018,.016),'Cyan' if i==1 else 'Steel',.001)
text('HMI status','JX',(.038,0,1.430),.021,(math.pi/2,0,-math.pi/2),'White')
for yy in [-.079,.079]:
 for zz in [1.255,1.430]:bolt((.045,yy,zz),r=.0025)
# Sensor head: sensor beams are not modelled or claimed as working sensors here.
box('Crosshead sensor beam',(.149,0,1.787),(.132,.275,.058),'Panel',.006)
box('Laser electronics housing',(.149,0,1.831),(.171,.145,.061),'Graphite',.006)
box('Laser optical band',(.149,0,1.840),(.175,.149,.014),'Glass',.004)
for side in [-1,1]:
 y=side*.120
 box('Scanner cheek bracket',(.149,y,1.764),(.148,.024,.112),'Steel',.004)
 cyl('Optical scanner drum',(.123,y,1.758),.031,.067,'Panel','Y',32,.002)
 cyl('Optical scanner aperture',(.123,y+side*.036,1.758),.024,.006,'Glass','Y',32,.001)
 cyl('Scanner aperture rim',(.123,y+side*.040,1.758),.027,.003,'Silver','Y',32,.0005)
 cyl('Scanner front glass',(.123,y+side*.042,1.758),.021,.002,'Glass','Y',32,.0004)
 for xx in [.106,.175]:bolt((xx,side*.135,1.805),'Y',.0025)
for i in range(6):box('Electronics cooling fin',(.151,-.076,1.812+i*.006),(.112,.003,.002),'Silver',.0003)
cyl('Beacon mounting collar',(.149,0,1.880),.052,.021,'Panel',vertices=32)
cyl('Red status beacon',(.149,0,1.910),.035,.040,'Red',vertices=32,bevel=.003)
cyl('Beacon cap',(.149,0,1.935),.041,.010,'Graphite',vertices=32,bevel=.001)

part('SM_JX_DriveWheel')
# One illustrative steer module. CN publishes the drive type, not its count
# or an underside drawing; this placement is an explicitly unverified estimate.
cyl('Drive tyre',(-.173,0,.100),.100,.073,'Rubber','Y',40,.006)
for side in [-1,1]:
 cyl('Drive hub',(-.173,side*.039,.100),.052,.006,'Steel','Y',24,.002)
 cyl('Axle nut',(-.173,side*.044,.100),.012,.008,'Silver','Y',6,.001)
box('Drive fork yoke',(-.173,0,.189),(.114,.115,.021),'Steel',.003)
for side in [-1,1]:box('Drive yoke side',(-.173,side*.049,.146),(.084,.015,.071),'Graphite',.003)
cyl('Steer kingpin',(-.173,0,.218),.038,.037,'Steel',vertices=24)

for side in [-1,1]:
 part('SM_JX_Caster'+('L' if side<0 else 'R'))
 cyl('Stabilizer tyre',(-.178,side*.281,.036),.036,.031,'Rubber','Y',24,.003)
 cyl('Stabilizer hub',(-.178,side*.300,.036),.018,.005,'Steel','Y',16,.001)
 box('Caster carrier',(-.178,side*.281,.071),(.055,.051,.025),'Graphite',.004)
 for i,xx in enumerate([1.08,1.16]):
  part('SM_JX_LoadRoller'+('L' if side<0 else 'R')+str(i+1))
  cyl('Polyurethane load roller',(xx,side*.23,.035),.035,.071,'Lime','Y',24,.002)
  for sy in [-1,1]:cyl('Roller hub',(xx,side*.23+sy*.037,.035),.015,.003,'Steel','Y',16,.001)

# Bake modifiers, export each articulation part in a shared chassis reference frame.
manifest={'model':'JXAGV JX-2000A Chinese product 95 / 1940mm',
 'units':'metres','forward_axis':'X','up_axis':'Z',
 'palette':{n:{'color':v[0],'metallic':v[1],'roughness':v[2],'emissive':.65 if n=='Cyan' else 0} for n,v in PALETTE.items()},
 'reference':'reference.json','parts':[],'status':'dimensioned exterior, not a validated dynamics plant'}
joined=[];allverts=[]
for name,objects in parts.items():
 bpy.ops.object.select_all(action='DESELECT')
 for o in objects:o.select_set(True)
 bpy.context.view_layer.objects.active=objects[0]
 bpy.ops.object.convert(target='MESH');bpy.ops.object.join()
 o=bpy.context.object;o.name=name;scene.cursor.location=(0,0,0);bpy.ops.object.origin_set(type='ORIGIN_CURSOR')
 bpy.ops.object.transform_apply(location=False,rotation=True,scale=True)
 # Valid UV islands also give Unreal a non-degenerate tangent basis for the
 # converted text, cylinders and bevel faces, despite using colour-only PBR.
 bpy.ops.object.mode_set(mode='EDIT');bpy.ops.mesh.select_all(action='SELECT')
 bpy.ops.uv.smart_project(angle_limit=math.radians(66),island_margin=.01)
 bpy.ops.object.mode_set(mode='OBJECT')
 pts=[o.matrix_world@v.co for v in o.data.vertices];allverts+=pts
 low=[min(v[i] for v in pts) for i in range(3)];high=[max(v[i] for v in pts) for i in range(3)]
 o.data.calc_loop_triangles()
 manifest['parts'].append({'name':name,'dimensions_m':[high[i]-low[i] for i in range(3)],
  'bounds_min_m':low,'bounds_max_m':high,'vertices':len(o.data.vertices),
  'triangles':len(o.data.loop_triangles),'material_slots':[m.name for m in o.data.materials]})
 bpy.ops.export_scene.fbx(filepath=str(OUT/(name+'.fbx')),use_selection=True,object_types={'MESH'},
  apply_unit_scale=True,axis_forward='X',axis_up='Z',add_leaf_bones=False,bake_anim=False)
 joined.append(o)
low=[min(v[i] for v in allverts) for i in range(3)];high=[max(v[i] for v in allverts) for i in range(3)]
manifest['assembled_bounds_m']={'min':low,'max':high,'size':[high[i]-low[i] for i in range(3)]}
manifest['triangles']=sum(p['triangles'] for p in manifest['parts'])
manifest['pivots_m']={'SM_JX_DriveWheel':[-.173,0,.100],
 **{'SM_JX_LoadRoller'+s+str(i+1):[x,sign*.23,.035] for s,sign in [('L',-1),('R',1)] for i,x in enumerate([1.08,1.16])}}
expected=[1.613,.735,1.94]
assert all(abs(a-b)<.002 for a,b in zip(manifest['assembled_bounds_m']['size'],expected)),manifest['assembled_bounds_m']
(OUT/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
bpy.ops.object.select_all(action='DESELECT')
for o in joined:o.select_set(True)
bpy.ops.export_scene.gltf(filepath=str(OUT/'JX2000A.glb'),export_format='GLB',use_selection=True)

# Studio presentation, rendered from actual authored meshes.
current='Studio';parts[current]=[]
ground=box('Studio ground',(0,0,-.021),(200,200,.04),'White',0)
groundmat=bpy.data.materials.new('Studio grey');groundmat.diffuse_color=(.16,.18,.21,1);groundmat.use_nodes=True
groundmat.node_tree.nodes.get('Principled BSDF').inputs['Base Color'].default_value=(.16,.18,.21,1)
groundmat.node_tree.nodes.get('Principled BSDF').inputs['Roughness'].default_value=.8
ground.data.materials.clear();ground.data.materials.append(groundmat)
scene.world.color=(.25,.25,.25);scene.world.use_nodes=True
scene.world.node_tree.nodes['Background'].inputs[0].default_value=(.35,.40,.49,1)
scene.world.node_tree.nodes['Background'].inputs[1].default_value=.45
def aim(o,p):o.rotation_euler=(Vector(p)-o.location).to_track_quat('-Z','Y').to_euler()
for name,pos,energy,size in [('Key',(-3,-4,5),600,4),('Rim',(2,3,4),850,3),('Fill',(-1,2,2.7),240,2.5)]:
 bpy.ops.object.light_add(type='AREA',location=pos);o=bpy.context.object;o.name=name;o.data.energy=energy;o.data.shape='DISK';o.data.size=size;aim(o,(.3,0,.9))
bpy.ops.object.camera_add(location=(-3.2,-4.5,2.6));cam=bpy.context.object;cam.data.type='ORTHO';cam.data.ortho_scale=2.65;aim(cam,(.35,0,.98));scene.camera=cam
scene.render.engine='CYCLES';scene.cycles.samples=24;scene.cycles.use_denoising=True
scene.render.resolution_x=1200;scene.render.resolution_y=1200;scene.render.resolution_percentage=100
scene.view_settings.view_transform='AgX';scene.render.image_settings.file_format='PNG'
scene.render.film_transparent=False
bpy.context.preferences.filepaths.save_version=0
bpy.context.preferences.filepaths.file_preview_type='NONE'
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'JX2000A.blend'))
if '--no-render' in sys.argv:
 print('JX_MODEL_AUTHOR PASS (no render) '+json.dumps(manifest['assembled_bounds_m']),flush=True)
 sys.exit(0)
scene.render.filepath=str(IMAGES/'JX2000A-Rear.png');bpy.ops.render.render(write_still=True)
cam.location=(3.7,-4.4,2.7);aim(cam,(.35,0,.98))
scene.render.filepath=str(IMAGES/'JX2000A-Forks.png');bpy.ops.render.render(write_still=True)
cam.location=(.48,-5,.97);aim(cam,(.48,0,.97));cam.data.ortho_scale=2.35
scene.render.filepath=str(IMAGES/'JX2000A-Side.png');bpy.ops.render.render(write_still=True)
print('JX_MODEL_AUTHOR PASS '+json.dumps(manifest['assembled_bounds_m'])+' triangles='+str(manifest['triangles']),flush=True)
