"""Independent checks of the authored convex rollers (not native cooked contact)."""
import hashlib,json
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull
from scipy.optimize import linprog

root=Path(__file__).resolve().parents[1]
spec=json.loads((root/'Config/Mecanum.json').read_text(encoding='utf-8-sig'))
vehicle=json.loads((root/'Config/Vehicle3D.json').read_text(encoding='utf-8-sig'))
radius=vehicle['wheel_radius_m'];small=spec['roller_max_radius_m'];rc=radius-small
half=spec['roller_half_length_m'];count=spec['roller_count'];c=np.sqrt(.5)
phi=np.linspace(0,np.pi/2,4097);points=[]
for x in np.linspace(-half,half,13):
    low,high=0,small
    for _ in range(40):
        a=(low+high)/2
        maximum=np.max((rc+a*np.cos(phi))**2+(abs(x)*c+a*c*np.sin(phi))**2)
        if maximum>radius**2:high=a
        else:low=a
    a=(low+high)/2
    for p in np.arange(32)*2*np.pi/32:points.append([x,a*np.sin(p),a*np.cos(p)])
points=np.array(points)
def roller(i,hand):
    theta=2*np.pi*i/count;radial=np.array([np.cos(theta),0,np.sin(theta)]);axle=np.array([0,1,0])
    axis=(-np.cross(axle,radial)+hand*axle)*c;y=np.cross(radial,axis)
    return rc*radial+points@np.stack([axis,y,radial])
hand_results=[]
for hand in (-1,1):
    bodies=[roller(i,hand) for i in range(count)];all_points=np.concatenate(bodies)
    # Maximal inscribed-ball radius in pair intersection: positive means overlap.
    intersections=[]
    hulls=[ConvexHull(p).equations for p in bodies]
    for i in range(count):
        eq=np.concatenate([hulls[i],hulls[(i+1)%count]])
        fit=linprog([0,0,0,-1],A_ub=np.column_stack([eq[:,:3],np.ones(len(eq))]),b_ub=-eq[:,3],bounds=[(None,None)]*4,method='highs')
        if not fit.success:raise RuntimeError('Convex pair check did not converge')
        intersections.append(float(fit.x[3]))
    supports=[]
    for p in np.linspace(0,2*np.pi/count,361):
        supports.append(np.max(all_points[:,0]*np.cos(p)+all_points[:,2]*np.sin(p)))
    width=2*np.max(abs(all_points[:,1]));radial_max=np.max(np.linalg.norm(all_points[:,[0,2]],axis=1))
    drop=radius-min(supports)
    hub_clearance=np.min(np.linalg.norm(all_points[:,[0,2]],axis=1))-spec['hub_radius_m']
    passed=hub_clearance>.001 and max(intersections)<0 and width<=spec['wheel_width_m'] and radial_max<=radius+1e-7 and drop<.002
    hand_results.append(dict(hand=hand,pass_geometry=bool(passed),max_adjacent_intersection_insphere_m=max(intersections),
        hub_min_radial_clearance_m=float(hub_clearance),actual_width_m=float(width),max_radial_m=float(radial_max),max_authored_support_drop_m=float(drop)))
result=dict(scope='Independent authored hull, 13x32 sampled profile; native convex cooking, stiffness, load rating and contact dynamics require separate tests.',
    pass_geometry=all(r['pass_geometry'] for r in hand_results),results=hand_results,
    config_sha256=hashlib.sha256((root/'Config/Mecanum.json').read_bytes()).hexdigest())
if spec.get('contact_geometry')=='inscribed_9_spheres':
    planes=ConvexHull(points).equations
    centers=np.linspace(-half*2/3,half*2/3,9)
    radii=np.array([np.min(-(planes[:,0]*x+planes[:,3])/np.linalg.norm(planes[:,:3],axis=1)) for x in centers])
    # Independent LP hull planes; native uses side-profile planes. All spheres
    # must be inscribed, so no new neighbour or hub overlap can be introduced.
    support_error=0.;support_drop=0.
    for p in np.linspace(0,2*np.pi/count,721):
        down=np.array([np.cos(p),0,np.sin(p)]);sphere_support=-1.
        authored=max(np.max(roller(i,1)@down) for i in range(count))
        for i in range(count):
            theta=2*np.pi*i/count;radial=np.array([np.cos(theta),0,np.sin(theta)]);axle=np.array([0,1,0])
            axis=(-np.cross(axle,radial)+axle)*c
            sphere_support=max(sphere_support,float(np.max((rc*radial+centers[:,None]*axis)@down+radii)))
        support_error=max(support_error,authored-sphere_support);support_drop=max(support_drop,radius-sphere_support)
    inscribed=all(np.max(planes[:,:3]@np.array([x,0,0])+planes[:,3]+r*np.linalg.norm(planes[:,:3],axis=1))<1e-10 for x,r in zip(centers,radii))
    result['sphere_contact']={'count':9,'centers_m':centers.tolist(),'radii_m':radii.tolist(),'inscribed':bool(inscribed),'ground_support_error_m':support_error,'ground_support_drop_m':support_drop,'local_axial_end_recess_m':float(half-max(centers+radii)),'mass_com_inertia':'Native copied from original authored barrel; sphere overlaps are not counted as material.'}
    result['pass_geometry']=result['pass_geometry'] and bool(inscribed) and support_error<.0002 and support_drop<.0003
out=root/'Saved/Validation/Mecanum-Geometry.json';out.parent.mkdir(parents=True,exist_ok=True)
out.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print(json.dumps(result))
raise SystemExit(0 if result['pass_geometry'] else 1)
