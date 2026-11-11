"""Independent small-displacement energy/contact oracle; no Unreal forces.

Planar rigid sectors on an elastic layer, coupled by Timoshenko-type seams.
Coordinates are inward radial displacement, tangent displacement, rc*rotation.
Units: m, N, rad. Material coefficients are design assumptions.
"""
import argparse
import json
import math
import numpy as np
from scipy.linalg import cho_factor, cho_solve
from scipy.optimize import minimize, nnls


def solve(count, height, phase=0., radius=.15, width=.1, depth=.05,
          foundation=1e8, mass=6., poisson=.49, nonlinear=True, native_state=None, self_test=False):
    n = count
    rc = radius-depth/2
    length = 2*math.pi*radius/n
    area = length*width
    young = foundation*depth
    shear = young/(2*(1+poisson))
    seam_length = 2*math.pi*rc/n
    radial = foundation*area
    tangent = shear/depth*area
    rotary = radial*length**2/12
    # Isotropic cohesive seam: objective spring energy, plus separate bending.
    seam_t = shear*width*depth/seam_length
    seam_r = shear*width*depth/seam_length
    seam_b = young*width*depth**3/(12*seam_length)
    ndof = 3*n
    k = np.zeros((ndof, ndof))
    gravity = np.zeros(ndof)
    constraints, rhs, vertex_ids, vertex_offsets = [], [], [], []
    e, t = [], []
    for i in range(n):
        a = (i+phase)*2*math.pi/n
        e.append(np.array([math.sin(a), -math.cos(a)]))
        t.append(np.array([math.cos(a), math.sin(a)]))
        j = 3*i
        k[j:j+3, j:j+3] += np.diag([radial, tangent, rotary/rc**2])
        gravity[j:j+3] = mass/n*9.81*np.array([math.cos(a), math.sin(a), 0.])
        # Exactly the same reference convex envelope, independently assembled.
        angles = np.linspace(-math.pi/n, math.pi/n, max(8,256//n)+1)
        for r, b in [(radius,b) for b in angles]+[(radius-depth,-math.pi/n),(radius-depth,math.pi/n)]:
            p = np.array([r*math.sin(a+b), -r*math.cos(a+b)])
            off = p-rc*e[-1]
            row = np.zeros(ndof)
            row[j:j+3] = [math.cos(a), math.sin(a), off[0]/rc]
            constraints.append(row)
            rhs.append(-height-p[1])
            vertex_ids.append(i)
            vertex_offsets.append(off)
    for i in range(n):
        j = (i+1)%n
        mid = (i+phase+.5)*2*math.pi/n
        em = np.array([math.sin(mid),-math.cos(mid)])
        tm = np.array([math.cos(mid),math.sin(mid)])
        anchor = rc*em
        for direction, stiffness in [(tm,seam_t),(em,seam_r)]:
            row = np.zeros(ndof)
            for index, sign in [(i,-1.),(j,1.)]:
                off = anchor-rc*e[index]
                spin = np.array([-off[1],off[0]])/rc
                row[3*index:3*index+3] = sign*np.array([-e[index]@direction,t[index]@direction,spin@direction])
            k += stiffness*np.outer(row,row)
        row = np.zeros(ndof)
        row[3*i+2], row[3*j+2] = -1/rc,1/rc
        k += seam_b*np.outer(row,row)
    a = np.array(constraints)
    b = np.array(rhs)
    factor = cho_factor(k)
    compliance = cho_solve(factor, a.T)
    diagonal = np.einsum('ij,ji->i',a,compliance)
    q = -cho_solve(factor,gravity)
    forces = np.zeros(len(b))
    for iteration in range(500 if nonlinear else 20000):
        change = 0.
        for j in range(len(b)):
            gap = a[j]@q-b[j]
            new = max(0.,forces[j]-gap/diagonal[j])
            delta = new-forces[j]
            if delta:
                q += compliance[:,j]*delta
                forces[j] = new
                change = max(change,abs(delta))
        if change<1e-5:
            break
    gaps = a@q-b
    residual = np.max(np.abs(k@q+gravity-a.T@forces))
    elastic = .5*q@k@q
    success = change<1e-5 and gaps.min()>-1e-8
    native_debug={}
    if nonlinear:
        e, t = np.array(e), np.array(t)
        mid = (np.arange(n)+phase+.5)*2*math.pi/n
        anchor = rc*np.column_stack([np.sin(mid),-np.cos(mid)])
        off0, off1 = anchor-rc*e, anchor-rc*np.roll(e,-1,axis=0)
        ids, offsets = np.array(vertex_ids), np.array(vertex_offsets)
        core_k = np.array([radial,tangent,rotary/rc**2])
        # True convex centroid, including its small gravity moment.
        aa=np.linspace(-math.pi/n,math.pi/n,max(8,256//n)+1)
        polygon=np.array([(radius*math.cos(x)-rc,radius*math.sin(x)) for x in aa]
                         +[((radius-depth)*math.cos(math.pi/n)-rc,(radius-depth)*math.sin(math.pi/n)),
                           ((radius-depth)*math.cos(math.pi/n)-rc,-(radius-depth)*math.sin(math.pi/n))])
        nxt=np.roll(polygon,-1,axis=0)
        cross=polygon[:,0]*nxt[:,1]-nxt[:,0]*polygon[:,1]
        centroid=float(((polygon[:,0]+nxt[:,0])*cross).sum()/(3*cross.sum()))
        angle=(np.arange(n)+phase)*2*math.pi/n

        def rotate(v, beta):
            c,s=np.cos(beta),np.sin(beta)
            return np.column_stack([c*v[:,0]-s*v[:,1],s*v[:,0]+c*v[:,1]])

        def spin(v):
            return np.column_stack([-v[:,1],v[:,0]])

        def state(x):
            v=x.reshape(n,3)
            beta=v[:,2]/rc
            centers=rc*e-v[:,0,None]*e+v[:,1,None]*t
            r0,r1=rotate(off0,beta),rotate(off1,np.roll(beta,-1))
            d=np.roll(centers,-1,axis=0)+r1-centers-r0
            return v,beta,centers,r0,r1,d

        def energy_grad(x):
            v,beta,centers,r0,r1,d=state(x)
            db=np.roll(beta,-1)-beta
            stored=.5*np.sum(core_k*v*v)+.5*seam_t*np.sum(d*d)+.5*seam_b*(db@db)
            energy=stored+mass/n*9.81*np.sum(centers[:,1]-centroid*np.cos(angle+beta))
            grad=core_k*v
            grad += mass/n*9.81*np.column_stack([np.cos(angle),np.sin(angle),centroid*np.sin(angle+beta)/rc])
            force=seam_t*d
            grad[:,0]+=np.einsum('ij,ij->i',force,e)
            grad[:,1]-=np.einsum('ij,ij->i',force,t)
            grad[:,2]-=np.einsum('ij,ij->i',force,spin(r0))/rc
            incoming=np.column_stack([-np.einsum('ij,ij->i',force,np.roll(e,-1,axis=0)),
                                      np.einsum('ij,ij->i',force,np.roll(t,-1,axis=0)),
                                      np.einsum('ij,ij->i',force,spin(r1))/rc])
            grad+=np.roll(incoming,1,axis=0)
            grad[:,2]+=seam_b*(2*beta-np.roll(beta,1)-np.roll(beta,-1))/rc
            return float(energy),grad.ravel(),float(stored)

        def contact(x):
            _,beta,centers,_,_,_=state(x)
            rot=rotate(offsets,beta[ids])
            gaps=height+centers[ids,1]+rot[:,1]
            jac=np.zeros_like(a)
            for j,i in enumerate(ids):
                jac[j,3*i:3*i+3]=[math.cos(angle[i]),math.sin(angle[i]),rot[j,0]/rc]
            return gaps,jac

        if self_test:
            probe=q+np.random.default_rng(15).normal(0,1e-4,ndof)
            energy,grad,_=energy_grad(probe)
            fd=np.zeros(ndof); cj=np.zeros_like(a); step=1e-7
            for j in range(ndof):
                plus,minus=probe.copy(),probe.copy();plus[j]+=step;minus[j]-=step
                fd[j]=(energy_grad(plus)[0]-energy_grad(minus)[0])/(2*step)
                cj[:,j]=(contact(plus)[0]-contact(minus)[0])/(2*step)
            assert np.max(np.abs(fd-grad))/max(1.,np.max(np.abs(grad)))<1e-6,'Energy gradient mismatch'
            assert np.max(np.abs(cj-contact(probe)[1]))<1e-7,'Contact Jacobian mismatch'
            v,beta,centers,_,_,_=state(probe)
            base=core_k*v+mass/n*9.81*np.column_stack([np.cos(angle),np.sin(angle),centroid*np.sin(angle+beta)/rc])
            internal=grad.reshape(n,3)-base
            for shift in [np.array([1.,0.]),np.array([0.,1.])]:
                dq=np.column_stack([-e@shift,t@shift,np.zeros(n)])
                assert abs(np.sum(internal*dq))<1e-7,'Internal force is not balanced'
            velocity=spin(centers)
            dq=np.column_stack([-np.einsum('ij,ij->i',e,velocity),np.einsum('ij,ij->i',t,velocity),np.full(n,rc)])
            assert abs(np.sum(internal*dq))<1e-7,'Internal moment is not balanced'

        result=minimize(lambda y:energy_grad(y/100)[0]/100,q*100,
                        jac=lambda y:energy_grad(y/100)[1]/10000,
                        constraints=[dict(type='ineq',fun=lambda y:contact(y/100)[0]*100,
                                          jac=lambda y:contact(y/100)[1])],
                        method='SLSQP',options=dict(ftol=1e-14,maxiter=1500))
        q=result.x/100
        gaps,jac=contact(q)
        _,grad,elastic=energy_grad(q)
        active=gaps<1e-7
        forces,residual=nnls(jac[active].T,grad,maxiter=10000)
        iteration=result.nit-1
        success=bool(result.success and gaps.min()>-1e-8 and residual/max(1.,forces.sum())<1e-5)
        if native_state:
            import csv
            qn=np.zeros(ndof)
            for row in csv.DictReader(open(native_state,encoding='utf-8-sig')):
                j=int(round(float(row['theta'])/(2*math.pi/n)-phase))%n
                qn[3*j:3*j+3]=[float(row['compression']),float(row['shear']),rc*float(row['rotation'])]
            ng,nj=contact(qn)
            _,gradient,stored=energy_grad(qn)
            free=[i for i in range(n) if ng[ids==i].min()>.005]
            native_debug=dict(native_energy_j=stored,native_min_gap_m=float(ng.min()),
                              native_free_gradient=gradient.reshape(n,3)[free].tolist(),
                              native_q=qn.reshape(n,3).tolist(),oracle_q=q.reshape(n,3).tolist())
    return dict(count=n,height_m=height,phase_fraction=phase,normal_n=float(forces.sum()),
                compression_m=float(q[0::3].max()),rotation_rad=float(np.abs(q[2::3]/rc).max()),
                energy_j=float(elastic),minimum_gap_m=float(gaps.min()),success=bool(success),
                equilibrium_residual_n=float(residual),iterations=iteration+1,**native_debug)


if __name__ == '__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--counts',type=int,nargs='+',default=[16,32,64])
    p.add_argument('--heights',type=float,nargs='+',default=[.14])
    p.add_argument('--phases',type=float,nargs='+',default=[0.,.5])
    p.add_argument('--config-root')
    p.add_argument('--native-state')
    p.add_argument('--self-test',action='store_true')
    args=p.parse_args()
    settings={}
    if args.config_root:
        from pathlib import Path
        root=Path(args.config_root)
        v=json.loads((root/'Vehicle3D.json').read_text(encoding='utf-8-sig'))
        tyre=json.loads((root/'Tyres.json').read_text(encoding='utf-8-sig'))
        settings=dict(radius=v['wheel_radius_m'],width=v['wheel_width_m'],depth=tyre['tread_depth_m'],
                      foundation=tyre['tread_foundation_n_per_m3'],mass=tyre['tread_mass_kg'],poisson=tyre.get('tread_poisson_ratio',.49))
    for h in args.heights:
        for n in args.counts:
            for phase in args.phases:
                print(json.dumps(solve(n,h,phase,native_state=args.native_state,self_test=args.self_test,**settings)),flush=True)
