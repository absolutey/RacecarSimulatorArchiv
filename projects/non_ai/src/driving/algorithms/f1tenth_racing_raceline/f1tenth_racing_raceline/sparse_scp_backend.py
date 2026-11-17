"""Experimental sparse convex SCP QP kernel; NOT an automatically qualified R-A3.

Uses sparse normal equations LU rather than dense inversion. Its output must pass
nonlinear joint path/speed validation before use in any trajectory/Reference.
"""
import math
import numpy as np


def solve_sparse_qp(hessian, gradient, constraints, bounds_low, bounds_high,
                    *, rho, tolerance, iterations, initial):
    from scipy import sparse
    from scipy.sparse.linalg import splu
    g=np.asarray(gradient,dtype=float).reshape(-1)
    h=sparse.csc_matrix(hessian,dtype=float)
    a=sparse.csc_matrix(constraints[0],dtype=float)
    b=np.asarray(constraints[1],dtype=float).reshape(-1)
    lo=np.asarray(bounds_low,dtype=float).reshape(-1)
    hi=np.asarray(bounds_high,dtype=float).reshape(-1)
    x=np.asarray(initial,dtype=float).reshape(-1)
    n=len(x)
    if h.shape!=(n,n) or a.shape[1]!=n or len(b)!=a.shape[0] or any(len(v)!=n for v in (g,lo,hi)):
        raise ValueError('sparse QP dimensions do not match')
    if not (np.isfinite(rho) and rho>0 and np.isfinite(tolerance) and tolerance>0 and iterations>0):
        raise ValueError('invalid sparse solver settings')
    if not all(np.isfinite(v).all() for v in (h.data,a.data,b,lo,hi,g,x)):
        raise ValueError('nonfinite sparse optimizer inputs')
    if np.any(lo>hi):raise ValueError('inverted sparse QP bounds')
    x=np.clip(x,lo,hi)
    rho=float(rho)
    # Physical dynamics and offset coupling are kept sparse via A.T @ A.
    lu=splu((h+rho*(a.T@a)+rho*sparse.identity(n,format='csc')).tocsc())
    z=np.minimum(a@x,b);y=x.copy()
    dz=np.zeros(len(b));dy=np.zeros(n)
    primal=dual=math.inf
    for iteration in range(1,iterations+1):
        x=lu.solve(-g+rho*(a.T@(z-dz))+rho*(y-dy))
        prev_z=z.copy();prev_y=y.copy()
        z=np.minimum(a@x+dz,b)
        y=np.clip(x+dy,lo,hi)
        dz+=a@x-z;dy+=x-y
        primal=max(float(np.max(np.abs(a@x-z))),float(np.max(np.abs(x-y))))
        dual=rho*max(float(np.max(np.abs(a.T@(z-prev_z)))),float(np.max(np.abs(y-prev_y))))
        ps=max(1.,float(np.max(np.abs(a@x))),float(np.max(np.abs(z))),float(np.max(np.abs(x))),float(np.max(np.abs(y))))
        ds=max(1.,float(np.max(np.abs(g))),float(np.max(np.abs(rho*a.T@dz))),float(np.max(np.abs(rho*dy))))
        if not np.isfinite(primal+dual):raise RuntimeError('nonfinite sparse iterate')
        if max(primal/ps,dual/ds)<=tolerance:
            if np.any(a@y>b+max(1e-6,tolerance*10)):
                raise RuntimeError('sparse QP output not primal feasible')
            return y,iteration
    raise RuntimeError(f'sparse QP did not converge (p={primal},d={dual})')


def locally_sparse_jacobian(geometry, corridor, offsets, epsilon, *, stencil_radius=3, tolerance=1e-8):
    """Finite-difference path geometry with explicit locality verification.

    Rejects geometry whose derivatives extend beyond the assumed stencil.
    This is an O(N^2) time / O(N) retained-derivative-memory reference
    implementation, NOT a fully optimized O(N) differentiation algorithm.
    """
    from scipy import sparse
    offsets=np.asarray(offsets,dtype=float)
    _,curvature,lengths,_=geometry(corridor,offsets)
    n=len(offsets);r=[];c=[];val_curv=[];val_len=[]
    for j in range(n):
        shifted=offsets.copy()
        direction=1. if offsets[j]+epsilon<=corridor.upper_offsets[j] else -1.
        shifted[j]+=direction*epsilon
        _,curv2,len2,_=geometry(corridor,shifted)
        dc=(curv2-curvature)/(direction*epsilon)
        dl=(len2-lengths)/(direction*epsilon)
        allowed={(j+i)%n for i in range(-stencil_radius,stencil_radius+1)}
        if any(abs(dc[k])>tolerance or abs(dl[k])>tolerance for k in range(n) if k not in allowed):
            raise ValueError('nonlocal geometry derivative: cannot use sparse stencil')
        for k in sorted(allowed):
            r.append(k);c.append(j);val_curv.append(dc[k]);val_len.append(dl[k])
    k=(n,n)
    return curvature,lengths,sparse.csc_matrix((val_curv,(r,c)),shape=k),sparse.csc_matrix((val_len,(r,c)),shape=k)
