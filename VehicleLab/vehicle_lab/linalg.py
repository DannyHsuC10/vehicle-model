"""Small-vector math and a damped constraint solver, using the standard library."""
import math


def add(a,b): return [x+y for x,y in zip(a,b)]
def sub(a,b): return [x-y for x,y in zip(a,b)]
def scale(a,s): return [x*s for x in a]
def dot(a,b): return sum(x*y for x,y in zip(a,b))
def norm(a): return math.sqrt(dot(a,a))
def cross(a,b): return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
def unit(a):
    n=norm(a)
    if n<1e-12: raise ValueError("Coincident points or degenerate axis")
    return scale(a,1/n)


def rotate(vector, rotvec):
    angle=norm(rotvec)
    if angle<1e-12: return list(vector)
    axis=scale(rotvec,1/angle)
    return add(add(scale(vector,math.cos(angle)),scale(cross(axis,vector),math.sin(angle))),
               scale(axis,dot(axis,vector)*(1-math.cos(angle))))


def transform(point, center, pose):
    return add(add(center,pose[:3]),rotate(sub(point,center),pose[3:]))


def solve_linear(matrix, rhs):
    size=len(rhs)
    a=[list(row)+[b] for row,b in zip(matrix,rhs)]
    for col in range(size):
        pivot=max(range(col,size),key=lambda r:abs(a[r][col]))
        if abs(a[pivot][col])<1e-18: raise ValueError("Singular geometry constraints")
        a[col],a[pivot]=a[pivot],a[col]
        factor=a[col][col]
        a[col]=[x/factor for x in a[col]]
        for row in range(size):
            if row!=col:
                factor=a[row][col]
                a[row]=[x-factor*y for x,y in zip(a[row],a[col])]
    return [row[-1] for row in a]


def solve_constraints(residual, initial, tolerance=2e-7, iterations=70):
    x=list(initial)
    damping=1e-8
    for _ in range(iterations):
        r=residual(x)
        error=max(map(abs,r))
        if error<tolerance: return x,error
        columns=[]
        for j in range(len(x)):
            trial=list(x)
            h=1e-6
            trial[j]+=h
            columns.append([(u-v)/h for u,v in zip(residual(trial),r)])
        n=len(x)
        normal=[[dot(columns[i],columns[j])+(damping if i==j else 0) for j in range(n)] for i in range(n)]
        delta=solve_linear(normal,[-dot(c,r) for c in columns])
        length=norm(delta)
        if length>.3: delta=scale(delta,.3/length)
        score=dot(r,r)
        for factor in (1.,.5,.25,.125,.0625):
            trial=add(x,scale(delta,factor))
            rr=residual(trial)
            if dot(rr,rr)<score:
                x=trial
                damping=max(1e-12,damping*.2)
                break
        else:
            damping*=20
    raise ValueError(f"Geometry has no converged solution (residual {error*1000:.3f} mm). Reduce travel or check mounts.")


def line_intersection(a,b,c,d):
    u=sub(b,a); w=sub(d,c)
    det=u[0]*w[1]-u[1]*w[0]
    if abs(det)<1e-10: return None
    t=((c[0]-a[0])*w[1]-(c[1]-a[1])*w[0])/det
    return add(a,scale(u,t))
