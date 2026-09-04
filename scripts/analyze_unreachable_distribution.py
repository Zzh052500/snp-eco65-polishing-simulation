#!/usr/bin/env python3
"""分析不可达点的空间分布."""
import re, math, numpy as np
from scipy.optimize import minimize
import eco65_fk as fkmod

CHAINS = fkmod.CHAINS
LIMITS = np.array([(-3.1067,3.1067),(-3.1067,2.3562),(-2.7925,2.5307),
                    (-3.1067,3.1067),(-3.1067,3.1067),(-3.1067,3.1067)])

def tf(xyz, rpy):
    M = np.eye(4)
    M[:3,:3] = fkmod.rpy_to_matrix(*rpy)
    M[:3,3] = xyz
    return M

def q_to_R(x,y,z,w):
    R = np.eye(3)
    R[0,0] = 1-2*(y*y+z*z); R[0,1] = 2*(x*y-z*w);  R[0,2] = 2*(x*z+y*w)
    R[1,0] = 2*(x*y+z*w);  R[1,1] = 1-2*(x*x+z*z); R[1,2] = 2*(y*z-x*w)
    R[2,0] = 2*(x*z-y*w);  R[2,1] = 2*(y*z+x*w);   R[2,2] = 1-2*(x*x+y*y)
    return R

def fk_all(q):
    T = {"baselink": np.eye(4)}
    for i,(name,parent,child,oxyz,orpy,axis) in enumerate(CHAINS):
        T[child] = T[parent] @ tf(oxyz,orpy) @ fkmod.axis_rot(axis,q[i])
    T["sand_tcp"] = T["Link6"] @ tf((-0.149,-0.154,0.083),(0,math.radians(-90),math.radians(-135)))
    return T

def R_to_axisangle(R):
    cos = (np.trace(R)-1)/2
    cos = max(-1, min(1, cos))
    th = math.acos(cos)
    if th < 1e-8: return np.zeros(3)
    v = np.array([R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1]])
    return v/(2*math.sin(th))*th

def pose_ik(tp, tR):
    def loss(q):
        T = fk_all(q)
        p = T["sand_tcp"][:3,3]
        R = T["sand_tcp"][:3,:3]
        e = p - tp
        re_ = R_to_axisangle(tR.T @ R)
        return float(np.linalg.norm(e)**2 + np.linalg.norm(re_)**2)

    best_q = None
    best_loss = 1e9
    for _ in range(10):
        s = np.random.uniform(-np.pi, np.pi, 6)
        res = minimize(loss, s, method='L-BFGS-B', bounds=LIMITS,
                      options=dict(maxiter=100, ftol=1e-7))
        if res.fun < best_loss:
            best_loss = res.fun
            best_q = res.x
    return best_q, best_loss

def fetch_poses(path="/tmp/tool_paths_captured.txt"):
    with open(path) as f: txt = f.read()
    poses = []
    for b in re.split(r'^- position:',txt,flags=re.MULTILINE)[1:]:
        m = re.search(r'x:\s*([\d.\-e+]+).*?y:\s*([\d.\-e+]+).*?z:\s*([\d.\-e+]+).*?x:\s*([\d.\-e+]+).*?y:\s*([\d.\-e+]+).*?z:\s*([\d.\-e+]+).*?w:\s*([\d.\-e+]+)',b,re.DOTALL)
        if not m: continue
        px,py,pz,ox,oy,oz,ow = map(float,m.groups())
        poses.append(dict(p=np.array([px,py,pz]), R=q_to_R(ox,oy,oz,ow), idx=len(poses)))
    return poses

def main():
    poses = fetch_poses()
    print(f"总路径点: {len(poses)}\n")

    reachable = []
    unreachable = []

    for pt in poses:
        Tp = np.eye(4)
        Tp[:3,:3] = pt['R']
        Tp[:3,3] = pt['p']
        q, loss = pose_ik(Tp[:3,3], Tp[:3,:3])

        if q is not None and loss < 1e-4:
            reachable.append(pt)
        else:
            unreachable.append(pt)

    print(f"可达: {len(reachable)}, 不可达: {len(unreachable)}\n")

    if unreachable:
        pos = np.array([p['p'] for p in unreachable])
        print("不可达点的位置统计:")
        print(f"  X: [{pos[:,0].min():.3f}, {pos[:,0].max():.3f}] mean={pos[:,0].mean():.3f}")
        print(f"  Y: [{pos[:,1].min():.3f}, {pos[:,1].max():.3f}] mean={pos[:,1].mean():.3f}")
        print(f"  Z: [{pos[:,2].min():.3f}, {pos[:,2].max():.3f}] mean={pos[:,2].mean():.3f}")

        # 聚类分析
        if len(unreachable) > 5:
            # X 范围分组
            x_min, x_max = pos[:,0].min(), pos[:,0].max()
            x_bins = [x_min, (x_min+x_max)/2, x_max]
            for i in range(len(x_bins)-1):
                cnt = np.sum((pos[:,0]>=x_bins[i]) & (pos[:,0]<x_bins[i+1]))
                print(f"  X∈[{x_bins[i]:.3f},{x_bins[i+1]:.3f}]: {cnt}个不可达点")

    if reachable:
        pos = np.array([p['p'] for p in reachable])
        print("\n可达点的位置统计:")
        print(f"  X: [{pos[:,0].min():.3f}, {pos[:,0].max():.3f}] mean={pos[:,0].mean():.3f}")
        print(f"  Y: [{pos[:,1].min():.3f}, {pos[:,1].max():.3f}] mean={pos[:,1].mean():.3f}")
        print(f"  Z: [{pos[:,2].min():.3f}, {pos[:,2].max():.3f}] mean={pos[:,2].mean():.3f}")

if __name__ == "__main__":
    main()
