#!/usr/bin/env python3
"""
测试最优布局对真实工具路径可达性的影响。
对比原始布局 vs 最优布局。
"""
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

def pose_ik_fast(tp, tR, seed=None):
    def loss(q):
        T = fk_all(q)
        p = T["sand_tcp"][:3,3]
        R = T["sand_tcp"][:3,:3]
        e = p - tp
        re_ = R_to_axisangle(tR.T @ R)
        return float(np.linalg.norm(e)**2 + np.linalg.norm(re_)**2)

    s = seed if seed is not None else np.zeros(6)
    bounds = LIMITS
    res = minimize(loss, s, method='L-BFGS-B', bounds=bounds, options=dict(maxiter=50, ftol=1e-6))

    if res.fun < 0.0025:
        T = fk_all(res.x)
        p = T["sand_tcp"][:3,3]
        R = T["sand_tcp"][:3,:3]
        pe = np.linalg.norm(p - tp)
        re_ = math.degrees(np.linalg.norm(R_to_axisangle(tR.T @ R)))
        return res.x, pe, re_
    return None, 1e9, 1e9

def test_placement(poses, Trel):
    ok = 0
    prev_q = None
    for pt in poses:
        Tp = np.eye(4)
        Tp[:3,:3] = pt['R']
        Tp[:3,3] = pt['p']
        Tq = Trel @ Tp
        q, pe, re_ = pose_ik_fast(Tq[:3,3], Tq[:3,:3], seed=prev_q)
        if q is not None and pe < 0.01 and re_ < 10.0:
            ok += 1
            prev_q = q
    return ok

def fetch_poses_from_file(path="/tmp/tool_paths_captured.txt"):
    with open(path) as f: txt = f.read()
    poses = []
    for b in re.split(r'^- position:',txt,flags=re.MULTILINE)[1:]:
        m = re.search(r'x:\s*([\d.\-e+]+).*?y:\s*([\d.\-e+]+).*?z:\s*([\d.\-e+]+).*?x:\s*([\d.\-e+]+).*?y:\s*([\d.\-e+]+).*?z:\s*([\d.\-e+]+).*?w:\s*([\d.\-e+]+)',b,re.DOTALL)
        if not m: continue
        px,py,pz,ox,oy,oz,ow = map(float,m.groups())
        poses.append(dict(p=np.array([px,py,pz]), R=q_to_R(ox,oy,oz,ow)))
    return poses

def main():
    poses = fetch_poses_from_file()
    n = len(poses)

    if n == 0:
        print("ERROR: 无工具路径数据。运行: ros2 topic echo /tool_paths > /tmp/tool_paths_captured.txt")
        return

    print(f"=== 工具路径可达性对比 ===\n")
    print(f"工具路径点数: {n}\n")

    # 原始布局 (0, 0, 0)
    Trel_orig = np.eye(4)
    ok_orig = test_placement(poses, Trel_orig)

    # 最优布局
    pitch = math.radians(10)
    yaw = math.radians(-140)
    dx = -0.40

    Rz = np.eye(4)
    c, s = math.cos(yaw), math.sin(yaw)
    Rz[:2,:2] = [[c, -s], [s, c]]

    Rx = np.eye(4)
    c, s = math.cos(pitch), math.sin(pitch)
    Rx[1:3,1:3] = [[c, -s], [s, c]]

    Tx = np.eye(4)
    Tx[0,3] = dx

    Trel_opt = Rz @ Rx @ Tx
    ok_opt = test_placement(poses, Trel_opt)

    print(f"原始布局 (x=-0.61, yaw=0°, pitch=0°):")
    print(f"  可达: {ok_orig}/{n} ({100*ok_orig/n:.1f}%)\n")

    print(f"最优布局 (x=-1.01, yaw=-140°, pitch=+10°):")
    print(f"  可达: {ok_opt}/{n} ({100*ok_opt/n:.1f}%)\n")

    improvement = ok_opt - ok_orig
    print(f"改善: {improvement:+d} 点 ({improvement/n*100:+.1f}%)")

if __name__ == "__main__":
    main()
