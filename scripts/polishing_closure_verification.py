#!/usr/bin/env python3
"""
ECO65 打磨闭环验证脚本。
验证优化布局下的完整打磨流程可行性。
"""
import re, math, json, numpy as np
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

def pose_ik(tp, tR, seed=None):
    """IK with multiple seeds for robustness."""
    def loss(q):
        T = fk_all(q)
        p = T["sand_tcp"][:3,3]
        R = T["sand_tcp"][:3,:3]
        e = p - tp
        re_ = R_to_axisangle(tR.T @ R)
        return float(np.linalg.norm(e)**2 + np.linalg.norm(re_)**2)

    # Try with seed first
    if seed is not None:
        res = minimize(loss, seed, method='L-BFGS-B', bounds=LIMITS,
                      options=dict(maxiter=100, ftol=1e-7))
        if res.fun < 1e-4:
            T = fk_all(res.x)
            p = T["sand_tcp"][:3,3]
            R = T["sand_tcp"][:3,:3]
            pe = np.linalg.norm(p - tp)
            re_ = math.degrees(np.linalg.norm(R_to_axisangle(tR.T @ R)))
            return res.x, pe, re_, res.fun

    # Try random seeds
    for _ in range(5):
        s = np.random.uniform(-np.pi, np.pi, 6)
        res = minimize(loss, s, method='L-BFGS-B', bounds=LIMITS,
                      options=dict(maxiter=100, ftol=1e-7))
        if res.fun < 1e-4:
            T = fk_all(res.x)
            p = T["sand_tcp"][:3,3]
            R = T["sand_tcp"][:3,:3]
            pe = np.linalg.norm(p - tp)
            re_ = math.degrees(np.linalg.norm(R_to_axisangle(tR.T @ R)))
            return res.x, pe, re_, res.fun

    return None, 1e9, 1e9, 1e9

def joint_distance(q1, q2):
    """Measure joint space distance."""
    return np.linalg.norm(q1 - q2) * 180 / np.pi  # degrees

def fetch_poses(path="/tmp/tool_paths_captured.txt"):
    with open(path) as f: txt = f.read()
    poses = []
    for b in re.split(r'^- position:',txt,flags=re.MULTILINE)[1:]:
        m = re.search(r'x:\s*([\d.\-e+]+).*?y:\s*([\d.\-e+]+).*?z:\s*([\d.\-e+]+).*?x:\s*([\d.\-e+]+).*?y:\s*([\d.\-e+]+).*?z:\s*([\d.\-e+]+).*?w:\s*([\d.\-e+]+)',b,re.DOTALL)
        if not m: continue
        px,py,pz,ox,oy,oz,ow = map(float,m.groups())
        poses.append(dict(p=np.array([px,py,pz]), R=q_to_R(ox,oy,oz,ow)))
    return poses

def simulate_polishing(poses):
    """Full polishing trajectory simulation."""
    print("=== 打磨流程闭环验证 ===\n")
    print(f"总路径点数: {len(poses)}\n")

    results = []
    prev_q = None
    reachable = 0
    unreachable = []
    large_jumps = []

    for i, pt in enumerate(poses):
        Tp = np.eye(4)
        Tp[:3,:3] = pt['R']
        Tp[:3,3] = pt['p']

        q, pe, re_, loss = pose_ik(Tp[:3,3], Tp[:3,:3], seed=prev_q)

        if q is not None and pe < 0.005 and re_ < 5.0:  # 5mm + 5deg
            reachable += 1
            results.append(dict(
                idx=i, status='OK', q=q, pe=pe, re_=re_,
                p=Tp[:3,3], jjump=0 if prev_q is None else joint_distance(q, prev_q)
            ))
            if prev_q is not None:
                jjump = joint_distance(q, prev_q)
                if jjump > 45:  # Large jump
                    large_jumps.append((i, jjump))
            prev_q = q
        else:
            unreachable.append(i)
            results.append(dict(
                idx=i, status='UNREACHABLE', pe=pe, re_=re_
            ))

    # Statistics
    print(f"可达点: {reachable}/{len(poses)} ({100*reachable/len(poses):.1f}%)")
    print(f"不可达点: {len(unreachable)}")
    if unreachable:
        print(f"  位置: {unreachable[:10]}" + ("..." if len(unreachable) > 10 else ""))

    if large_jumps:
        print(f"\n大关节跳变(>45°): {len(large_jumps)}")
        for idx, jjump in large_jumps[:5]:
            print(f"  点 {idx}: {jjump:.1f}°")

    # Coverage
    if reachable > 0:
        avg_pe = np.mean([r['pe'] for r in results if r['status']=='OK'])
        avg_re = np.mean([r['re_'] for r in results if r['status']=='OK'])
        avg_jjump = np.mean([r['jjump'] for r in results if r['status']=='OK' and r['jjump']>0])
        print(f"\n均值 (可达点):")
        print(f"  位置误差: {avg_pe*1000:.2f} mm")
        print(f"  姿态误差: {avg_re:.2f}°")
        print(f"  关节跳变: {avg_jjump:.1f}°")

    print(f"\n验证结论: {'✓ 打磨可行' if reachable > len(poses)*0.8 else '✗ 可达性不足'}")
    return results

def main():
    try:
        poses = fetch_poses()
        if not poses:
            print("ERROR: 无工具路径数据")
            return
        simulate_polishing(poses)
    except Exception as e:
        print(f"ERROR: {e}")

if __name__ == "__main__":
    main()
