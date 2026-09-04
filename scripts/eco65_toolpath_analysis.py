#!/usr/bin/env python3
"""
分析 eco65 对一条 tool path(位置+姿态约束)的可达性,
确定把工件平移多少(dx)能让整条 tool path 可达。

用法: 直接跑, 会 docker exec 抓取 /tool_paths。
"""
import subprocess, re, math, json
import numpy as np
from scipy.optimize import least_squares
import eco65_fk as fkmod

# --- eco65 链 & fk 带姿态 ---
CHAINS=fkmod.CHAINS
def tf(xyz,rpy):
    M=np.eye(4); M[:3,:3]=fkmod.rpy_to_matrix(*rpy); M[:3,3]=xyz; return M
def q_to_R(q):
    x,y,z,w=q
    R=np.eye(3)
    R[0,0]=1-2*(y*y+z*z); R[0,1]=2*(x*y-z*w);   R[0,2]=2*(x*z+y*w)
    R[1,0]=2*(x*y+z*w);   R[1,1]=1-2*(x*x+z*z); R[1,2]=2*(y*z-x*w)
    R[2,0]=2*(x*z-y*w);   R[2,1]=2*(y*z+x*w);   R[2,2]=1-2*(x*x+y*y)
    return R
def R_to_axisangle(R):
    # 返回 3-vector (旋转向量)
    cos=(np.trace(R)-1)/2
    cos=max(-1,min(1,cos))
    th=math.acos(cos)
    if th<1e-8: return np.zeros(3)
    v=np.array([R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1]])
    return v/(2*math.sin(th))*th
def fk_all(q):
    T={"baselink":np.eye(4)}
    for i,(name,parent,child,oxyz,orpy,axis) in enumerate(CHAINS):
        # 重算与 eco65_fk.fk_full 相同: T_parent @ T_origin @ Rot
        T[child]=T[parent]@tf(oxyz,orpy)@fkmod.axis_rot(axis,q[i])
    T["sand_tcp"]=T["Link6"]@tf((-0.149,-0.154,0.083),(0,math.radians(-90),math.radians(-135)))
    return T

LIMITS=[(-3.1067,3.1067),(-3.1067,2.3562),(-2.7925,2.5307),
        (-3.1067,3.1067),(-3.1067,3.1067),(-3.1067,3.1067)]

def pose_ik(target_pos, target_R, seed=None, max_nfev=6000):
    """求 sand_tcp 达到 target_pos + target_R 的关节角."""
    def resid(q):
        T=fk_all(q)
        p=T["sand_tcp"][:3,3]
        R=T["sand_tcp"][:3,:3]
        # 位置残差
        e_pos=list(p-np.array(target_pos))
        # 姿态残差 (轴角)
        Rerr=target_R.T@R
        e_rot=list(R_to_axisangle(Rerr))
        # 关节极限软惩罚
        pen=[]
        for lo,hi in LIMITS:
            pen.append(0.0)
        return e_pos+e_rot+pen
    best=None; bestc=1e9
    seeds=[]
    if seed is not None: seeds.append(np.array(seed))
    seeds.append(np.zeros(6))
    for j in range(6):
        s=np.zeros(6); s[j]=0.6; seeds.append(s)
        s=np.zeros(6); s[j]=-0.6; seeds.append(s)
    for s in seeds:
        try:
            r=least_squares(resid,s,max_nfev=max_nfev,xtol=1e-9,ftol=1e-9,gtol=1e-9)
        except Exception: continue
        # cost 只看前6维(位置3+姿态3)
        cost=np.sum(np.array(r.fun[:6])**2)
        if cost<bestc: bestc=cost; best=r.x
    return best, math.sqrt(bestc)

def main():
    # 抓 tool path
    out=subprocess.run(['docker','exec','snp_automate_2023_sim','bash','-lc',
        'source /opt/ros/jazzy/setup.bash; timeout 6 ros2 topic echo /tool_paths --once'],
        capture_output=True,text=True).stdout
    blocks=re.split(r'- position:',out)
    poses=[]
    for b in blocks[1:]:
        m=re.search(r'x:\s*(-?[\d.eE+]+)\n\s*y:\s*(-?[\d.eE+]+)\n\s*z:\s*(-?[\d.eE+]+)',b)
        if not m: continue
        px,py,pz=map(float,m.groups())
        o=re.search(r'orientation:\n\s*x:\s*(-?[\d.eE+]+)\n\s*y:\s*(-?[\d.eE+]+)\n\s*z:\s*(-?[\d.eE+]+)\n\s*w:\s*(-?[\d.eE+]+)',b)
        if not o: continue
        ox,oy,oz,ow=map(float,o.groups())
        poses.append(dict(p=np.array([px,py,pz]), R=q_to_R((ox,oy,oz,ow))))
    print(f"工具路径点数: {len(poses)}")
    if not poses: return
    xs=[p['p'][0] for p in poses]
    print(f"原始 x 范围: [{min(xs):.3f}, {max(xs):.3f}]")

    # 对每个候选平移 dx(负=移近), 测全部点带姿态可达率
    print("\n=== 平移 dx 后整条 tool path 可达性(位置+姿态) ===")
    for dx in [0.0,-0.1,-0.2,-0.25,-0.3,-0.35,-0.4,-0.45,-0.5]:
        ok=0; worst=0; worst_pt=None
        prev_q=None
        fail_reason=[]
        for pt in poses:
            tp=np.array([pt['p'][0]+dx, pt['p'][1], pt['p'][2]])
            q,err=pose_ik(tp, pt['R'], seed=prev_q)
            if err<0.02:
                ok+=1; prev_q=q
            else:
                worst=max(worst,err)
                if worst_pt is None or err>worst_pt[1]: worst_pt=(tp,err)
                prev_q=q if q is not None else prev_q
        rate=ok/len(poses)
        print(f"  dx={dx:+.2f}: 可达 {ok}/{len(poses)} ({rate*100:.0f}%)"
              + (f"  最差 {worst*1000:.0f}mm @ {[round(v,2) for v in worst_pt[0]]}" if worst_pt else ""))

if __name__=="__main__":
    main()
