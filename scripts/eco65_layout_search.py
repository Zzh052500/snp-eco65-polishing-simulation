#!/usr/bin/env python3
"""
eco65 底座摆放搜索 (2026-09-04): 在真实 /tool_paths 上扫 底座偏航/俯仰/平移,
找出让 sand_tcp 带姿态(位置+姿态)全部可达的机器人摆放。

模型: 工具路径位姿在 baselink 系。若把机器人绕 baselink z 额外偏航 ψ_extra(世界系),
则工件(含路径)在 baselink 里表现为绕 z 转 -ψ_extra。同理, 绕桌 x 俯仰 / 沿 x,z 平移
对应路径整体做逆变换。搜索这些相对变换, 对每条候选求 FK-位姿 IK。

用法: python3 eco65_layout_search.py
"""
import subprocess, re, math, sys
import numpy as np
from scipy.optimize import least_squares
import eco65_fk as fkmod

CHAINS = fkmod.CHAINS
def tf(xyz, rpy):
    M = np.eye(4); M[:3,:3] = fkmod.rpy_to_matrix(*rpy); M[:3,3] = xyz; return M
def q_to_R(x,y,z,w):
    R=np.eye(3)
    R[0,0]=1-2*(y*y+z*z); R[0,1]=2*(x*y-z*w);  R[0,2]=2*(x*z+y*w)
    R[1,0]=2*(x*y+z*w);  R[1,1]=1-2*(x*x+z*z); R[1,2]=2*(y*z-x*w)
    R[2,0]=2*(x*z-y*w);  R[2,1]=2*(y*z+x*w);   R[2,2]=1-2*(x*x+y*y)
    return R
def R_to_axisangle(R):
    cos=(np.trace(R)-1)/2; cos=max(-1,min(1,cos)); th=math.acos(cos)
    if th<1e-8: return np.zeros(3)
    v=np.array([R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1]])
    return v/(2*math.sin(th))*th
def rot(axis,ang):
    T=np.eye(4); R=np.eye(3); c=math.cos(ang); s=math.sin(ang); C=1-c
    x,y,z=axis
    R[0,0]=x*x*C+c; R[0,1]=x*y*C-z*s; R[0,2]=x*z*C+y*s
    R[1,0]=y*x*C+z*s; R[1,1]=y*y*C+c; R[1,2]=y*z*C-x*s
    R[2,0]=z*x*C-y*s; R[2,1]=z*y*C+x*s; R[2,2]=z*z*C+c
    T[:3,:3]=R; return T

def fk_all(q):
    T={"baselink":np.eye(4)}
    for i,(name,parent,child,oxyz,orpy,axis) in enumerate(CHAINS):
        T[child]=T[parent]@tf(oxyz,orpy)@fkmod.axis_rot(axis,q[i])
    T["Link6"]=T["Link6"]
    T["sand_tcp"]=T["Link6"]@tf((-0.149,-0.154,0.083),(0,math.radians(-90),math.radians(-135)))
    return T
LIMITS=[(-3.1067,3.1067),(-3.1067,2.3562),(-2.7925,2.5307),
        (-3.1067,3.1067),(-3.1067,3.1067),(-3.1067,3.1067)]
def _in(q,tol=1e-6): return all(lo-tol<=qi<=hi+tol for (lo,hi),qi in zip(LIMITS,q))

def pose_ik(tp,tR,frame="sand_tcp",seed=None,n_rand=60,max_nfev=2500):
    def resid(q):
        T=fk_all(q); p=T[frame][:3,3]; R=T[frame][:3,:3]
        e=list(p-np.array(tp)); e+=list(R_to_axisangle(tR.T@R))
        pen=0.0
        for (lo,hi),qi in zip(LIMITS,q):
            if qi<lo: pen+=(lo-qi)**2*1e4
            elif qi>hi: pen+=(qi-hi)**2*1e4
        return e+[pen]
    rng=np.random.default_rng(3)
    seeds=[]
    if seed is not None: seeds.append(np.array(seed))
    seeds.append(np.zeros(6))
    for _ in range(n_rand): seeds.append(np.array([rng.uniform(lo,hi) for lo,hi in LIMITS]))
    best=None;bestc=1e9
    for s in seeds:
        try: r=least_squares(resid,s,max_nfev=max_nfev,xtol=1e-9,ftol=1e-9,gtol=1e-9)
        except Exception: continue
        q=r.x
        if not _in(q): continue
        c=float(np.sum(np.array(r.fun[:6])**2))
        if c<bestc: bestc=c;best=q
    if best is None: return None,1e9,1e9
    T=fk_all(best); p=T[frame][:3,3]; R=T[frame][:3,:3]
    pe=np.linalg.norm(p-np.array(tp)); re_=math.degrees(np.linalg.norm(R_to_axisangle(tR.T@R)))
    return best,pe,re_

def fetch_tool_path():
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
        poses.append(dict(p=np.array([px,py,pz]),R=q_to_R(ox,oy,oz,ow)))
    return poses

def test_placement(poses, Trel, frame, seed_chase=True):
    """Trel: 施加到每个路径位姿的变换(P' = Trel P). 返回 可达点数/总数."""
    ok=0; prev=None; worst_rot=0; worst_pos=0
    for pt in poses:
        Tp=np.eye(4); Tp[:3,:3]=pt['R']; Tp[:3,3]=pt['p']
        Tq=Trel@Tp
        q,pe,re_=pose_ik(Tq[:3,3],Tq[:3,:3],frame=frame,seed=prev)
        if q is not None and pe<0.005 and re_<5.0:
            ok+=1; prev=q
        else:
            prev=q if q is not None else prev
            worst_rot=max(worst_rot,re_); worst_pos=max(worst_pos,pe)
    return ok,worst_pos,worst_rot

def main():
    frame=sys.argv[1] if len(sys.argv)>1 else "sand_tcp"
    poses=fetch_tool_path()
    n=len(poses)
    print(f"工具路径点: {n}   frame: {frame}")
    if not n:
        print("空路径"); return
    # 单轴扫描
    print("\n[1] 底座偏航扫描 (绕 baselink z 转, 正=机器人在世界系逆时针)")
    for deg in range(-140,150,10):
        Trel=rot((0,0,1),math.radians(-deg))
        ok,wp,wr=test_placement(poses,Trel,frame)
        flag="  <== 全可达" if ok==n else ""
        print(f"  yaw={deg:+4d}deg: 可达 {ok}/{n}" + (f"  最差pos {wp*1000:.0f}mm / rot {wr:.0f}deg" if ok<n else "") + flag)
    print("\n[2] 底座俯仰扫描 (绕 baselink x 转)")
    for deg in range(-30,35,5):
        Trel=rot((1,0,0),math.radians(-deg))
        ok,wp,wr=test_placement(poses,Trel,frame)
        flag="  <== 全可达" if ok==n else ""
        print(f"  pitch={deg:+3d}deg: 可达 {ok}/{n}" + (f"  最差pos {wp*1000:.0f}mm / rot {wr:.0f}deg" if ok<n else "") + flag)
    print("\n[3] 底座沿 x 平移扫描 (相对当前位置)")
    for dxm in np.arange(-0.3,0.31,0.1):
        Trel=tf((dxm,0,0),(0,0,0))
        ok,wp,wr=test_placement(poses,Trel,frame)
        flag="  <== 全可达" if ok==n else ""
        print(f"  dx={dxm:+.2f}m: 可达 {ok}/{n}" + (f"  最差pos {wp*1000:.0f}mm / rot {wr:.0f}deg" if ok<n else "") + flag)

if __name__=="__main__":
    main()
