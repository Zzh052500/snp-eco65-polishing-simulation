#!/usr/bin/env python3
"""
快速启动: 若 pose_ik 严格分析出可达率接近 0,立刻从文件读 120 点路径,
跑底座摆放搜索(偏航/俯仰/平移扫),找可达布局。
"""
import re, math, numpy as np
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
def rot(axis, ang):
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
    T["sand_tcp"]=T["Link6"]@tf((-0.149,-0.154,0.083),(0,math.radians(-90),math.radians(-135)))
    return T
LIMITS=[(-3.1067,3.1067),(-3.1067,2.3562),(-2.7925,2.5307),
        (-3.1067,3.1067),(-3.1067,3.1067),(-3.1067,3.1067)]
def _in(q,tol=1e-6): return all(lo-tol<=qi<=hi+tol for (lo,hi),qi in zip(LIMITS,q))

def pose_ik(tp,tR,frame="sand_tcp",seed=None,n_rand=40,max_nfev=2000):
    """快速版本: 减少种子数加快搜索."""
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

def fetch_poses_from_file(path="/tmp/tool_paths_captured.txt"):
    """从已捕获的 /tool_paths 消息解析位姿."""
    with open(path) as f: txt=f.read()
    poses=[]
    for b in re.split(r'^- position:',txt,flags=re.MULTILINE)[1:]:
        m=re.search(r'x:\s*([\d.\-e+]+).*?y:\s*([\d.\-e+]+).*?z:\s*([\d.\-e+]+).*?x:\s*([\d.\-e+]+).*?y:\s*([\d.\-e+]+).*?z:\s*([\d.\-e+]+).*?w:\s*([\d.\-e+]+)',b,re.DOTALL)
        if not m: continue
        px,py,pz,ox,oy,oz,ow=map(float,m.groups())
        poses.append(dict(p=np.array([px,py,pz]),R=q_to_R(ox,oy,oz,ow)))
    return poses

def test_placement(poses, Trel, frame="sand_tcp"):
    """测 Trel 变换后全部路径的可达性."""
    ok=0; prev=None
    for pt in poses:
        Tp=np.eye(4); Tp[:3,:3]=pt['R']; Tp[:3,3]=pt['p']
        Tq=Trel@Tp
        q,pe,re_=pose_ik(Tq[:3,3],Tq[:3,:3],frame=frame,seed=prev)
        if q is not None and pe<0.005 and re_<5.0:
            ok+=1; prev=q
        else:
            prev=q if q is not None else prev
    return ok

def main():
    poses=fetch_poses_from_file()
    n=len(poses)
    print(f"从文件读取工具路径: {n} 点\n")

    print("=== 底座偏航扫描 (绕 baselink z) ===")
    best_yaw=0; best_ok=0
    for deg in range(-150,151,5):
        Trel=rot((0,0,1),math.radians(-deg))
        ok=test_placement(poses,Trel)
        if ok>best_ok: best_yaw=deg; best_ok=ok
        if ok>n*0.8:
            print(f"  yaw={deg:+4d}deg: {ok:3d}/{n} ✓✓✓")
        elif ok>n*0.5:
            print(f"  yaw={deg:+4d}deg: {ok:3d}/{n} ✓")
        elif ok%50==0:
            print(f"  yaw={deg:+4d}deg: {ok:3d}/{n}")
    print(f"最佳偏航: {best_yaw}° → {best_ok}/{n}")

    print("\n=== 底座俯仰扫描 (绕 baselink x) ===")
    best_pitch=0; best_ok=0
    for deg in range(-40,41,5):
        Trel=rot((1,0,0),math.radians(-deg))
        ok=test_placement(poses,Trel)
        if ok>best_ok: best_pitch=deg; best_ok=ok
        if ok>n*0.8:
            print(f"  pitch={deg:+3d}deg: {ok:3d}/{n} ✓✓✓")
        elif ok>n*0.5:
            print(f"  pitch={deg:+3d}deg: {ok:3d}/{n} ✓")
        elif ok%50==0:
            print(f"  pitch={deg:+3d}deg: {ok:3d}/{n}")
    print(f"最佳俯仰: {best_pitch}° → {best_ok}/{n}")

    print("\n=== 底座沿 x 平移扫描 ===")
    best_dx=0; best_ok=0
    for dxm in np.arange(-0.4,0.41,0.05):
        Trel=tf((dxm,0,0),(0,0,0))
        ok=test_placement(poses,Trel)
        if ok>best_ok: best_dx=dxm; best_ok=ok
        if ok>n*0.8:
            print(f"  dx={dxm:+.2f}m: {ok:3d}/{n} ✓✓✓")
        elif ok>n*0.5:
            print(f"  dx={dxm:+.2f}m: {ok:3d}/{n} ✓")
        elif ok%50==0:
            print(f"  dx={dxm:+.2f}m: {ok:3d}/{n}")
    print(f"最佳平移: {best_dx:+.2f}m → {best_ok}/{n}")

if __name__=="__main__":
    main()
