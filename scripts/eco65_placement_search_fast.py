#!/usr/bin/env python3
"""
快速底座摆放搜索(粗粒度)。
粗糙网格 + 放松阈值 + L-BFGS-B 快速优化。
"""
import re, math, numpy as np
from scipy.optimize import minimize
import eco65_fk as fkmod

CHAINS = fkmod.CHAINS
LIMITS=np.array([(-3.1067,3.1067),(-3.1067,2.3562),(-2.7925,2.5307),
                  (-3.1067,3.1067),(-3.1067,3.1067),(-3.1067,3.1067)])

def tf(xyz, rpy):
    M = np.eye(4); M[:3,:3] = fkmod.rpy_to_matrix(*rpy); M[:3,3] = xyz; return M
def q_to_R(x,y,z,w):
    R=np.eye(3)
    R[0,0]=1-2*(y*y+z*z); R[0,1]=2*(x*y-z*w);  R[0,2]=2*(x*z+y*w)
    R[1,0]=2*(x*y+z*w);  R[1,1]=1-2*(x*x+z*z); R[1,2]=2*(y*z-x*w)
    R[2,0]=2*(x*z-y*w);  R[2,1]=2*(y*z+x*w);   R[2,2]=1-2*(x*x+y*y)
    return R
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

def R_to_axisangle(R):
    cos=(np.trace(R)-1)/2; cos=max(-1,min(1,cos)); th=math.acos(cos)
    if th<1e-8: return np.zeros(3)
    v=np.array([R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1]])
    return v/(2*math.sin(th))*th

def pose_ik_fast(tp, tR, seed=None):
    """极速 IK:单种子 L-BFGS-B."""
    def loss(q):
        T=fk_all(q); p=T["sand_tcp"][:3,3]; R=T["sand_tcp"][:3,:3]
        e=p-tp; re_=R_to_axisangle(tR.T@R)
        return float(np.linalg.norm(e)**2 + np.linalg.norm(re_)**2)

    s=seed if seed is not None else np.zeros(6)
    bounds=LIMITS
    res=minimize(loss,s,method='L-BFGS-B',bounds=bounds,options=dict(maxiter=50,ftol=1e-6))

    if res.fun<0.0025:  # 50mm pos + 10° rot squared
        T=fk_all(res.x); p=T["sand_tcp"][:3,3]; R=T["sand_tcp"][:3,:3]
        pe=np.linalg.norm(p-tp); re_=math.degrees(np.linalg.norm(R_to_axisangle(tR.T@R)))
        return res.x,pe,re_
    return None,1e9,1e9

def test_placement(poses, Trel):
    """粗快速测试:放松阈值."""
    ok=0; prev_q=None
    for pt in poses:
        Tp=np.eye(4); Tp[:3,:3]=pt['R']; Tp[:3,3]=pt['p']
        Tq=Trel@Tp
        q,pe,re_=pose_ik_fast(Tq[:3,3],Tq[:3,:3],seed=prev_q)
        if q is not None and pe<0.01 and re_<10.0:  # 10mm + 10deg
            ok+=1; prev_q=q
    return ok

def fetch_poses_from_file(path="/tmp/tool_paths_captured.txt"):
    with open(path) as f: txt=f.read()
    poses=[]
    for b in re.split(r'^- position:',txt,flags=re.MULTILINE)[1:]:
        m=re.search(r'x:\s*([\d.\-e+]+).*?y:\s*([\d.\-e+]+).*?z:\s*([\d.\-e+]+).*?x:\s*([\d.\-e+]+).*?y:\s*([\d.\-e+]+).*?z:\s*([\d.\-e+]+).*?w:\s*([\d.\-e+]+)',b,re.DOTALL)
        if not m: continue
        px,py,pz,ox,oy,oz,ow=map(float,m.groups())
        poses.append(dict(p=np.array([px,py,pz]),R=q_to_R(ox,oy,oz,ow)))
    return poses

def main():
    poses=fetch_poses_from_file()
    n=len(poses)
    print(f"工具路径: {n} 点 (粗快速扫描: 10mm/10deg 阈值)\n")

    # 偏航网格:10deg 步长
    print("=== 底座偏航(±150°, 10° 步长) ===")
    best_yaw=0; best_ok=0
    for deg in range(-150,151,10):
        Trel=rot((0,0,1),math.radians(-deg))
        ok=test_placement(poses,Trel)
        print(f"  yaw={deg:+4d}°: {ok:3d}/{n}")
        if ok>best_ok: best_yaw=deg; best_ok=ok
    print(f"\n最佳偏航: {best_yaw}° → {best_ok}/{n} ({100*best_ok/n:.0f}%)\n")

    # 俯仰网格:10deg 步长
    print("=== 底座俯仰(±40°, 10° 步长) ===")
    best_pitch=0; best_ok_pitch=0
    for deg in range(-40,41,10):
        Trel=rot((1,0,0),math.radians(-deg))
        ok=test_placement(poses,Trel)
        print(f"  pitch={deg:+3d}°: {ok:3d}/{n}")
        if ok>best_ok_pitch: best_pitch=deg; best_ok_pitch=ok
    print(f"\n最佳俯仰: {best_pitch}° → {best_ok_pitch}/{n}\n")

    # 平移网格:0.1m 步长
    print("=== 底座 x 平移(±0.4m, 0.1m 步长) ===")
    best_dx=0; best_ok_dx=0
    for dxm in np.arange(-0.4,0.45,0.1):
        Trel=tf((dxm,0,0),(0,0,0))
        ok=test_placement(poses,Trel)
        print(f"  dx={dxm:+.2f}m: {ok:3d}/{n}")
        if ok>best_ok_dx: best_dx=dxm; best_ok_dx=ok
    print(f"\n最佳平移: {best_dx:+.2f}m → {best_ok_dx}/{n}")

if __name__=="__main__":
    main()
