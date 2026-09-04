#!/usr/bin/env python3
"""
eco65 扫描轨迹设计: 数值 IK 搜索 + 轨迹生成。

思路:
  模拟模式下 reconstruction_sim_node 直接加载 part_scan.ply, 扫描轨迹是一条
  合法的演示运动。目标是让 sand_tcp(工具) 扫过工件上方区域, 轨迹从零位出发、
  绕工件移动、再回零位。

  用 scipy.optimize.least_squares 求解让 sand_tcp 到达工件上方目标点所需的关节角。
"""
import math
import numpy as np
from scipy.optimize import least_squares
import eco65_fk as fkmod

def rpy_to_matrix(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    Ry = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
    Rz = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
    return Rz @ Ry @ Rx

def tf(xyz, rpy):
    M = np.eye(4); M[:3,:3]=rpy_to_matrix(*rpy); M[:3,3]=xyz; return M

def axis_rot(axis, th):
    T=np.eye(4); R=np.eye(3); x,y,z=axis
    c=math.cos(th); s=math.sin(th); C=1-c
    R[0,0]=x*x*C+c; R[0,1]=x*y*C-z*s; R[0,2]=x*z*C+y*s
    R[1,0]=y*x*C+z*s; R[1,1]=y*y*C+c; R[1,2]=y*z*C-x*s
    R[2,0]=z*x*C-y*s; R[2,1]=z*y*C+x*s; R[2,2]=z*z*C+c
    T[:3,:3]=R; return T

CHAINS=fkmod.CHAINS

def fk(q):
    """返回 baselink 下各 frame 名->4x4."""
    T={"baselink":np.eye(4)}
    for i,(name,parent,child,oxyz,orpy,axis) in enumerate(CHAINS):
        T[child]=T[parent] @ tf(oxyz,orpy) @ axis_rot(axis,q[i])
    # sand_tcp (固定于 Link6)
    T["sand_tcp"]= T["Link6"] @ tf((-0.149,-0.154,0.083),(0,math.radians(-90),math.radians(-135)))
    T["camera_frame"]= T["Link6"] @ tf((0.0385,-0.09268,0.07127),(1.58196,2.32626,1.57371))
    return T

LIMITS=[(-3.1067,3.1067),(-3.1067,2.3562),(-2.7925,2.5307),
        (-3.1067,3.1067),(-3.1067,3.1067),(-3.1067,3.1067)]

def solve_ik(target_xyz, frame="sand_tcp", seed=None, verbose=False):
    """求 sand_tcp/camera_frame 到达 target_xyz 的关节角(最小二乘, 只约束位置)."""
    def resid(q):
        T=fk(q)
        p=T[frame][:3,3]
        # 惩罚远离关节极限(软约束)
        pen=0.0
        for i,(lo,hi) in enumerate(LIMITS):
            if q[i]<lo: pen+=(lo-q[i])**2*10
            elif q[i]>hi: pen+=(q[i]-hi)**2*10
        return list(p-target_xyz)+[pen]
    # 多个 seed 尝试
    seeds=[]
    if seed is not None: seeds.append(np.array(seed))
    # 基础种子: 各关节小幅变化
    for j in range(6):
        s=np.zeros(6); s[j]=0.5; seeds.append(s)
        s=np.zeros(6); s[j]=-0.5; seeds.append(s)
    best=None; best_cost=1e9
    for s in seeds:
        r=least_squares(resid,s,max_nfev=4000,xtol=1e-10,ftol=1e-10,gtol=1e-10)
        cost=np.sum(r.fun[:-1]**2)
        if cost<best_cost:
            best_cost=cost; best=r.x
    if verbose:
        print(f"  IK target {frame}->{target_xyz}: error={math.sqrt(best_cost)*1000:.1f} mm")
    return best, math.sqrt(best_cost)

def main():
    print("="*70)
    print("eco65 工件扫描可达性 & 轨迹设计")
    print("工件(part)中心在 baselink: (0.797, 0.010, 0.155)")
    print("  x∈[0.568,1.027]  y∈[-0.225,0.245]  z∈[0.087,0.227]")
    print("="*70)

    # 1. 探测: 相机能到工件上方哪些点
    targets=[
        ("工件中心上方",(0.797,0.010,0.42)),
        ("工件左前",(0.60,0.00,0.40)),
        ("工件右前",(0.90,0.00,0.40)),
        ("工件左侧",(0.80,-0.20,0.40)),
        ("工件右侧",(0.80,0.20,0.40)),
        ("工件近端",(1.00,0.00,0.40)),
    ]
    print("\n[相机 camera_frame 到工件上方各点]")
    qs={}
    for name,t in targets:
        q,err=solve_ik(t,frame="camera_frame",verbose=False)
        ok="OK" if err<0.03 else f"远于30mm(err={err*1000:.0f}mm)"
        print(f"  {name:8s} {t}: q={[round(x,3) for x in q]}  {ok}")
        # 校验关节极限
        for i,(lo,hi) in enumerate(LIMITS):
            if q[i]<lo-1e-6 or q[i]>hi+1e-6:
                print(f"    !! joint{i+1} OOB: {q[i]:.3f} not in [{lo:.3f},{hi:.3f}]")
        qs[name]=q

    # 2. 也试试 sand_tcp
    print("\n[sand_tcp(工具) 到工件上方]")
    for name,t in targets:
        q,err=solve_ik(t,frame="sand_tcp",verbose=False)
        print(f"  {name:8s} {t}: q={[round(x,3) for x in q]}  err={err*1000:.0f}mm")

if __name__=="__main__":
    main()
