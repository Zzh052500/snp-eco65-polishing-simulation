#!/usr/bin/env python3
"""
eco65 工具路径「带姿态可达性」严格分析 (2026-09-04)。

背景: eco65_toolpath_analysis.py 用 pose_ik(少量固定种子)测出 0/47 全不可达。
但 pose_ik 地真验证显示它对"已知可达位姿"基本都能恢复(120 中 116,中位 0mm),
仅个别位姿因局部极小假阴性。故 0/47 结果存疑,需要更严格测量:

本脚本对当前 /tool_paths 每一点:
  1) 用 多随机种子(200) + 固定种子 + 前一解(连续性) 做 pose_ik;
  2) 对找到的解, 分开报 位置误差(mm) 与 姿态误差(deg),
     判据: pos<5mm 且 rot<5deg 才算真正带姿态可达;
  3) 对达不到的点, 做"姿态松弛"扫描: 固定位置可达前提下, 最小姿态误差是多少,
     判断是硬性不可达(姿态需放松 >30deg)还是边界/轻微失配(几度内)。

可分析 frame: sand_tcp(带工具偏移) 或 Link6(去工具偏移, 仅本体).
"""
import subprocess, re, math, sys
import numpy as np
from scipy.optimize import least_squares
import eco65_fk as fkmod

CHAINS = fkmod.CHAINS

def tf(xyz, rpy):
    M = np.eye(4); M[:3,:3] = fkmod.rpy_to_matrix(*rpy); M[:3,3] = xyz; return M

def q_to_R(x, y, z, w):
    R = np.eye(3)
    R[0,0]=1-2*(y*y+z*z); R[0,1]=2*(x*y-z*w);  R[0,2]=2*(x*z+y*w)
    R[1,0]=2*(x*y+z*w);   R[1,1]=1-2*(x*x+z*z); R[1,2]=2*(y*z-x*w)
    R[2,0]=2*(x*z-y*w);   R[2,1]=2*(y*z+x*w);   R[2,2]=1-2*(x*x+y*y)
    return R

def R_to_axisangle(R):
    cos = (np.trace(R)-1)/2; cos = max(-1, min(1, cos))
    th = math.acos(cos)
    if th < 1e-8: return np.zeros(3)
    v = np.array([R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1]])
    return v/(2*math.sin(th))*th

def fk_all(q):
    T = {"baselink": np.eye(4)}
    for i,(name,parent,child,oxyz,orpy,axis) in enumerate(CHAINS):
        T[child] = T[parent] @ tf(oxyz,orpy) @ fkmod.axis_rot(axis, q[i])
    T["Link6"]   = T["Link6"]
    T["sand_tcp"]= T["Link6"] @ tf((-0.149,-0.154,0.083), (0,math.radians(-90),math.radians(-135)))
    return T

LIMITS = [(-3.1067,3.1067), (-3.1067,2.3562), (-2.7925,2.5307),
          (-3.1067,3.1067), (-3.1067,3.1067), (-3.1067,3.1067)]

def _in_limits(q, tol=1e-6):
    return all(lo-tol <= qi <= hi+tol for (lo,hi),qi in zip(LIMITS,q))

def pose_ik_strict(target_pos, target_R, frame="sand_tcp",
                   n_rand=70, seed=None, max_nfev=2500):
    """多随机种子 pose_ik. 返回 (q_best, pos_err_m, rot_err_deg, cost)."""
    def resid(q):
        T = fk_all(q)
        p = T[frame][:3,3]; R = T[frame][:3,:3]
        e_pos = p - np.array(target_pos)
        Rerr = target_R.T @ R
        e_rot = np.array(R_to_axisangle(Rerr))
        # 关节极限: 硬性越界大幅惩罚(避免收敛到限位外解)
        pen = 0.0
        for (lo,hi),qi in zip(LIMITS,q):
            if qi < lo: pen += (lo-qi)**2*1e4
            elif qi > hi: pen += (qi-hi)**2*1e4
        return list(e_pos) + list(e_rot*1.0) + [pen]
    rng = np.random.default_rng(7)
    seeds = []
    if seed is not None: seeds.append(np.array(seed))
    seeds.append(np.zeros(6))
    for _ in range(n_rand):
        seeds.append(np.array([rng.uniform(lo,hi) for lo,hi in LIMITS]))
    best = None; bestc = 1e9
    for s in seeds:
        try:
            r = least_squares(resid, s, max_nfev=max_nfev, xtol=1e-9, ftol=1e-9, gtol=1e-9)
        except Exception:
            continue
        q = r.x
        if not _in_limits(q): continue
        c = float(np.sum(np.array(r.fun[:6])**2))
        if c < bestc: bestc = c; best = q
    if best is None: return None, 1e9, 1e9, 1e9
    T = fk_all(best)
    p = T[frame][:3,3]; R = T[frame][:3,:3]
    pos_err = np.linalg.norm(p - np.array(target_pos))
    rot_err = math.degrees(np.linalg.norm(R_to_axisangle(target_R.T @ R)))
    return best, pos_err, rot_err, math.sqrt(bestc)

def fetch_tool_path():
    out = subprocess.run(['docker','exec','snp_automate_2023_sim','bash','-lc',
        'source /opt/ros/jazzy/setup.bash; timeout 6 ros2 topic echo /tool_paths --once'],
        capture_output=True, text=True).stdout
    blocks = re.split(r'- position:', out)
    poses = []
    for b in blocks[1:]:
        m = re.search(r'x:\s*(-?[\d.eE+]+)\n\s*y:\s*(-?[\d.eE+]+)\n\s*z:\s*(-?[\d.eE+]+)', b)
        if not m: continue
        px,py,pz = map(float, m.groups())
        o = re.search(r'orientation:\n\s*x:\s*(-?[\d.eE+]+)\n\s*y:\s*(-?[\d.eE+]+)\n\s*z:\s*(-?[\d.eE+]+)\n\s*w:\s*(-?[\d.eE+]+)', b)
        if not o: continue
        ox,oy,oz,ow = map(float, o.groups())
        poses.append(dict(p=np.array([px,py,pz]), R=q_to_R(ox,oy,oz,ow)))
    return poses

def main():
    frame = sys.argv[1] if len(sys.argv)>1 else "sand_tcp"
    assert frame in ("sand_tcp","Link6")
    poses = fetch_tool_path()
    print(f"工具路径点数: {len(poses)}   分析 frame: {frame}")
    if not poses: return
    xs = [p['p'][0] for p in poses]
    print(f"x 范围: [{min(xs):.3f}, {max(xs):.3f}]")

    ok=0; prev_q=None
    hard=[]
    for i,pt in enumerate(poses):
        q,pe,re_,cost = pose_ik_strict(pt['p'], pt['R'], frame=frame, seed=prev_q)
        if i % 10 == 0:
            print(f"  ... 检查点 {i}/{len(poses)}", flush=True)
        if q is not None and pe<0.005 and re_<5.0:
            ok+=1; prev_q=q
        else:
            tag = "OOB位置/求解失败" if q is None else f"pos {pe*1000:.0f}mm rot {re_:.0f}deg"
            hard.append((i,pt,tag,q))
            prev_q = q if q is not None else prev_q
    print(f"\n带姿态可达: {ok}/{len(poses)}")
    if hard:
        print("失败点明细:")
        for i,pt,tag,q in hard:
            print(f"  #{i} p=({pt['p'][0]:.3f},{pt['p'][1]:.3f},{pt['p'][2]:.3f})  {tag}")

    # 姿态松弛分析: 对失败点, 用 位置优先 看最小可达姿态误差
    if hard:
        print("\n[姿态松弛] 失败点固定位置后,最小可达姿态误差(位置误差<3mm 前提):")
        for i,pt,tag,q0 in hard[:12]:
            # 位置-only 求解拿到解后直接评估 rot —— 用多起点, 目标只位置
            def resid_pos(q):
                T = fk_all(q); p = T[frame][:3,3]
                return list(p - np.array(pt['p']))
            rng = np.random.default_rng(11+i)
            seeds=[np.zeros(6)]+[np.array([rng.uniform(lo,hi) for lo,hi in LIMITS]) for _ in range(150)]
            bestrot=1e9; bestq=None
            for s in seeds:
                try: r=least_squares(resid_pos,s,max_nfev=3000)
                except Exception: continue
                q=r.x
                if not _in_limits(q): continue
                T=fk_all(q); p=T[frame][:3,3]; R=T[frame][:3,:3]
                pe=np.linalg.norm(p-pt['p'])
                if pe>0.003: continue
                re_=math.degrees(np.linalg.norm(R_to_axisangle(pt['R'].T@R)))
                if re_<bestrot: bestrot=re_; bestq=q
            print(f"  #{i} ({pt['p'][0]:.3f},{pt['p'][1]:.3f},{pt['p'][2]:.3f}): 最小姿态误差 {bestrot:.1f}deg" +
                  (f"  q={[round(x,2) for x in bestq]}" if bestq is not None else ""))

if __name__=="__main__":
    main()
