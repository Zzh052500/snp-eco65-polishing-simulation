#!/usr/bin/env python3
"""
eco65 正运动学(FK) + 工件包围盒分析。

用途: 为 SNP 扫描设计一套 eco65 关节空间扫描轨迹。
- 依赖: numpy, xml.etree.ElementTree(纯标准库), binary ply 解析(纯标准库).
- 输出: 在 baselink 坐标系下相机/法兰/工具 的位姿。
"""
import sys
import math
import struct
import xml.etree.ElementTree as ET
import numpy as np

import numpy as np

def rpy_to_matrix(r, p, y):
    # R = Rz(y) * Ry(p) * Rx(r)
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx

def tf(xyz, rpy):
    M = np.eye(4)
    M[:3, :3] = rpy_to_matrix(*rpy)
    M[:3, 3] = xyz
    return M

def axis_rot(axis, theta):
    """绕 axis(单位向量) 旋转 theta 的齐次变换."""
    T = np.eye(4)
    R = np.eye(3)
    x, y, z = axis
    c = math.cos(theta); s = math.sin(theta); C = 1 - c
    R[0, 0] = x*x*C + c;       R[0, 1] = x*y*C - z*s; R[0, 2] = x*z*C + y*s
    R[1, 0] = y*x*C + z*s;     R[1, 1] = y*y*C + c;   R[1, 2] = y*z*C - x*s
    R[2, 0] = z*x*C - y*s;     R[2, 1] = z*y*C + x*s; R[2, 2] = z*z*C + c
    T[:3, :3] = R
    return T

# 关节链(从 URDF 读取,但直接硬编码已核对的 origin/axis)
# baselink 是根。每个关节: (name, parent, child, origin_xyz, origin_rpy, axis)
CHAINS = [
    ("joint1", "baselink", "Link1", (0, 0, 0.1625), (0, 0, 0), (0, 0, 1)),
    ("joint2", "Link1", "Link2", (-0.086, 0, 0), (1.5708, 0, 0), (0, 0, -1)),
    ("joint3", "Link2", "Link3", (0, 0.26, 0), (3.1416, 0, 1.5708), (0, 0, 1)),
    ("joint4", "Link3", "Link4", (0.24, 0, -0.05888), (0, 0, 1.5708), (0, 0, 1)),
    ("joint5", "Link4", "Link5", (0.00040486, -0.10983, 0), (1.5708, 0, 0), (0, 0, 1)),
    ("joint6", "Link5", "Link6", (0, 0.0795, 0), (-1.5708, 0, 0), (0, 0, 1)),
]

# 固定关节(末端)
FIXED = [
    # tool0_to_camera: 相机
    ("camera_frame", "Link6", vec_cam_xyz:=None, vec_cam_rpy:=None),  # 用 calibration.yaml
    # sand_tcp_joint
    ("sand_tcp", "Link6", (-0.149, -0.154, 0.083), (0, math.radians(-90), math.radians(-135))),
]

def compute_fk(q, camera_cal=None):
    """q: 6 个关节角(rad)。返回 baselink 下的各 frame 4x4."""
    frames = {"baselink": np.eye(4)}
    T = np.eye(4)
    for (name, parent, child, oxyz, orpy, axis) in CHAINS:
        T = T @ tf(oxyz, orpy) @ axis_rot(axis, q[len(frames) - 1] if False else 0)
        # 需要正确索引; 重写为累积
        frames[child] = T
    return frames

def fk_full(q, camera_cal=None):
    """正确版本: 累积 T_parent @ T_origin @ T_axis(theta)."""
    frames = {"baselink": np.eye(4)}
    joint_index = {}
    for i, (name, parent, child, oxyz, orpy, axis) in enumerate(CHAINS):
        joint_index[name] = i
    T = {}
    T["baselink"] = np.eye(4)
    for (name, parent, child, oxyz, orpy, axis) in CHAINS:
        T[child] = T[parent] @ tf(oxyz, orpy) @ axis_rot(axis, q[joint_index[name]])
        frames[child] = T[child]
    frames["baselink"] = np.eye(4)
    # 固定 frame
    # 相机
    if camera_cal is not None:
        pos = camera_cal["camera_mount_to_camera_pos"]
        rpy = camera_cal["camera_mount_to_camera_rpy"]
        frames["camera_frame"] = T["Link6"] @ tf((pos["x"], pos["y"], pos["z"]),
                                                 (rpy["x"], rpy["y"], rpy["z"]))
    # sand_tcp
    Tcp = T["Link6"] @ tf((-0.149, -0.154, 0.083),
                          (0, math.radians(-90), math.radians(-135)))
    frames["sand_tcp"] = Tcp
    frames["Link6"] = T["Link6"]
    frames["flange"] = T["Link6"]
    return frames

def read_ply_bounds(path):
    """读取二进制 ply 顶点包围盒."""
    with open(path, "rb") as f:
        data = f.read()
    header_end = data.find(b"end_header") + len(b"end_header")
    header = data[:header_end].decode("ascii", errors="ignore")
    n_vert = 0
    for line in header.splitlines():
        if line.startswith("element vertex"):
            n_vert = int(line.split()[2])
    # vertex: 3x float (xyz) + 4x uchar (rgba) = 16 bytes
    VERT_STRIDE = 16
    body = data[header_end:]
    xyz = np.zeros((n_vert, 3), dtype=np.float32)
    for i in range(n_vert):
        base = i * VERT_STRIDE
        xyz[i, 0] = struct.unpack_from("<f", body, base + 0)[0]
        xyz[i, 1] = struct.unpack_from("<f", body, base + 4)[0]
        xyz[i, 2] = struct.unpack_from("<f", body, base + 8)[0]
    return xyz, xyz.min(axis=0), xyz.max(axis=0)

def main():
    # 读 calibration
    cal = {}
    with open("/home/liangfx/snp/config/calibration.yaml") as f:
        yaml_txt = f.read()
    # 简化: 直接硬编码 calibration.yaml 值
    camera_cal = {
        "camera_mount_to_camera_pos": {"x": 0.03850114747059883, "y": -0.09268336808180952, "z": 0.071274308624225333},
        "camera_mount_to_camera_rpy": {"x": 1.5819644105520236, "y": 2.3262610409212181, "z": 1.5737142986439765},
    }

    print("=" * 70)
    print("eco65 FK 分析 (baselink 坐标系) — baselink 在 table 的 (-0.61,0,0.723)")
    print("=" * 70)

    # 零位测试
    q0 = [0, 0, 0, 0, 0, 0]
    frames = fk_full(q0, camera_cal)
    print("\n[零位 q=[0,0,0,0,0,0]]")
    for k in ["Link6", "camera_frame", "sand_tcp"]:
        p = frames[k][:3, 3]
        print(f"  {k}: xyz = ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})")

    # 工件包围盒
    part_path = "/home/liangfx/snp/meshes/part_scan.ply"
    arr, amin, amax = read_ply_bounds(part_path)
    print("\n[part_scan.ply 包围盒, 在 baselink(=原base_link) 坐标系]")
    print(f"  min = ({amin[0]:.3f}, {amin[1]:.3f}, {amin[2]:.3f})")
    print(f"  max = ({amax[0]:.3f}, {amax[1]:.3f}, {amax[2]:.3f})")
    print(f"  center = ({(amin[0]+amax[0])/2:.3f}, {(amin[1]+amax[1])/2:.3f}, {(amin[2]+amax[2])/2:.3f})")

if __name__ == "__main__":
    main()
