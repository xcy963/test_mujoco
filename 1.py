import numpy as np
import math

from scipy.spatial.transform import Rotation as R
# xyz="-0.010293002166729 0.0269637057926049 -0.0254082202127092"
cos_sita = math.cos(1.83259)
sin_sita = math.sin(1.83259)

R_mat = np.array(
    [
        [0,0,-1],
        [-sin_sita,cos_sita,0],
        [cos_sita,sin_sita,0]
    ]
)

t = np.array([-0.010293002166729 ,0.0269637057926049, -0.0254082202127092])

print()

trans = np.array([0.13816 , 0.0384621, 0.0123112]) + R_mat@t

print(trans)


# mujuco是xyz欧拉角,但是urdf是xyz固定角度,相当逆天,两个矩阵是反着乘的

def euler_zyx_to_xyz(angles_zyx):
    """
    将 ZYX 欧拉角 (yaw, pitch, roll) 转成 XYZ 欧拉角 (roll_x, pitch_y, yaw_z)
    angles_zyx: (3,) 数组 [z, y, x]，单位：弧度
    返回: (3,) 数组 [x', y', z']，同样是弧度
    """
    angles_zyx = np.asarray(angles_zyx, dtype=float)

    # 先按 ZYX 顺序生成旋转（这里 SciPy 默认是“内在旋转 intrinsic”）
    r = R.from_euler('zyx', angles_zyx)

    # 再用 XYZ 顺序把同一个旋转分解出来
    angles_xyz = r.as_euler('xyz')

    return angles_xyz


zyx = [0,-1.5708 , 1.83259  ]   # [yaw_z, pitch_y, roll_x]，单位弧度
xyz = euler_zyx_to_xyz(zyx)

print("XYZ:", xyz)