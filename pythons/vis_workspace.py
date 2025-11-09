import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation

class RobotWorkspaceVisualizer:
    def __init__(self):
        # self.kinematics_checker = kinematics_checker  # 您的运动学验证函数
        pass
        
    def visualize_workspace_3d(self, bounds, resolution=50):
        """
        三维工作空间可视化
        bounds: [x_min, x_max, y_min, y_max, z_min, z_max]
        resolution: 每个维度的采样点数
        """
        # 创建网格
        x = np.linspace(bounds[0], bounds[1], resolution)
        y = np.linspace(bounds[2], bounds[3], resolution)
        z = np.linspace(bounds[4], bounds[5], resolution)
        
        # 初始化可达性矩阵
        reachable = np.zeros((resolution, resolution, resolution), dtype=bool)
        
        # 采样每个点
        for i, xi in enumerate(x):
            for j, yj in enumerate(y):
                for k, zk in enumerate(z):
                    # 这里需要您的逆运动学算法将(x,y,z)转换为(s1,s2,s3)
                    # 假设您有一个逆运动学函数：inverse_kinematics(x, y, z)
                    s1, s2, s3 = self.inverse_kinematics(xi, yj, zk)
                    reachable[i, j, k] = self.kinematics_checker(s1, s2, s3)
            
            print(f"Progress: {i+1}/{resolution}")
        
        # 可视化
        self.plot_3d_workspace(x, y, z, reachable)
        
    def plot_3d_workspace(self, x, y, z, reachable):
        fig = plt.figure(figsize=(12, 9))
        ax = fig.add_subplot(111, projection='3d')
        
        # 提取可达点的坐标
        reachable_points = np.where(reachable)
        if len(reachable_points[0]) > 0:
            xs = x[reachable_points[0]]
            ys = y[reachable_points[1]]
            zs = z[reachable_points[2]]
            
            ax.scatter(xs, ys, zs, c='green', alpha=0.6, s=1, label='可达')
        
        ax.set_xlabel('X轴')
        ax.set_ylabel('Y轴')
        ax.set_zlabel('Z轴')
        ax.set_title('机械臂工作空间')
        ax.legend()
        plt.show()

    def kinematics_checker(self,s1, s2, s3):
        r = Rotation.from_euler('ZYX', [s1, s2, s3])
        mat_after = r.as_matrix()
        # print(f"测试矩阵\n{mat_after}")
        v1_x =  
        return mat_after
    
    def solve_one(self,vi,wi):
        pass

if __name__ == "__main__":
    instance = RobotWorkspaceVisualizer()
    instance.kinematics_checker(1.57,-1.57,0)