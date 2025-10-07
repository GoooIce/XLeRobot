import math
import numpy as np
from typing import List, Union, Tuple

from lerobot.robots.robot import Robot
from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig
from lerobot.robots.utils import make_robot_from_config
import numpy as np
from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig
from lerobot.cameras import ColorMode, Cv2Rotation
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig

def create_real_robot(port, camera_index, uid: str = "so101") -> Robot:
    """
    创建真实机器人的包装函数

    将字符串UID映射到真实机器人配置。主要用于用户在fork仓库时节省代码量，
    他们可以在这个文件中编辑相机、id等设置。

    Args:
        port: 机器人USB端口
        camera_index: 相机索引
        uid: 机器人唯一标识符，默认为"so101"

    Returns:
        Robot: 配置好的机器人实例
    """
    if uid == "so101":
        robot_config = SO101FollowerConfig(
            port= port,
            use_degrees=True,
            # 手机相机用户可以使用下面的注释设置
            cameras = {
                "base_camera": OpenCVCameraConfig(index_or_path= camera_index,  # 替换为find_cameras.py中找到的相机索引
                fps=30,
                width=640,
                height=480,
                color_mode=ColorMode.RGB,
                rotation=Cv2Rotation.NO_ROTATION)
            },
            # Intel RealSense相机用户需要修改自己硬件的序列号或名称
            # cameras={
            #     "base_camera": RealSenseCameraConfig(serial_number_or_name="146322070293", fps=30, width=640, height=480)
            # },
            id="robot1",
        )
        real_robot = make_robot_from_config(robot_config)
        return real_robot


class SO101Kinematics:
    """
    SO101机器人臂运动学类

    所有公共方法都使用度作为输入/输出单位。
    """

    def __init__(self, l1=0.1159, l2=0.1350):
        """
        初始化运动学参数

        Args:
            l1: 第一连杆长度（上臂），默认0.1159米
            l2: 第二连杆长度（下臂），默认0.1350米
        """
        self.l1 = l1  # 第一连杆长度（上臂）
        self.l2 = l2  # 第二连杆长度（下臂）

    def inverse_kinematics(self, x, y, l1=None, l2=None):
        """
        计算两连杆机械臂的逆运动学，考虑关节偏移

        Args:
            x: 末端执行器x坐标
            y: 末端执行器y坐标
            l1: 上臂长度（默认使用实例值）
            l2: 下臂长度（默认使用实例值）

        Returns:
            joint2_deg, joint3_deg: 关节角度（度）(shoulder_lift, elbow_flex)
        """
        # 如果未提供参数，使用实例值
        if l1 is None:
            l1 = self.l1
        if l2 is None:
            l2 = self.l2

        # 计算关节2和关节3在theta1和theta2中的偏移
        theta1_offset = math.atan2(0.028, 0.11257)  # 关节2为0时的theta1偏移
        theta2_offset = math.atan2(0.0052, 0.1349) + theta1_offset  # 关节3为0时的theta2偏移

        # 计算原点到目标点的距离
        r = math.sqrt(x**2 + y**2)
        r_max = l1 + l2  # 最大可达距离

        # 如果目标点超出最大工作空间，将其缩放到边界
        if r > r_max:
            scale_factor = r_max / r
            x *= scale_factor
            y *= scale_factor
            r = r_max

        # 如果目标点小于最小工作空间（|l1-l2|），将其缩放
        r_min = abs(l1 - l2)
        if r < r_min and r > 0:
            scale_factor = r_min / r
            x *= scale_factor
            y *= scale_factor
            r = r_min

        # 使用余弦定理计算theta2
        cos_theta2 = -(r**2 - l1**2 - l2**2) / (2 * l1 * l2)

        # 将cos_theta2限制在有效范围[-1, 1]以避免域错误
        cos_theta2 = max(-1.0, min(1.0, cos_theta2))

        # 计算theta2（肘部角度）
        theta2 = math.pi - math.acos(cos_theta2)

        # 计算theta1（肩部角度）
        beta = math.atan2(y, x)
        gamma = math.atan2(l2 * math.sin(theta2), l1 + l2 * math.cos(theta2))
        theta1 = beta + gamma
        
        # Convert theta1 and theta2 to joint2 and joint3 angles
        joint2 = theta1 + theta1_offset
        joint3 = theta2 + theta2_offset
        
        # Ensure angles are within URDF limits
        joint2 = max(-0.1, min(3.45, joint2))
        joint3 = max(-0.2, min(math.pi, joint3))
        
        # Convert from radians to degrees
        joint2_deg = math.degrees(joint2)
        joint3_deg = math.degrees(joint3)

        # Apply coordinate system transformation
        joint2_deg = 90 - joint2_deg
        joint3_deg = joint3_deg - 90
        
        return joint2_deg, joint3_deg
    
    def forward_kinematics(self, joint2_deg, joint3_deg, l1=None, l2=None):
        """
        Calculate forward kinematics for a 2-link robotic arm
        
        Parameters:
            joint2_deg: Shoulder lift joint angle in degrees
            joint3_deg: Elbow flex joint angle in degrees
            l1: Upper arm length (default uses instance value)
            l2: Lower arm length (default uses instance value)
            
        Returns:
            x, y: End effector coordinates
        """
        # Use instance values if not provided
        if l1 is None:
            l1 = self.l1
        if l2 is None:
            l2 = self.l2
            
        # Convert degrees to radians and apply inverse transformation
        joint2_rad = math.radians(90 - joint2_deg)
        joint3_rad = math.radians(joint3_deg + 90)
        
        # Calculate joint2 and joint3 offsets
        theta1_offset = math.atan2(0.028, 0.11257)
        theta2_offset = math.atan2(0.0052, 0.1349) + theta1_offset
        
        # Convert joint angles back to theta1 and theta2
        theta1 = joint2_rad - theta1_offset
        theta2 = joint3_rad - theta2_offset
        
        # Forward kinematics calculations
        x = l1 * math.cos(theta1) + l2 * math.cos(theta1 + theta2 - math.pi)
        y = l1 * math.sin(theta1) + l2 * math.sin(theta1 + theta2 - math.pi)
        
        return x, y

    
    def generate_sinusoidal_velocity_trajectory(
        self,
        start_point: Union[List[float], np.ndarray],
        end_point: Union[List[float], np.ndarray],
        control_freq: float = 100.0,  # Hz
        total_time: float = 5.0,      # seconds
        velocity_amplitude: float = 1.0,  # m/s
        velocity_period: float = 2.0,     # seconds
        phase_offset: float = 0.0         # radians
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Generate a straight-line trajectory with sinusoidal velocity profile.
        
        Parameters:
        -----------
        start_point : array-like
            3D coordinates of starting point [x, y, z]
        end_point : array-like  
            3D coordinates of ending point [x, y, z]
        control_freq : float
            Control frequency in Hz
        total_time : float
            Total trajectory time in seconds
        velocity_amplitude : float
            Amplitude of velocity oscillation in m/s
        velocity_period : float
            Period of velocity oscillation in seconds
        phase_offset : float
            Phase offset in radians
            
        Returns:
        --------
        trajectory : np.ndarray
            Array of 3D positions (n_points, 3)
        velocities : np.ndarray
            Array of velocity magnitudes (n_points,)
        time_array : np.ndarray
            Time array (n_points,)
        """
        
        # Convert to numpy arrays
        start = np.array(start_point, dtype=float)
        end = np.array(end_point, dtype=float)
        
        # Calculate direction and distance
        direction_vector = end - start
        total_distance = np.linalg.norm(direction_vector)
        direction_unit = direction_vector / total_distance if total_distance > 0 else np.zeros(3)
        
        # Generate time array
        dt = 1.0 / control_freq
        n_points = int(total_time * control_freq) + 1
        time_array = np.linspace(0, total_time, n_points)
        
        # Calculate angular frequency
        omega = 2 * np.pi / velocity_period
        
        # Generate sinusoidal velocity profile
        base_velocity = total_distance / total_time  # Average velocity needed
        velocities = base_velocity + velocity_amplitude * np.sin(omega * time_array + phase_offset)
        
        # Ensure non-negative velocities (optional - remove if negative velocities are desired)
        velocities = np.maximum(velocities, 0.1 * base_velocity)
        
        # Integrate velocity to get position along the path
        positions_1d = np.zeros(n_points)
        for i in range(1, n_points):
            positions_1d[i] = positions_1d[i-1] + velocities[i-1] * dt
        
        # Scale positions to fit exactly between start and end points
        if positions_1d[-1] > 0:
            positions_1d = positions_1d * (total_distance / positions_1d[-1])
        
        # Convert 1D positions to 3D trajectory
        trajectory = np.zeros((n_points, 3))
        for i in range(n_points):
            progress = positions_1d[i] / total_distance if total_distance > 0 else 0
            trajectory[i] = start + progress * direction_vector
        
        return trajectory, velocities, time_array
    # Example usage
    # if __name__ == "__main__":
    #     # Define start and end points
    #     start = [0, 0, 0]
    #     end = [5, 3, 2]
        
    #     # Generate trajectory
    #     trajectory, velocities, time_array = generate_sinusoidal_velocity_trajectory(
    #         start_point=start,
    #         end_point=end,
    #         control_freq=100.0,
    #         total_time=6.0,
    #         velocity_amplitude=0.8,
    #         velocity_period=1.5,
    #         phase_offset=0
    #     )
        
    #     print(f"Generated {len(trajectory)} trajectory points")
    #     print(f"Total distance: {np.linalg.norm(np.array(end) - np.array(start)):.3f}")
    #     print(f"Time duration: {time_array[-1]:.2f} seconds")
    #     print(f"Average velocity: {velocities.mean():.3f} m/s")
    #     print(f"Velocity range: {velocities.min():.3f} to {velocities.max():.3f} m/s")
        
    #     print("\nFirst few trajectory points:")
    #     for i in range(0, min(10, len(trajectory)), 2):
    #         print(f"t={time_array[i]:.2f}s: pos=[{trajectory[i,0]:.3f}, {trajectory[i,1]:.3f}, {trajectory[i,2]:.3f}], vel={velocities[i]:.3f} m/s")
