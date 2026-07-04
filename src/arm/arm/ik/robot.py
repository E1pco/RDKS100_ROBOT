"""
纯 Python 机器人运动学实现
"""

import numpy as np
import math
from scipy.spatial.transform import Rotation as R

# 硬件驱动是可选的（仅在实际控制硬件时需要）
try:
    from ..driver.ftservo_controller import ServoController
    from ..driver.ftservo_driver import FTServo
except ImportError:
    ServoController = None
    FTServo = None

# 支持直接运行和模块导入
try:
    from .et import ET, ETS
    from .solver import IKResult, ikine_LM as _ikine_LM, ikine_GN as _ikine_GN, ikine_NR as _ikine_NR, ikine_QP as _ikine_QP
except ImportError:
    # 直接运行时使用绝对导入
    import sys
    import os
    sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
    from ik.et import ET, ETS
    from ik.solver import IKResult, ikine_LM as _ikine_LM, ikine_GN as _ikine_GN, ikine_NR as _ikine_NR, ikine_QP as _ikine_QP


def atan2(first, second):
    """保留3位小数的 atan2"""
    return round(math.atan2(first, second), 3)


def sin(radians_angle):
    """保留3位小数的 sin"""
    return round(math.sin(radians_angle), 3)


def cos(radians_angle):
    """保留3位小数的 cos"""
    return round(math.cos(radians_angle), 3)


def acos(value):
    """保留3位小数的 acos"""
    return round(math.acos(value), 3)

class IKResult:
    """IK 求解结果封装类，兼容 roboticstoolbox 接口"""
    def __init__(self, success, q, reason=""):
        self.success = success
        self.q = q
        self.reason = reason


class Robot:
    """
    机器人封装类，提供与 roboticstoolbox 兼容的 API
    
    Attributes
    ----------
    ets : ETS
        Elementary Transform Sequence
    n : int
        关节数量
    qlim : np.ndarray
        关节限位 (2, n)
    joint_names : list
        关节名称列表
    gear_sign : dict
        各关节的方向符号 (+1/-1)
    gear_ratio : dict
        各关节的减速比
    """
    
    def __init__(self, ets, qlim=None, joint_names=None, gear_sign=None, gear_ratio=None):
        """
        初始化机器人模型
        
        Parameters
        ----------
        ets : ETS
            机器人运动学链
        qlim : np.ndarray, optional
            关节限位 (2, n)，第一行为下限，第二行为上限
        joint_names : list, optional
            关节名称列表
        gear_sign : dict, optional
            各关节的方向符号
        gear_ratio : dict, optional
            各关节的减速比
        """
        self.ets = ets
        self.n = ets.n
        self.qlim = qlim
        self.joint_names = joint_names or [f"joint_{i}" for i in range(self.n)]
        self.gear_sign = gear_sign or {name: +1 for name in self.joint_names}
        self.gear_ratio = gear_ratio or {name: 1.0 for name in self.joint_names}
        
        # 延迟初始化 ServoController（仅在需要时）
        self._servo = None
        
        # 将 qlim 设置到 ETS 对象上（IK solver 会从 ets.qlim 读取）
        if qlim is not None:
            self.ets.qlim = qlim
    
    def set_servo_controller(self, controller):
        """
        手动设置 ServoController 实例
        
        Parameters
        ----------
        controller : ServoController
            舵机控制器实例
        """
        self._servo = controller
    
    @property
    def servo(self):
        """懒加载 ServoController"""
        if self._servo is None:
            try:
                # 寻找 servo_config.json 的正确路径
                import os
                config_paths = [
                    "servo_config.json",
                    "driver/servo_config.json",
                    os.path.join(os.path.dirname(__file__), "..", "driver", "servo_config.json"),
                    os.path.join(os.path.dirname(__file__), "..", "servo_config.json"),
                ]
                
                config_path = None
                for path in config_paths:
                    if os.path.exists(path):
                        config_path = path
                        break
                
                if config_path is None:
                    print(f"⚠️ 无法找到 servo_config.json，尝试的路径: {config_paths}")
                    return None
                
                self._servo = ServoController(
                    port="/dev/ttyACM0",
                    baudrate=1_000_000,
                    config_path=config_path
                )
            except Exception as e:
                print(f"⚠️ 无法初始化 ServoController: {e}")
                self._servo = None
        return self._servo

    def q_to_servo_targets(self, q_rad, joint_names=None, home_pose=None, 
                            counts_per_rev=4096, gear_ratio=None, gear_sign=None):
        """
        将关节角度（弧度）转换为舵机目标步数
        
        Parameters
        ----------
        q_rad : array-like
            关节角度数组（弧度）
        joint_names : list of str
            关节名称列表
        home_pose : dict, optional
            各关节的中位步数 {"joint_name": home_position}
            若为 None，则使用 self.servo.home_pose
        counts_per_rev : int
            每转编码器计数（默认4096）
        gear_ratio : dict, optional
            齿轮比 {"joint_name": ratio}
        gear_sign : dict, optional
            方向符号 {"joint_name": +1 or -1}
        
        Returns
        -------
        targets : dict
            舵机目标位置 {"joint_name": target_steps}
        """
        # 如果未提供 home_pose，从 servo 获取
        if home_pose is None:
            if self.servo is None:
                raise ValueError("home_pose 必须提供，或者 ServoController 必须可用")
            home_pose = self.servo.home_pose
        
        if gear_ratio is None:
            gear_ratio = self.gear_ratio
        if gear_sign is None:
            gear_sign = self.gear_sign
        if joint_names is None:
            joint_names = self.joint_names
        counts_per_rad = counts_per_rev / (2 * 3.141592653589793)  # 2*pi
        targets = {}
        
        for i, name in enumerate(joint_names):
            steps = int(round(
                home_pose[name] + 
                gear_sign[name] * gear_ratio[name] * q_rad[i] * counts_per_rad
            ))
            targets[name] = steps
        
        return targets
    def read_joint_angles(self, joint_names=None, home_pose=None, gear_sign=None, gear_ratio=None, verbose=True):
        """
        读取舵机实际位置并计算关节角度
        
        Parameters
        ----------
        joint_names : list of str
            关节名称列表
        home_pose : dict, optional
            各关节的中位步数 {"joint_name": home_position}
            若为 None，则使用 self.servo.home_pose
        gear_sign : dict, optional
            方向符号 {"joint_name": +1 or -1}，默认为 self.gear_sign
        gear_ratio : dict, optional
            齿轮比 {"joint_name": ratio}，默认为 self.gear_ratio
        verbose : bool
            是否打印详细信息（默认 True）
        
        Returns
        -------
        q : np.ndarray
            关节角度数组（弧度）
        """
        if self.servo is None:
            raise RuntimeError("ServoController 不可用，无法读取舵机位置")
        
        # 如果未提供，使用默认值
        if joint_names is None:
            joint_names = self.joint_names
        if gear_sign is None:
            gear_sign = self.gear_sign
        if gear_ratio is None:
            gear_ratio = self.gear_ratio
        if home_pose is None:
            home_pose = self.servo.home_pose
        positions = self.servo.read_servo_positions(joint_names=joint_names, verbose=False)
        q = np.zeros(len(joint_names))
        counts_per_rad = 4096 / (2 * np.pi)
        
        if verbose:
            print("\n📡 读取关节角度:")
        
        for i, name in enumerate(joint_names):
            current = positions[name]
            delta = current - home_pose[name]
            q[i] = gear_sign[name] * delta / (counts_per_rad * gear_ratio[name])
            
            if verbose:
                print(f" {name:15s} : 步数={current:4d}, Δ={delta:+5d} → q={q[i]:+.4f} rad ")
        
        return q
    def fkine(self, q):
        """
        正运动学计算
        
        Parameters
        ----------
        q : array_like
            关节角度
            
        Returns
        -------
        np.ndarray
            4x4 齐次变换矩阵
        """
        return self.ets.fkine(q)
    def fk(self, qpos_data, joint_indices=None):
        """
        并返回末端执行器位姿向量 [X, Y, Z, Roll, Pitch, Yaw]

        Parameters
        ----------
        qpos_data : np.ndarray
            关节角度向量（可以比机器人关节多，会根据 joint_indices 提取）
        joint_indices : list or np.ndarray, optional
            要使用的关节索引。如果为 None，则使用前 n 个关节

        Returns
        -------
        np.ndarray
            末端执行器位姿 [X, Y, Z, Roll, Pitch, Yaw]
        """
        # 如果提供了关节索引，使用索引提取关节角度
        if joint_indices is not None:
            if max(joint_indices) >= len(qpos_data):
                raise Exception(
                    f"Joint index {max(joint_indices)} out of range for qpos_data "
                    f"with length {len(qpos_data)}"
                )
            q = qpos_data[joint_indices]
        else:
            # 否则，检查长度并提取前 n 个
            if len(qpos_data) < self.n:
                raise Exception(
                    f"The dimensions of qpos_data ({len(qpos_data)}) "
                    f"is less than the robot joint dimensions ({self.n})"
                )
            q = qpos_data[:self.n]

        # 计算正运动学，获取齐次变换矩阵
        T = self.fkine(q)

        # 提取位置
        X, Y, Z = T[0, 3], T[1, 3], T[2, 3]

        # 提取旋转矩阵并计算欧拉角 (XYZ -> Roll, Pitch, Yaw)
        R_mat = T[:3, :3]

        beta = atan2(-R_mat[2, 0], math.sqrt(R_mat[0, 0]**2 + R_mat[1, 0]**2))

        if cos(beta) != 0:
            alpha = atan2(R_mat[1, 0] / cos(beta), R_mat[0, 0] / cos(beta))
            gamma = atan2(R_mat[2, 1] / cos(beta), R_mat[2, 2] / cos(beta))
        else:
            # 万向节锁情况
            alpha = 0
            gamma = atan2(R_mat[0, 1], R_mat[1, 1])

        return np.array([X, Y, Z, gamma, beta, alpha])
    
    def ikine_LM(self, Tep, q0=None, ilimit=100, slimit=10, tol=1e-3, mask=None, 
                 k=1.0, method='chan'):
        """
        使用 Levenberg-Marquardt 方法求解逆运动学
        
        Parameters
        ----------
        Tep : np.ndarray
            目标位姿 (4x4 齐次变换矩阵)
        q0 : array_like, optional
            初始关节角度，默认为零向量
        ilimit : int
            最大迭代次数
        slimit : int
            搜索次数限制
        tol : float
            收敛容差
        mask : array_like, optional
            位姿权重 [x, y, z, roll, pitch, yaw]，0 表示忽略该维度
        k : float
            LM 阻尼系数
        method : str
            LM 更新方法 ('chan', 'wampler', 'sugihara')
            
        Returns
        -------
        IKResult
            求解结果，包含 .success, .q, .reason 属性
        """
        return _ikine_LM(self.ets, Tep, q0, ilimit, slimit, tol, mask, k, method)
    
    def ikine_GN(self, Tep, q0=None, ilimit=50, tol=1e-3, mask=None, pinv=False):
        """
        使用 Gauss-Newton 方法求解逆运动学
        
        Parameters
        ----------
        Tep : np.ndarray
            目标位姿 (4x4 齐次变换矩阵)
        q0 : array_like, optional
            初始关节角度
        ilimit : int
            最大迭代次数
        tol : float
            收敛容差
        mask : array_like, optional
            位姿权重
        pinv : bool
            是否使用伪逆
            
        Returns
        -------
        IKResult
            求解结果
        """
        return _ikine_GN(self.ets, Tep, q0, ilimit, tol, mask, pinv)
    
    def ikine_NR(self, Tep, q0=None, ilimit=50, tol=1e-3, mask=None, pinv=False):
        """使用 Newton-Raphson 方法求解逆运动学"""
        return _ikine_NR(self.ets, Tep, q0, ilimit, tol, mask, pinv)
    
    def ikine_QP(self, Tep, q0=None, ilimit=50, tol=1e-3, mask=None, 
                 kj=0.01, ks=1.0):
        """
        使用二次规划方法求解逆运动学
        
        kj: 关节正则化系数
        ks: 步长缩放系数
        """
        return _ikine_QP(self.ets, Tep, q0, ilimit, tol, mask, kj, ks)

def create_so101_5dof():
    """
    SO-101 五自由度机械臂（基于 URDF 简化结构）ET 建模
    包含关节限位（通过 ERobot.qlim 设置）
    """

    # ---------------------------
    # 1) URDF 同步的关节限位
    # ---------------------------
    qlim = np.array([
        [-1.91986, -1.74533, -1.69,    -1.65806, -2.74385],
        [ 1.91986,  1.74533,  1.69,     1.65806,  2.84121]
    ])


    # E1 = ET.tx(0.002798)
    # E2 = ET.tz(0.05031)
    # E3 = ET.Rz()
    
    # # to joint 2
    # E4 = ET.tx(0.02957)
    # E5 = ET.tz(0.11590)
    # E6 = ET.Ry()
    
    # # to joint 3
    # E7 = ET.tx(0.11323)
    # E8 = ET.tz(0.00500)
    # E9 = ET.Ry()

    # # to joint 4
    # E10 = ET.tx(0.0650)
    # E11 = ET.tz(0.00519)
    # E12 = ET.Ry()
    
    # # to joint 5
    # E13 = ET.tx(0.02413)
    # E14 = ET.tz(0)
    # E15 = ET.Rx()  
    
    # E17 = ET.tx(0.07440)
        # to joint 1
    E1 = ET.tx(0.0612)
    E2 = ET.tz(0.0598)
    E3 = ET.Rz()
    
    # to joint 2
    E4 = ET.tx(0.02943)
    E5 = ET.tz(0.05504)
    E6 = ET.Ry()
    
    # to joint 3
    E7 = ET.tz(0.1127)
    E8 = ET.tx(0.02798)
    E9 = ET.Ry()

    # to joint 4
    E10 = ET.tx(0.13504)
    E11 = ET.tz(0.00519)
    E12 = ET.Ry()
    
    # to joint 5
    E13 = ET.tx(0.0593)
    E14 = ET.tz(0.00996)
    E15 = ET.Rx()  
    
    #E17 = ET.tx(0.09538)
    # to gripper

    ets = E1 * E2 * E3 *E4 * E5 * E6 * E7 * E8 * E9 * E10 * E11 * E12 * E13 * E14 * E15 

    
    # 关节名称
    joint_names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
    
    # 各关节的方向符号 (+1/-1)
    gear_sign = {
        "shoulder_pan": -1,
        "shoulder_lift": +1,
        "elbow_flex":   +1,
        "wrist_flex":   +1,
        "wrist_roll":   -1,
    }
    
    # 各关节的减速比
    gear_ratio = {
        "shoulder_pan": 1.0,
        "shoulder_lift": 1.0,
        "elbow_flex":   1.0,
        "wrist_flex":   1.0,
        "wrist_roll":   1.0,
    }
    
    return Robot(ets, qlim, joint_names=joint_names, gear_sign=gear_sign, gear_ratio=gear_ratio)



def create_so101_5dof_gripper():
    # to joint 1
    E1 = ET.tx(0.0612)
    E2 = ET.tz(0.0598)
    E3 = ET.Rz()
    
    # to joint 2
    E4 = ET.tx(0.02943)
    E5 = ET.tz(0.05504)
    E6 = ET.Ry()
    
    # to joint 3
    E7 = ET.tz(0.1127)
    E8 = ET.tx(0.02798)
    E9 = ET.Ry()

    # to joint 4
    E10 = ET.tx(0.13504)
    E11 = ET.tz(0.00519)
    E12 = ET.Ry()
    
    # to joint 5
    E13 = ET.tx(0.0593)
    E14 = ET.tz(0.00996)
    E15 = ET.Rx()  
    
    E17 = ET.tx(0.09538)
    # to gripper
   
    ets = E1*E2*E3*E4 * E5 * E6 * E7 * E8 * E9 * E10 * E11 * E12 * E13 * E14 * E15 *E17# E1 * E2 * E3 * E17 
    joint_names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
    # Set joint limits
    qlim = np.array([
        [-1.91986, -1.74533, -1.69,    -1.65806, -2.74385],
        [ 1.91986,  1.74533,  1.69,     1.65806,  2.84121]
    ])
    gear_sign = {
        "shoulder_pan": -1,
        "shoulder_lift": +1,
        "elbow_flex":   +1,
        "wrist_flex":   +1,
        "wrist_roll":   -1,
    }
    
    # 各关节的减速比
    gear_ratio = {
        "shoulder_pan": 1.0,
        "shoulder_lift": 1.0,
        "elbow_flex":   1.0,
        "wrist_flex":   1.0,
        "wrist_roll":   1.0,
    }
    
    return Robot(ets, qlim, joint_names=joint_names, gear_sign=gear_sign, gear_ratio=gear_ratio)

def create_so101_6dof_gripper():
    # to joint 1
    E1 = ET.tx(0.0612)
    E2 = ET.tz(0.0598)
    E3 = ET.Rz()
    
    # to joint 2
    E4 = ET.tx(0.02943)
    E5 = ET.tz(0.05504)
    E6 = ET.Ry()
    
    # to joint 3
    E7 = ET.tz(0.1127)
    E8 = ET.tx(0.02798)
    E9 = ET.Ry()

    # to joint 4
    E10 = ET.tx(0.13504)
    E11 = ET.tz(0.00519)
    E12 = ET.Ry()
    
    # to joint 5
    E13 = ET.tx(0.025)
    E14 = ET.tz(0.04)
    E15 = ET.Rz()
    
    # to joint 6
    E16 = ET.tz(-0.04)
    E17 = ET.tx(0.058)
    E18 = ET.Rx()
    
    E19 = ET.tx(0.09538)
    # to gripper
   
    ets = E1*E2*E3*E4 * E5 * E6 * E7 * E8 * E9 * E10 * E11 * E12 * E13 * E14 * E15 *E16*E17*E18*E19
    joint_names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw","wrist_roll"]
    # Set joint limits
    qlim = np.array([
        [-1.91986, -1.74533, -1.69,    -1.65806, -1.81986,-2.74385],
        [ 1.91986,  1.74533,  1.69,     1.65806,  1.81986,2.84121]
    ])
    gear_sign = {
        "shoulder_pan": -1,
        "shoulder_lift": +1,
        "elbow_flex":   +1,
        "wrist_flex":   +1,
        "wrist_yaw":   +1,
        "wrist_roll":   -1,
    }
    
    # 各关节的减速比
    gear_ratio = {
        "shoulder_pan": 1.0,
        "shoulder_lift": 1.0,
        "elbow_flex":   1.0,
        "wrist_flex":   1.0,
        "wrist_yaw":   1.0,
        "wrist_roll":   1.0,
    }
    
    return Robot(ets, qlim, joint_names=joint_names, gear_sign=gear_sign, gear_ratio=gear_ratio)
def create_so101():
    """
    基于 DH 参数的 SO-101 机械臂建模
    严格按照给定的 DH 参数构建 ETS 链
    """
    import math
    pi = math.pi
    
    # 基于 DH 参数构建 ETS 链
    # ETS = Rz(q1 - 1e-5) * Tz(0.0624) * Tx(0.038835) *
    #       Rz(q2 + 0.00038) * Tz(0.0542) * Tx(0.030399) * Rx(-pi/2) *
    #       Rz(q3 - 1.3279) * Tz(-0.018278) * Tx(0.116) *
    #       Rz(q4 + 1.2889) * Tx(0.135) *
    #       Rz(q5 - 1.5325) * Tz(0.0181) * Rx(pi/2) *
    #       Rz(q6 - 2.78913) * Tz(-0.0845) * Tx(0.0202) * Rx(pi/2)
    
    # Joint 1 (q1)
    E1 = ET.Rz()  # q1 joint
    E2 = ET.Rz(-1e-5)  # offset
    E3 = ET.tz(0.0624)
    E4 = ET.tx(0.038835)
    
    # Joint 2 (q2)  
    E5 = ET.Rz()  # q2 joint
    E6 = ET.Rz(0.00038)  # offset
    E7 = ET.tz(0.0542)
    E8 = ET.tx(0.030399)
    E9 = ET.Rx(-pi/2)
    
    # Joint 3 (q3)
    E10 = ET.Rz()  # q3 joint
    E11 = ET.Rz(-1.3279)  # offset
    E12 = ET.tz(-0.018278)
    E13 = ET.tx(0.116)
    
    # Joint 4 (q4)
    E14 = ET.Rz()  # q4 joint
    E15 = ET.Rz(1.2889)  # offset
    E16 = ET.tx(0.135)
    
    # Joint 5 (q5)
    E17 = ET.Rz()  # q5 joint
    E18 = ET.Rz(-1.5325)  # offset
    E19 = ET.tz(0.0181)
    E20 = ET.Rx(pi/2)
    
    # Joint 6 (q6) - gripper
    # E21 = ET.Rz()  # q6 joint (gripper)
    E22 = ET.Rz(-2.78913)  # offset
    E23 = ET.tz(-0.0845)
    E24 = ET.tx(0.0202)
    E25 = ET.Rx(pi/2)
    
    # 构建完整的 ETS 链
    ets = (E1 * E2 * E3 * E4 * 
           E5 * E6 * E7 * E8 * E9 *
           E10 * E11 * E12 * E13 *
           E14 * E15 * E16 *
           E17 * E18 * E19 * E20 *
           E22 * E23 * E24 * E25)
    
    # 关节名称 (6个关节)
    joint_names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
    
    # 关节限位 (6个关节)
    qlim = np.array([
        [-1.91986, -1.74533, -1.69, -1.65806, -2.74385], 
        [ 1.91986,  1.74533,  1.69,  1.65806,  2.84121]
    ])
    
    # 各关节的方向符号
    gear_sign = {
        "shoulder_pan": -1,
        "shoulder_lift": -1,
        "elbow_flex":   -1,
        "wrist_flex":   -1,
        "wrist_roll":   +1,
    }
    
    # 各关节的减速比
    gear_ratio = {
        "shoulder_pan": 1.0,
        "shoulder_lift": 1.0,
        "elbow_flex":   1.0,
        "wrist_flex":   1.0,
        "wrist_roll":   1.0,
    }
    
    return Robot(ets, qlim=qlim, joint_names=joint_names, gear_sign=gear_sign, gear_ratio=gear_ratio)



def get_robot(robot="so101"):
    """
    获取指定的机器人模型
    
    Parameters
    ----------
    robot : str
        机器人类型： 'so101_5dof'
        
    Returns
    -------
    ETS or None
        机器人的运动学模型
    """

    if robot == "so101_5dof":
        return create_so101_5dof()
    else:
        print(f"Sorry, we don't support {robot} robot now")
        return None

def smooth_joint_motion(q_now, q_new, robot, max_joint_change=0.1):
    """
    平滑关节运动，限制单步最大变化量
    
    Parameters
    ----------
    q_now : np.ndarray
        当前关节角度
    q_new : np.ndarray
        新的关节角度
    robot : ETS
        机器人运动学模型
    max_joint_change : float
        单步允许的最大关节变化量
        
    Returns
    -------
    np.ndarray
        平滑后的关节角度
    """
    q_smoothed = q_new.copy()
    
    for i in range(len(q_new)):
        delta = q_new[i] - q_now[i]
        if abs(delta) > max_joint_change:
            delta = np.sign(delta) * max_joint_change
        q_smoothed[i] = q_now[i] + delta
    
    return q_smoothed


if __name__ == "__main__":
    robot = create_so101_5dof()
    qpos_data = np.array([0.0, -0.5, 0.5, 0.0, 0.0])
    T = robot.fkine(qpos_data)
    print(T)
