"""
Arm - 舵机驱动、逆运动学和遥操作节点

主要模块:
- driver: FT 舵机串口驱动
- ik: 逆运动学求解器
- follower_node: follower 遥操作 ROS2 节点
"""

from .driver.ftservo_controller import ServoController
from .driver.ftservo_driver import FTServo
from .ik.robot import Robot, create_so101_5dof, get_robot, smooth_joint_motion
from .ik.solver import IKResult, ikine_LM, ikine_GN, ikine_NR, ikine_QP
from .ik.et import ET, ETS

__version__ = "0.2.0"
__all__ = [
    "ServoController",
    "FTServo",
    "Robot",
    "create_so101_5dof",
    "get_robot",
    "smooth_joint_motion",
    "IKResult",
    "ikine_LM",
    "ikine_GN",
    "ikine_NR",
    "ikine_QP",
    "ET",
    "ETS",
]
