import numpy as np
import time
import threading
import sys, os
import mujoco
import pinocchio as pin
import tsid


tool_path = os.path.dirname(os.path.abspath(__file__)) + "/tools"
sys.path.append(tool_path)

from alexbot_sim import alexbot_sim
from robot_dynamics import RobotDynamics
from tsid_controller import TSIDController  # 假设你把TSIDController封装在这个文件中

def rotation_from_quat(q):
    return pin.Quaternion(q).matrix()

if __name__ == "__main__":
    base_path = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(base_path, "../model/robot_model/alexbot.urdf")
    mesh_dir = os.path.join(base_path, "../models/meshes/")
    xml_path = os.path.join(base_path, "../model/scene/alexbot_scene.xml")

    # 加载仿真和模型
    sim = alexbot_sim(xml_path)
    robot_dynamics = RobotDynamics(urdf_path, mesh_dir, True)
    tsid_ctrl = TSIDController(robot_dynamics.robot)

    # 启动仿真线程
    sim_thread = threading.Thread(target=sim.run)
    sim_thread.start()

    model = sim.model
    data = sim.data
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'base_link')

    try:
        while True:
            # 获取仿真状态
            angles, vels = sim.get_joint_states()
            imu_pos = np.array(data.xpos[body_id])
            imu_quat = np.array(data.xquat[body_id])
            imu_rot = rotation_from_quat(imu_quat)

            # 构建状态向量
            q_pin = np.zeros(robot_dynamics.model.nq)
            q_pin[:3] = imu_pos
            q_pin[3:7] = imu_quat
            q_pin[7:] = angles

            v_pin = np.zeros(robot_dynamics.model.nv)
            v_pin[:6] = 0  # base速度暂时设为0
            v_pin[6:] = vels

            # TSID 控制
            tau, dv = tsid_ctrl.compute(sim.data.time, q_pin, v_pin, imu_rot)
            if tau is not None:
                sim.set_joint_torques(tau[:robot_dynamics.n_actuated])  # 只取10个关节的力矩

            time.sleep(0.002)
    except KeyboardInterrupt:
        print("退出程序")
