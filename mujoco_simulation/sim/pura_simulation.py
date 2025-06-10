from alexbot_sim import alexbot_sim
import threading
import sys, os, time
import numpy as np
import mujoco

tool_path = os.path.dirname(os.path.abspath(__file__)) + "/tools"
sys.path.append(tool_path)

from robot_dynamics import RobotDynamics


if __name__ == "__main__":
    print(os.path.dirname(os.path.abspath(__file__)))
    sim = alexbot_sim(os.path.dirname(os.path.abspath(__file__)) + "/../model/scene/alexbot_scene.xml")
    robot_dynamics = RobotDynamics(os.path.dirname(os.path.abspath(__file__)) + "/../model/robot_model/alexbot.urdf",os.path.dirname(os.path.abspath(__file__)) + "/../models/meshes/", True)

    robot_dynamics.print_model_info()

    # 启动仿真线程
    sim_thread = threading.Thread(target=sim.run)
    sim_thread.start()

    body_id = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, 'base_link')

    try:
        while True:
            
            # 获取仿真状态
            imu_pos = np.array(sim.data.xpos[body_id])
            imu_quat = np.array(sim.data.xquat[body_id])

            angles, vels = sim.get_joint_states()

            q = np.zeros(robot_dynamics.model.nq)
            q[:3] = imu_pos
            q[3:7] = imu_quat
            q[7:] = angles
            qd = np.zeros(robot_dynamics.model.nv)
            qd[:6] = sim.data.cvel[body_id]
            qd[6:] = vels
            # 计算动力学
            M, b = robot_dynamics.compute_dynamics(q, qd)
            # 计算前向动力学
            qdd = np.zeros(robot_dynamics.model.nv)

            # 使用逆动力学计算力矩
            tau = robot_dynamics.inverse_dynamics(q, qd, qdd)
            

            sim.set_joint_torques(tau[6:])

            # print("关节角度:", angles)
            # print("关节速度:", vels)

            time.sleep(0.001)  # 0.001秒更新一次
    except KeyboardInterrupt:
        print("退出程序")
