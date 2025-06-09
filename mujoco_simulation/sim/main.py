from alexbot_sim import alexbot_sim
import threading
import sys, os, time
import numpy as np

tool_path = os.path.dirname(os.path.abspath(__file__)) + "/tools"
sys.path.append(tool_path)

from robot_dynamics import RobotDynamics


if __name__ == "__main__":
    print(os.path.dirname(os.path.abspath(__file__)))
    sim = alexbot_sim(os.path.dirname(os.path.abspath(__file__)) + "/../model/scene/alexbot_scene.xml")
    robot_dynamics = RobotDynamics(os.path.dirname(os.path.abspath(__file__)) + "/../model/robot_model/alexbot.urdf",os.path.dirname(os.path.abspath(__file__)) + "/../models/meshes/", False)

    # 启动仿真线程
    sim_thread = threading.Thread(target=sim.run)
    sim_thread.start()

    try:
        while True:

            new_torques = np.random.uniform(-0.2, 0.2, size=sim.n_joints)
            sim.set_joint_torques(new_torques)

            angles, vels = sim.get_joint_states()
            # print("关节角度:", angles)
            # print("关节速度:", vels)

            time.sleep(0.001)  # 0.001秒更新一次
    except KeyboardInterrupt:
        print("退出程序")
