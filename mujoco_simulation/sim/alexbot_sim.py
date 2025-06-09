import mujoco
import mujoco.viewer
import time
import numpy as np
import threading

class alexbot_sim:
    def __init__(self, model_path):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        self.n_joints = 10 # 关节自由度
        self.dt = self.model.opt.timestep

        # 初始化力矩存储，外部可直接访问修改
        self.current_torques = np.zeros(self.n_joints)

        # 线程锁，避免多线程冲突
        self.lock = threading.Lock()

    def update_joint_state(self):
        self.joint_angles = self.data.qpos[:self.n_joints].copy()
        self.joint_velocities = self.data.qvel[:self.n_joints].copy()

    def set_joint_torques(self, torques):
        torques = np.array(torques)
        assert torques.shape[0] == self.n_joints, "torques长度必须等于关节数"
        with self.lock:
            self.current_torques[:] = torques

    def get_joint_states(self):
        self.update_joint_state()
        return self.joint_angles, self.joint_velocities

    def step(self):
        mujoco.mj_step(self.model, self.data)

    def run(self):
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            print("开始仿真，关闭窗口以退出")
            try:
                while viewer.is_running():
                    start_time = time.time()

                    # 读力矩时加锁
                    with self.lock:
                        self.data.ctrl[:self.n_joints] = self.current_torques

                    self.step()
                    viewer.sync()

                    elapsed = time.time() - start_time
                    to_sleep = self.dt - elapsed
                    if to_sleep > 0:
                        time.sleep(to_sleep)

            except Exception as e:
                print(f"仿真异常终止: {e}")
