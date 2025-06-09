import numpy as np
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper

class RobotDynamics:
    def __init__(self, urdf_path, package_dirs=None, floating_base=True):
        if package_dirs is None:
            package_dirs = []

        # 使用浮动基还是固定 base_link
        if floating_base:
            print("[RobotDynamics] Using floating base model.")
            root_joint = pin.JointModelFreeFlyer()
            self.robot = RobotWrapper.BuildFromURDF(urdf_path, package_dirs, root_joint)
        else:
            self.robot = RobotWrapper.BuildFromURDF(urdf_path, package_dirs)
        # 构造机器人模型
        
        self.model = self.robot.model
        self.data = self.robot.data

        self.nq = self.model.nq
        self.nv = self.model.nv

        print(f"[RobotDynamics] Loaded model from: {urdf_path}")
        print(f"  nq = {self.nq}, nv = {self.nv}")

    def get_dof(self):
        return self.nv

    def compute_dynamics(self, q, qd):
        pin.computeAllTerms(self.model, self.data, q, qd)
        M = self.data.M.copy()
        b = pin.nonLinearEffects(self.model, self.data, q, qd)
        return M, b

    def forward_dynamics(self, q, qd, tau):
        M, b = self.compute_dynamics(q, qd)
        qdd = np.linalg.solve(M, tau - b)
        return qdd

    def inverse_dynamics(self, q, qd, qdd):
        tau = pin.rnea(self.model, self.data, q, qd, qdd)
        return tau

    def print_model_info(self):
        print("Degrees of freedom (nv):", self.nv)
        print("Configuration space dimension (nq):", self.nq)
        print("Joint names:")
        for name in self.model.names:
            print("  -", name)
