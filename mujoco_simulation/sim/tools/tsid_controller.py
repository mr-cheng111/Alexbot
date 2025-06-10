import numpy as np
import pinocchio as pin
import tsid
from tsid import RobotWrapper

class TSIDController:
    def __init__(self, robot):
        self.robot = robot
        self.model = robot.model
        self.data = robot.data

        self.formulation = tsid.InverseDynamicsFormulationAccForce("tsid", self.robot, False)
        self.formulation.computeProblemData(0.0, np.zeros(self.robot.nq), np.zeros(self.robot.nv))

        # 支持腿名
        self.foot_frames = ["leftlink5", "rightlink5"]
        self.contacts = {}

        for frame in self.foot_frames:
            contact = tsid.Contact6d(
            "contact_" + frame,          # contact name
            self.robot,                  # tsid.RobotWrapper instance
            frame,                       # contact frame name (e.g., "left_foot")
            np.identity(3),              # local frame placement (3x3)
            np.array([0.0, 0.0, 1.0]),   # contact normal (pointing up)
            0.3,                         # friction coefficient
            10.0,                        # min normal force
            1000.0,                      # max normal force
            1e-5                         # force regularization weight (optional, defaults to 0)
            )
            contact.setKp(100. * np.ones(6))
            contact.setKd(20. * np.ones(6))
            self.contacts[frame] = contact

        self.support_foot = "leftlink5"
        self.swing_foot = "rightlink5"
        self.in_contact = {frame: False for frame in self.foot_frames}
        self._set_contact(self.support_foot, True)
        self._set_contact(self.swing_foot, False)

        self.com_task = tsid.TaskComEquality("task-com", self.robot)
        self.com_task.setKp(100. * np.ones(3))
        self.com_task.setKd(20. * np.ones(3))
        self.formulation.addMotionTask(self.com_task, 1.0, 1, 0.0)

        self.body_task = tsid.TaskSE3Equality("task-body", self.robot, "base_link")
        self.body_task.setKp(200. * np.ones(6))
        self.body_task.setKd(40. * np.ones(6))
        self.formulation.addMotionTask(self.body_task, 1.0, 1, 0.0)

        self.swing_task = tsid.TaskSE3Equality("task-swing", self.robot, self.swing_foot)
        self.swing_task.setKp(100. * np.ones(6))
        self.swing_task.setKd(20. * np.ones(6))
        self.swing_enabled = False

        self.solver = tsid.SolverHQuadProgFast("qp solver")
        self.solver.resize(self.formulation.nVar, self.formulation.nEq, self.formulation.nIn)

        self.time = 0.0
        self.dt = 0.001
        self.step_count = 0
        self.swing_duration = 1.0  # 抬腿时长

    def _set_contact(self, foot, enable):
        if enable and not self.in_contact[foot]:
            self.formulation.addRigidContact(self.contacts[foot])
            self.in_contact[foot] = True
        elif not enable and self.in_contact[foot]:
            self.formulation.removeRigidContact(foot)
            self.in_contact[foot] = False

    def _switch_legs(self):
        self.support_foot, self.swing_foot = self.swing_foot, self.support_foot
        self._set_contact(self.support_foot, True)
        self._set_contact(self.swing_foot, False)
        self.swing_enabled = True
        self.swing_start_time = self.time
        self.swing_task = tsid.TaskSE3Equality("task-swing", self.robot, self.swing_foot)
        self.swing_task.setKp(100. * np.ones(6))
        self.swing_task.setKd(20. * np.ones(6))
        self.formulation.addMotionTask(self.swing_task, 1.0, 1.0, 0.0)

    def compute(self, q, v):
        self.time += self.dt
        HQPData = self.formulation.computeProblemData(self.time, q, v)

        # CoM 任务
        com_ref = self.robot.com(q)
        com_ref[2] = 0.6
        com_sample = tsid.TrajectorySample(3)
        com_sample.pos(com_ref)
        com_sample.vel(np.zeros(3))
        com_sample.acc(np.zeros(3))
        self.com_task.setReference(com_sample)

        # 姿态任务
        p_des = pin.SE3(np.eye(3), np.array([0., 0., 0.6]))
        sample = tsid.TrajectorySample(6, 6)
        sample.pos(p_des)
        sample.vel(np.zeros(6))
        sample.acc(np.zeros(6))
        self.body_task.setReference(sample)

        # Swing leg 抬腿轨迹
        if self.swing_enabled:
            t_swing = self.time - self.swing_start_time
            foot_pos = self.robot.framePosition(q, self.model.getFrameId(self.swing_foot)).translation
            foot_traj = np.copy(foot_pos)
            foot_traj[2] += 0.05 * np.sin(np.pi * t_swing / self.swing_duration)
            traj_sample = tsid.TrajectorySample(6, 6)
            traj_sample.pos(pin.SE3(np.eye(3), foot_traj))
            traj_sample.vel(np.zeros(6))
            traj_sample.acc(np.zeros(6))
            self.swing_task.setReference(traj_sample)

            if t_swing >= self.swing_duration:
                # 放下脚并切换支撑腿
                self.formulation.removeMotionTask("task-swing", 0.0)
                self._switch_legs()
                self.swing_enabled = False

        # QP 解算
        sol = self.solver.solve(HQPData)
        if not sol.success:
            print("[TSID] QP 解算失败")
            return None, None

        tau = self.formulation.getActuatorForces(sol)
        dv = self.formulation.getAccelerations(sol)
        return tau, dv
