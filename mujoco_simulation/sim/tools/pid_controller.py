import numpy as np

class PIDController:
    def __init__(self, kp, ki, kd, dt):
        self.kp = kp  # 比例增益
        self.ki = ki  # 积分增益
        self.kd = kd  # 微分增益
        self.dt = dt  # 时间步长

        self.integral = 0.0  # 积分项
        self.previous_error = 0.0  # 上一时刻的误差

    def compute(self, setpoint, feedback_value):
        error = setpoint - feedback_value  # 当前误差

        # 计算积分项
        self.integral += error * self.dt

        # 计算微分项
        derivative = (error - self.previous_error) / self.dt

        # PID 控制输出
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        # 更新上一时刻的误差
        self.previous_error = error

        return output
    
    def reset(self):
        """重置积分和误差"""
        self.integral = 0.0
        self.previous_error = 0.0

    def set_gains(self, kp, ki, kd):
        """设置 PID 增益"""
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def get_gains(self):
        """获取当前 PID 增益"""
        return self.kp, self.ki, self.kd