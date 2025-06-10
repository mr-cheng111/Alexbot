// src/robot_dynamics.cpp
#include "robot_dynamics.h"
#include <iostream>

RobotDynamics::RobotDynamics(const std::string& urdf_path, const std::string& mesh_dir, bool floating_base) 
{
    if (floating_base) 
    {
        pinocchio::JointModelFreeFlyer root_joint;      // 定义浮动基关节
        std::string root_joint_name = "root_joint";     // 必须指定名称

        // 解析URDF并构建带浮动基的模型
        model = pinocchio::urdf::buildModel(urdf_path, root_joint, root_joint_name, model, true);
        } 
    else 
    {
        model = pinocchio::urdf::buildModel(urdf_path, model);

    }
    data = pinocchio::Data(model);
    std::cout << "[RobotDynamics] Model loaded: " << urdf_path << "\n";
}

void RobotDynamics::printModelInfo() 
{
    std::cout << "nq = " << model.nq << ", nv = " << model.nv << "\n";
    for (const auto& name : model.names) std::cout << "- " << name << "\n";
}

void RobotDynamics::computeDynamics(const Eigen::VectorXd& q, const Eigen::VectorXd& qd, Eigen::MatrixXd& M, Eigen::VectorXd& b) 
{
    pinocchio::computeAllTerms(model, data, q, qd);
    M = data.M;
    b = pinocchio::nonLinearEffects(model, data, q, qd);
}

Eigen::VectorXd RobotDynamics::inverseDynamics(const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const Eigen::VectorXd& qdd) 
{
    return pinocchio::rnea(model, data, q, qd, qdd);
}
