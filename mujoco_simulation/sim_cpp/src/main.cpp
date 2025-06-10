#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>
#include "robot_sim.h"
#include "robot_dynamics.h"

volatile bool run_flag = true;
void signalHandler(int signum) 
{
    run_flag = false;
}

int main(int argc, char** argv)
{
    if(argc < 3)
    {
        std::cerr << "Usage: " << argv[0] << " <mujoco_xml_path> <urdf_path>" << std::endl;
        return -1;
    }

    std::string mujoco_xml_path = argv[1];
    std::string urdf_path = argv[2];

    // 初始化仿真和动力学
    robot_sim sim(mujoco_xml_path);
    RobotDynamics robot_dynamics(urdf_path, "", true);

    robot_dynamics.printModelInfo();

    // 注册信号中断，优雅退出
    std::signal(SIGINT, signalHandler);

    // 启动仿真线程
    std::thread sim_thread([&sim]() 
    {
        sim.run();
    });

    int body_id = mj_name2id(sim.model, mjOBJ_BODY, "base_link");
    std::vector<double> angles(sim.model->nq - 7, 0.0); // 假设有7个关节角度需要设置
    std::vector<double> vels(sim.model->nv - 6, 0.0); // 假设有6个关节速度需要设置
    // 获取 base_link 的body id
    double* imu_pos = sim.data->xpos + 3 * body_id;
    // 四元数：4个double (w, x, y, z)
    double* imu_quat = sim.data->xquat + 4 * body_id;
    Eigen::VectorXd q = Eigen::VectorXd::Zero(robot_dynamics.model.nq);
    Eigen::VectorXd qd = Eigen::VectorXd::Zero(robot_dynamics.model.nv);

    try
    {
        while(run_flag)
        {
            sim.getJointStates(angles, vels);
            
            // 构造q和qd
            q.segment<3>(0) = Eigen::Map<Eigen::Vector3d>(imu_pos);
            q.segment<4>(3) = Eigen::Map<Eigen::Vector4d>(imu_quat);

            for(int i=7; i<robot_dynamics.model.nq; ++i)
                q[i] = angles[i - 7];

            for(int i=6; i<robot_dynamics.model.nv; ++i)
                qd[i] = vels[i - 6];

            // 计算动力学
            Eigen::MatrixXd M;
            Eigen::VectorXd b;
            robot_dynamics.computeDynamics(q, qd, M, b);

            // 逆动力学
            Eigen::VectorXd qdd = Eigen::VectorXd::Zero(robot_dynamics.model.nv);
            Eigen::VectorXd tau = robot_dynamics.inverseDynamics(q, qd, qdd);
            Eigen::VectorXd tau_joints = tau.segment(6, tau.size() - 6); // 去除前6个关节的力矩
            std::vector<double> tau_vec(tau_joints.data(), tau_joints.data() + tau_joints.size());

            // 发送力矩，去除浮动基部分
            sim.setJointTorques(tau_vec);

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    catch(...)
    {
        std::cerr << "Exception caught, exiting..." << std::endl;
    }

    // 退出，等待仿真线程结束
    sim_thread.join();

    return 0;
}

