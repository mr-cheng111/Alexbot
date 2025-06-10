// robot_sim.h
#pragma once

#include <string>
#include <vector>
#include <mutex>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

class robot_sim 
{
public:
    mjModel* model = nullptr;
    mjData* data = nullptr;

public:
    explicit robot_sim(const std::string& model_path);
    ~robot_sim();

    // 运行主循环（仿真+渲染）
    void run();

    // 设置关节力矩输入
    void setJointTorques(const std::vector<double>& torques);

    // 获取当前关节位置和速度
    void getJointStates(std::vector<double>& q, std::vector<double>& qd);
    // GLFW回调函数
    void keyboardCB(GLFWwindow* window, int key, int scancode, int act, int mods);
    void mouseButtonCB(GLFWwindow* window, int button, int act, int mods);
    void cursorPosCB(GLFWwindow* window, double xpos, double ypos);
    void scrollCB(GLFWwindow* window, double xoffset, double yoffset);
    void applyInteractionForce(double dx, double dy, bool is_force, bool mod_ctrl);
    // 新增双击选中和拖动相关回调
    void handleMouseButton(int button, int action, int mods, double xpos, double ypos);
    void handleCursorPos(double xpos, double ypos);

    // 施加力和力矩
    void applyForceToSelected(const mjtNum force[3]);
    void applyTorqueToSelected(const mjtNum torque[3]);

    // 在render中调用，绘制力和力矩箭头
    void renderForceTorque();
    
private:
    // 初始化GLFW和MuJoCo GUI相关资源
    void initGUI();

    // 进行一步仿真
    void step();
    // 渲染当前仿真状态
    void render();
    bool button_left = false;
    bool button_middle = false;
    bool button_right = false;
    // 鼠标位置
    double cursor_xpos = 0;
    double cursor_ypos = 0;
    int selected_geomid = -1;      // 当前选中物体的几何体ID
    double last_click_time = 0.0;  // 上次点击时间，用于双击判断
    double click_threshold = 0.3;  // 双击时间阈值，单位秒

    double mouse_start_x = 0;      // 鼠标拖动起始点x
    double mouse_start_y = 0;      // 鼠标拖动起始点y
    bool is_dragging = false;      // 是否处于拖动状态

    // 当前施加的力和力矩（世界坐标系）
    mjtNum applied_force[3] = {0,0,0};
    mjtNum applied_torque[3] = {0,0,0};
    // 鼠标点击时间
    double last_click_time = 0;
    int selected_body = -1;
    mjtNum selection_point[3];
    
private:
    int n_joints;
    double dt;

    std::vector<double> current_torques;
    std::mutex mtx;

    // MuJoCo可视化结构
    mjvCamera cam;
    mjvOption opt;
    mjvScene scn;
    mjrContext con;

    // MuJoCo GUI相关
    int width, height;


    // GLFW窗口
    GLFWwindow* window = nullptr;
};
