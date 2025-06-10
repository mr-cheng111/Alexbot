// src/robot_sim.cpp
#include "robot_sim.h"
#include <iostream>
#include <chrono>
#include <thread>

robot_sim::robot_sim(const std::string& model_path) 
{
    model = mj_loadXML(model_path.c_str(), 0, 0, 0);
    if (!model) throw std::runtime_error("Failed to load MuJoCo model");
    data = mj_makeData(model);
    n_joints = 10;
    dt = model->opt.timestep;
    current_torques.resize(n_joints, 0.0);

    // 初始化MuJoCo可视化相关对象
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_makeScene(model, &scn, 1000);
    mjr_defaultContext(&con);
}

robot_sim::~robot_sim() 
{
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    if (window) {
        glfwDestroyWindow(window);
        glfwTerminate();
    }

    mj_deleteData(data);
    mj_deleteModel(model);
}

void robot_sim::initGUI()
{
    glfwSetErrorCallback([](int error, const char* desc) 
    {
        std::cerr << "GLFW error " << error << ": " << desc << std::endl;
    });

    if (!glfwInit()) 
    {
        throw std::runtime_error("Failed to initialize GLFW");
    }

    window = glfwCreateWindow(1024, 768, "MuJoCo Robot Simulation", nullptr, nullptr);
    if (!window) 
    {
        glfwTerminate();
        throw std::runtime_error("Failed to create GLFW window");
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // vsync

    // Esc键关闭窗口回调
    glfwSetKeyCallback(window, [](GLFWwindow* win, int key, int scancode, int action, int mods)
    {
        if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) 
        {
            glfwSetWindowShouldClose(win, GLFW_TRUE);
        }
    });


    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(this->model, &scn, 10000);                // space for 2000 objects
    mjr_makeContext(this->model, &con, mjFONTSCALE_150);   // model-specific context

    
    // 2. 加载可视化回调函数
    // install GLFW mouse and keyboard callbacks
    glfwSetWindowUserPointer(window, this);
    glfwSetKeyCallback(window, [](GLFWwindow* win, int key, int scancode, int act, int mods) 
    {
        auto sim = static_cast<robot_sim*>(glfwGetWindowUserPointer(win));
        if (sim) sim->keyboardCB(win, key, scancode, act, mods);
    });

    glfwSetMouseButtonCallback(window, [](GLFWwindow* win, int button, int action, int mods) 
    {
        auto sim = static_cast<robot_sim*>(glfwGetWindowUserPointer(win));
        if (sim) sim->mouseButtonCB(win, button, action, mods);
    });

    glfwSetCursorPosCallback(window, [](GLFWwindow* win, double xpos, double ypos) 
    {
        auto sim = static_cast<robot_sim*>(glfwGetWindowUserPointer(win));
        if (sim) sim->cursorPosCB(win, xpos, ypos);
    });

    glfwSetScrollCallback(window, [](GLFWwindow* win, double xoffset, double yoffset) 
    {
        auto sim = static_cast<robot_sim*>(glfwGetWindowUserPointer(win));
        if (sim) sim->scrollCB(win, xoffset, yoffset);
    });

    // 3. 设置初始化条件
    // frame
    opt.frame = mjFRAME_WORLD;
    // camera parameters
    // 注意：相机的坐标以及缩放参数可根据合适的值调整，这个可以在实时仿真中输出并打印
    double arr_view[] = {8.0, 90.0, -45.0, 0.000000, 0.000000, 0.000000};
    cam.distance = arr_view[0];
    cam.azimuth = arr_view[1];
    cam.elevation = arr_view[2];
    cam.lookat[0] = arr_view[3];
    cam.lookat[1] = arr_view[4];
    cam.lookat[2] = arr_view[5];

    glfwGetFramebufferSize(window, &width, &height);

}

void robot_sim::step() 
{
    mj_step(model, data);
}

void robot_sim::render()
{
    // 更新场景
    mjv_updateScene(model, data, &opt, nullptr, &cam, mjCAT_ALL, &scn);

    // 获取窗口大小
    mjrRect viewport = {0, 0, width, height};

    // 清屏并渲染
    mjr_render(viewport, &scn, &con);
}

void robot_sim::run() 
{
    initGUI();
    glfwMakeContextCurrent(window);

    std::cout << "Starting simulation and rendering loop..." << std::endl;

    while (!glfwWindowShouldClose(window)) 
    {
        // 上锁写控制量
        {
            std::lock_guard<std::mutex> lock(mtx);
            for (int i = 0; i < n_joints; ++i) {
                data->ctrl[i] = current_torques[i];
            }
        }

        step();

        // 渲染
        render();

        glfwSwapBuffers(window);
        glfwPollEvents();

        // 限制仿真步长，避免跑得太快
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void robot_sim::setJointTorques(const std::vector<double>& torques) 
{
    if (torques.size() != n_joints) throw std::runtime_error("Torque size mismatch");
    std::lock_guard<std::mutex> lock(mtx);
    current_torques = torques;
}

void robot_sim::getJointStates(std::vector<double>& q, std::vector<double>& qd) 
{
    q.resize(n_joints);
    qd.resize(n_joints);
    for (int i = 0; i < n_joints; ++i) 
    {
        q[i] = data->qpos[i];
        qd[i] = data->qvel[i];
    }
}
// keyboard callback
void robot_sim::keyboardCB(GLFWwindow* window, int key, int scancode, int act, int mods) 
{
    // backspace: reset simulation
    if(act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE) 
    {
        mj_resetData(model, data);
        mj_forward(model, data);
    }
}

// mouse button callback
void robot_sim::mouseButtonCB(GLFWwindow* window, int button, int act, int mods) 
{
    button_left   = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right  = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
    glfwGetCursorPos(window, &cursor_xpos, &cursor_ypos);

    this->handleMouseButton(button, act, mods, cursor_xpos, cursor_ypos);

    if (button == GLFW_MOUSE_BUTTON_LEFT && act == GLFW_PRESS) 
    {
        double current_time = glfwGetTime();
        if (current_time - last_click_time < 0.3) 
        {
            // 双击
            int x = static_cast<int>(cursor_xpos);
            int y = static_cast<int>(cursor_ypos);

            
            float aspect = (float)width / (float)height;
            float relx = (float)x / (float)width;
            float rely = 1.0f - (float)y / (float)height;

            mjtNum selpnt[3] = {0};
            int geomid[1] = {-1};
            int flexid[1] = {-1};
            int skinid[1] = {-1};

            selected_body = mjv_select(
                model, data, &opt, aspect,
                relx, rely,
                &scn,
                selpnt,
                geomid,
                flexid,
                skinid);

            if (selected_body >= 0) {
                std::cout << "Selected body ID: " << selected_body << std::endl;
                std::copy(selpnt, selpnt + 3, selection_point);

                mjs_addLight(model->bo, const mjsDefault* defspec) 

            }
        }
        last_click_time = current_time;
    }
}

// mouse move callback
void robot_sim::cursorPosCB(GLFWwindow* window, double xpos, double ypos) 
{
   double dx = xpos - cursor_xpos;
    double dy = ypos - cursor_ypos;
    cursor_xpos = xpos;
    cursor_ypos = ypos;

    if (!button_left && !button_right) return;
    if (selected_body < 0) return;

    bool mod_ctrl = (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS ||
                     glfwGetKey(window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS);

    // 左键: 施加力，右键: 施加转矩
    if (mod_ctrl) 
    {
        if (button_left) 
        {
            applyInteractionForce(dx, dy, true, mod_ctrl);
        } 
        else if (button_right) 
        {
            applyInteractionForce(dx, dy, false, mod_ctrl);
        }
    } 
    else 
    {
        // 正常相机控制
        int width, height;
        glfwGetWindowSize(window, &width, &height);

        bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                          glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

        mjtMouse action;
        if(button_right)
            action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
        else if(button_left)
            action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
        else
            action = mjMOUSE_ZOOM;

        mjv_moveCamera(model, action, dx/height, dy/height, &scn, &cam);
    }
}

// scroll callback
void robot_sim::scrollCB(GLFWwindow* window, double xoffset, double yoffset) 
{
    // emulate vertical mouse motion = 5% of window height
    mjtMouse action = mjMOUSE_ZOOM;
    mjv_moveCamera(model, action, 0, -0.05*yoffset, &scn, &cam);
}

void robot_sim::applyInteractionForce(double dx, double dy, bool is_force, bool mod_ctrl) {
    if (selected_body < 0) return;

    // 获取 body 的 ID 和参考点
    int id = model->body_dofadr[selected_body];

    // 设置施加的力或转矩
    mjtNum* xfrc = data->xfrc_applied + 6 * selected_body;

    double scale = 0.01;  // 力/转矩放缩因子

    if (is_force) {
        xfrc[0] = scale * dx; // fx
        xfrc[1] = scale * -dy; // fy
        xfrc[2] = 0;
    } else {
        xfrc[3] = scale * dx; // mx
        xfrc[4] = scale * -dy; // my
        xfrc[5] = 0;
    }
}
