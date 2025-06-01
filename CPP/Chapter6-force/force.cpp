#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <mujoco/mjtnum.h>
#include <thread>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

// MuJoCo data structures
mjModel *m = NULL; // MuJoCo model
mjData *d = NULL;  // MuJoCo data
mjvCamera cam;     // abstract camera
mjvOption opt;     // visualization options
mjvScene scn;      // abstract scene
mjrContext con;    // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;

// keyboard callback
void keyboard(GLFWwindow *window, int key, int scancode, int act, int mods) {
  // backspace: reset simulation
  if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
    mj_resetData(m, d);
    mj_forward(m, d);
  }
}

// mouse button callback
void mouse_button(GLFWwindow *window, int button, int act, int mods) {
  // update button state
  button_left =
      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  button_middle =
      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  button_right =
      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

  // update mouse position
  glfwGetCursorPos(window, &lastx, &lasty);
}

// mouse move callback
void mouse_move(GLFWwindow *window, double xpos, double ypos) {
  // no buttons down: nothing to do
  if (!button_left && !button_middle && !button_right) {
    return;
  }

  // compute mouse displacement, save
  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx = xpos;
  lasty = ypos;

  // get current window size
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // get shift key state
  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

  // determine action based on mouse button
  mjtMouse action;
  if (button_right) {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (button_left) {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    action = mjMOUSE_ZOOM;
  }

  // move camera
  mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

// scroll callback
void scroll(GLFWwindow *window, double xoffset, double yoffset) {
  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

std::vector<float> get_sensor_data(const mjModel *model, const mjData *data,
                                   const std::string &sensor_name) {
  int sensor_id = mj_name2id(model, mjOBJ_SENSOR, sensor_name.c_str());
  if (sensor_id == -1) {
    std::cout << "no found sensor" << std::endl;
    return std::vector<float>();
  }
  int data_pos = model->sensor_adr[sensor_id];
  std::vector<float> sensor_data(model->sensor_dim[sensor_id]);
  for (int i = 0; i < sensor_data.size(); i++) {
    sensor_data[i] = data->sensordata[data_pos + i];
  }
  return sensor_data;
}

// main function
int main(int argc, const char **argv) {

  char error[1000] = "Could not load binary model";
  m = mj_loadXML("../../../API-MJC/force.xml", 0, error, 1000);

  // make data
  d = mj_makeData(m);

  // init GLFW
  if (!glfwInit()) {
    mju_error("Could not initialize GLFW");
  }

  // create window, make OpenGL context current, request v-sync
  GLFWwindow *window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // initialize visualization data structures
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);

  // create scene and context
  mjv_makeScene(m, &scn, 2000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  // install GLFW mouse and keyboard callbacks
  glfwSetKeyCallback(window, keyboard);
  glfwSetCursorPosCallback(window, mouse_move);
  glfwSetMouseButtonCallback(window, mouse_button);
  glfwSetScrollCallback(window, scroll);

  mj_step(m, d);
  /*--------box--------*/
  //获取施加力的body的id
  int box_id = mj_name2id(m, mjOBJ_BODY, "box");
  //施加外部力
  mjtNum box_force[3] = {0.0, 0.0, 9.8};
  mjtNum box_torque[3] = {0.0, 0.0, 0.0};
  mjtNum box_point[3] = {1.0, 0.0, 0.2};
  int red_point = mj_name2id(m, mjOBJ_SITE, "red_point");
  mjtNum *point = d->site_xpos + red_point * 3;
  // mj_applyFT(m, d, box_force, box_torque, box_point, box_id, d->qfrc_applied);
  /*--------box--------*/

  /*--------力——加速度--------*/
  //获取施加力的body的id
  int sphere_id = mj_name2id(m, mjOBJ_BODY, "sphere");
  //施加外部力
  mjtNum sphere_force[3] = {0.3, 0.0, 0.0};
  mjtNum sphere_torque[3] = {0.0, 0.0, 0.0};
  mjtNum sphere_point[3] = {0.0, 0.0, 0.0};
  // mj_applyFT(m, d, sphere_force, sphere_torque, sphere_point, sphere_id,
  //            d->qfrc_applied);
  /*--------力——加速度--------*/

  /*--------扭矩--------*/
  int pointer_id = mj_name2id(m, mjOBJ_BODY, "pointer");
  //施加外部力
  mjtNum pointer_force[3] = {0.0, 0.0, 0.0};
  mjtNum pointer_torque[3] = {0.0, 0.0, 0.0};
  mjtNum pointer_point[3] = {0.0, 0.0, 0.0};
  // mj_applyFT(m, d, pointer_force, pointer_torque, pointer_point, pointer_id,
  //            d->qfrc_applied);
  /*--------扭矩--------*/

  auto step_start = std::chrono::high_resolution_clock::now();
  while (!glfwWindowShouldClose(window)) {

    std::cout<<"nv:"<<m->nv<<"  nefc:"<<d->nefc<<std::endl;

    /*--------box--------*/
    // mju_zero(d->qfrc_applied, m->nv);
    // mj_applyFT(m, d, box_force, box_torque, point, box_id, d->qfrc_applied);
    // mj_step(m, d);
    /*--------box--------*/

    /*--------box--------*/
    // mjtNum *box_xfrc_applied = d->xfrc_applied + box_id * 6;
    // box_xfrc_applied[0] = 0.0; // fx
    // box_xfrc_applied[1] = 0.0; // fy
    // box_xfrc_applied[2] = 9.8; // fz
    // box_xfrc_applied[3] = 0.0; // tx
    // box_xfrc_applied[4] = 0.0; // ty
    // box_xfrc_applied[5] = 0.0; // tz
    // mj_step(m, d);
    /*--------box--------*/

    /*--------力——加速度--------*/
    // d->ctrl[0] = 0.6;
    // mjtNum *sphere_xfrc_applied = d->xfrc_applied + sphere_id * 6;
    // sphere_xfrc_applied[0] = 0.0; // fx
    // sphere_xfrc_applied[1] = 0.0; // fy
    // sphere_xfrc_applied[2] = 0.0; // fz
    // sphere_xfrc_applied[3] = 0.0; // tx
    // sphere_xfrc_applied[4] = 0.0; // ty
    // sphere_xfrc_applied[5] = 0.0; // tz
    // mj_step(m, d);
    // std::cout << "qfrc_passive:" << d->qfrc_passive[0]
    //           << "  qfrc_actuator:" << d->qfrc_actuator[0]
    //           << "  qfrc_applied:" << d->qfrc_applied[0]
    //           << "  qfrc_bias:" << d->qfrc_bias[0]
    //           << "  efc_force:" << d->efc_force[0] << std::endl;
    // auto lin_acc = get_sensor_data(m, d, "lin_acc");
    // std::cout << "lin_acc:" << lin_acc[0] << std::endl;
    // auto lin_vel = get_sensor_data(m, d, "lin_vel");
    // std::cout << "lin_vel:" << lin_vel[0] << std::endl;
    // auto lin_pos = get_sensor_data(m, d, "lin_pos");
    // std::cout << "lin_pos:" << lin_pos[0] << std::endl;
    // mjtNum acc = (d->qfrc_passive[0] + d->qfrc_actuator[0] +
    //               d->qfrc_applied[0] + d->qfrc_bias[0] + d->efc_force[0]) /
    //              m->body_mass[sphere_id];
    // std::cout << "计算加速度:" << acc << std::endl;
    /*--------力——加速度--------*/

    /*--------扭矩--------*/
    d->ctrl[1] = 0.6;
    mjtNum *pointer_xfrc_applied = d->xfrc_applied + pointer_id * 6;
    pointer_xfrc_applied[0] = 0.0; // fx
    pointer_xfrc_applied[1] = 0.0; // fy
    pointer_xfrc_applied[2] = 0.0; // fz
    pointer_xfrc_applied[3] = 0.0; // tx
    pointer_xfrc_applied[4] = 0.0; // ty
    pointer_xfrc_applied[5] = 0.0; // tz
    mj_step(m, d);
    std::cout << "qfrc_passive:" << d->qfrc_passive[1]
              << "  qfrc_actuator:" << d->qfrc_actuator[1]
              << "  qfrc_applied:" << d->qfrc_applied[1]
              << "  qfrc_bias:" << d->qfrc_bias[1]
              << "  efc_force:" << d->efc_force[1] << std::endl;
    mjtNum tau = d->qfrc_passive[1] + d->qfrc_actuator[1] + d->qfrc_applied[1] +
                 d->qfrc_bias[1] + d->efc_force[1];
    std::cout << "计算扭矩:" << tau << std::endl;
    auto t = get_sensor_data(m, d, "torque");
    std::cout << "测量扭矩:" << t[2] << std::endl;
    auto pivot_pos = get_sensor_data(m, d, "pivot_pos");
    std::cout << "pivot_pos:" << pivot_pos[0] << std::endl;
    auto pivot_vel = get_sensor_data(m, d, "pivot_vel");
    std::cout << "pivot_vel:" << pivot_vel[0] << std::endl;
    /*--------扭矩--------*/

    //同步时间
    auto current_time = std::chrono::high_resolution_clock::now();
    double elapsed_sec =
        std::chrono::duration<double>(current_time - step_start).count();
    double time_until_next_step = m->opt.timestep * 5 - elapsed_sec;
    if (time_until_next_step > 0.0) {
      auto sleep_duration = std::chrono::duration<double>(time_until_next_step);
      std::this_thread::sleep_for(sleep_duration);
    }

    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
  }

  // free visualization storage
  mjv_freeScene(&scn);
  mjr_freeContext(&con);

  // free MuJoCo model and data
  mj_deleteData(d);
  mj_deleteModel(m);

  // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
  glfwTerminate();
#endif

  return 1;
}
