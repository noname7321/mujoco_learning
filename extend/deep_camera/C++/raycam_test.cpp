#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <mujoco/mjmodel.h>
#include <mujoco/mjrender.h>
#include <mujoco/mjspec.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>
#include <opencv2/highgui.hpp>
#include <string>
#include <thread>

#include "opencv2/opencv.hpp"
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include "RayCasterCamera.hpp"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

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

void get_cam_image(mjvCamera *cam, int width, int height, int stereo) {
  mjrRect viewport2 = {0, 0, width, height};
  int before_stereo = scn.stereo;
  scn.stereo = stereo;
  // mujoco更新渲染
  mjv_updateCamera(m, d, cam, &scn);
  mjr_render(viewport2, &scn, &con);
  scn.stereo = before_stereo;
  // 渲染完成读取图像
  unsigned char *rgbBuffer = new unsigned char[width * height * 3];
  float *depthBuffer = new float[width * height];
  mjr_readPixels(rgbBuffer, depthBuffer, viewport2, &con);

  // 创建浮点类型的深度图像
  cv::Mat depth_float(height, width, CV_32FC1, depthBuffer);

  // 定义深度范围（0-8米）
  const float min_depth_m = 0.0f; // 最小深度（0米）
  const float max_depth_m = 8.0f; // 最大深度（8米）

  // 假设相机参数（根据你的实际设置调整）
  const float near_clip = 0.1f; // 近裁剪面（米）
  const float far_clip = 50.0f; // 远裁剪面（米）

  // 将非线性深度缓冲区值转换为线性深度（米）
  cv::Mat linear_depth_m = depth_float.clone();
  linear_depth_m = far_clip * near_clip /
                   (far_clip - (far_clip - near_clip) * linear_depth_m);

  // 裁剪深度到0-8米范围
  cv::Mat depth_clipped = linear_depth_m.clone();
  depth_clipped.setTo(min_depth_m, linear_depth_m < min_depth_m); // 小于0设为0
  depth_clipped.setTo(max_depth_m, linear_depth_m > max_depth_m); // 大于8设为8

  // 映射0-8米到0-255像素值（距离越小越亮）
  cv::Mat depth_visual;
  // 计算映射参数：scale = 255/(max_depth_m - min_depth_m), shift = 0
  float scale = 255.0f / (max_depth_m - min_depth_m);
  // 反转映射：距离越小值越大（越亮）
  cv::Mat inverted_depth = max_depth_m - depth_clipped;
  inverted_depth.convertTo(depth_visual, CV_8UC1, scale);
  cv::flip(depth_visual, depth_visual, 0);
  // 显示深度图
  // cv::imshow("deep camera2", depth_visual);

  cv::Mat image(height, width, CV_8UC3, rgbBuffer);
  // 反转图像以匹配OpenGL渲染坐标系
  cv::flip(image, image, 0);
  // 颜色顺序转换这样要使用bgr2rgb而不是rgb2bgr
  cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
  cv::imshow("Image", image);
  cv::waitKey(1);
  // 释放内存
  delete[] rgbBuffer;
  delete[] depthBuffer;
}

// main function
int main(int argc, const char **argv) {

  char error[1000] = "Could not load binary model";
  m = mj_loadXML("../../deep_ray.xml", 0, error, 1000);

  // make data
  d = mj_makeData(m);

  // init GLFW
  if (!glfwInit()) {
    mju_error("Could not initialize GLFW");
  }

  // create window, make OpenGL context current, request v-sync
  GLFWwindow *window = glfwCreateWindow(2560, 1440, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // initialize visualization data structures
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);

  // create scene and context
  mjv_makeScene(m, &scn, 20000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  // install GLFW mouse and keyboard callbacks
  glfwSetKeyCallback(window, keyboard);
  glfwSetCursorPosCallback(window, mouse_move);
  glfwSetMouseButtonCallback(window, mouse_button);
  glfwSetScrollCallback(window, scroll);

  /*--------可视化配置--------*/
  // opt.flags[mjtVisFlag::mjVIS_CONTACTPOINT] = true;
  opt.flags[mjtVisFlag::mjVIS_CAMERA] = true;
  // opt.flags[mjtVisFlag::mjVIS_CONVEXHULL] = true;
  // opt.flags[mjtVisFlag::mjVIS_COM] = true;
  // opt.label = mjtLabel::mjLABEL_BODY;
  opt.frame = mjtFrame::mjFRAME_CAMERA;
  /*--------可视化配置--------*/

  /*--------场景渲染--------*/
  // scn.flags[mjtRndFlag::mjRND_WIREFRAME] = true;
  // scn.flags[mjtRndFlag::mjRND_SEGMENT] = true;
  // scn.flags[mjtRndFlag::mjRND_IDCOLOR] = true;
  /*--------场景渲染--------*/

#define box_num 5
  int box_idx[5];
  for (int i = 0; i < 5; i++) {
    std::string geom_name = "box" + std::to_string(i + 1);
    box_idx[i] = mj_name2id(m, mjOBJ_GEOM, geom_name.c_str());
  }
  mjtNum *boxs_pos[box_num];
  for (int i = 0; i < 5; i++) {
    boxs_pos[i] = d->geom_xpos + box_idx[i] * 3;
  }

  // 相机初始化
  mjvCamera cam2;
  int camID = mj_name2id(m, mjOBJ_CAMERA, "look_box");
  if (camID == -1) {
    std::cerr << "Camera not found" << std::endl;
  } else {
    mjv_defaultCamera(&cam2);
    cam2.fixedcamid = camID;
    cam2.type = mjCAMERA_FIXED;
  }

  mjtNum dis_range[2] = {0.01, 8};
  RayCasterCamera dc(m, d, camID, 24.0, 20.955, 16.0/9.0, 160, 90, dis_range);
  while (!glfwWindowShouldClose(window)) {
    auto step_start = std::chrono::high_resolution_clock::now();

    mj_step(m, d);

    get_cam_image(&cam2, 1000, 500, mjtStereo::mjSTEREO_NONE);

    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);

    float rgba[4] = {0, 0, 1, 0.5};

    // 深度相机绘制
    //  dc.compute_ray_vec();
    // 计算时长 ms
    auto start = std::chrono::high_resolution_clock::now();
    dc.get_distance();
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "nray:" << dc.nray << "  get_distance time: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end -
                                                                       start)
                     .count()
              << "ms" << std::endl;

    cv::Mat image = dc.get_image();
    cv::resize(image, image, cv::Size(800, 450));
    cv::imshow("deep camera", image);
    cv::waitKey(1);
    dc.draw_deep_ray(&scn, 1, true);
    // dc.draw_deep(&scn,10);

    mjr_render(viewport, &scn, &con);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();

    // 同步时间
    auto current_time = std::chrono::high_resolution_clock::now();
    double elapsed_sec =
        std::chrono::duration<double>(current_time - step_start).count();
    double time_until_next_step = m->opt.timestep - elapsed_sec;
    if (time_until_next_step > 0.0) {
      auto sleep_duration = std::chrono::duration<double>(time_until_next_step);
      std::this_thread::sleep_for(sleep_duration);
    }
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
