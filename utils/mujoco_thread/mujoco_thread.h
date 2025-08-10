#pragma once
#include <GLFW/glfw3.h>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <mujoco/mujoco.h>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <tuple>

class mujoco_thread {
  friend class mj_env;

public:
  mujoco_thread() = default;
  mujoco_thread(std::string model_file, double max_FPS = 60, int width = 1200,
                int height = 900, std::string title = "MUJOCO");
  ~mujoco_thread();

  void load_model(std::string model_file);
  void load_model(mjModel *m);
  void set_window_size(int width, int height);
  void set_window_title(std::string title);
  void set_max_FPS(double max_FPS);

  // 调用之后关闭窗口会停止仿真
  void connect_windows_sim();
  void sim();
  virtual void step() = 0;
  virtual void step_unlock();
  virtual void vis_cfg();
  // 渲染
  void render();
  int id = 0;
  void close_render();

private:
  // MuJoCo data structures
  mjModel *m = nullptr; // MuJoCo model
  mjData *d = nullptr;  // MuJoCo data
  mjvCamera cam;        // abstract camera
  mjvOption opt;        // visualization options
  mjvScene scn;         // abstract scene
  mjrContext con;       // custom GPU context

  // mouse interaction
  bool button_left = false;
  bool button_middle = false;
  bool button_right = false;
  double lastx = 0;
  double lasty = 0;
  double last_mouse_x = 0;
  double last_mouse_y = 0;
  bool ctrl_pressed = false;
  int selected_body = -1;
  double last_click_time = 0;

  double fps = 0.0;
  int width = 1200;
  int height = 900;
  std::string title = "MUJOCO";

  GLFWwindow *window = nullptr;
  std::mutex m_mtx;
  std::atomic_bool is_step{true};
  std::atomic_bool is_sim{true};
  std::atomic_bool connect_windows{false}; // 是否关闭窗口停止仿真

  // 窗口操作

  static void static_keyboard(GLFWwindow *window, int key, int scancode,
                              int act, int mods);
  static void static_mouse_move(GLFWwindow *window, double xpos, double ypos);
  static void static_mouse_button(GLFWwindow *window, int button, int act,
                                  int mods);
  static void static_scroll(GLFWwindow *window, double xoffset, double yoffset);

  void keyboard(int key, int scancode, int act, int mods);
  void mouse_move(double xpos, double ypos);
  void mouse_button(int button, int act, int mods);
  void scroll(double xoffset, double yoffset);

  // 可视化
  std::atomic_bool is_show{false};
  // 可以销毁信号
  std::atomic_bool is_render_close{false};
  void destroyRender();
  void initRender(int width, int height, std::string title);
  std::thread render_thread;

  // 计算并更新 FPS
  double max_FPS = 60;
  double min_render_time; // ms
  void updateRender();

  std::vector<mjtNum> get_sensor_data(const mjModel *model, const mjData *data,
                                      const std::string &sensor_name);
  void draw_line(mjvScene *scn, mjtNum *from, mjtNum *to, float rgba[4]);
};
