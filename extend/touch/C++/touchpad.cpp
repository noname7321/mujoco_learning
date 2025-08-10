#include "mujoco_thread.h"
#include "opencv2/opencv.hpp"
#include <GLFW/glfw3.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class mj_env : public mujoco_thread {
public:
  mj_env(std::string model_file, double max_FPS = 60) {
    load_model(model_file);
    set_window_size(1920, 1080);
    set_window_title("MUJOCO Soft Contact");
    set_max_FPS(max_FPS);
    /*--------读取传感器--------*/
    std::cout << "sensor_num: " << m->nsensor << std::endl;
    auto print = [&](int num, int *adr) {
      std::cout << "num:" << num << std::endl;
      std::cout << "name:";
      for (int i = 0; i < num; i++)
        std::cout << "  " << m->names + adr[i];
      std::cout << std::endl;
    };
    // print(m->nsensor, m->name_sensoradr);
    // 获取touch_point0000位置
    int sensor_id = mj_name2id(m, mjOBJ_SENSOR, "touch_point0000");
    if (sensor_id == -1) {
      std::cout << "no found sensor" << std::endl;
    }
    for (int x = 0; x < 20; x++) {
      for (int y = 0; y < 20; y++) {
        int idx = x * 20 + y;
        touch_point_adr[x][y] = m->sensor_adr[idx];
      }
    }
    std::cout << std::endl;
  }

  int touch_point_adr[20][20];
  cv::Mat touch;

  void vis_cfg() {
    /*--------可视化配置--------*/
    opt.flags[mjtVisFlag::mjVIS_CONTACTPOINT] = true;
    opt.flags[mjtVisFlag::mjVIS_CONTACTFORCE] = true;
    // opt.flags[mjtVisFlag::mjVIS_CAMERA] = true;
    // opt.flags[mjtVisFlag::mjVIS_CONVEXHULL] = true;
    // opt.flags[mjtVisFlag::mjVIS_COM] = true;
    // opt.label = mjtLabel::mjLABEL_BODY;
    opt.frame = mjtFrame::mjFRAME_WORLD;
    /*--------可视化配置--------*/

    /*--------场景渲染--------*/
    scn.flags[mjtRndFlag::mjRND_WIREFRAME] = true;
    // scn.flags[mjtRndFlag::mjRND_SEGMENT] = true;
    // scn.flags[mjtRndFlag::mjRND_IDCOLOR] = true;
    /*--------场景渲染--------*/
  }
  void step() {
    mj_step(m, d);
    touch = cv::Mat(20, 20, CV_8UC1);
    for (int x = 0; x < 20; x++) {
      for (int y = 0; y < 20; y++) {
        int idx = x * 20 + y;
        mjtNum data = mju_norm3(d->sensordata + touch_point_adr[x][y]);
        data = mju_clip(data, 0.0, 3.0);
        touch.at<uchar>(x, y) = data / 3 * 255;
      }
    }
  }
  void step_unlock() {
    cv::resize(touch, touch, cv::Size(200, 200));
    cv::imshow("touch", touch);
    cv::waitKey(1);
  }
};

// main function
int main(int argc, const char **argv) {

  mj_env mujoco("../../touch_pad.xml", 170);
  mujoco.connect_windows_sim();
  mujoco.render();
  mujoco.sim();
}
