#include "mujoco_thread.h"
#include "opencv2/opencv.hpp"
#include <GLFW/glfw3.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class mj_env : public mujoco_thread {
public:
  mj_env(std::string model_file, double max_FPS = 60)
      : mujoco_thread(model_file, max_FPS) {
    /*--------读取信息--------*/
    std::cout << "n flex: " << m->nflex << std::endl;
    std::cout << "n flex elem: " << m->nflexelem << std::endl;
    std::cout << "n flex elemdata: " << m->nflexelemdata << std::endl;
    std::cout << "n flex nflexvert: " << m->nflexvert << std::endl;
  }
  void vis_cfg() {
    /*--------可视化配置--------*/
    // opt.flags[mjtVisFlag::mjVIS_CONTACTPOINT] = true;
    // opt.flags[mjtVisFlag::mjVIS_CONTACTFORCE] = true;
    // opt.flags[mjtVisFlag::mjVIS_CAMERA] = true;
    // opt.flags[mjtVisFlag::mjVIS_CONVEXHULL] = true;
    // opt.flags[mjtVisFlag::mjVIS_COM] = true;
    // opt.label = mjtLabel::mjLABEL_BODY;
    // opt.frame = mjtFrame::mjFRAME_WORLD;
    /*--------可视化配置--------*/

    /*--------场景渲染--------*/
    scn.flags[mjtRndFlag::mjRND_WIREFRAME] = true;
    // scn.flags[mjtRndFlag::mjRND_SEGMENT] = true;
    // scn.flags[mjtRndFlag::mjRND_IDCOLOR] = true;
    /*--------场景渲染--------*/
  }
  cv::Mat touch_elem, touch_vert;
  int x_n_elem = 30,y_n_elem = 15;
  int x_n_vert = 16,y_n_vert = 16;
  void step() {
    mj_step(m, d);
    // std::cout << "contact msg" << std::endl;
    touch_elem = cv::Mat::zeros(cv::Size(x_n_elem, y_n_elem), CV_8UC1);
    touch_vert = cv::Mat::zeros(cv::Size(x_n_vert, y_n_vert), CV_8UC1);
    mjContact *cont;
    for (int j = 0; j < d->ncon; j++) {
      cont = d->contact + j;
      if (cont->flex[0] != -1 || cont->flex[1] != -1) {
        if (cont->elem[0] != -1 || cont->elem[1] != -1) {
          for (int k = 0; k < 2; k++) {
            if (cont->elem[k] != -1) {
              mjtNum force_torque[6];
              mj_contactForce(m, d, j, force_torque);
              mjtNum force = mju_norm3(force_torque);
              int y = (int)(cont->elem[k] / x_n_elem);
              int x = cont->elem[k] % x_n_elem;
              touch_elem.at<uchar>(y, x) = mju_clip(force / 3 * 255, 0.0, 255.0);
            }
          }
        } else if (cont->vert[0] != -1 || cont->vert[1] != -1) {
          for (int k = 0; k < 2; k++) {
            if (cont->vert[k] != -1) {
              mjtNum force_torque[6];
              mj_contactForce(m, d, j, force_torque);
              mjtNum force = mju_norm3(force_torque);
              int y = (int)(cont->vert[k] / x_n_vert);
              int x = cont->vert[k] % x_n_vert;
              if(x<x_n_vert&&y<y_n_vert)
                touch_vert.at<uchar>(y, x) = mju_clip(force / 1 * 255, 0.0, 255.0);
            }
          }
        }
      }
    }
  }
  void step_unlock() {
    cv::resize(touch_elem, touch_elem, cv::Size(300, 300));
    cv::resize(touch_vert, touch_vert, cv::Size(320, 320));
    cv::imshow("touch_elem", touch_elem);
    cv::imshow("touch_vert", touch_vert);
    cv::waitKey(1);
  }
};

// main function
int main(int argc, const char **argv) {

  mj_env mujoco("../../touch_flex.xml", 170);
  mujoco.connect_windows_sim();
  mujoco.render();
  mujoco.sim();
}
