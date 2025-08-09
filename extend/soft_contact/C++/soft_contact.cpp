#include "mujoco_thread.h"
#include "opencv2/opencv.hpp"
#include <GLFW/glfw3.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class mj_env : public mujoco_thread {
public:
  mj_env(std::string model_file, double max_FPS = 60)
      : mujoco_thread(model_file, max_FPS) {}
  void vis_cfg() {
    /*--------可视化配置--------*/
    // opt.flags[mjtVisFlag::mjVIS_CONTACTPOINT] = true;
    opt.flags[mjtVisFlag::mjVIS_CONTACTFORCE] = true;
    // opt.flags[mjtVisFlag::mjVIS_CAMERA] = true;
    // opt.flags[mjtVisFlag::mjVIS_CONVEXHULL] = true;
    // opt.flags[mjtVisFlag::mjVIS_COM] = true;
    opt.label = mjtLabel::mjLABEL_CONTACTFORCE;
    // opt.frame = mjtFrame::mjFRAME_WORLD;
    /*--------可视化配置--------*/

    /*--------场景渲染--------*/
    scn.flags[mjtRndFlag::mjRND_WIREFRAME] = true;
    // scn.flags[mjtRndFlag::mjRND_SEGMENT] = true;
    // scn.flags[mjtRndFlag::mjRND_IDCOLOR] = true;
    /*--------场景渲染--------*/
  }
  int cnt = 0;
  void step() {

    // cnt++;
    // if (cnt <= 220) {
    mj_step(m, d);
    auto pos = get_sensor_data(m, d, "ball_pos");
    auto acc = get_sensor_data(m, d, "ball_acc");
    auto vel = get_sensor_data(m, d, "ball_vel");
    std::cout << "acc:" << acc[2] << std::endl;
    std::cout << "vel:" << vel[2] << std::endl;
    std::cout << "r(vel-pos):" << 0.1 - pos[2] << std::endl;
    std::cout << "r(efc_pos-efc_margin):" << d->efc_pos[0] - d->efc_margin[0] << std::endl;
    std::cout << "n efc:" << std::endl;
    mjtNum efc_a = 0;
    for (int i = 0; i < d->nefc; i++) {
      int id = d->efc_id[i];
      auto cnt = d->contact[id];
      std::cout << "  KBIP:" << d->efc_KBIP[i * 4 + 0] << " "
                << d->efc_KBIP[i * 4 + 1] << " " << d->efc_KBIP[i * 4 + 2]
                << " " << d->efc_KBIP[i * 4 + 3] << std::endl;
      std::cout << "  efc_force:" << d->efc_force[i] << std::endl;
      std::cout << "  efc_aref:" << d->efc_aref[i] << std::endl;
    }
    // }
  }
  void step_unlock() {}
};

// main function
int main(int argc, const char **argv) {

  mj_env mujoco("../../scene.xml", 170);
  mujoco.connect_windows_sim();
  mujoco.render();
  mujoco.sim();
}
