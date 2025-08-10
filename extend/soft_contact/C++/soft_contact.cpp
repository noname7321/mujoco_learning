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
  mj_env(std::string model_file, double max_FPS = 60){
    load_model(model_file);
    set_window_size(2560,1440);
    set_window_title("MUJOCO Soft Contact");
    set_max_FPS(max_FPS);
  }
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

  // 在碰撞后第n步停止仿真
  bool is_contact_stop = true;
  int n_contact_step = 100;
  int contact_step = 0;
  void step() {
    mj_step(m, d);
    auto pos = get_sensor_data(m, d, "ball_pos");
    auto a0 = mju_norm3(m->opt.gravity);
    mjtNum err_a = 0;
    std::cout << "a0: " << a0 << std::endl;
    std::cout << "r(R-pos): " << 0.1 - pos[2] << std::endl;
    std::cout << "r(efc_pos-efc_margin): " << d->efc_pos[0] - d->efc_margin[0]
              << std::endl;
    std::cout << "n efc:" << std::endl;
    if (d->nefc > 0 && is_contact_stop) {
      contact_step++;
      if (contact_step > n_contact_step) {
        is_step.store(false);
      }
    }
    int ball_id = mj_name2id(m, mjtObj::mjOBJ_BODY, "ball");
    for (int i = 0; i < d->nefc; i++) {
      int id = d->efc_id[i];
      std::cout << "  KBIP: " << d->efc_KBIP[i * 4 + 0] << " "
                << d->efc_KBIP[i * 4 + 1] << " " << d->efc_KBIP[i * 4 + 2]
                << " " << d->efc_KBIP[i * 4 + 3] << std::endl;
      std::cout << "  efc_vel: " << d->efc_vel[i] << std::endl;
      std::cout << "  efc_force: " << d->efc_force[i] << std::endl;
      err_a = d->efc_force[i] / m->body_mass[ball_id];
      std::cout << "  efc_acc: " << err_a << std::endl;
      std::cout << "  efc_aref: " << d->efc_aref[i] << std::endl;
    }
    std::cout << "a1: " << a0 - err_a << std::endl;
  }
  void step_unlock() {}
};

// main function
int main(int argc, const char **argv) {

  mj_env mujoco("../../scene.xml",60);
  mujoco.connect_windows_sim();
  mujoco.render();
  mujoco.sim();
}
