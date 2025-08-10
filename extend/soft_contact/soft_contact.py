import time
import mujoco
import mujoco.viewer

m = mujoco.MjModel.from_xml_path('./scene.xml')
d = mujoco.MjData(m)

ball_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "ball")
# 在碰撞后第n步停止仿真
is_step = True
is_contact_stop = True
n_contact_step = 150
contact_step = 0
with mujoco.viewer.launch_passive(m, d) as viewer:
  viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
  while viewer.is_running():
    
    step_start = time.time()
    if d.nefc > 0 and is_contact_stop:
      contact_step += 1
      if contact_step > n_contact_step:
        is_step = False
    if is_step:
      mujoco.mj_step(m, d)
    
      a0 = mujoco.mju_norm3(m.opt.gravity)
      err_a = 0
      print("a0: ", a0)
      print("r(R-pos): ", 0.1-d.sensor("ball_pos").data[2])
      print("r(efc_pos-efc_margin): ", d.efc_pos - d.efc_margin)
      print("n efc: ", d.nefc)
      for i in range(d.nefc):
        id = d.efc_id[i]
        print("  KBIP: ", d.efc_KBIP[i][0], d.efc_KBIP[i][1],
              d.efc_KBIP[i][2], d.efc_KBIP[i][3])
        print("  efc_vel: ", d.efc_vel[i])
        print("  efc_force: ", d.efc_force[i])
        err_a = d.efc_force[i] / m.body_mass[ball_id]
        print("  efc_acc: ", d.efc_pos[i])
        print("  efc_aref: ", d.efc_aref[i])
      print("a1: ", a0 - err_a)

    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)