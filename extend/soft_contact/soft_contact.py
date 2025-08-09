import time
import mujoco
import mujoco.viewer

m = mujoco.MjModel.from_xml_path('./scene.xml')
d = mujoco.MjData(m)

with mujoco.viewer.launch_passive(m, d) as viewer:
  while viewer.is_running():
    
    
    step_start = time.time()
    mujoco.mj_step(m, d)
    
    print("acc:", 0.1-d.sensor("ball_acc").data[2])
    print("vel:", 0.1-d.sensor("ball_vel").data[2])
    print("r(vel-pos):", 0.1-d.sensor("ball_pos").data[2])
    print("r(efc_pos-efc_margin):", d.efc_pos - d.efc_margin)
    print("n efc:", d.nefc)
    for i in range(d.nefc):
      id = d.efc_id[i]
      print("  KBIP:", d.efc_KBIP[i][0], d.efc_KBIP[i][1],
            d.efc_KBIP[i][2], d.efc_KBIP[i][3])
      print("  efc_force:", d.efc_force[i])
      print("  efc_aref:", d.efc_aref[i])

    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)