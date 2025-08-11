import time
import math

import mujoco
import mujoco.viewer

m = mujoco.MjModel.from_xml_path('../../API-MJCF/pointer.xml')
d = mujoco.MjData(m)

with mujoco.viewer.launch_passive(m, d) as viewer:
  # Close the viewer automatically after 30 wall-seconds.
  start = time.time()
  cnt = 0
  while viewer.is_running() and time.time() - start < 30:
    step_start = time.time()
    
    # d.ctrl[1] = math.sin(cnt)
    mujoco.mj_step(m, d)
    
    # print(m.njnt)
    # print(m.names)
    bsae_id = mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,"pointer")
    # print(bsae_id)
    # print(d.xpos[bsae_id])
    # imu_id = mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SITE,"imu")
    # print(d.site_xpos[imu_id])
    # w x y z
    print(d.xquat[bsae_id])
      
    cnt += 0.005

    # Example modification of a viewer option: toggle contact points every two seconds.
    with viewer.lock():
      viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)