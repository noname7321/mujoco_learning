import time
import math

import mujoco
import mujoco.viewer
import cv2
import glfw
import numpy as np

m = mujoco.MjModel.from_xml_path('scene.xml')
d = mujoco.MjData(m)

with mujoco.viewer.launch_passive(m, d) as viewer:
  viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = False
  start = time.time()
  while viewer.is_running():
    
    
    step_start = time.time()
    mujoco.mj_step(m, d)
    
    print(0.1-d.sensor("ball_pos").data[2])


    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)