import time
import math

import mujoco
import mujoco.viewer
import cv2
import glfw
import numpy as np

m = mujoco.MjModel.from_xml_path('../touch_flex.xml')
d = mujoco.MjData(m)

x_n_elem = 30
y_n_elem = 15
x_n_vert = 16
y_n_vert = 16
with mujoco.viewer.launch_passive(m, d) as viewer:
  start = time.time()
  while viewer.is_running():
    
    step_start = time.time()
    mujoco.mj_step(m, d)
    
    touch_elem = np.zeros((y_n_elem,x_n_elem), dtype=np.uint8)
    touch_vert = np.zeros((y_n_vert,x_n_vert), dtype=np.uint8)
    for j in range(d.ncon):
      con = d.contact[j]
      if con.flex[0] != -1 or con.flex[1] != -1:
        if con.elem[0] != -1 or con.elem[1] != -1:
          for k in range(2):
            if con.elem[k] != -1:
              force_torque = np.zeros(6)
              mujoco.mj_contactForce(m,d,j,force_torque)
              force = mujoco.mju_norm3(force_torque[0:3])
              y = int(con.elem[k] / x_n_elem)
              x = con.elem[k] % x_n_elem
              data = mujoco.mju_clip(force / 3 * 255, 0.0, 255.0)
              touch_elem[y,x] = data
        elif con.vert[0] != -1 or con.vert[1] != -1:
          for k in range(2):
            if con.vert[k] != -1:
              force_torque = np.zeros(6)
              mujoco.mj_contactForce(m,d,j,force_torque)
              force = mujoco.mju_norm3(force_torque[0:3])
              y = int(con.vert[k] / x_n_vert)
              x = con.vert[k] % x_n_vert
              data = mujoco.mju_clip(force / 1 * 255, 0.0, 255.0)
              touch_vert[y,x] = data

    touch_elem = cv2.resize(touch_elem,(300,300))
    touch_vert = cv2.resize(touch_vert,(320,320))
    cv2.imshow("touch_elem",touch_elem)
    cv2.imshow("touch_vert",touch_vert)
    cv2.waitKey(1)


    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)