import time
import math

import mujoco
import mujoco.viewer
import cv2
import numpy as np

m = mujoco.MjModel.from_xml_path('../touch_pad.xml')
d = mujoco.MjData(m)

print(f"sensor_num: {m.nsensor}")

sensor_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, "touch_point0000")
if sensor_id == -1:
    print("no found sensor")

touch_point_adr = [[0] * 20 for _ in range(20)]
for x in range(20):
    for y in range(20):
        idx = x * 20 + y
        touch_point_adr[x][y] = m.sensor_adr[idx]


with mujoco.viewer.launch_passive(m, d) as viewer:
  start = time.time()
  while viewer.is_running():
    
    step_start = time.time()
    mujoco.mj_step(m, d)
    
    touch = np.zeros((20, 20), dtype=np.uint8)
    for x in range(20):
      for y in range(20):
          adr = touch_point_adr[x][y]
          data = mujoco.mju_norm3(d.sensordata[adr:adr+3])
          touch[x,y] = mujoco.mju_clip(data, 0.0, 3.0) / 3 *255
    
    touch = cv2.resize(touch,(200,200))
    cv2.imshow("touch",touch)
    cv2.waitKey(1)


    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)