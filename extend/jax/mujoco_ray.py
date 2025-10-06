import time
import math

import mujoco
import mujoco.viewer
import cv2
import glfw
import numpy as np

m = mujoco.MjModel.from_xml_path('../../API-MJCF/deep_ray.xml')
d = mujoco.MjData(m)

camID = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, "look_box")
cam_pos = m.cam_pos[camID*3:camID*3+3]

box_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "box1")
box2_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "box2")

box_num = 5
box_idx = np.zeros((box_num, 1), dtype=np.int32)
boxs_pos = np.zeros((box_num, 3))
for i in range(box_num):
  geom_name = "box" + str(i+1)
  box_idx[i] = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
  boxs_pos[i] = d.geom_xpos[box_idx[i]]

with mujoco.viewer.launch_passive(m, d) as viewer:
  start = time.time()
  while viewer.is_running():
    
    step_start = time.time()
    mujoco.mj_step(m, d)

    
    '''--------多射线--------'''
    num_ray = 200000
    num_vec = np.random.rand(num_ray*3, 1)
    geomid = np.zeros((num_ray, 1), dtype=np.int32)
    geomgroup = np.ones((6, 1))
    dist = np.zeros((num_ray, 1))
    dist_ratio = np.zeros((num_ray, 1))
    pnt = cam_pos.reshape(3, 1)
    start = time.time()
    mujoco.mj_multiRay(m, d, pnt, num_vec, geomgroup, 1, -1, geomid, dist_ratio, num_ray, 999)
    elapsed = time.time() - start
    print(f'Compilation took {elapsed*1000} ms.')
    '''--------多射线--------'''

    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)