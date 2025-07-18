import time
import math

import mujoco
import mujoco.viewer
import cv2
import glfw
import numpy as np

m = mujoco.MjModel.from_xml_path('../../API-MJC/vis_cfg.xml')
d = mujoco.MjData(m)

glfw.init()
glfw.window_hint(glfw.VISIBLE,glfw.FALSE)
window = glfw.create_window(1920,1080,"mujoco",None,None)
glfw.make_context_current(window)
#创建相机
camera = mujoco.MjvCamera()
camID = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, "look_bunny")#look_bunny bunny_eyes
camera.fixedcamid = camID
camera.type = mujoco.mjtCamera.mjCAMERA_FIXED 
scene = mujoco.MjvScene(m, maxgeom=2000)
context = mujoco.MjrContext(m, mujoco.mjtFontScale.mjFONTSCALE_150)

def get_image(w,h,stereo=mujoco.mjtStereo.mjSTEREO_NONE):
    before_stereo = scene.stereo
    scene.stereo = stereo
    viewport = mujoco.MjrRect(0, 0, w, h)
    # 更新场景
    mujoco.mjv_updateScene(
        m, d, mujoco.MjvOption(), 
        None, camera, mujoco.mjtCatBit.mjCAT_ALL, scene
    )
    '''--------设置分割颜色--------'''
    for i in range(scene.ngeom):
        geom = scene.geoms[i]
        if geom.objid == bunny_id and geom.objtype == mujoco.mjtObj.mjOBJ_GEOM:
            r = 254
            g = 0
            b = 255
            geom.segid = b << 16 | g << 8 | r
    '''--------设置分割颜色--------'''
    # 渲染到缓冲区
    mujoco.mjr_render(viewport, scene, context)
    scene.stereo = before_stereo
    # 读取 RGB 数据（格式为 HWC, uint8）
    rgb = np.zeros((h, w, 3), dtype=np.uint8)
    mujoco.mjr_readPixels(rgb, None, viewport, context)
    cv_image = cv2.cvtColor(np.flipud(rgb), cv2.COLOR_RGB2BGR)
    return cv_image


with mujoco.viewer.launch_passive(m, d) as viewer:
  '''--------可视化配置--------'''
  viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CAMERA] = True
  '''--------可视化配置--------'''
  
  '''--------场景渲染--------'''
  # scene.flags[mujoco.mjtRndFlag.mjRND_WIREFRAME] = True
  # scene.flags[mujoco.mjtRndFlag.mjRND_SEGMENT] = True #随机颜色分割
  # scene.flags[mujoco.mjtRndFlag.mjRND_IDCOLOR] = True #segid
  bunny_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "bunny")
  '''--------场景渲染--------'''
  
  # Close the viewer automatically after 30 wall-seconds.
  start = time.time()
  while viewer.is_running():
    
    step_start = time.time()
    mujoco.mj_step(m, d)
    
    img = get_image(1024,640,mujoco.mjtStereo.mjSTEREO_SIDEBYSIDE)
    # img = get_image(1024,640)
    cv2.imshow("img",img)
    cv2.waitKey(1)


    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)