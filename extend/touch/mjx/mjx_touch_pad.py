import time

import jax
from jax import numpy as jp
import mujoco
from mujoco import mjx
import mujoco.viewer
import cv2

jax.config.update('jax_platform_name', 'cpu')
m = mujoco.MjModel.from_xml_path("../touch_pad.xml")
d = mujoco.MjData(m)
mx = mjx.put_model(m)
dx = mjx.put_data(m, d)

print(f'Default backend: {jax.default_backend()}')
step_fn = mjx.step
  
# 用jax.jit编译mjx的仿真步进
print('JIT-compiling the model physics step...')
start = time.time()
step_fn = jax.jit(step_fn).lower(mx, dx).compile()
elapsed = time.time() - start
print(f'Compilation took {elapsed}s.')
  
print(dx.ctrl.device)

viewer = mujoco.viewer.launch_passive(m, d)
with viewer:
  while True:
    start = time.time()

    dx = dx.replace(
        ctrl=jp.array(d.ctrl),
        act=jp.array(d.act),
        xfrc_applied=jp.array(d.xfrc_applied),
    )
    dx = dx.replace(
        qpos=jp.array(d.qpos), qvel=jp.array(d.qvel), time=jp.array(d.time)
    )  # handle resets
    mx = mx.tree_replace({
        'opt.gravity': m.opt.gravity,
        'opt.tolerance': m.opt.tolerance,
        'opt.ls_tolerance': m.opt.ls_tolerance,
        'opt.timestep': m.opt.timestep,
    })
      

    dx = step_fn(mx, dx)

    elapsed = time.time() - start
  #   print(f'mjx step time {elapsed}s.')
    
    mjx.get_data_into(d, m, dx)
    viewer.sync()
