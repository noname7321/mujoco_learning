import time

import jax
from jax import numpy as jnp
import mujoco
from mujoco import mjx
import mujoco.viewer

jax.config.update('jax_platform_name', 'gpu')
m = mujoco.MjModel.from_xml_path("./car.xml")
d = mujoco.MjData(m)
mx = mjx.put_model(m)
dx = mjx.put_data(m, d)

print(f'Default backend: {jax.default_backend()}')

# 用jax编译ray
mjx_ray = mjx.ray
mjx_ray = jax.jit(mjx_ray)


step_fn = mjx.step

# 用jax.jit编译mjx的仿真步进
print('JIT-compiling the model physics step...')
start = time.time()
step_fn = jax.jit(step_fn).lower(mx, dx).compile()
elapsed = time.time() - start
print(f'Compilation took {elapsed}s.')


print(dx.ctrl.device)
viewer = mujoco.viewer.launch_passive(m, d)
while True:
  start = time.time()

  d.ctrl[0] = 1
  dx = dx.replace(
      ctrl=jnp.array(d.ctrl),
      act=jnp.array(d.act),
      xfrc_applied=jnp.array(d.xfrc_applied),
  )
  
  dx = step_fn(mx, dx)

  elapsed = time.time() - start
#   print(f'mjx step time {elapsed}s.')
  
  mjx.get_data_into(d, m, dx)
  viewer.sync()

