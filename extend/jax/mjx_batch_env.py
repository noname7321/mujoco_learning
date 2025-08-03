import time

import jax
from jax import numpy as jnp
import jax.tree_util as jtu
import mujoco
from mujoco import mjx
import mujoco.viewer
import copy

jax.config.update('jax_platform_name', 'gpu')
m = mujoco.MjModel.from_xml_path("/home/albusgive/software/mujoco-3.3.4/model/car/car.xml")
d = mujoco.MjData(m)
mx = mjx.put_model(m)
dx = mjx.put_data(m, d)

num_envs = 1
dummy_input = jnp.zeros(num_envs)
batch = jax.vmap(lambda _: dx)(dummy_input)
print('JIT-compiling the model physics step...')
start = time.time()
jit_step = jax.jit(jax.vmap(mjx.step, in_axes=(None, 0)))
elapsed = time.time() - start
print(f'Compilation took {elapsed}s.')
batch = jit_step(mx, batch)

viewer = mujoco.viewer.launch_passive(m, d)

while True:
  start = time.time()

  # print(batch.qpos[0])
  print(batch.time[0])
  batch = batch.replace(ctrl=batch.ctrl.at[0].set(jnp.array(d.ctrl)),
                        act=batch.act.at[0].set(jnp.array(d.act)),
                        xfrc_applied=batch.xfrc_applied.at[0].set(jnp.array(d.xfrc_applied)),
                        )
  batch = jit_step(mx, batch)
  elapsed = time.time() - start
  # print(f'mjx step time {elapsed}s.')

  # mujoco_data = mjx.get_data(m, batch)[0]
  
  dx0 = jtu.tree_map(lambda x: x[0], batch)
  dx0 = jtu.tree_map_with_path(lambda path, x: x[0], batch)
  
  mjx.get_data_into(d, m, dx0)
  
  viewer.sync()
