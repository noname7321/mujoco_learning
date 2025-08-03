import jax
import jax.numpy as jnp
import mujoco as mj
from mujoco import mjx
import jax.tree_util as jtu  # 使用新API替代过时的jax.tree_map

# 1. 创建原始模型
xml_string = """
<mujoco>
  <worldbody>
    <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
    <geom type="plane" size="1 1 0.1" rgba=".9 0 0 1"/>
    <body pos="0 0 1">
      <joint type="free"/>
      <geom type="box" size=".1 .1 .1" rgba="0 .9 0 1"/>
    </body>
  </worldbody>
</mujoco>
"""

m = mj.MjModel.from_xml_string(xml_string)
d = mj.MjData(m)

# 2. 转换为 MJX 格式
mjx_model = mjx.put_model(m)
mjx_data = mjx.put_data(m, d)

# 3. 批量创建环境（使用新的jax.tree_util API）
batch = jtu.tree_map(
    lambda x: jnp.repeat(x[jnp.newaxis], 4096, axis=0),
    mjx_data
)

# 4. 创建步进函数（修复参数问题）
def step_wrapper(model, data):
    # 调用mjx.step并返回更新后的数据
    return mjx.step(model, data)

# 5. 向量化并编译
vmap_step = jax.vmap(step_wrapper, in_axes=(None, 0))
jit_step = jax.jit(vmap_step)

# 6. 执行步进（注意：现在只需要2个参数）
batch = jit_step(mjx_model, batch)

# 7. 检查结果
print("批量环境数量:", batch.qpos.shape[0])
print("第一个环境的qpos:", batch.qpos[0])
print("所有环境的qpos是否相同:", jnp.all(batch.qpos[0] == batch.qpos[1]))