import jax
import jax.numpy as jnp
from jax import grad, vmap  
import time

# 求导
# def func1(x):
#     return jnp.square(x)
# def func2(x):
#     return jnp.sum(jnp.square(x))

# x_small = jnp.arange(3.)
# derivative_fn1 = vmap(grad(func1)) 
# print(derivative_fn1(x_small))    
# derivative_fn2 = grad(func2)
# print(derivative_fn2(x_small))
# print(vmap(func1)(x_small))

# 求偏导
# def g(x,y):
#     return x * y + jnp.sin(x)
# grad_g = grad(g,argnums=0)
# print(grad_g(1.0,2.0))

# def g(x,y):
#     return x + y
# vmap_g = vmap(g,in_axes=(0,None))
# x = jnp.array([1.0,2.0])
# y = 5.0

# print(vmap_g(x,y))
# print(g(x,y))

# gpu_device = jax.devices("gpu")[0]
# cpu_device = jax.devices("cpu")[0]
# print(gpu_device)
# x = jnp.array([1.0,2.0,3.0])
# print(x.device)

# xx=jax.device_put(x,device=cpu_device)
# print(xx.device)

# x_on_cpu = jax.device_get(x)
# print(x_on_cpu.device)

# 累加
# def f(carry,x):
#     return carry + x, carry + x
# carry, process = jax.lax.scan(f,jnp.array([1,2,3,4]),jnp.array([1,1,1,1]))
# print(carry)
# print(process)

# 判断 第一次调用jax会评估两个函数并判断返回值类型，之后再次使用将直接输出返回值而不是调用函数
# @jax.jit
# def true_fn(x):
#     print("true")
#     return x * 2

# @jax.jit
# def false_fn(x):
#     print("false")
#     return x / 2

# result = jax.lax.cond(True, true_fn, false_fn, 0.0)
# print(result)
# result = jax.lax.cond(True, true_fn, false_fn, 1.0)
# print(result)
# result = jax.lax.cond(False, true_fn, false_fn, 2.0)
# print(result) 
# result = jax.lax.cond(False, true_fn, false_fn, 3.0)
# print(result)
# result = jax.lax.cond(False, true_fn, false_fn, 4.0)
# print(result)

jax.config.update('jax_platform_name', 'cpu')
array = jnp.zeros((4,3))
print(array)
array = array.at[0,0].set(10)
print(array)
a = array[0,0]
print(a)