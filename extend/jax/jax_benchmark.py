import jax
import jax.numpy as jnp
from time import perf_counter

# 打印使用的设备信息
print(f"JAX 使用的设备: {jax.devices()[0]}")
print(f"JAX 后端平台: {jax.lib.xla_bridge.get_backend().platform}\n")

def benchmark(size=8192, dtype=jnp.float32, num_iter=10):
    """基准测试函数：测量大型矩阵乘法的性能"""
    # 创建随机矩阵
    key = jax.random.PRNGKey(42)
    a = jax.random.normal(key, (size, size), dtype=dtype)
    b = jax.random.normal(key, (size, size), dtype=dtype)
    
    # 预热：确保JIT编译完成
    _ = jnp.dot(a, b).block_until_ready()
    
    # 正式计时
    start = perf_counter()
    for _ in range(num_iter):
        c = jnp.dot(a, b)
        c.block_until_ready()  # 确保计算完成
    end = perf_counter()
    
    # 计算统计数据
    duration = end - start
    avg_time = duration / num_iter
    flops = 2 * size ** 3  # 每次矩阵乘法的浮点运算次数
    tflops = (flops / avg_time) * 1e-12 / num_iter
    
    print(f"矩阵大小: {size}x{size} | 数据类型: {dtype}")
    print(f"总耗时: {duration:.4f} 秒 | 平均每次: {avg_time:.4f} 秒")
    print(f"计算吞吐量: {tflops:.2f} TFLOPS")
    return tflops

# 运行基准测试
if __name__ == "__main__":
    # 测试不同精度 (GPU上float32更快，TPU上bfloat16更快)
    dtypes = [jnp.float32, jnp.float16, jnp.bfloat16] if jax.default_backend() != "tpu" else [jnp.bfloat16]
    
    for dtype in dtypes:
        try:
            tflops = benchmark(size=4096, dtype=dtype)
            print(f"✅ {dtype} 测试完成\n" + "="*50)
        except Exception as e:
            print(f"❌ {dtype} 测试失败: {str(e)}\n")