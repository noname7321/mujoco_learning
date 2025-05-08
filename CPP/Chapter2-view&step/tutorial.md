# View
## 加载模型
mj_loadModel和mj_loadXML，分别是加载mujoco编译好的模型和mjcf文件
## 数据结构
mjModel，用来存储模型文件信息
mjData，用来存储仿真数据
mjvCamera，相机和相机传感器
mjvOption，可视化配置
mjvScene，渲染场景
mjrContext，渲染上下文

## 仿真运行
**mj_step**        
前向动力学（计算加速度 qacc）
数值积分（更新状态 qpos, qvel）
处理碰撞和约束
更新传感器数据

**mj_step1**       
计算当前状态下的广义加速度 (qacc)
处理碰撞检测，但不应用约束

**mj_step2**       
应用用户设置的 ctrl 和 xfrc_applied
处理约束（接触力、关节限位等）
数值积分更新状态 (qpos, qvel)
更新仿真时间 d->time

**mj_forward**     
前向动力学，但不推进仿真时间
给定关节位置(qpos)、速度(qvel)和关节力矩(τ)，计算关节加速度(qacc)

**mj_inverse**
给定关节位置(qpos)、速度(qvel)、加速度(qacc)，计算所需的关节力矩(τ)






