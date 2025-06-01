# 作用力
**平动**      
$$ m \cdot a = F $$
**旋转**
$$ I \cdot \alpha = \tau $$  
$$ F/\tau = 外部力+驱动力+被动力+约束力+偏置力 $$  

# 外部力
我们查看文档计算部分可以看到有qfrc_passive,qfrc_actuator,qfrc_applied三个力分别对应被动力，驱动力，外部力       

![](../../MJCF/asset/force.png)
只要看手册的api或者头文件中，找到mj_applyFT函数应用外部力。
![](../../MJCF/asset/forcefunc1.png)
或者还可以使用xfrc_applied直接作用外部力在质心上。
![](../../MJCF/asset/frc_.png)
**这里也说明了是笛卡尔力。**
&emsp;&emsp;mj_applyFT函数的参数，是三维的力，三维扭矩，三维坐标(worldbody坐标系)，bodyid。qfrc_target可以直接使用d->qfrc_applied。mj_applyFT是对于body在 **“自由度”** 上施加力。于是我们可以使用两个方式对body施加外部力。       
&emsp;&emsp;qfrc_target还可以是以下这些被动力等qfrc_xxx的力
![](../../MJCF/asset/mj_passive.png)

**mj_Data接口演示（作用在质心上）：**
```C++
int bullet_id = mj_name2id(m, mjOBJ_BODY, "box");
mjtNum *set_torque = d->xfrc_applied + bullet_id * 6;
```

**mj_applyFT函数接口演示（可以调整施力点）:**
```C++
int bullet_id = mj_name2id(m, mjOBJ_BODY, "box");
mjtNum force[3] = {0.0, 0.0, 9.81};
mjtNum torque[3] = {0.0, 0.0, 0.0};
mjtNum point[3] = {0.0, 0.0, 0.0}; 
mj_applyFT(m, d, force, torque, point, id, d->qfrc_applied);
```
**mj_applyFT每次调用都是增量式，如果我们想清除力可以使用mju_zero，如mju_zero(d->qfrc_applied, m->nv);**

# 驱动力
![](../../MJCF/asset/efc_force.png)
&emsp;&emsp;mjData.qfrc_actuator是驱动器执行的力，不同驱动器最终会计算出力或者扭矩作用到关节上。        

# 被动力
&emsp;&emsp;mjData.qfrc_passive是被动力，关节参数的damping，stiffness,摩擦力，流体阻力都会最终计算到改力中。        
$$ damping_force = (0-qvel)*damping $$
$$ stiffness_force = (0-qpos)*stiffness $$

# 约束力
&emsp;&emsp;mjData.efc_force是约束力，关节的frictionloss，equality计算出来的合力为改力。        
&emsp;&emsp;jar = Jac*qacc-aref 残差=雅可比*关节加速度-参考伪加速度

$$
frictionlossforce = 
\begin{cases}
    frictionloss, & \text{if } jar <= -InverseConstraintMass ⋅ floss  \\
    -frictionloss,  & \text{else if } jar>=InverseConstraintMass ⋅ floss   \\
    （没看懂）,  & \text{else }
\end{cases}
$$
**源码实现(SRC/engine/engine_core_constraint.c)**
![](../../MJCF/asset/frictionloss.png)

# 偏置力
![](../../MJCF/asset/qfrc_bias.png)
&emsp;&emsp;mjData.qfrc_bias科里奥利力等，由引擎自动计算。        