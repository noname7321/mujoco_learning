# friction
## option（椭圆/金字塔）
cone=[pyramidal, elliptic] 摩擦力近似计算方法,默认pyramidal计算速度快，elliptic效果更好
impratio="1" 椭圆摩擦的比例，更高的数值摩擦力会更硬一些，防止打滑，不建议在使用pyramidal的时候调大，适合机械臂抓取打滑时调整参数
## 优先级
**摩擦计算的时候会先根据geom的priority选择使用那个geom的friction参数，如果相同就会选择最大的使用**
源码位置：engine/engine_collision_driver.c: mj_contactParam
![](../asset/friction2.png)
![](../asset/friction1.png)

## condim设置
|condim|作用|
|---|----|
|1|只有垂直于接触面的法向力，两个物体碰撞或者挤压产生的力|
|3|可以生成接触面上的力，也就是常见的摩擦力，friction第一个参数调控|
|4|增加了抑制在绕接触面法线旋转的摩擦力，friction第二个参数调控|
|6|会抑制滚动，滚动摩擦力是滚动过程中接触变形产生的能量损耗，friction第三个参数调控|

## 计算
### 约束力维度
摩擦力在mujoco中是约束力的一部分，根据不同condim会产生不同数量的约束力，elliptic产生condim相同维度的约束力，pyramidal则会产生2*(condim-1)维度的约束力
![](../asset/friction_compute.png)
**condim和约束力维度：**
|condim|elliptic|pyramidal|
|-|-|-|
|1|e1|e1|
|3|e1~e3|e1~e4|
|4|e1~e4|e1~e6|
|5|e1~e6|e1~e10|

**约束力对应摩擦系数：**
|elliptic|pyramidal||
|-|-|-|
|e1|e1|None|
|e1~e3|e1~e4|friction1|
|e4|e5,e6|friction2|
|e5,e6|e7~e10|friction3|

**源码：**
engine/engine_core_constraint.c：getposdim
![](../asset/friction_dim.png)
engine/engine_core_constraint.c：mj_makeImpedance
**下面friction是一个长度为5的数组，fri是mjcf中定义的friction参数**
![](../asset/friction_data.png)
![](../asset/friction_compute2.png)

官方文档：
[Contact](https://mujoco.readthedocs.io/en/latest/computation/index.html#contact)
[Friction cones](https://mujoco.readthedocs.io/en/latest/computation/index.html#friction-cones)