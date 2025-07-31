# friction
## option（椭圆/金字塔）
cone=[pyramidal, elliptic] 默认pyramidal，速度快，elliptic效果更好
impratio="1" 椭圆摩擦的比例，更高的数值摩擦力会更硬一些，防止打滑，不建议在使用pyramidal的时候调大
**摩擦计算的时候会先根据priority选择使用那个geom的friction参数，如果相同就会选择最大的使用**
engine/engine_collision_driver.c: mj_contactParam
![](../asset/friction2.png)
![](../asset/friction1.png)

**condim设置**
|condim|作用|
|---|----|
|1|只有平面的摩擦力，斜面不会作用,一个自由度，δx方向，friction第一个参数调控|
|3|有多个方向的摩擦力，斜面也会生效，三个自由度,δxδyδz，friction第一个参数调控|
|4|只有平面的摩擦力，不会抑制滚动，但是会作用抑制扭矩，有滑冰效果,四个自由度，δxδyδz,τx,friction第二个参数调控|
|6|只有平面的摩擦力，会抑制滚动，滚动摩擦力是滚动过程中接触变形产生的能量损耗,六个自由度，δxδyδz,τxτyτz,friction第三个参数调控|