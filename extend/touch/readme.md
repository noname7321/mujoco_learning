# Touch Pad/Flex
刚性，柔性材料触摸实现
非插件方式实现，实现参考：mujoco源码engine_sensor.c中mjSENS_TOUCH部分
## mujoco version
touch_pad:mujoco3.x.x
touch_flex:大于mujoco3.3.1，其中elastic2d参数在3.3.3之后版本使用
## MJCF
### touch pad
用replicate复制body然后添加forcec传感器，演示见touch_pad.xml
### touch flex
直接建模flexcomp无mjcf自带传感器，通过读取flex中的elem和vert接触力数据实现
## C++
build:
```
cd build
cmake ..
make
```
### touch pad
run： `./touch_pad`
按顺序读取传感器数据
### touch flex
run： `./touch_flex`
一个flexcomp会生成一个flex，count表示flex中vert数量，elem数量则是三角区域数量，比count数量多。
vert一般会和平面接触如plane做为支撑，elem一般会和其他物体接触如球。
## Python
实现原理同C++
run：
`python touch_pad.py` and `python touch_flex.py`