# 传感器数据获取

![](../../MJCF/asset/sensor_adr.png)
**sensordata的索引需要依靠mjData的sensor_adr获取，这个可以使用sensor的id**
**这个我们要注意传感器具有的数据量，有的传感器是一个值，而有的传感器是三个值。我们可以使用mjModel中的sensor_dim获得传感器输出的参数量**
<font color=Green>*演示：*</font>

```python
def get_sensor_data(sensor_name):
    sensor_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name)
    if sensor_id == -1:
        raise ValueError(f"Sensor '{sensor_name}' not found in model!")
    start_idx = m.sensor_adr[sensor_id]
    dim = m.sensor_dim[sensor_id]
    sensor_values = d.sensordata[start_idx : start_idx + dim]
    return sensor_values
```
对于传感器其他的属性在 mjModel中可以直接获得。如下：
![](../../MJCF/asset/modelsensors.png)
**现在可以通过 MjData.sensor("sensorname").data 获取传感器数据**

### 读取相机画面
&emsp;&emsp;相机来源一般是在模型文件中创建相机，或者创建一个相机手动控制，就像 base中
与人交互的画面就是手动创建的相机。我们读取相机的步骤为：
1. 初始化glfw
2. 创建相机
3. 更新场景
4. 读取图像
5. 通过 opencv将图像转换
```
MJAPI void mjr_readPixels(unsigned char* rgb, float* depth,
mjrRect viewport, const mjrContext* con);
```
将渲染画面转成rgb图像和深度图像。
**获取相机视角演示：**
初始化:
```python
# 初始化glfw
glfw.init()
glfw.window_hint(glfw.VISIBLE,glfw.FALSE)
window = glfw.create_window(1200,900,"mujoco",None,None)
glfw.make_context_current(window)
#创建相机
camera = mujoco.MjvCamera()
camID = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, "this_camera")
camera.fixedcamid = camID
camera.type = mujoco.mjtCamera.mjCAMERA_FIXED 
scene = mujoco.MjvScene(m, maxgeom=1000)
context = mujoco.MjrContext(m, mujoco.mjtFontScale.mjFONTSCALE_150)
mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, context)

def get_image(w,h):
    # 定义视口大小
    viewport = mujoco.MjrRect(0, 0, w, h)
    # 更新场景
    mujoco.mjv_updateScene(
        m, d, mujoco.MjvOption(), 
        None, camera, mujoco.mjtCatBit.mjCAT_ALL, scene
    )
    # 渲染到缓冲区
    mujoco.mjr_render(viewport, scene, context)
    # 读取 RGB 数据（格式为 HWC, uint8）
    rgb = np.zeros((h, w, 3), dtype=np.uint8)
    mujoco.mjr_readPixels(rgb, None, viewport, context)
    cv_image = cv2.cvtColor(np.flipud(rgb), cv2.COLOR_RGB2BGR)
    return cv_image
```
获取图像:
```python
def get_image(w,h):
    # 定义视口大小
    viewport = mujoco.MjrRect(0, 0, w, h)
    # 更新场景
    mujoco.mjv_updateScene(
        m, d, mujoco.MjvOption(), 
        None, camera, mujoco.mjtCatBit.mjCAT_ALL, scene
    )
    # 渲染到缓冲区
    mujoco.mjr_render(viewport, scene, context)
    # 读取 RGB 数据（格式为 HWC, uint8）
    rgb = np.zeros((h, w, 3), dtype=np.uint8)
    mujoco.mjr_readPixels(rgb, None, viewport, context)
    cv_image = cv2.cvtColor(np.flipud(rgb), cv2.COLOR_RGB2BGR)
    return cv_image

img = get_image(640,480)
    cv2.imshow("img",img)
    cv2.waitKey(1)
```



