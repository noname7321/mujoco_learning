# 传感器数据获取

![](../../MJCF/asset/sensor_adr.png)
**sensordata的索引需要依靠mjData的sensor_adr获取，这个可以使用sensor的id**
**这个我们要注意传感器具有的数据量，有的传感器是一个值，而有的传感器是三个值。我们可以使用mjModel中的sensor_dim获得传感器输出的参数量**
<font color=Green>*演示：*</font>

```C++
std::vector<float> get_sensor_data(const mjModel *model, const mjData *data,
                                   const std::string &sensor_name) {
  int sensor_id = mj_name2id(model, mjOBJ_SENSOR, sensor_name.c_str());
  if (sensor_id == -1) {
    std::cout << "no found sensor" << std::endl;
    return std::vector<float>();
  }
  int data_pos = model->sensor_adr[sensor_id];
  std::vector<float> sensor_data(model->sensor_dim[sensor_id]);
  for (int i = 0; i < sensor_data.size(); i++) {
    sensor_data[i] = data->sensordata[data_pos + i];
  }
  return sensor_data;
}
```
对于传感器其他的属性在 mjModel中可以直接获得。如下：
![](../../MJCF/asset/modelsensors.png)

### 读取相机画面
&emsp;&emsp;相机来源一般是在模型文件中创建相机，或者创建一个相机手动控制，就像 base中
与人交互的画面就是手动创建的相机。我们读取相机的步骤为：
1. 获取相机 ID
2. 确定图像大小
3. 渲染
4. 读取图像
5. 通过 opencv将图像转成 Mat
```
MJAPI void mjr_readPixels(unsigned char* rgb, float* depth,
mjrRect viewport, const mjrContext* con);
```
将渲染画面转成rgb图像和深度图像。
获取相机视角演示：
相机初始化:
```C++
mjvCamera cam2;//bsae中全局变量
int camID = mj_name2id(m, mjOBJ_CAMERA, "cam2armor1");
if (camID == -1)
{
std::cerr << "Camera not found" << std::endl;
}
else
{
std::cout << "Camera ID: " << camID << std::endl;
// 获取摄像机的位置
const double *cam_pos = &m->cam_pos[3 * camID];
std::cout << "Camera Position: (" << cam_pos[0] << ", " << cam_pos[1] << ", " << cam_pos[2] << ")" << std::endl;
// 获取摄像机的视野角度
double cam_fovy = m->cam_fovy[camID];
std::cout << "Camera FOV Y: " << cam_fovy << " degrees" << std::endl;
// 给相机初始化
mjv_defaultCamera(&cam2);
// 这里给相机id和类型即可，mujoco就能找到相机位置
cam2.fixedcamid = camID;
cam2.type = mjCAMERA_FIXED;
}
```
获取图像:
```C++
// 设置图像大小,要小于opengl窗口大小,否则会图像出现问题
int width = 1200;
int height = 900;
mjrRect viewport2 = {0, 0, width, height};
// mujoco更新渲染
mjv_updateCamera(m, d, &cam2, &scn);
mjr_render(viewport2, &scn, &con);
// 渲染完成读取图像
unsigned char *rgbBuffer = new unsigned char[width * height * 3];
float *depthBuffer = new float[width * height]; //深度图
mjr_readPixels(rgbBuffer, depthBuffer, viewport2, &con);
cv::Mat image(height, width, CV_8UC3, rgbBuffer);
// 反转图像以匹配OpenGL渲染坐标系
cv::flip(image, image, 0);
// 颜色顺序转换这样要使用bgr2rgb而不是rgb2bgr
cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
cv::imshow("Image", image);
cv::waitKey(1);
// 释放内存
delete[] rgbBuffer;
delete[] depthBuffer;
```



