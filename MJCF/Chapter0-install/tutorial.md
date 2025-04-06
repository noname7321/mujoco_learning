# mujoco安装
## 编译安装
* 克隆项目
  ``` git clone https://github.com/google-deepmind/mujoco.git ```
* 编译（建议开启魔法）
```
cd mujoco
mkdir build
cmake ..
cmake --build . 多线程编译使用 cmake --build . -j线程数
```  
* 选择安装位置（推荐/opt）  
  `cmake -DCMAKE_INSTALL_PREFIX=/opt/mujoco .`
* `sudo cmake --install .`
### 测试
```
cd bin/
./simulate ../model/humanoid.xml
```

## release版本
在github上下载对应平台的压缩包解压即可
https://github.com/google-deepmind/mujoco/releases
### 测试
```
cd bin/
./simulate ../model/humanoid/humanoid.xml
```

## python安装
  `pip install mujoco`
### 测试
 `python -m mujoco.viewer` or `python3 -m mujoco.viewer`
 或者加载模型
 `python -m mujoco.viewer --mjcf=/path/to/some/mjcf.xml` or
 `python3 -m mujoco.viewer --mjcf=/path/to/some/mjcf.xml`