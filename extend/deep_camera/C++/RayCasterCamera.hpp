#include <mujoco/mjtnum.h>
#pragma onece
#include "opencv2/opencv.hpp"
#include <mujoco/mujoco.h>
#include <cmath>

class RayCasterCamera {
public:
  RayCasterCamera();
  /** @brief 初始化相机 - 使用焦距和孔径
   * @param m mjModel
   * @param d mjData
   * @param cam_id 相机id
   * @param focal_length 焦距 (cm)
   * @param horizontal_aperture 水平孔径 (cm)
   * @param aspect_ratio 宽高比 (宽/高)
   * @param h_ray_num 水平射线数量
   * @param v_ray_num 垂直射线数量
   * @param dis_range 距离范围 [最小，最大] (M)
   */
  RayCasterCamera(mjModel *m, mjData *d, int cam_id, mjtNum focal_length, 
                  mjtNum horizontal_aperture, mjtNum aspect_ratio,
                  int h_ray_num, int v_ray_num, mjtNum dis_range[2]);
  ~RayCasterCamera();
  void init(mjModel *m, mjData *d, int cam_id, mjtNum focal_length, 
            mjtNum horizontal_aperture, mjtNum aspect_ratio,
            int h_ray_num, int v_ray_num, mjtNum dis_range[2]);
  /** @brief 绘制测量射线 在mjv_updateScene和mjr_render中间
   * @param ratio 绘制比例
   * @param edge 仅绘制边缘
   */
  void draw_deep_ray(mjvScene *scn, int ratio, bool edge = false);
  /** @brief 绘制距离线段 在mjv_updateScene和mjr_render中间
   * @param ratio 绘制比例
   */
  void draw_deep(mjvScene *scn, int ratio);
  /** @brief 获取距离 数值存放在dist中*/
  void get_distance();
  /** @brief 反或距离深度图OpenCV格式*/
  cv::Mat get_image();

  /** @brief 获取dist中索引
  * @param h 水平索引
  * @param v 垂直索引
  无效索引返回-1
  */
  int get_idx(int h, int v);

  mjtNum *dist; // 距离 h_ray_num * v_ray_num
  int nray;     // 射线数量

private:
  mjModel *m;
  mjData *d;
  int cam_id;
  mjtNum *pos;
  mjtNum quat[4];
  mjtNum *mat;         // 相机的旋转矩阵
  mjtNum focal_length = 24.0; // 焦距 (cm)
  mjtNum horizontal_aperture = 20.955; // 水平孔径 (cm)
  mjtNum aspect_ratio = 16.0/9.0; // 宽高比
  int h_ray_num = 500;
  int v_ray_num = 500;
  mjtNum h_pixel_size = 0.0; // 像素水平尺寸 (cm)
  mjtNum v_pixel_size = 0.0; // 像素垂直尺寸 (cm)
  mjtNum deep_max = 8;
  mjtNum deep_min = 0.05;
  mjtNum deep_min_ratio;
  mjtNum *_ray_vec; // h_ray_num * v_ray_num * 3 相对于相机坐标系的偏转
  mjtNum *ray_vec;  // h_ray_num * v_ray_num * 3 世界坐标系下的偏转
  int *geomids;
  mjtNum *dist_ratio;

  void _init();
  int _get_idx(int h, int v);
  // 计算射线向量
  void compute_ray_vec();
  // 计算射线向量，基于虚拟平面方法
  void compute_ray_vec_virtual_plane();
  void draw_line(mjvScene *scn, mjtNum *from, mjtNum *to, mjtNum width,
                 float *rgba);
};

RayCasterCamera::RayCasterCamera(mjModel *m, mjData *d, int cam_id, mjtNum focal_length,
                       mjtNum horizontal_aperture, mjtNum aspect_ratio,
                       int h_ray_num, int v_ray_num, mjtNum dis_range[2]) {
  init(m, d, cam_id, focal_length, horizontal_aperture, aspect_ratio, 
       h_ray_num, v_ray_num, dis_range);
}

RayCasterCamera::~RayCasterCamera() {
  delete[] _ray_vec;
  delete[] ray_vec;
  delete[] geomids;
  delete[] dist;
  delete[] dist_ratio;
}

void RayCasterCamera::init(mjModel *m, mjData *d, int cam_id, mjtNum focal_length,
                      mjtNum horizontal_aperture, mjtNum aspect_ratio,
                      int h_ray_num, int v_ray_num, mjtNum dis_range[2]) {
  this->m = m;
  this->d = d;
  this->cam_id = cam_id;
  this->focal_length = focal_length;
  this->horizontal_aperture = horizontal_aperture;
  this->aspect_ratio = aspect_ratio;
  this->h_ray_num = h_ray_num;
  this->v_ray_num = v_ray_num;
  
  // 计算像素尺寸
  h_pixel_size = horizontal_aperture / h_ray_num;
  v_pixel_size = (horizontal_aperture / aspect_ratio) / v_ray_num;
  
  deep_min = dis_range[0];
  deep_max = dis_range[1];
  _init();
}

void RayCasterCamera::_init() {
  nray = h_ray_num * v_ray_num;
  deep_min_ratio = deep_min / deep_max;

  pos = d->cam_xpos + cam_id * 3;
  mat = d->cam_xmat + cam_id * 9;

  _ray_vec = new mjtNum[h_ray_num * v_ray_num * 3];
  ray_vec = new mjtNum[h_ray_num * v_ray_num * 3];
  geomids = new int[h_ray_num * v_ray_num];
  dist = new mjtNum[h_ray_num * v_ray_num];
  dist_ratio = new mjtNum[h_ray_num * v_ray_num];
  compute_ray_vec_virtual_plane();
}

int RayCasterCamera::get_idx(int v, int h) {
  int idx = _get_idx(v, h);
  if (idx < 0 || idx >= nray) {
    std::cerr << "Index out of range" << std::endl;
    return -1;
  }
  return idx;
}

int RayCasterCamera::_get_idx(int v, int h) { return v * h_ray_num + h; }

void RayCasterCamera::compute_ray_vec() {
  for (int i = 0; i < v_ray_num; i++) {
    for (int j = 0; j < h_ray_num; j++) {
      int idx = _get_idx(i, j) * 3;
      mju_mulMatVec3(ray_vec + idx, mat, _ray_vec + idx);
    }
  }
}

void RayCasterCamera::get_distance() {
  compute_ray_vec();
  mj_multiRay(m, d, pos, ray_vec, NULL, 1, -1, geomids, dist_ratio, nray,
              deep_max);
  for (int i = 0; i < nray; i++) {
    if (geomids[i] == -1) {
      dist_ratio[i] = 1;
    } else if (dist_ratio[i] > 1) {
      dist_ratio[i] = 1;
    } else if (dist_ratio[i] < deep_min_ratio) {
      dist_ratio[i] = deep_min_ratio;
    }
    dist[i] = deep_max * dist_ratio[i];
  }
}

cv::Mat RayCasterCamera::get_image() {
  cv::Mat image(v_ray_num, h_ray_num, CV_8UC1);
  for (int i = 0; i < v_ray_num; i++) {
    for (int j = 0; j < h_ray_num; j++) {
      int idx = _get_idx(i, j);
      if (geomids[idx] == -1) {
        image.at<uchar>(i, j) = 0;
      } else {
        image.at<uchar>(i, j) = 255 - dist_ratio[idx] * 255;
      }
    }
  }
  return image;
}

void RayCasterCamera::draw_line(mjvScene *scn, mjtNum *from, mjtNum *to,
                           mjtNum width, float *rgba) {
  scn->ngeom += 1;
  mjvGeom *geom = scn->geoms + scn->ngeom - 1;
  mjv_initGeom(geom, mjGEOM_SPHERE, NULL, NULL, NULL, rgba);
  mjv_connector(geom, mjGEOM_LINE, width, from, to);
}

void RayCasterCamera::compute_ray_vec_virtual_plane() {
  // 计算虚拟平面的尺寸
  mjtNum vertical_aperture = horizontal_aperture / aspect_ratio;
  
  // 计算虚拟平面的边界
  mjtNum half_width = horizontal_aperture / 2.0;
  mjtNum half_height = vertical_aperture / 2.0;
  
  // 计算虚拟平面上的点（相机坐标系）
  for (int i = 0; i < v_ray_num; i++) {
    for (int j = 0; j < h_ray_num; j++) {
      // 计算像素在虚拟平面上的位置
      mjtNum x = (j + 0.5 - h_ray_num/2.0) * h_pixel_size;
      mjtNum y = (v_ray_num/2.0 - i - 0.5) * v_pixel_size; // Y轴翻转
      mjtNum z = -focal_length;
      
      // 计算从相机原点到虚拟平面上该点的向量
      mjtNum point_on_plane[3] = {x, y, z};
      
      // 归一化这个向量得到射线方向
      mjtNum norm = mju_norm3(point_on_plane);
      mjtNum dir[3] = {
        point_on_plane[0] / norm,
        point_on_plane[1] / norm,
        point_on_plane[2] / norm
      };
      
      // 存储射线方向（注意：MuJoCo中的射线方向需要乘以距离）
      int idx = _get_idx(i, j) * 3;
      _ray_vec[idx + 0] = dir[0] * deep_max;
      _ray_vec[idx + 1] = dir[1] * deep_max;
      _ray_vec[idx + 2] = dir[2] * deep_max;
    }
  }
}

void RayCasterCamera::draw_deep_ray(mjvScene *scn, int ratio, bool edge) {
  float color[4] = {0.0, 1.0, 0.0, 0.5};
  auto draw = [&](int idx) {
    mjtNum end[3] = {pos[0], pos[1], pos[2]};
    mju_addTo3(end, ray_vec + idx);
    draw_line(scn, pos, end, 2, color);
  };
  if (edge) {
    for (int i = 0; i < v_ray_num; i += ratio) {
      int idx = _get_idx(i, 0) * 3;
      draw(idx);
      idx = _get_idx(i, h_ray_num - 1) * 3;
      draw(idx);
    }
    for (int j = 0; j < h_ray_num; j += ratio) {
      int idx = _get_idx(0, j) * 3;
      draw(idx);
      idx = _get_idx(v_ray_num - 1, j) * 3;
      draw(idx);
    }
  } else {
    for (int i = 0; i < v_ray_num; i += ratio) {
      for (int j = 0; j < h_ray_num; j += ratio) {
        int idx = _get_idx(i, j) * 3;
        draw(idx);
      }
    }
  }
}

void RayCasterCamera::draw_deep(mjvScene *scn, int ratio) {
  float color[4] = {0.0, 0.0, 1.0, 0.2};
  for (int i = 0; i < v_ray_num; i += ratio) {
    for (int j = 0; j < h_ray_num; j += ratio) {
      int idx = _get_idx(i, j);
      mjtNum end[3] = {pos[0], pos[1], pos[2]};
      mju_addToScl3(end, ray_vec + (idx * 3), dist_ratio[idx]);
      draw_line(scn, pos, end, 2, color);
    }
  }
}