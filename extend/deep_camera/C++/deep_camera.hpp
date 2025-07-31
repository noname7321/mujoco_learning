#include <mujoco/mjtnum.h>
#pragma onece
#include "opencv2/opencv.hpp"
#include <mujoco/mujoco.h>

class DeepCamera {
public:
  DeepCamera();
  /** @brief 初始化相机
   * @param m mjModel
   * @param d mjData
   * @param cam_id 相机id
   * @param fov_h 水平视场角 (度)
   * @param fov_v 垂直视场角 (度)
   * @param h_ray_num 水平射线数量
   * @param v_ray_num 垂直射线数量
   * @param dis_range 距离范围 [最小，最大] (M)
   */
  DeepCamera(mjModel *m, mjData *d, int cam_id, mjtNum fov_h, mjtNum fov_v,
             int h_ray_num, int v_ray_num, mjtNum dis_range[2]);
  /** @brief 初始化相机
   * @param m mjModel
   * @param d mjData
   * @param cam_id 相机id
   * @param fov_h 水平视场角 (度)
   * @param fov_v 垂直视场角 (度)
   * @param h_res 水平分辨率 (度)
   * @param v_res 垂直分辨率 (度)
   * @param dis_range 距离范围 [最小，最大] (M)
   */
  DeepCamera(mjModel *m, mjData *d, int cam_id, mjtNum fov_h, mjtNum fov_v,
             mjtNum h_res, mjtNum v_res, mjtNum dis_range[2]);
  ~DeepCamera();
  void init(mjModel *m, mjData *d, int cam_id, mjtNum h, mjtNum v,
            int h_ray_num, int v_ray_num, mjtNum dis_range[2]);
  void init(mjModel *m, mjData *d, int cam_id, mjtNum h, mjtNum v, mjtNum h_res,
            mjtNum v_res, mjtNum dis_range[2]);
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

  mjtNum *dist; //距离 h_ray_num * v_ray_num
  int nray;     // 射线数量

private:
  mjModel *m;
  mjData *d;
  int cam_id;
  mjtNum *pos;
  mjtNum quat[4];
  mjtNum *mat;
  mjtNum fov_h = 50.0; // 水平总视场角 (度)
  mjtNum fov_v = 50.0; // 垂直总视场角 (度)
  int h_ray_num = 500;
  int v_ray_num = 500;
  mjtNum h_res = 0.0;
  mjtNum v_res = 0.0;
  mjtNum deep_max = 8;
  mjtNum deep_min = 0.05;
  mjtNum deep_min_ratio;
  mjtNum *_ray_vec; // h_ray_num * v_ray_num * 3
  mjtNum *ray_vec;  // h_ray_num * v_ray_num * 3
  int *geomids;
  mjtNum *dist_ratio;

  void _init();
  int _get_idx(int h, int v);
  // 计算射线向量
  void compute_ray_vec();
  // 计算射线向量，以相机坐标系为参考系
  void compute_ray_vec_in_cam();
  void draw_line(mjvScene *scn, mjtNum *from, mjtNum *to, mjtNum width,
                 float *rgba);
};

DeepCamera::DeepCamera(mjModel *m, mjData *d, int cam_id, mjtNum fov_h,
                       mjtNum fov_v, int h_ray_num, int v_ray_num,
                       mjtNum dis_range[2]) {
  init(m, d, cam_id, fov_h, fov_v, h_ray_num, v_ray_num, dis_range);
}

DeepCamera::DeepCamera(mjModel *m, mjData *d, int cam_id, mjtNum fov_h,
                       mjtNum fov_v, mjtNum h_res, mjtNum v_res,
                       mjtNum dis_range[2]) {
  init(m, d, cam_id, fov_h, fov_v, h_res, v_res, dis_range);
}

DeepCamera::~DeepCamera() {
  delete[] _ray_vec;
  delete[] ray_vec;
  delete[] geomids;
  delete[] dist;
  delete[] dist_ratio;
}

void DeepCamera::init(mjModel *m, mjData *d, int cam_id, mjtNum fov_h,
                      mjtNum fov_v, int h_ray_num, int v_ray_num,
                      mjtNum dis_range[2]) {
  this->m = m;
  this->d = d;
  this->cam_id = cam_id;
  this->fov_h = fov_h;
  this->fov_v = fov_v;
  this->h_ray_num = h_ray_num;
  this->v_ray_num = v_ray_num;
  h_res = fov_h / h_ray_num;
  v_res = fov_v / v_ray_num;
  deep_min = dis_range[0];
  deep_max = dis_range[1];
  _init();
}

void DeepCamera::init(mjModel *m, mjData *d, int cam_id, mjtNum fov_h,
                      mjtNum fov_v, mjtNum h_res, mjtNum v_res,
                      mjtNum dis_range[2]) {
  this->m = m;
  this->d = d;
  this->cam_id = cam_id;
  this->fov_h = fov_h;
  this->fov_v = fov_v;
  this->h_res = h_res;
  this->v_res = v_res;
  h_ray_num = fov_h / h_res;
  v_ray_num = fov_v / v_res;
  deep_min = dis_range[0];
  deep_max = dis_range[1];
  _init();
}

void DeepCamera::_init() {
  nray = h_ray_num * v_ray_num;
  deep_min_ratio = deep_min / deep_max;

  pos = d->cam_xpos + cam_id * 3;
  mat = d->cam_xmat + cam_id * 9;

  _ray_vec = new mjtNum[h_ray_num * v_ray_num * 3];
  ray_vec = new mjtNum[h_ray_num * v_ray_num * 3];
  geomids = new int[h_ray_num * v_ray_num];
  dist = new mjtNum[h_ray_num * v_ray_num];
  dist_ratio = new mjtNum[h_ray_num * v_ray_num];
  compute_ray_vec_in_cam();
}

int DeepCamera::get_idx(int v, int h) {
  int idx = _get_idx(v, h);
  if (idx < 0 || idx >= nray) {
    std::cerr << "Index out of range" << std::endl;
    return -1;
  }
  return idx;
}

int DeepCamera::_get_idx(int v, int h) { return v * h_ray_num + h; }

void DeepCamera::compute_ray_vec() {
  for (int i = 0; i < v_ray_num; i++) {
    for (int j = 0; j < h_ray_num; j++) {
      int idx = _get_idx(i, j) * 3;
      mju_mulMatVec3(ray_vec + idx, mat, _ray_vec + idx);
    }
  }
}

void DeepCamera::get_distance() {
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

cv::Mat DeepCamera::get_image() {
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

void DeepCamera::draw_line(mjvScene *scn, mjtNum *from, mjtNum *to,
                           mjtNum width, float *rgba) {
  scn->ngeom += 1;
  mjvGeom *geom = scn->geoms + scn->ngeom - 1;
  mjv_initGeom(geom, mjGEOM_SPHERE, NULL, NULL, NULL, rgba);
  mjv_connector(geom, mjGEOM_LINE, width, from, to);
}

void DeepCamera::compute_ray_vec_in_cam() {

  mjtNum ref_vec[3] = {0.0, 0.0, -deep_max};
  mjtNum axis_x[3] = {1.0, 0.0, 0.0};
  mjtNum quat_x[4];
  mjtNum axis_y[3] = {0.0, 1.0, 0.0};
  mjtNum quat_y[4];
  mjtNum combined_quat[4];
  mjtNum res_vec[3];
  mjtNum start_h_angle = fov_h / 2.0;
  mjtNum start_v_angle = fov_v / 2.0;
  for (int i = 0; i < v_ray_num; i++) {
    mjtNum angle_x = ((start_v_angle - v_res * i) / 180.0) * mjPI;
    mju_axisAngle2Quat(quat_x, axis_x, angle_x);
    for (int j = 0; j < h_ray_num; j++) {

      mjtNum angle_y = ((start_h_angle - h_res * j) / 180.0) * mjPI;
      mju_axisAngle2Quat(quat_y, axis_y, angle_y);
      mju_mulQuat(combined_quat, quat_x, quat_y);
      mju_rotVecQuat(res_vec, ref_vec, combined_quat);

      int idx = _get_idx(i, j) * 3;
      _ray_vec[idx + 0] = res_vec[0];
      _ray_vec[idx + 1] = res_vec[1];
      _ray_vec[idx + 2] = res_vec[2];
    }
  }
}

void DeepCamera::draw_deep_ray(mjvScene *scn, int ratio, bool edge) {
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

void DeepCamera::draw_deep(mjvScene *scn, int ratio) {
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
