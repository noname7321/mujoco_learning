import numpy as np
import cv2
import math
import mujoco

class DeepCamera:
    def __init__(self, model, data, cam_id, fov_h, fov_v, h_res,
                 v_res, dis_range):
        self.model = model
        self.data = data
        self.cam_id = cam_id
        
        self.fov_h = fov_h
        self.fov_v = fov_v
        self.h_res = h_res
        self.v_res = v_res
        self.h_ray_num = int(fov_h / h_res)
        self.v_ray_num = int(fov_v / v_res)
        self.deep_min = dis_range[0]
        self.deep_max = dis_range[1]

        self.nray = self.v_ray_num * self.h_ray_num
        self.deep_min_ratio = self.deep_min / self.deep_max

        self.pos = self.data.cam_xpos[self.cam_id]
        self.mat = self.data.cam_xmat[self.cam_id]
        
        self._ray_vec = np.zeros((self.v_ray_num*self.h_ray_num*3,1))
        self.ray_vec = np.zeros((self.v_ray_num*self.h_ray_num*3,1))
        self.geomids = np.zeros((self.nray,1),dtype=np.int32)
        self.dist = np.zeros((self.nray,1))
        self.dist_ratio = np.zeros((self.nray,1),dtype=np.float64)
        
        self.compute_ray_vec_in_cam()

    def compute_ray_vec_in_cam(self):
        ref_vec = np.array([0.0, 0.0, -self.deep_max])
        axis_x = np.array([1.0, 0.0, 0.0])
        axis_y = np.array([0.0, 1.0, 0.0])
        
        start_h_angle = self.fov_h / 2.0
        start_v_angle = self.fov_v / 2.0
        
        for i in range(self.v_ray_num):
            for j in range(self.h_ray_num):
                angle_x = (start_v_angle - self.v_res * i)/180 * mujoco.mjPI
                quat_x = np.zeros(4)
                mujoco.mju_axisAngle2Quat(quat_x, axis_x, angle_x)
                
                angle_y = (start_h_angle - self.h_res * j)/180 * mujoco.mjPI
                quat_y = np.zeros(4)
                mujoco.mju_axisAngle2Quat(quat_y, axis_y, angle_y)
                
                combined_quat = np.zeros(4)
                mujoco.mju_mulQuat(combined_quat, quat_y, quat_x)
                
                res_vec = np.zeros(3)
                mujoco.mju_rotVecQuat(res_vec, ref_vec, combined_quat)
                
                idx = self._get_idx(i, j) * 3
                self._ray_vec[idx] = res_vec[0]
                self._ray_vec[idx+1] = res_vec[1]
                self._ray_vec[idx+2] = res_vec[2]

    def compute_ray_vec(self):
        for i in range(self.v_ray_num):
            for j in range(self.h_ray_num):
                idx = self._get_idx(i, j) * 3
                res = np.zeros(3)
                vec = np.array([self._ray_vec[idx], self._ray_vec[idx+1], self._ray_vec[idx+2]])
                mujoco.mju_mulMatVec3(res, self.mat, vec)
                self.ray_vec[idx] = res[0]
                self.ray_vec[idx+1] = res[1]
                self.ray_vec[idx+2] = res[2]

    def get_distance(self):
        self.compute_ray_vec()
        
        pos = self.pos.reshape(3, 1)
        geomgroup = np.zeros((6, 1))
        mujoco.mj_multiRay(
            self.model,
            self.data,
            pos,
            self.ray_vec,
            geomgroup, 
            1,    
            -1,   
            self.geomids,
            self.dist_ratio,
            self.nray,
            self.deep_max
        )
        for i in range(self.nray):
            if self.geomids[i] == -1:
                self.dist_ratio[i] = 1.0
            elif self.dist_ratio[i] > 1.0:
                self.dist_ratio[i] = 1.0
            elif self.dist_ratio[i] < self.deep_min_ratio:
                self.dist_ratio[i] = self.deep_min_ratio
    
            self.dist[i] = self.deep_max * self.dist_ratio[i]
                

    def get_image(self):
        img = np.zeros((self.v_ray_num, self.h_ray_num), dtype=np.uint8)
        for i in range(self.v_ray_num):
            for j in range(self.h_ray_num):
                idx = self._get_idx(i, j)
                if self.geomids[idx] == -1:
                    img[i, j] = 0
                else:
                    img[i, j] = 255 - int(self.dist_ratio[idx] * 255)
        return img

    def _get_idx(self, v, h):
        return v * self.h_ray_num + h

    def get_idx(self, i, j):
        if 0 <= i < self.v_ray_num and 0 <= j < self.h_ray_num:
            return self._get_idx(i, j)
        print("Index out of range")
        return -1