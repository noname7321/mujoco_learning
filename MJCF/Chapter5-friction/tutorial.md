# friction
对于两个物体之间产生摩擦的过程中，摩擦力系数μ将会取两个物体的几何平均值，即
![](../asset/mu.png)
 无论是滑动，扭矩还是滚动都是这样取μ。我们在geom的friction中可以设置这三个摩擦系数。

## 滑动摩擦
在 mujoco中，滑动摩擦力模型和我们中学学的是一样的。
<center>f=μ∙Fn</center>
#### 旋转摩擦
旋转摩擦是物体绕接触点的法向轴旋转时的摩擦力。比如车轮。
<center>τ=μ∙r∙Fnormal</center>
● r是接触面上相对运动的半径
● Fnormal是接触的法向力
#### 滚动摩擦
滚动摩擦是两个物体接触时，阻止一个物体在另一个物体表面上滚动的摩擦力。滚动摩擦
产生的阻力通常远小于滑动摩擦，因为滚动摩擦主要是由于接触面的变形而引起的。
<center>τ=μ∙r∙Fnormal</center>
● r是滚动物体的半径
● Fnormal是接触的法向力

## 滑动摩擦力求解：
&emsp;&emsp;在仿真过程中旋转和滚动摩擦的都很小，可以在默认给定的值附近微调，所以我们主要计算的是滑动摩擦。
![](../asset/slove_mu1.png)
![](../asset/slove_mu2.png)
此时该方程最理想情况是有唯一解，但是现实中我们不一定会去两两测量摩擦系数（方程数量不为），方程也可能是无解状态。所以我们可以使用最小二乘的方法求出近似解，即：
C++ Eigen库实现：
```C++
MatrixXd A(6, 4);
A << 1, 1, 0, 0,
1, 0, 1, 0,
1, 0, 0, 1,
0, 1, 1, 0,
0, 1, 0, 1,
0, 0, 1, 1;
VectorXd b(6);
b << 3, 4, 5, 6, 7, 8;
JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);
VectorXd x = svd.solve(b);
std::cout << "A is:\n"
<< A.transpose() << std::endl;
std::cout << "b is:\n"
<< b.transpose() << std::endl;
std::cout << "x: " << std::endl;
std::cout << x.transpose() << std::endl;
```
python实现：
```python
import numpy as np
A = np.array([
    [1, 1, 0, 0],
    [1, 0, 1, 0],
    [1, 0, 0, 1],
    [0, 1, 1, 0],
    [0, 1, 0, 1],
    [0, 0, 1, 1]
], dtype=np.float64)

b = np.array([3, 4, 5, 6, 7, 8], dtype=np.float64)
U, s, Vh = np.linalg.svd(A, full_matrices=False)
threshold = 1e-15
s_pinv = np.zeros(s.shape)
for i in range(len(s)):
    if s[i] > threshold:
        s_pinv[i] = 1.0 / s[i]
Sigma_pinv = np.diag(s_pinv)
x = Vh.T @ Sigma_pinv @ U.T @ b
print("A.T:\n", A.T)
print("b:\n", b)
print("x:\n", x)
```