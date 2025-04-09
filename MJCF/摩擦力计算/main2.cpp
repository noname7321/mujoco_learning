#include <iostream>
#include <Eigen/Dense>

int main()
{
    // 定义矩阵A和向量b
    Eigen::MatrixXd A(4, 4);
    Eigen::VectorXd b(4);

    // 填充矩阵A
    A << 1, 1, 0, 0,
        1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 1;

    // 填充向量b
    b << 1, 2, 3, 10;

    // 尝试求解线性方程组 A * x = b
    // Eigen::VectorXd x = A.colPivHouseholderQr().solve(b);

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd x = svd.solve(b);

    // 输出结果
    std::cout << "解为: \n"
              << x << std::endl;

    return 0;
}
