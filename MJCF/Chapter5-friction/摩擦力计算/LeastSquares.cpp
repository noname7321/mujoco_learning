#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;
//MatrixXd 是列向量
int main()
{
    // std::vector<double> U = {3, 4, 5, 6, 7, 9, 9, 10};
    // int n = 4;
    // int rows = n, cols = n * (n - 1) / 2;
    // MatrixXd A(cols, rows);
    // int num1 = 0, num2 = 1;
    // for (int i = 0; i < cols; i++)
    // {
    //     if (num2 == rows)
    //     {
    //         num1++;
    //         num2 = num1 + 1;
    //     }
    //     A(i, num1) = 1;
    //     A(i, num2) = 1;
    //     num2++;
    // }
    // std::cout << "A is:\n"
    //           << A << std::endl;

    // VectorXd b(cols);
    // for (int i = 0; i < cols; i++)
    // {
    //     b(i) = U[i];
    // }
    // std::cout << "b is:\n"
    //           << b.transpose() << std::endl;

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

    // 计算并打印残差
    VectorXd r = A * x - b; // 计算残差
    std::cout << "残差：\n"
              << r.transpose() << std::endl;

    return 0;
}
