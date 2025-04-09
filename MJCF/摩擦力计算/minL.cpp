#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;
/*最小范数求解
Ax=b
x*= A^T(AA^T)^-1b
*/
int main()
{
    std::vector<double> U = {3, 4, 5, 6, 7, 8, 9, 10};
    int n = 4;
    int rows = n, cols = n * (n - 1) / 2;
    MatrixXd A(cols, rows);
    int num1 = 0, num2 = 1;
    for (int i = 0; i < cols; i++)
    {
        if (num2 == rows)
        {
            num1++;
            num2 = num1 + 1;
        }
        A(i, num1) = 1;
        A(i, num2) = 1;
        num2++;
    }
    std::cout << "A is:\n"
              << A << std::endl;

    VectorXd b(cols);
    for (int i = 0; i < cols; i++)
    {
        b(i) = U[i];
    }
    std::cout << "b is:\n"
              << b.transpose() << std::endl;

    // 求解最小范数问题
    VectorXd x = A.transpose() * (A * A.transpose()).inverse() * b;

    // 输出解 x
    std::cout << "解得 x: " << std::endl;
    std::cout << x.transpose() << std::endl;

    // 计算并打印残差
    VectorXd r = A * x - b; // 计算残差
    std::cout << "残差：\n"
              << r.transpose() << std::endl;

    return 0;
}
