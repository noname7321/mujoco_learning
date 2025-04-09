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