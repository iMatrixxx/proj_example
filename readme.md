旋转($\lambda$)、缩放(s)、平移($c_x, c_y$)的转换矩阵求解：
$$\begin{bmatrix}
   x_u \\
   y_u \\
\end{bmatrix}=\begin{bmatrix}
   s*cos\lambda & -s*sin\lambda & c_x \\
   s*sin\lambda & s*cos\lambda & c_y \\
\end{bmatrix}\begin{bmatrix}
   x_m \\
   y_m \\
   1 \\ 
\end{bmatrix}$$

$$\begin{bmatrix}
   x_u \\
   y_u \\
\end{bmatrix}=\begin{bmatrix}
   x_a & -x_b & c_x \\
   x_b & x_a & c_y \\
\end{bmatrix}\begin{bmatrix}
   x_m \\
   y_m \\
   1 \\ 
\end{bmatrix}$$

$$\begin{bmatrix}
   x_u \\
   y_u \\
\end{bmatrix}=\begin{bmatrix}
   x_m & -y_m & 1 & 0 \\
   y_m & x_m & 0 & 1 \\
\end{bmatrix}\begin{bmatrix}
   x_a \\
   x_b \\
   c_x \\
   c_y 
\end{bmatrix}$$