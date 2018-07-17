# 解析計算ログ

## ある角速度近傍での角速度ダイナミクス
機体の姿勢ダイナミクスは一般に以下の非線形方程式で記述される。

$$
\boldsymbol{\dot{\omega}}=I^{-1}\{\boldsymbol{\tau} -\boldsymbol{\omega}\times(I\boldsymbol{\omega)}\}
$$

これを $\boldsymbol{\omega}=(\acute{\omega}_{x}, \acute{\omega}_y, \acute{\omega}_z)$ 近傍でテイラー展開することにより、局所的な線形常微分方程式を導く。

$$
\begin{eqnarray}
  \boldsymbol{\dot{\omega}}&=&I^{-1}\{\boldsymbol{\tau} -\boldsymbol{\omega}\times(I\boldsymbol{\omega)}\}\\
  &=&
  - \left(
    \begin{array}{cc}
    \frac{I_y - I_z}{I_x}\omega_y\omega_z\\
    \frac{I_z-I_x}{I_y}\omega_x\omega_z\\
    \frac{I_x-I_y}{I_z}\omega_x\omega_y\\
    \end{array}
    \right)
  + I^{-1}\boldsymbol{\tau}\\
  &=&
  - \left(
    \begin{array}{cc}
      \frac{I_y - I_z}{I_x} \{\acute{\omega}_{y}\acute{\omega}_{z}+\acute{\omega}_{z}(\omega_{y}-\acute{\omega}_{y})+\acute{\omega}_{y}(\omega_{z}-\acute{\omega}_{z})\}\\
      \frac{I_z-I_x}{I_y}\{\acute{\omega}_{x}\acute{\omega}_{z}+\acute{\omega}_{z}(\omega_{x}-\acute{\omega}_{x})+\acute{\omega}_{x}(\omega_{z}-\acute{\omega}_{z})\}\\
      \frac{I_x-I_y}{I_z}\{\acute{\omega}_{x}\acute{\omega}_{y}+\acute{\omega}_{y}(\omega_{x}-\acute{\omega}_{x})+\acute{\omega}_{x}(\omega_{y}-\acute{\omega}_{y})\}\\
    \end{array}
    \right)
    + I^{-1} \boldsymbol{\tau}\\
  &=&
  -
  \left[ \begin{array}{cc}
  0 & \frac{I_y - I_z}{I_x}\acute{\omega}_{z} & \frac{I_y - I_z}{I_x}\acute{\omega}_{y} \\
  \frac{I_z-I_x}{I_y}\acute{\omega}_{z} & 0 & \frac{I_z-I_x}{I_y}\acute{\omega}_{x}\\
  \frac{I_x-I_y}{I_z}\acute{\omega}_{x} & \frac{I_x-I_y}{I_z}\acute{\omega}_{y} & 0\\
  \end{array} \right]

  \left( \begin{array}{cc}
    \omega_{x}\\
    \omega_{y}\\
    \omega_{z}\\
  \end{array} \right)

  -
  \left( \begin{array}{cc}
    \frac{I_y - I_z}{I_x}\acute{\omega}_{y}\acute{\omega}_{z}\\
    \frac{I_z - I_x}{I_y}\acute{\omega}_{x}\acute{\omega}_{z}\\
    \frac{I_x - I_y}{I_z}\acute{\omega}_{x}\acute{\omega}_{y}\\
  \end{array} \right)

  + I^{-1} \boldsymbol{\tau}


\end{eqnarray}
$$
