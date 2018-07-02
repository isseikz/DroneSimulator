# Quadrotor.pyに関して

## 2018/07/02
分配則を以下の過程で設計した．
まず，機体プロペラが生成する機体軸に対する力とモーメントは下式で得られる．
$$
\begin{equation}
\left[
\begin{array}{cc}
-1 & -1 & -1 & -1 \\
0 & d & 0 & -d\\
d & 0 & -d & 0\\
\kappa_{t} & -\kappa_{t} & \kappa_{t} & -\kappa_{t}
\end{array}
\right]
\left(
\begin{array}{cc}
f_1\\
f_2\\
f_3\\
f_4\\
\end{array}
\right)
=
\left(
  \begin{array}{cc}
  F_{total}\\
  M_{x}\\
  M_{y}\\
  M_{z}
  \end{array}
\right)
\end{equation}
$$

このとき，$d$は機体の重心からプロペラまでの腕の長さを表し，$\kappa_{t}$ はプロペラの推力と回転によって生じる反トルクとの関係である．
$d, \kappa_{t}$ はともに非零であるから，この行列は逆行列を持つ．したがって，制御則が要求する推力$F_{req}$ ,モーメント $M_{req}$ を用いて以下の式を得られる．
$$
\begin{eqnarray*}
\left(
\begin{array}{cc}
f_1\\
f_2\\
f_3\\
f_4\\
\end{array}
\right)

&=&

\left[
\begin{array}{cc}
-1 & -1 & -1 & -1 \\
0 & d & 0 & -d\\
d & 0 & -d & 0\\
\kappa_{t} & -\kappa_{t} & \kappa_{t} & -\kappa_{t}
\end{array}
\right]^{-1}

\left(
  \begin{array}{cc}
  F_{req, total}\\
  M_{req, x}\\
  M_{req, y}\\
  M_{req, z}
  \end{array}
\right)\\

&=&
\frac{1}{4\kappa_{t}d^2}
\left[
\begin{array}{cc}

\end{array}
\right]

\left(
  \begin{array}{cc}
  F_{req, total}\\
  M_{req, x}\\
  M_{req, y}\\
  M_{req, z}
  \end{array}
\right)

\end{eqnarray*}
$$
