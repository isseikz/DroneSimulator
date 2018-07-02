# Quadrotor.pyに関して

## 2018/07/02 分配則

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
\frac{1}{4\kappa_{t}d}
\left[
  \begin{array}{cc}
    -\kappa_{t}d & 0 & 2\kappa{t}   &  d \\
    -\kappa_{t}d &  2\kappa{t} & 0  & -d \\
    -\kappa_{t}d & 0 & -2\kappa{t}  &  d \\
    -\kappa_{t}d & -2\kappa{t} & 0  & -d \\
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

ただし、$\kappa_{t}$ が小さい（モデル機では$10^{-2}$ 程度）ため、要求するyawing量が大きいと、プロペラに対して大きな入力を与えることになるため、注意が必要である。場合によっては、$M_{req, z}$ のみ別のゲインで計算することが必要。

## 2018/07/02 プロペラの故障度
プロペラの故障度を、変数`self.normality`を用いて表現した。
$$
f_{real,i}=nf_{i},\\n:[0,1]
$$
