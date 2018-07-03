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

## 2018/07/03 Yawing制御を放棄する分配則
1基のプロペラが故障した状態を想定して、Yawingの制御を放棄した分配則を設計した。
一般のクアッドロータ機では、機体は総推力（1方向の位置）、モーメント（3軸姿勢）を制御する。この時の各プロペラの出力と力・モーメントは2018/07/02に記した。プロペラが1基故障したとき、例えば本項では4番プロペラ（プロペラの番号と位置の関係は、1:前, 2:左, 3: 後, 4:右）の回転と推力が失われた場合を想定する。この時の力とモーメントの関係は$f_{4}=0$ から下式で表せる。

$$
\begin{equation}
\left[
\begin{array}{cc}
-1 & -1 & -1\\
0 & d & 0\\
d & 0 & -d\\
\kappa_{t} & -\kappa_{t} & \kappa_{t}
\end{array}
\right]
\left(
\begin{array}{cc}
f_1\\
f_2\\
f_3\\
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

Yawingの制御を放棄するとは、結局はこの逆行列を作るために左辺の行列の4行目を無視するということだ。つまり、

$$
\begin{eqnarray*}
\left(
\begin{array}{cc}
f_1\\
f_2\\
f_3\\
\end{array}
\right)

&=&

\left[
\begin{array}{cc}
-1 & -1 & -1\\
0 & d & 0\\
d & 0 & -d\\
\end{array}
\right]^{-1}

\left(
  \begin{array}{cc}
  F_{req, total}\\
  M_{req, x}\\
  M_{req, y}\\
  \end{array}
\right)\\

&=&
\frac{1}{2d}
\left[
  \begin{array}{cc}
    -d & -1 &  1 \\
    0  &  2 &  0 \\
    -d & -1 & -1 \\
  \end{array}
\right]

\left(
  \begin{array}{cc}
  F_{req, total}\\
  M_{req, x}\\
  M_{req, y}\\
  \end{array}
\right)

\end{eqnarray*}
$$

このとき、機体に加わる力とモーメントは下式で表される。

$$
\begin{eqnarray*}
\left(
\begin{array}{cc}
F    \\
M_{y}\\
M_{y}\\
M_{z}\\
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
\right]

\left[
\begin{array}{cc}

\frac{1}{2d}

\left[
\begin{array}{cc}
-1 & -1 & -1\\
0 & d & 0\\
d & 0 & -d\\
\end{array}
\right]^{-1}

\left(
  \begin{array}{cc}
  F_{req, total}\\
  M_{req, x}\\
  M_{req, y}\\
  \end{array}
\right)\\

0
\end{array}
\right]\\

&=&

\left(
  \begin{array}{cc}
  F_{req}+M_{req,x}\\
  M_{req, x}\\
  M_{req, y}\\
  -(F_{req}+2M_{req,x}/d)
  \end{array}
\right)

\end{eqnarray*}
$$

## 2018/07/03 機体位置のシミュレーション
機体位置を表すシステムの設計を行った．
機体の重心位置の運動方程式は下式で表せる．
$$
\ddot{x}=\frac{1}{m}R
\left(
\begin{array}{cc}
0 \\
0 \\
F_{total}
\end{array}
\right)
-\frac{D}{m}
$$

このとき，mは機体質量，Dは外力（主に空気等による抵抗）を表す．Rは機体座標系のベクトルを慣性座標系に変換する行列で，回転行列(Rotation Matrix)である．機体の回転をEuler角（Roll  $\phi$ , Pitch $\theta$ , Yaw $\psi$ ）で表現すると，
$$
\begin{eqnarray}
R(\phi,\theta,\psi)&=&R(\phi)R(\theta)R(\psi)\\
&=&
\left[
\begin{array}{cc}
1 & 0 & 0\\
0 & cos\phi & -sin\phi\\
0 & sin\phi & cos\phi\\
\end{array}
\right]
\left[
\begin{array}{cc}
cos\theta & 0 & sin\theta\\
0 & 1 & 0\\
-sin\theta & 0 & cos\theta
\end{array}
\right]
\left[
\begin{array}{cc}
cos\psi & -sin\psi & 0\\
sin\psi & cos\psi  & 0\\
0 & 0 & 1\\
\end{array}
\right]
\end{eqnarray}
$$

## 2018/07/03 位置制御
機体位置の制御を，PD制御によって行う制御則を実装した．
PD制御は，目標位置との差を $\tilde{x}=x-x_{nom}$ とすれば，以下のシステムで表現できる．
$$
\ddot{\tilde{x}} = -2\zeta\omega\dot{\tilde{x}}-\omega^2\tilde{x}
$$
とくに，目標位置を固定した場合には，目標位置の微分は零になるため，
$$
\ddot{x} = -2\zeta\omega\dot{x}-\omega^2\tilde{x}
$$
これと重力等の外乱の補償力の合計に対して，機体推力の方向を一致させれば目標位置に向かって飛行をする．上式で求めた $\ddot{x}$ から，機体が出すべき推力 $F_{req}$ とモーメント $M_{req}$ が得られる．
$$
\begin{eqnarray}
F_{req} =
\frac{m}{cos\theta cos\phi}
\left(
  \begin{array}{cc}
0 & 0&1
  \end{array}
\right)
(\ddot{\tilde{x}}-g)\\

M_{req}=-2\zeta\omega{\dot{P}}-\omega^2{(-\frac{\tilde{x}}{|\tilde{x}|}\times{n_z})}
\end{eqnarray}
$$

ここで，$P, n_z$ はそれぞれ角速度ベクトル，機体座標系のz方向を表す． 座標系を右手系かつ地面方向を正にとっているため，推力方向と$z$方向は真逆となる．
