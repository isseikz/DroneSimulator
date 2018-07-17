# モデル予測制御におけるC/GMRES法
+ based on the pontryagin minimum principle (PMP)
+ deduce a sequence of matrix equations iteratively

## C/GMRES method
+ discrete-time property of the solution algorithm
+ the discrete form of the dynamical model can be deduced by the forward difference method
$$
x(t+1)=x(t)+f(x(t),u(t))\Delta\tau
$$
+ suppose that the predictive horizon is divided into N-step
$$
x_t(k)=x(t+k), u_t(k)=u(t+k),(0\le{k}\le{N},t\rightarrow\infty)
$$
+ for given the initial conditions $x_t(0)=x_t$ , find an optimal control action  $u^{* }$ to minimize the following performance index $(k=0,1,...,N-1,\Delta\tau=\frac{T}{N})$
$$
J=\Phi(x_t(N),N)+\sum_{k=0}^{N-1}L[x_t(k),x_d(k),u_t(k)]\Delta\tau\\
subject\ to\\
x_t(k+1)=x_t(k)+f(x_t(k),u_t(k))\Delta\tau\\
x_t(0)=x_t\\
C(x_t(k),u_t(k))=0\\
u_{min}\le{u_t(k)}\le{u_{max}}
$$
+ inequality constraints can be converted to equality constraints by introducing a dummy input $u_t^{' }$ 
