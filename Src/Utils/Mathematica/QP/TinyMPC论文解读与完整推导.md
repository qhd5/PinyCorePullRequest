# TinyMPC 论文解读与完整推导

> 论文：**TinyMPC: Model-Predictive Control on Resource-Constrained Microcontrollers**  
> 作者：Anoushka Alavilli、Khai Nguyen、Sam Schoedel、Brian Plancher、Zachary Manchester  
> 本次阅读版本：arXiv:2310.16985v4，2025-08-12（以所提供 PDF 首页为准）  
> 原文：[本地 PDF](<C:/Users/guo29/Desktop/MPC/TinyMPC.pdf>)；项目地址：<https://tinympc.org>  
> 文档性质：整理前述对话形成的中文学习笔记。涵盖研究动机、问题建模、第 III 部分完整推导、价值函数递推细节、实验解释。本文不是逐字翻译。

## 目录

1. [核心结论与研究动机](#1-核心结论与研究动机)
2. [符号与原始 MPC 问题](#2-符号与原始-mpc-问题)
3. [第 III-A 部分：ADMM 分裂](#3-第-iii-a-部分admm-分裂)
4. [LQR 子问题的完整动态规划推导](#4-lqr-子问题的完整动态规划推导)
5. [复制变量、对偶变量与收敛](#5-复制变量对偶变量与收敛)
6. [第 III-B 部分：预计算与稳态 Riccati](#6-第-iii-b-部分预计算与稳态-riccati)
7. [第 III-C 部分：惩罚参数切换](#7-第-iii-c-部分惩罚参数切换)
8. [完整算法与计算复杂度](#8-完整算法与计算复杂度)
9. [论文实验与结果解读](#9-论文实验与结果解读)
10. [理论边界与常见误解](#10-理论边界与常见误解)
11. [公式速查与推荐阅读顺序](#11-公式速查与推荐阅读顺序)

---

## 1. 核心结论与研究动机

### 1.1 论文研究什么？

论文研究：**如何让带约束的模型预测控制 MPC，在算力与内存有限的微控制器上实时运行。**

其核心路线是：

\[
\boxed{\text{带约束 MPC}\to\text{ADMM 分裂}\to\text{LQR + 约束投影}\to\text{离线预计算与在线向量递推}}
\]

三个关键组成部分：

| 组成部分 | 作用 |
|---|---|
| LQR / Riccati 递推 | 利用动力学沿预测时间链式连接的结构，高效计算状态与控制序列 |
| ADMM | 将动力学与性能优化、状态与输入集合约束分成易处理的子问题 |
| 预计算与缓存 | 将重复使用的矩阵计算提前完成，在线主要执行矩阵乘向量、向量运算与投影 |

TinyMPC 的主要价值在求解器设计：利用控制问题结构适配 MCU，而不是更换 MPC 的基本控制目标。

### 1.2 为什么考虑 MPC？

以 PID 为例，控制器可能先产生控制请求，再按执行器能力限幅：

\[
u=K_pe+K_i\int e\,dt+K_d\dot e,
\qquad u_{\mathrm{actual}}=\operatorname{clip}(u,u_{\min},u_{\max}).
\]

抗积分饱和等设计可以改善饱和工况，但这种基本计算方式没有显式规划未来约束。MPC 将限制直接写入优化：在预测未来运动时，同时考虑电流、力矩、速度、位置边界，并决定是否提前制动。

每个控制周期，MPC：

1. 获取当前状态。
2. 预测未来一段时间的运动。
3. 求解满足约束且代价较小的控制序列。
4. 只执行第一个控制输入，下一周期重新预测和优化。

这称为滚动时域控制。其困难是每个控制周期都要解优化问题；TinyMPC 主要降低这部分计算和存储负担。

### 1.3 本文如何区分信息来源？

- **论文内容**：原文提出的模型、算法结构与实验报告。
- **补充推导**：为理解原文而展开的配方、动态规划、等价变形和数值例子。
- **工程建议与边界分析**：根据算法结构得出的实现建议，不代表论文已逐项验证。

本文涉及的代码生成功能、二阶锥和定点实现等，应按所读论文版本理解；不据此推断当前软件仓库的功能状态。

## 2. 符号与原始 MPC 问题

### 2.1 下标和符号约定

沿用论文：状态时间下标为 \(k=1,\ldots,N\)，输入为 \(k=1,\ldots,N-1\)。ADMM 迭代次数用上标 \(j\) 表示。

| 符号 | 含义 | 维度 |
|---|---|---|
| \(x_k\) | 状态 | \(n\) |
| \(u_k\) | 控制输入 | \(m\) |
| \(A,B\) | 离散动力学矩阵 | \(n\times n, n\times m\) |
| \(Q,Q_f,R\) | 阶段状态、终端状态、输入二次权重 | \(n\times n,n\times n,m\times m\) |
| \(q_k,q_f,r_k\) | 线性代价系数 | \(n,n,m\) |
| \(z_k,w_k\) | 状态、输入的复制变量 | \(n,m\) |
| \(\lambda_k,\mu_k\) | 未缩放对偶变量 | \(n,m\) |
| \(y_k,g_k\) | 缩放对偶变量 | \(n,m\) |
| \(\rho\) | ADMM 惩罚参数 | 正标量 |
| \(P_k,p_k,c_k\) | 价值函数的二次、一次、常数系数 | \(n\times n,n,1\) |
| \(K_k,d_k\) | 状态反馈增益、仿射前馈项 | \(m\times n,m\) |

### 2.2 动力学与约束

\[
x_{k+1}=Ax_k+Bu_k,
\qquad x_1=\hat x.
\]

其中 \(\hat x\) 为本控制周期的状态估计。初始状态固定是 MPC 必需条件，本文显式补出，原文部分紧凑公式没有重复列出。

状态与输入约束为：

\[
x_k\in\mathcal X,\qquad u_k\in\mathcal U.
\]

例如电流、速度限幅：

\[
-I_{\max}\le i_k\le I_{\max},\qquad
-v_{\max}\le v_k\le v_{\max}.
\]

### 2.3 代价函数

\[
\begin{aligned}
J(x,u)={}&\frac12x_N^\top Q_fx_N+q_f^\top x_N\\
&+\sum_{k=1}^{N-1}\left[
\frac12x_k^\top Qx_k+q_k^\top x_k
+\frac12u_k^\top Ru_k+r_k^\top u_k
\right].
\end{aligned}
\]

假设 \(Q\succeq0,Q_f\succeq0,R\succ0\)，约束集合为凸集。当这些集合由线性等式、不等式描述时，得到凸二次规划 QP。

轨迹跟踪常写成：

\[
\frac12(x_k-x_k^{\mathrm{ref}})^\top Q(x_k-x_k^{\mathrm{ref}}).
\]

展开并忽略与决策变量无关的常数：

\[
\frac12x_k^\top Qx_k-(Qx_k^{\mathrm{ref}})^\top x_k.
\]

因此 \(q_k=-Qx_k^{\mathrm{ref}}\)；输入参考对应 \(r_k=-Ru_k^{\mathrm{ref}}\)。固定二次权重时，更新参考主要改变线性项。

## 3. 第 III-A 部分：ADMM 分裂

### 3.1 引入复制变量

令：

\[
x_k=z_k,\qquad u_k=w_k.
\]

让 \(x,u\) 负责动力学与性能，让 \(z,w\) 负责集合约束。定义集合指示函数：

\[
I_{\mathcal X}(z)=
\begin{cases}
0,&z\in\mathcal X,\\
+\infty,&z\notin\mathcal X.
\end{cases}
\]

对序列的指示函数可理解为各时刻指示函数之和。原问题等价改写为：

\[
\begin{aligned}
\min_{x,u,z,w}\quad &J(x,u)+I_{\mathcal X}(z)+I_{\mathcal U}(w)\\
\mathrm{s.t.}\quad &x_1=\hat x,\quad x_{k+1}=Ax_k+Bu_k,\\
&x_k=z_k,\quad u_k=w_k.
\end{aligned}
\]

这里的复制变量不是允许硬约束违反的软约束松弛量；一致性收敛时，恢复原约束。

### 3.2 增广拉格朗日函数：论文式（13）

\[
\begin{aligned}
\mathcal L_\rho={}&J(x,u)+I_{\mathcal X}(z)+I_{\mathcal U}(w)\\
&+\sum_{k=1}^{N}\left[\lambda_k^\top(x_k-z_k)
+\frac\rho2\|x_k-z_k\|^2\right]\\
&+\sum_{k=1}^{N-1}\left[\mu_k^\top(u_k-w_k)
+\frac\rho2\|u_k-w_k\|^2\right].
\end{aligned}
\]

动力学约束仍保留在 \(x,u\) 子问题中。线性乘子项协调一致性，二次项惩罚不一致。

**记号说明：**论文式（13）排版将乘子也列在最小化符号下；严格的原始—对偶解释中，乘子不是普通最小化变量。ADMM 对原始变量交替最小化，并对乘子执行对偶上升。

### 3.3 原始变量更新：式（14）—（15）

固定第 \(j\) 次迭代的 \(z,w,\lambda,\mu\)，求 \(x,u\)。以状态项为例：

\[
\begin{aligned}
&\frac12x_k^\top Qx_k+q_k^\top x_k
+\lambda_k^\top(x_k-z_k)+\frac\rho2\|x_k-z_k\|^2\\
={}&\frac12x_k^\top Qx_k+q_k^\top x_k
+\lambda_k^\top x_k-\lambda_k^\top z_k\\
&+\frac\rho2x_k^\top x_k-\rho z_k^\top x_k+\frac\rho2z_k^\top z_k\\
={}&\frac12x_k^\top(Q+\rho I)x_k
+(q_k+\lambda_k-\rho z_k)^\top x_k+\text{常数}.
\end{aligned}
\]

所以：

\[
\boxed{
\begin{aligned}
\tilde Q&=Q+\rho I_n,&\tilde q_k&=q_k+\lambda_k-\rho z_k,\\
\tilde R&=R+\rho I_m,&\tilde r_k&=r_k+\mu_k-\rho w_k,\\
\tilde Q_f&=Q_f+\rho I_n,&\tilde q_f&=q_f+\lambda_N-\rho z_N.
\end{aligned}}
\]

得到论文式（14）对应的子问题：

\[
\begin{aligned}
\min_{x,u}\quad &\frac12x_N^\top\tilde Q_fx_N+\tilde q_f^\top x_N\\
&+\sum_{k=1}^{N-1}\left[
\frac12x_k^\top\tilde Qx_k+\tilde q_k^\top x_k
+\frac12u_k^\top\tilde Ru_k+\tilde r_k^\top u_k\right]\\
\mathrm{s.t.}\quad &x_1=\hat x,\quad x_{k+1}=Ax_k+Bu_k.
\end{aligned}
\]

集合约束已移出这一子问题，剩下的是带线性项的 LQR。

### 3.4 缩放对偶变量：式（16）

定义：

\[
y_k=\lambda_k/\rho,\qquad g_k=\mu_k/\rho.
\]

则：

\[
\boxed{
\begin{aligned}
\tilde q_k&=q_k+\rho(y_k-z_k),\\
\tilde r_k&=r_k+\rho(g_k-w_k),\\
\tilde q_f&=q_f+\rho(y_N-z_N).
\end{aligned}}
\]

在模型、权重和 \(\rho\) 固定时，ADMM 迭代只改变这些线性项，二次矩阵保持不变。这是预计算的基础。

## 4. LQR 子问题的完整动态规划推导

本节展开原文引用的 Riccati 递推，特别保留前述对话中“4.4”的所有关键细节。

### 4.1 价值函数与 Bellman 方程

设下一时刻价值函数：

\[
V_{k+1}(x_{k+1})=\frac12x_{k+1}^\top P_{k+1}x_{k+1}
+p_{k+1}^\top x_{k+1}+c_{k+1}.
\]

终端条件：

\[
P_N=\tilde Q_f,\qquad p_N=\tilde q_f.
\]

希望求出：

\[
V_k(x_k)=\frac12x_k^\top P_kx_k+p_k^\top x_k+c_k.
\]

暂记 \(x=x_k,u=u_k,P=P_{k+1},p=p_{k+1},q=\tilde q_k,r=\tilde r_k\)。Bellman 方程为：

\[
\begin{aligned}
V_k(x)=\min_u\Big[&\frac12x^\top\tilde Qx+q^\top x
+\frac12u^\top\tilde Ru+r^\top u\\
&+\frac12(Ax+Bu)^\top P(Ax+Bu)
+p^\top(Ax+Bu)+c_{k+1}\Big].
\end{aligned}
\]

### 4.2 展开动力学带来的代价

因为 \(P=P^\top\)：

\[
\begin{aligned}
\frac12(Ax+Bu)^\top P(Ax+Bu)
={}&\frac12x^\top A^\top PAx
+u^\top B^\top PAx
+\frac12u^\top B^\top PBu.
\end{aligned}
\]

这里原先的两个交叉项相等：

\[
x^\top A^\top PBu=u^\top B^\top PAx.
\]

一次项为：

\[
p^\top(Ax+Bu)=(A^\top p)^\top x+(B^\top p)^\top u.
\]

定义：

\[
H=\tilde R+B^\top PB,\quad G=B^\top PA,\quad h=B^\top p+r.
\]

得到：

\[
\begin{aligned}
V_k(x)={}&\frac12x^\top(\tilde Q+A^\top PA)x
+(q+A^\top p)^\top x+c_{k+1}\\
&+\min_u\left[\frac12u^\top Hu+u^\top(Gx+h)\right].
\end{aligned}
\]

其中 \(H\in\mathbb R^{m\times m},G\in\mathbb R^{m\times n},h\in\mathbb R^m\)。由于 \(\tilde R\succ0,P\succeq0\)，有 \(H\succ0\)。

### 4.3 求最优输入与最小代价

记 \(b=Gx+h\)，配方：

\[
\frac12u^\top Hu+u^\top b
=\frac12(u+H^{-1}b)^\top H(u+H^{-1}b)
-\frac12b^\top H^{-1}b.
\]

第一个二次型最小为零，于是：

\[
u^*=-H^{-1}b=-H^{-1}Gx-H^{-1}h.
\]

定义：

\[
\boxed{
\begin{aligned}
K_k&=(\tilde R+B^\top P_{k+1}B)^{-1}B^\top P_{k+1}A,\\
d_k&=(\tilde R+B^\top P_{k+1}B)^{-1}(B^\top p_{k+1}+\tilde r_k).
\end{aligned}}
\]

所以：

\[
\boxed{u_k^*=-K_kx_k-d_k}.
\]

相应的最小代价修正为：

\[
\min_u\left[\frac12u^\top Hu+u^\top(Gx+h)\right]
=-\frac12(Gx+h)^\top H^{-1}(Gx+h).
\]

### 4.4 将最优输入代回：逐项推导 \(P_k,p_k,c_k\)

代回后：

\[
\begin{aligned}
V_k(x)={}&\frac12x^\top(\tilde Q+A^\top PA)x
+(q+A^\top p)^\top x+c_{k+1}\\
&-\frac12(Gx+h)^\top H^{-1}(Gx+h).
\end{aligned}
\]

完整展开最后一项：

\[
\begin{aligned}
&-\frac12(Gx+h)^\top H^{-1}(Gx+h)\\
={}&-\frac12x^\top G^\top H^{-1}Gx
-\frac12x^\top G^\top H^{-1}h\\
&-\frac12h^\top H^{-1}Gx
-\frac12h^\top H^{-1}h.
\end{aligned}
\]

由于 \(H^{-1}\) 对称，两个中间项相等，合并得：

\[
-\frac12x^\top G^\top H^{-1}Gx
-x^\top G^\top H^{-1}h
-\frac12h^\top H^{-1}h.
\]

**注意：交叉项合并后，一次项前不再有 \(1/2\)。**

因此：

\[
\begin{aligned}
V_k(x)={}&\frac12x^\top
\left[\tilde Q+A^\top PA-G^\top H^{-1}G\right]x\\
&+\left[q+A^\top p-G^\top H^{-1}h\right]^\top x\\
&+c_{k+1}-\frac12h^\top H^{-1}h.
\end{aligned}
\]

与价值函数标准形式比较，得到以下三条递推。

#### 4.4.1 二次项递推

\[
P_k=\tilde Q+A^\top PA-G^\top H^{-1}G.
\]

恢复下标：

\[
\boxed{
\begin{aligned}
P_k={}&\tilde Q+A^\top P_{k+1}A\\
&-A^\top P_{k+1}B
(\tilde R+B^\top P_{k+1}B)^{-1}B^\top P_{k+1}A.
\end{aligned}}
\]

这就是离散 Riccati 递推。

#### 4.4.2 一次项递推

\[
p_k=q+A^\top p-G^\top H^{-1}h.
\]

因为 \(K_k=H^{-1}G\)，所以 \(G^\top H^{-1}=K_k^\top\)，从而：

\[
\begin{aligned}
p_k&=q+A^\top p-K_k^\top(B^\top p+r)\\
&=q+(A^\top-K_k^\top B^\top)p-K_k^\top r.
\end{aligned}
\]

得到：

\[
\boxed{p_k=\tilde q_k+(A-BK_k)^\top p_{k+1}-K_k^\top\tilde r_k}.
\]

#### 4.4.3 常数项递推

\[
\boxed{c_k=c_{k+1}-\frac12h^\top H^{-1}h}.
\]

由于常数项不影响后续的最优控制输入，若不需要最优代价值，可以不存储、不递推 \(c_k\)。前面若省略了轨迹跟踪展开产生的常数，这里的价值函数也相应差一个与决策无关的量，但最优策略不变。

### 4.5 与论文的 \(P_k\) 写法对应

论文前文使用：

\[
\boxed{P_k=\tilde Q+K_k^\top\tilde RK_k
+(A-BK_k)^\top P_{k+1}(A-BK_k)}.
\]

这是直接代入 \(u=-Kx-d\) 后收集二次项得到的。下一状态为：

\[
x_{k+1}=(A-BK)x-Bd.
\]

三个来源分别为当前状态代价、当前输入代价和未来价值函数。展开上述矩阵：

\[
\begin{aligned}
&\tilde Q+K^\top\tilde RK+(A-BK)^\top P(A-BK)\\
={}&\tilde Q+A^\top PA-A^\top PBK-K^\top B^\top PA
+K^\top(\tilde R+B^\top PB)K.
\end{aligned}
\]

利用 \(HK=B^\top PA\)，后两项抵消，得到：

\[
\tilde Q+A^\top PA-A^\top PBK
=\tilde Q+A^\top PA-A^\top PBH^{-1}B^\top PA.
\]

与 Riccati 形式完全一致。

### 4.6 与论文的 \(p_k\) 写法对应：为什么 \(d_k\) 消失？

论文前文形式为：

\[
\begin{aligned}
p_k={}&\tilde q_k+(A-BK_k)^\top(p_{k+1}-P_{k+1}Bd_k)\\
&+K_k^\top(\tilde Rd_k-\tilde r_k).
\end{aligned}
\]

令 \(F=A-BK\)。直接代入 \(u=-Kx-d,x_{k+1}=Fx-Bd\)，各项对一次项系数的贡献为：

| 代价项 | 对一次项系数的贡献 |
|---|---|
| \(q^\top x\) | \(q\) |
| \(\frac12u^\top\tilde Ru\) | \(K^\top\tilde Rd\) |
| \(r^\top u\) | \(-K^\top r\) |
| \(\frac12x_{k+1}^\top Px_{k+1}\) | \(-F^\top PBd\) |
| \(p^\top x_{k+1}\) | \(F^\top p\) |

相加即：

\[
p_k=q+F^\top p-K^\top r+(K^\top\tilde R-F^\top PB)d.
\]

计算 \(d\) 的系数：

\[
\begin{aligned}
K^\top\tilde R-F^\top PB
&=K^\top\tilde R-(A-BK)^\top PB\\
&=K^\top(\tilde R+B^\top PB)-A^\top PB\\
&=K^\top H-A^\top PB.
\end{aligned}
\]

由 \(HK=B^\top PA\) 取转置，得：

\[
K^\top H=A^\top PB.
\]

所以该系数为零，最终：

\[
p_k=q+F^\top p-K^\top r.
\]

**\(d_k\) 不是被忽略，而是利用最优性条件后，显式含 \(d_k\) 的项恰好抵消。** 线性项信息仍通过 \(p_{k+1}\) 和 \(\tilde r_k\) 影响递推。

### 4.7 标量例子：检查所有系数

取：

\[
A=B=1,\quad\tilde Q=\tilde R=1,\quad P_{k+1}=2,\quad p_{k+1}=3,
\quad\tilde q_k=4,\quad\tilde r_k=5,\quad c_{k+1}=0.
\]

则：

\[
V_k(x)=\min_u\left[
\frac12x^2+4x+\frac12u^2+5u+(x+u)^2+3(x+u)
\right].
\]

整理：

\[
V_k(x)=\frac32x^2+7x+\min_u\left[\frac32u^2+(2x+8)u\right].
\]

最优输入与最小代价：

\[
u^*=-\frac{2x+8}{3},\qquad
V_k(x)=\frac32x^2+7x-\frac{(2x+8)^2}{6}.
\]

展开得：

\[
\boxed{V_k(x)=\frac56x^2+\frac53x-\frac{32}{3}}.
\]

注意价值函数是 \(\frac12P_kx^2+p_kx+c_k\)，因此：

\[
P_k=\frac53,\qquad p_k=\frac53,\qquad c_k=-\frac{32}{3}.
\]

使用递推核对：

\[
H=3,\quad K=\frac23,\quad d=\frac83,
\]

\[
P_k=1+2-\frac{2^2}{3}=\frac53,
\]

\[
p_k=4+\left(1-\frac23\right)3-\frac23\cdot5=\frac53.
\]

两种计算一致。该例说明：展开后 \(x^2\) 的系数不是 \(P_k\)，而是 \(P_k/2\)。

## 5. 复制变量、对偶变量与收敛

### 5.1 约束投影：论文式（17）

完成 \(x,u\) 更新后，状态复制变量求解：

\[
z_k^{j+1}=\arg\min_{z_k}\left[
I_{\mathcal X}(z_k)+(\lambda_k^j)^\top(x_k^{j+1}-z_k)
+\frac\rho2\|x_k^{j+1}-z_k\|^2\right].
\]

用 \(y_k^j=\lambda_k^j/\rho\) 配方：

\[
(\lambda_k^j)^\top(x_k^{j+1}-z_k)+\frac\rho2\|x_k^{j+1}-z_k\|^2
=\frac\rho2\|z_k-(x_k^{j+1}+y_k^j)\|^2-\frac\rho2\|y_k^j\|^2.
\]

最后一项与 \(z_k\) 无关，得到欧氏投影：

\[
\boxed{
\begin{aligned}
z_k^{j+1}&=\operatorname{proj}_{\mathcal X}(x_k^{j+1}+y_k^j),\\
w_k^{j+1}&=\operatorname{proj}_{\mathcal U}(u_k^{j+1}+g_k^j).
\end{aligned}}
\]

盒约束下按元素限幅：

\[
[w_k^{j+1}]_i=\min\left([u_{\max}]_i,
\max([u_{\min}]_i,[u_k^{j+1}+g_k^j]_i)\right).
\]

这是优化迭代内部的一步，后续 LQR 更新会响应投影造成的不一致。一般凸集的投影不一定是线性映射，也不一定便宜。

### 5.2 对偶更新：论文式（18）

未缩放形式：

\[
\lambda_k^{j+1}=\lambda_k^j+\rho(x_k^{j+1}-z_k^{j+1}),
\]

\[
\mu_k^{j+1}=\mu_k^j+\rho(u_k^{j+1}-w_k^{j+1}).
\]

固定 \(\rho\) 时除以 \(\rho\)：

\[
\boxed{
\begin{aligned}
y_k^{j+1}&=y_k^j+x_k^{j+1}-z_k^{j+1},\\
g_k^{j+1}&=g_k^j+u_k^{j+1}-w_k^{j+1}.
\end{aligned}}
\]

### 5.3 残差与可行性

将序列堆叠为 \(v=[x;u],a=[z;w]\)。该等式分裂的标准残差可写为：

\[
e_{\mathrm{pri}}^{j+1}=v^{j+1}-a^{j+1},
\qquad e_{\mathrm{dual}}^{j+1}=\rho(a^{j+1}-a^j).
\]

对偶残差整体符号不影响范数判断。论文说明检查两类残差，但此处不将某种具体阈值公式冒充为原文实现。

当残差趋零时，动力学侧和集合约束侧一致。对于可解问题，在适当的凸 ADMM 条件下，精确子问题更新可获得最优解收敛。若使用稳态近似替代不匹配的有限时域子问题，不能不加分析地沿用精确 ADMM 的全部结论。

## 6. 第 III-B 部分：预计算与稳态 Riccati

### 6.1 两层预计算

第一层：固定 \(A,B,Q,R,Q_f,\rho,N\) 时，有限时域的 \(P_k,K_k,H_k^{-1}\) 本就不依赖当前的 ADMM 线性项，可离线计算完整序列，在线只更新 \(p_k,d_k\)。

第二层：TinyMPC 利用稳态解，仅存一套矩阵，进一步减少存储。

### 6.2 稳态代数 Riccati 方程

在相应稳定化条件下，长时域反向递推趋于稳定解：

\[
\begin{aligned}
P_\infty={}&\tilde Q+A^\top P_\infty A\\
&-A^\top P_\infty B(\tilde R+B^\top P_\infty B)^{-1}B^\top P_\infty A.
\end{aligned}
\]

对应：

\[
K_\infty=(\tilde R+B^\top P_\infty B)^{-1}B^\top P_\infty A.
\]

稳定化 Riccati 解的存在并非对任意系统自动成立；例如通常需要可稳定化等条件。这里 \(\tilde Q=Q+\rho I\succ0\)，但仍需关注动力学的可稳定化性。

### 6.3 缓存：论文式（19）

\[
\boxed{C_1=(\tilde R+B^\top P_\infty B)^{-1},\qquad
C_2=(A-BK_\infty)^\top}.
\]

论文 III-B 的公式省略了部分波浪号。本文保留波浪号，强调用于当前原始变量子问题的是修改后的代价。

### 6.4 在线递推：论文式（20）

将 \(P_{k+1},K_k\) 替换为稳态矩阵：

\[
\boxed{
\begin{aligned}
d_k&=C_1(B^\top p_{k+1}+\tilde r_k),\\
p_k&=\tilde q_k+C_2p_{k+1}-K_\infty^\top\tilde r_k.
\end{aligned}}
\]

然后从当前状态正向展开：

\[
u_k=-K_\infty x_k-d_k,\qquad x_{k+1}=Ax_k+Bu_k.
\]

在线主要为矩阵乘向量，不再逐次求解 Riccati 矩阵。所谓避免求逆、分解，是将相应工作放到离线；不是说数学表达式中不出现逆矩阵。

### 6.5 稳态替代何时精确？

一般有限时域满足 \(P_N=\tilde Q_f\)，未必等于 \(P_\infty\)。长时域使远离终端的递推趋近稳态，但不会自动让终端附近也完全相等。

如果选取：

\[
\boxed{\tilde Q_f=P_\infty},
\]

由于 \(P_\infty\) 是 Riccati 映射的不动点，则所有阶段都有：

\[
P_k=P_\infty,\qquad K_k=K_\infty.
\]

此时统一矩阵与该有限时域子问题精确一致。一般终端权重不匹配时，统一使用稳态矩阵涉及近似；论文 III-B 没有展开其误差界。

对于采用 \(\tilde Q_f=Q_f+\rho I\) 的分裂，还要注意匹配条件针对的是**修改后的终端 Hessian**。改变 \(\rho\) 后，固定的 \(Q_f\) 不会自动对每组稳态缓存都满足匹配条件。

## 7. 第 III-C 部分：惩罚参数切换

### 7.1 为什么调节 \(\rho\)？

ADMM 对问题尺度和 \(\rho\) 敏感。较小 \(\rho\) 可能使一致性推进较慢，较大 \(\rho\) 则可能过度强调与当前复制变量接近；并非越大越好。

改变 \(\rho\) 会改变：

\[
\tilde Q=Q+\rho I,\qquad\tilde R=R+\rho I,
\]

进而改变 \(P_\infty,K_\infty,C_1,C_2\)。在线重新计算会削弱预计算优势。

### 7.2 论文方案：多组缓存

预先选择 \(\rho_1,\ldots,\rho_L\)，为每个值准备一组矩阵。在线根据原始、对偶残差切换缓存，以存储换取在线计算速度。

论文说明采用改编自 OSQP 的残差驱动方案，但 III-C 没有给出完整候选值、切换阈值和公式，无法从该节恢复唯一实现。

常见残差平衡直觉是：原始残差明显偏大时倾向增大 \(\rho\)，对偶残差明显偏大时倾向减小。这里是动机解释，不是原文完整参数规则。

### 7.3 缩放对偶变量的一致性

若切换 \(\rho\) 时保持未缩放乘子不变，由定义应调整：

\[
\boxed{y_{\mathrm{new}}=\frac{\rho_{\mathrm{old}}}{\rho_{\mathrm{new}}}y_{\mathrm{old}},\qquad
g_{\mathrm{new}}=\frac{\rho_{\mathrm{old}}}{\rho_{\mathrm{new}}}g_{\mathrm{old}}}.
\]

同时切换缓存并重算修改后的线性项。这是由变量定义推得的实现细节，原文没有单独展开。

## 8. 完整算法与计算复杂度

### 8.1 算法流程

```text
Offline:
    Discretize the dynamics and choose weights and horizon
    For each candidate rho:
        Compute modified weights and steady-state Riccati solution
        Cache Kinf, C1, C2

Each control cycle:
    Read the estimated state
    Update reference costs and constraint sets
    Initialize or warm-start z, w, y, g

    Repeat:
        Build modified linear costs
        Set p[N] = modified terminal linear cost

        For k = N-1 down to 1:
            d[k] = C1 * (B^T * p[k+1] + modified_r[k])
            p[k] = modified_q[k] + C2 * p[k+1] - Kinf^T * modified_r[k]

        Set x[1] = measured state
        For k = 1 to N-1:
            u[k] = -Kinf * x[k] - d[k]
            x[k+1] = A * x[k] + B * u[k]

        Project x+y and u+g onto the constraint sets
        Update scaled dual variables
        Check residuals
        If rho changes, switch caches and update scaled variables

    Apply the first control input
```

热启动通常利用上一周期的解；具体移位、终端补齐和对偶变量处理属于实现选择，不能仅凭论文伪代码确定。

### 8.2 控制周期与求解迭代不同

- 控制周期：一次采样、优化、执行的物理周期。
- ADMM 迭代：一个控制周期内的求解器循环。
- 预测时间步：描述未来状态序列，不是 ADMM 迭代次数。

控制频率也不必等于预测模型离散频率，若要换算预测时间长度，需核对具体实现的模型时间步。

### 8.3 复杂度分析（补充）

对稠密矩阵、简单投影，一次迭代主要量级为：

\[
O\!\left(N(n^2+nm+m^2)\right).
\]

固定维数时随时域线性增长。缓存矩阵不必逐时刻重复存储；状态、输入、复制变量、对偶变量等向量序列仍占 \(O(N(n+m))\) 量级存储。若存储多组 \(\rho\) 缓存，还要增加与候选组数有关的矩阵存储。

该分析描述数学工作区的量级，不等于编译产物或整机 RAM 的实际字节数。

## 9. 论文实验与结果解读

### 9.1 微控制器基准测试

平台：Teensy 4.1，Cortex-M7，600 MHz，论文列出 7.75 MB Flash、512 kB 紧耦合 RAM 的测试配置/限制。TinyMPC 使用 C++ 和 Eigen；OSQP 使用代码生成的 C 实现。

原文设置：目标容差 \(10^{-3}\)，约束容差 \(10^{-4}\)，最大迭代次数 4000；两者均热启动，OSQP 关闭 solution polishing，其他参数尽量对齐。

测试通过改变状态维数、输入维数和预测长度比较随机可控线性系统上的跟踪问题，并加入状态估计噪声。

| 项目 | 论文报告 |
|---|---|
| 最大加速 | 约 8.85 倍 |
| 内存示例 | 固定 \(m=4,N=10\) 时，OSQP 在 \(n=16\) 超出论文 512 kB 限制；TinyMPC 在 \(n=32\) 约占 400 kB |
| 最大测试实例 | 696 个决策变量、490 个线性等式、392 个线性不等式 |

**图 3 的时间指标是每次迭代平均耗时。** 不能把 8.85 倍直接推广为所有问题的端到端求解加速；总耗时还取决于迭代次数等因素。内存数字也应按论文测量口径理解，不直接当作任意移植版本的工作区需求。

### 9.2 Crazyflie 实机平台

| 项目 | 配置 |
|---|---|
| 平台 | 27 g Crazyflie 2.1 |
| MCU | STM32F405，Cortex-M4，168 MHz |
| SRAM / Flash | 192 kB / 1 MB |
| 状态 / 输入维数 | \(n=12,m=4\) |
| 模型 | 悬停点附近的六自由度动力学线性化，采用论文引用的四元数姿态表述 |
| 输入 | 电机 PWM 指令 |
| 外部定位 | OptiTrack，以 100 Hz 发送位姿 |

论文称对应 OSQP 实例无法装入该 MCU 可用内存，因此飞行实验比较 Crazyflie 固件中的 PID、Mellinger、INDI、Brescianini 等控制器。不能将这一结果理解为 OSQP 在 F405 上任何规模的问题都无法运行。

### 9.3 八字轨迹跟踪

- 控制频率 500 Hz，\(N=15\)。
- 较快轨迹周期 3 s，最大速度约 1.5 m/s，姿态偏离悬停约 20°。
- 论文报告该跟踪实验每步最多 7 次迭代，满足 2 ms 求解预算。
- 高速工况下 TinyMPC 完成跟踪，对比的 Mellinger、Brescianini 控制器发生坠机。

这些结论针对论文的模型、参数和轨迹，不构成对所有控制场景的排名，也不意味着任意问题 7 次迭代就足够。

### 9.4 90° 姿态误差恢复

| 控制器 | 论文报告的最大位置误差 |
|---|---:|
| TinyMPC | 约 23 cm |
| PID | 约 40 cm |
| Brescianini | 约 65 cm |

论文报告 INDI、Mellinger 在该实验中未能稳定飞机。图 5 展示限幅前 PWM 指令，传统控制器存在超出执行器范围的请求；这不代表电机实际执行了超过硬件上限的 PWM。

### 9.5 动态避障

- 控制频率降为 100 Hz，预测长度增为 \(N=20\)。
- 无线发送移动棍端点位置，在线重新线性化球形障碍约束。
- 最大问题为 316 个决策变量、248 个线性等式、172 个线性不等式。
- 额外要求留在 \(x=0\) 垂直平面时，论文报告最大偏离约 5 cm。

作者明确指出约束在激活与不激活之间切换时需要更多迭代。物理系统偏离约束平面也提醒我们：数值优化约束不能直接等同于真实系统零误差的约束保证。

## 10. 理论边界与常见误解

### 10.1 不是任意非线性 MPC 的直接解法

核心针对线性动力学、凸二次目标和凸约束。非线性系统通常需要线性化或额外外层算法。90° 恢复实验展示了特定条件下的实际能力，不等于任意大姿态、任意工况下的全局保证。

### 10.2 避障本身通常非凸

球形障碍外部约束：

\[
\|p-p_{\mathrm{obs}}\|\ge r
\]

一般为非凸约束。在线线性化得到局部凸约束，不等价于求解任意环境的全局路径规划；规划方向、可行性和线性化点选择仍重要。

### 10.3 未收敛时两组变量各自可行，不代表原问题可行

ADMM 中 \(x,u\) 满足动力学，\(z,w\) 满足集合约束；在它们尚未一致时，不能同时获得原问题的全部可行性。最终输入限幅只能保证对应输入边界，不能自动保证未来状态约束。

### 10.4 凸性不等于递归可行性和闭环稳定性

求解器收敛、当前问题可行、下一周期仍可行、真实闭环稳定，是不同问题。需要结合模型误差、终端设计、扰动、约束和求解精度分析。本文不将一般凸优化性质解释为完整控制安全证明。

### 10.5 预计算依赖不变结构

频繁修改 \(A,B,Q,R,\rho\) 可能需要重算或切换缓存。TinyMPC 尤其适合模型和权重相对固定、参考与简单约束经常变化的问题。

### 10.6 便宜的投影不是所有约束的共同性质

盒约束投影很便宜。一般凸集的投影可能本身需要求解子问题，降低分裂方法优势。跨时刻耦合约束也可能破坏逐时间步独立投影的简单结构。

### 10.7 机载优化不等于完全独立自主感知

论文优化在机载 MCU 完成，但实验依赖外部动作捕捉和障碍位置传输。证明的是机载实时优化能力，不是完整的无外部定位自主系统。

## 11. 公式速查与推荐阅读顺序

### 11.1 核心公式速查

| 论文位置 | 核心内容 |
|---|---|
| 式（13） | 引入 \(z,w\) 后的增广拉格朗日函数 |
| 式（14）—（15） | 原始变量更新变成修改代价后的 LQR |
| 式（16） | \(\tilde q=q+\rho(y-z),\ \tilde r=r+\rho(g-w)\) |
| 式（17） | \(z^+=\operatorname{proj}_{\mathcal X}(x^++y),\ w^+=\operatorname{proj}_{\mathcal U}(u^++g)\) |
| 式（18） | 缩放对偶变量累积一致性残差 |
| 式（19） | 缓存 \(C_1,C_2\) 与稳态增益 |
| 式（20） | 在线反向向量递推 \(d_k,p_k\) |

最重要的两组连接是：

\[
\boxed{\tilde q_k=q_k+\rho(y_k-z_k),\quad
\tilde r_k=r_k+\rho(g_k-w_k)}
\]

以及：

\[
\boxed{
\begin{aligned}
d_k&=C_1(B^\top p_{k+1}+\tilde r_k),\\
p_k&=\tilde q_k+C_2p_{k+1}-K_\infty^\top\tilde r_k.
\end{aligned}}
\]

第一组将约束协调信息传入 LQR；第二组说明缓存矩阵后如何只通过向量递推响应这些变化。

### 11.2 推荐阅读顺序

1. 理解 MPC 的滚动求解和只执行首个输入。
2. 掌握价值函数二次形式、配方和 Riccati 递推，重点看本文第 4 节。
3. 对照原文 III-A 式（13）—（18），理解分裂后的三步更新。
4. 精读 III-B 式（19）—（20），区分精确有限时域缓存和稳态替代。
5. 阅读 III-C，理解 \(\rho\) 与矩阵缓存的关联。
6. 用图 3 的每次迭代指标、飞行实验的完整周期预算分别评估性能。

TinyMPC 的整体逻辑是：**MPC 定义优化目标；ADMM 分离集合约束；Riccati 利用动力学结构；预计算降低在线矩阵计算和存储开销。** 是否能在具体 MCU 上稳定、实时地工作，还取决于模型、维数、约束类型、终端设计、求解精度和调度预算。
