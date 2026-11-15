# 打符跟踪：从图像到空间，再投影回图像

依据本工程 2026-10-29 的实际源码编写。本文面向第一次接触 PnP、旋转矩阵和 EKF 的读者。先读第1～8节打通坐标链，再读滤波与预测。公式描述当前实现，不代表实际相机和机械标定已经验收。

## 1. 先知道整个程序在做什么

相机提供二维点；PnP结合已知物理尺寸求初始空间位姿；跟踪器保持空间状态，并将它投影回图像，与新检测比较、校正。不能仅凭一个像素点直接得到三维距离。

```mermaid
flowchart LR
    A[原图像素 R与四端点] --> B[PnP 初始位姿]
    B --> C[OpenCV相机系]
    C --> D[RMCS相机系]
    D --> E[手眼与曝光时刻IMU]
    E --> F[世界系状态与EKF]
    F --> G[五个板位的空间位置]
    G --> H[逆外参转回相机]
    H --> I[内参和畸变投影到原图]
    I --> J[关联并校正]
    J --> F
```

| 源码 | 阅读时关注什么 |
|---|---|
| [pose/rune_camera.cpp](pose/rune_camera.cpp) | 位姿模块内部读取相机标定、IMU基变换、动态手眼外参 |
| [track/buff_tracker.cpp](track/buff_tracker.cpp) | 观测组织、模型生命周期及位姿模块调用 |
| [pose/rune_pose.cpp](pose/rune_pose.cpp) | R与种子板选择、PnP、几何筛选、初始角度 |
| [pose/rune_projection.cpp](pose/rune_projection.cpp) | 世界特征→相机→像素的实际观测函数 |
| [track/rune_model.cpp](track/rune_model.cpp) | 状态、EKF预测、像素关联与校正 |
| [predict/rune_state.cpp](predict/rune_state.cpp) | 恒速/正弦积分、未来点、空间瞄准点 |
| [buff_state.hpp](buff_state.hpp) | 公共状态及单位 |
| [conversion.hpp](../../rm_utils/algorithm/conversion.hpp) | OpenCV与RMCS换轴 |
| [orientation.hpp](../../rm_utils/algorithm/orientation.hpp) | IMU双侧基变换及四元数插值 |
| [pose/rune_pnp.cpp](pose/rune_pnp.cpp) | PnP点序、求解、结果换轴 |

**先分清三个“中心”：**机构中心是五块符板旋转所绕的中心；R标的三维位置沿法线突出0.1m，并不恰在该中心；每块符板还有自己的靶心。`state.x/y/z`表示机构中心，不是R标中心，也不是某一块符板中心。

## 2. 符号与最少的数学基础

本书统一用列向量：

$$
p_A=R_{A\leftarrow B}p_B+t_{A\leftarrow B}.
$$

读法：把“B坐标系表示的同一个点”转换成“A坐标系表示”。旋转矩阵是3×3，点和平移是3×1；平移是B原点在A系的位置。这里是描述坐标的变化，不是命令机器人真的移动。

- 旋转只改变朝向；平移改变原点。方向向量只乘旋转，空间点还要加平移。
- 连乘从右往左执行：`R_AB * R_BC * p_C`先从C到B，再从B到A，顺序不能交换。
- 正交旋转矩阵满足 $R^{-1}=R^T$、$R^TR=I$、$\det R=1$。
- 长度用m，像素用px，角度用rad，时间用s；$180^\circ=\pi\,rad$。配置中少数门限以degree存储，源码会换算。

若需要连同平移一起写：

$$
T_{A\leftarrow B}=\begin{bmatrix}R_{A\leftarrow B}&t_{A\leftarrow B}\\0&1\end{bmatrix},\quad
\begin{bmatrix}p_A\\1\end{bmatrix}=T_{A\leftarrow B}\begin{bmatrix}p_B\\1\end{bmatrix}.
$$

逆变换**先减平移再逆旋转**：

$$p_B=R_{A\leftarrow B}^T(p_A-t_{A\leftarrow B}).$$

例如相机原点在世界系的 $(1,0,0)$，且无旋转，世界点 $(4,0,0)$在相机里就是$(3,0,0)$，不是$(5,0,0)$。

## 3. 本项目究竟有哪几个坐标系

| 记号 | 名称 | 轴/原点约定 |
|---|---|---|
| I | 原图像素系 | 左上为原点，u向右、v向下；只有二维位置 |
| O | OpenCV光学相机系 | 原点在光心，X向右、Y向下、Z向前 |
| C | RMCS内部相机系 | 同一光心，x向前、y向左、z向上 |
| G | 云台参考系 | 手眼标定目标系，采用项目约定的前/左/上轴 |
| B | IMU本体基 | 串口姿态采用的基，借助 `R_gimbal2imubody` 与G基联系 |
| W | 跟踪世界/odom系 | 姿态经基变换后的参考系；不是地理坐标，也没有底盘平移里程计 |
| S | 符机构局部系 | 机构中心为原点，x指背面，y/z位于符面；初始种子板在+z侧 |

C与O只是换轴，**光心相同，没有平移**。G原点与光心不重合，才需要手眼平移。W的轴随配置和IMU参考定义，并不保证“x永远朝目标”。

### 3.1 最容易踩坑的实际边界

当前动态外参只包含“云台旋转＋相机相对云台的固定偏移”。代码没有接收机器人在场地中的位置，因此不能声称已消除底盘平移。可把它理解为有世界参考朝向、但平移原点按当前云台原点建模的坐标表达；机器人平移需要由目标状态重新估计，或后续接入里程计。

## 4. 第一步：像素和相机内参

相机内参为：

$$K=\begin{bmatrix}f_x&0&c_x\\0&f_y&c_y\\0&0&1\end{bmatrix}.$$

$f_x,f_y$单位是像素，$(c_x,c_y)$是主点，不一定刚好在图像正中心。无畸变针孔模型：

$$u=f_x X_O/Z_O+c_x,\qquad v=f_y Y_O/Z_O+c_y.$$

反过来，已去畸变的像素只能确定一条射线：

$$p_O=Z_O\begin{bmatrix}(u-c_x)/f_x\\(v-c_y)/f_y\\1\end{bmatrix}.$$

**这里的 $Z_O$ 未知。** PnP通过多个像素与已知物理点的对应关系恢复尺度；上式不能直接对带畸变的原始像素使用。

### 4.1 五参数畸变

本项目读取 $[k_1,k_2,p_1,p_2,k_3]$。先令$x_n=X_O/Z_O$、$y_n=Y_O/Z_O$、$r^2=x_n^2+y_n^2$，则：

$$L=1+k_1r^2+k_2r^4+k_3r^6,$$
$$x_d=x_nL+2p_1x_ny_n+p_2(r^2+2x_n^2),$$
$$y_d=y_nL+p_1(r^2+2y_n^2)+2p_2x_ny_n,$$
$$u=f_xx_d+c_x,\qquad v=f_yy_d+c_y.$$

前两行在归一化平面中计算，最后才转像素。源码交给OpenCV `projectPoints`处理，无需自己手写一份替代。

模型输入的640×480不是相机标定尺寸。检测点必须先撤销letterbox回原图，再用原图内参。若真的把原图按$s_x,s_y$缩放，则$f_x,c_x$乘$s_x$、$f_y,c_y$乘$s_y$；裁剪还要调整主点。当前tracker会检查输入尺寸与标定尺寸一致。

## 5. 第二步：PnP如何得到初始空间位姿

### 5.1 先摆好已知物理点

[buff_types.hpp](../buff_types.hpp)使用半径 $r=0.7m$、靶面端点半径 $a=0.15m$、R突出量 $d=0.1m$：

| PnP顺序 | S系坐标(m) |
|---|---|
| R图标 | $(-d,0,0)$ |
| T外端 | $(0,0,r+a)$ |
| L侧端 | $(0,a,r)$ |
| B内端 | $(0,0,r-a)$ |
| R侧端 | $(0,-a,r)$ |

表内“R图标”和“R侧端”是两个东西。图像中的五个对应点按`icon,T,L,B,R`送入PnP。深大原始网络顺序是`top,left,机构R,right,bottom`，不会原封不动当作该PnP顺序。

当前PnP根据离R最远/最近找T/B，再用侧向投影区分L/R；不使用检测框四角。测量端点要对应这里的真实物理尺寸，换灯排或换机械点后必须重新确认尺寸，不能只改标签。

### 5.2 PnP求的是“符局部→相机”

每个物理点先从S的RMCS轴转换为OpenCV式的局部轴（下一节给换轴矩阵），再与原图像素配对：

$$p_{O,i}=R_{O\leftarrow S_O}p_{S_O,i}+t_{O\leftarrow S_O},\qquad
z_i=\pi(K,D,p_{O,i}).$$

$\pi$表示投影函数，$D$表示畸变。求解器寻找使投影贴近观测的R和t；可以用下式理解误差目标，不代表EPNP本身按此逐步迭代：

$$\min_{R,t}\sum_i\|z_i-\pi(K,D,Rp_{S_O,i}+t)\|^2.$$

源码先EPNP，再ITERATIVE细化，`Rodrigues(rvec)`将旋转向量转成矩阵。**t是机构原点在相机系的位置，不是“相机在机构系的位置”。**

PnP之后还要检查深度、五点重投影、符面倾角、法线朝向和其他待击板布局；`solvePnP`返回true不等于空间结果一定正确。当前初始化沿用1～2块待击板的限制，已击打圆环不能单独初始化。

## 6. 第三步：OpenCV与RMCS换轴

源码常量为：

$$A=\begin{bmatrix}0&0&1\\-1&0&0\\0&-1&0\end{bmatrix}.$$

于是：

$$p_C=Ap_O=(Z_O,-X_O,-Y_O)^T,$$
$$p_O=A^Tp_C=(-y_C,-z_C,x_C)^T.$$

例如光学系$(0.2,-0.1,3)$表示右0.2m、上0.1m、前3m；内部相机系变为$(3,-0.2,0.1)$。

PnP的输入物体轴和输出相机轴都采用OpenCV式，转换旋转要两边一起换：

$$R_{C\leftarrow S}=A R_{O\leftarrow S_O} A^T,\qquad
 t_{C\leftarrow S}=A t_{O\leftarrow S_O}.$$

只有向量换轴才是左乘A；旋转算子的输入与输出基都变了，所以是$A R A^T$。这就是`opencv2ros_rotation`与`opencv2ros_position`写法不同的原因。

## 7. 第四步：手眼标定＋IMU，得到相机到世界

### 7.1 手眼配置的含义

记YAML里的：

$$H=R_{G\leftarrow O}=\texttt{R\_camera2gimbal},\qquad
 t_{GO}=\texttt{t\_camera2gimbal}.$$

H接收OpenCV光学系向量，输出云台系向量；平移单位m，表示光心在云台系的位置。它们不是云台到相机的逆外参。YAML的9个数字按行排列。

由于程序内部已经使用C系，需要：

$$R_{G\leftarrow C}=H A^T.$$

平移原点没有换，不需要再乘A；`t_camera2gimbal`本来就在G系。

### 7.2 IMU使用的是双侧基变换

令$B$为配置`R_gimbal2imubody`，$Q(t)$为串口姿态四元数对应的旋转矩阵。当前`gimbal_orientation`实际执行：

$$R_{W\leftarrow G}(t)=B^T Q(t) B.$$

四元数形式：

$$q_g=q_B^{-1}\otimes q_{imu}\otimes q_B.$$

这里**不是**简单的$q_{imu}\otimes q_B$：按当前串口约定，姿态的输入、输出参考基一起转换。以后换IMU协议时必须重新核对，不能把此式当成所有IMU通用公式。

四元数补充：单位四元数用共轭等于求逆；Eigen构造函数为`Quaterniond(w,x,y,z)`，但`coeffs()`排列是x/y/z/w；$q$与$-q$表示相同旋转。时间插值前选择最短路径，再用SLERP，不直接逐项线性插值欧拉角。

### 7.3 组合成当前帧外参

$$R_{W\leftarrow C}=R_{W\leftarrow G}H A^T,$$
$$t_{W\leftarrow C}=R_{W\leftarrow G}t_{GO}.$$

因此完整坐标链为：

$$\boxed{p_W=R_{W\leftarrow G}(H A^T p_C+t_{GO})}.$$

从光学系直接出发也可写为：

$$\boxed{p_W=R_{W\leftarrow G}(H p_O+t_{GO})}.$$

对照源码`set_imu_orientation`：先得到gimbal姿态，再旋转手眼的朝向**和平移**。只旋转朝向、不旋转相机偏移，会导致云台转动时目标位置漂移。

通用机器人链应还有云台原点的世界平移$t_{WG}$：$p_W=R_{WG}(Hp_O+t_{GO})+t_{WG}$。**当前实现没有这一项，等价于在本轮建模中取$t_{WG}=0$。**

固定相机离线回放可使用`fixed_gimbal_to_world_quaternion_wxyz`；不采用手眼时，也可直接配置内部C→W的`camera_to_world_*`。两种配置不能混填。离线固定外参不能替代手持/运动相机的逐帧姿态。

### 7.4 用一个数字例子验证顺序

假设$H=A$（理想安装），$t_{GO}=(0.1,0,0)^T$；光学点$(0.2,-0.1,3)^T$先变成云台系$(3.1,-0.2,0.1)^T$。若云台绕世界z轴转$90^\circ$：

$$R_z(90^\circ)=\begin{bmatrix}0&-1&0\\1&0&0\\0&0&1\end{bmatrix},\quad
p_W=(0.2,3.1,0.1)^T.$$

反算时先乘$R_z^T$，再减$(0.1,0,0)$，最后乘$H^T$，应还原$(0.2,-0.1,3)$。这些是教学数字，不是本机标定值。

## 8. 第五步：从PnP位姿得到跟踪状态

设PnP换轴后得到$R_{CS},t_{CS}$，则：

$$c_W=R_{WC}t_{CS}+t_{WC},\qquad R_{WS}=R_{WC}R_{CS}.$$

$c_W$就是`state.x/y/z`。取$R_{WS}$第一列为符面法线$n$：

$$\psi=\operatorname{atan2}(n_y,n_x),\qquad
\beta=\arcsin(|n_z|).$$

$\beta$用于倾角验证；跟踪只保留符面偏航$\psi$，并不维护完整六自由度符面姿态。若实际符面大幅倾斜，不能靠放宽门限把不适用的几何变正确。

先消去偏航$R_{seed}=R_z(-\psi)R_{WS}$，源码取：

$$\theta=\operatorname{atan2}(-R_{seed}[1,2],R_{seed}[2,2]).$$

下标从0开始。它等价于从剩余绕x旋转中提取种子板转角，不能直接拿图像中R→板心的二维角度替代；透视和云台姿态会影响图像角度。

## 9. 第六步：跟踪状态如何生成图像观测

状态顺序固定为：

$$s=(x,y,z,\omega,\theta,\psi)^T.$$

对第$i$块板（$i=0,1,2,3,4$），角度相差$72^\circ$：

$$\alpha_i=\theta+2\pi i/5,$$
$$l_i=(0,-r\sin\alpha_i,r\cos\alpha_i)^T,$$
$$p_{W,i}=c_W+R_z(\psi)l_i.$$

R标对应局部点$l_R=(-d,0,0)^T$，也先绕符面偏航再加机构中心。符面转角$\theta$不改变R位置。

接着逐步逆变换：

$$p_{C,i}=R_{WC}^T(p_{W,i}-t_{WC}),$$
$$p_{O,i}=A^Tp_{C,i},$$
$$\hat z_i=\pi(K,D,p_{O,i}).$$

这是`RuneObservable::update`的完整观测函数$h_i(s)$。内部相机系的“前方”是x正方向，换回OpenCV后才检查Z深度；当前实现要求光学深度大于0.1m。

## 10. 第七步：EKF为何能减少抖动

EKF不把每帧PnP结果直接平均；初始化后主要比较**R和符板中心的二维像素观测**，通过空间模型调整状态。四端点主要服务于初始化和几何检查。

### 10.1 预测：先按运动模型猜下一帧

$$\theta^-_{k}=\theta_{k-1}+\omega_{k-1}\Delta t,$$

其余状态按当前模型保持，$\psi$规范到角度区间。$\Delta t$取相邻曝光/媒体时间，不能拿循环耗时代替。

$$F=I,\quad F_{\theta,\omega}=\Delta t,\qquad P^-=FPF^T+Q.$$

$P$是状态不确定度，$Q$是过程噪声。当前源码直接加所配置Q，未额外按$\Delta t$离散化；解释参数时应以此实现为准。

### 10.2 关联：哪个检测对应哪个板位

$$e_{ij}=z_i-h_j(s^-),\quad S_j=H_jP^-H_j^T+R_{obs},$$
$$d_{ij}^2=e_{ij}^TS_j^{-1}e_{ij}.$$

这叫平方马氏距离：同样的像素偏差，在不确定度大时与不确定度小时意义不同。匈牙利匹配选择对应关系并拒绝超门限候选；R只能匹配R，板心只能匹配五个板位。检测数组下标不等于固定板位编号。

### 10.3 校正：用残差纠正状态

$$K=P^-H^T(HP^-H^T+R_{obs})^{-1},$$
$$s^+=s^-+K(z-h(s^-)),$$
$$P^+=(I-KH)P^-(I-KH)^T+KR_{obs}K^T.$$

这里K是卡尔曼增益，不是第4节的相机内参；实际源码还会对协方差对称化。$R_{obs}$大表示更不相信像素观测，绝不是“越小越精准”。

雅可比$H=\partial h/\partial s$描述“状态变化一点，像素会变多少”。当前代码用针孔投影导数：

$$J_\pi=\begin{bmatrix}f_x/Z&0&-f_xX/Z^2\\0&f_y/Z&-f_yY/Z^2\end{bmatrix},$$
$$H\approx J_\pi A^TR_{WC}^T\frac{\partial p_W}{\partial s}.$$

板位的$\partial l/\partial\theta=(0,-r\cos\alpha,-r\sin\alpha)^T$；中心三维导数为单位阵，角速度列对瞬时投影为0。**均值投影包含畸变，但此雅可比未展开畸变导数，是当前实现的近似。**

### 10.4 大符正弦拟合：RMCS 状态与深大鲁棒方法的衔接

输入仍是 RMCS EKF 校正后的连续转角和曝光时间，不重新计算靶心、不重复换叶补偿。恒速拟合和原来的模型选择规则保持不变；`fit_sine()` 改为参考深大 RP-26Rune 的可分离 LM–IRLS：

$$\theta(t)=A\cos(\Omega\tau)+B\sin(\Omega\tau)+v\tau+C,\qquad \tau=t-t_{ref}.$$

固定频率后，四个线性参数由加权最小二乘求出；内层 IRLS 用 Cauchy 权重降低异常相位影响，外层 LM 优化频率。时间与转角先去中心化，输出再恢复原来的绝对时间和速度相位定义。

- 保留 RMCS 的 6 秒窗口、1.5 秒最短拟合跨度和 3 秒时间权重半衰期；正弦拟合至少需要 6 个样本。
- 频率搜索范围仍为 1.80～2.20 rad/s；内层最多 12 次、外层最多 20 次迭代，不增加新依赖或 YAML 参数。
- 鲁棒尺度由初始残差的 MAD 估计，并在本次频率搜索中固定，避免异常段抬高尺度、不同频率比较不同损失。
- 对外 `cost` 仍为时间加权均方误差，保证与恒速拟合比较时口径一致；内部频率优化使用 Cauchy 损失。
- 电控模式的处理沿用当前实现，本次没有新增按 mode 强制选择恒速或正弦模型的策略。

来源记录见 [SOURCE_SZU.json](../../rm_utils/SOURCE_SZU.json)，许可证见 [LICENSE_SZU](../../rm_utils/LICENSE_SZU)。这次只借鉴拟合方法，没有移入深大的几何重建、相位换叶、弹道或调试框架。

`buff_predict_test` 覆盖顺反转、离散异常点和末端短时偏移、大时间戳、频率边界，以及“像素观测 → RMCS EKF → 正弦拟合 → 未来角度”的完整算法调用。合成测试结果不能替代实机视频的命中率与帧率测试。

## 11. 第八步：未来状态、瞄准角与时间

恒速外推：$\theta(t+\tau)=\theta(t)+\omega\tau$。

正弦角速度$\omega(t)=v+a\sin\phi(t)$、$\phi(t+\tau)=\phi(t)+\Omega\tau$，积分得到：

$$\theta(t+\tau)=\theta(t)+v\tau+\frac{a}{\Omega}[\cos\phi(t)-\cos(\phi(t)+\Omega\tau)].$$

$\Omega$是速度振荡的频率参数，不是机构当前转速$\omega$。源码在$|\Omega|$很小时回退恒速。未来计算只修改状态副本，不推进EKF当前后验。

未来空间点仍按第9节计算。几何朝向可写为：

$$yaw=\operatorname{atan2}(y,x),\quad pitch_{up}=\operatorname{atan2}(z,\sqrt{x^2+y^2}).$$

但**这不是完整火控公式**：shoot还要计入弹道、飞行时间和固定延迟；本工程火控方向重建用$z=-\sin(pitch)$，其pitch正方向与上式抬头为正的几何pitch不同。不得直接用`xyz2ypd`结果替换弹道解算。串口发送目标绝对角，电控再减当前云台角；此处不再重复相减。

### 时间须知

一帧的图像、姿态、检测和模型时间必须绑定同一曝光时刻$t_e$。实机使用串口姿态历史对$t_e$插值，不能取推理结束时的“最新姿态”冒充。火控外推则需要考虑从$t_e$到当前及命中时刻的延迟。相机帧率、模型处理频率、结果延迟是三个不同指标。

## 12. 初学者核对清单与排错

| 现象 | 优先核对 |
|---|---|
| 云台不动时位置合理，转起来漂 | IMU基变换、曝光时间匹配、手眼平移是否随姿态旋转 |
| 左右/上下相反 | OpenCV与RMCS换轴，旋转矩阵方向，有没有重复换轴 |
| 距离差约1000倍 | mm与m是否混用、物理尺寸单位 |
| 图上五点好看但PnP不稳定 | 点位是否对应真实尺寸、R突出量、点序、相机内参与图像尺寸 |
| 偏差随画边增大 | 畸变参数、裁剪/缩放后的主点 |
| 开始几帧正常随后越跟越偏 | 关联错误、错误中心观测、时间戳、模型假设是否适合实际运动 |
| 有框却不初始化 | R/四端点是否完整、PnP及布局门限、仅已击打板不能初始化 |
| 检测暂丢还在显示点 | 可能是预测；不能当成本帧真实检测 |

实用的核对顺序：先静止相机检查重投影→单独绕yaw转动→单独绕pitch转动→加入相机偏移→检查动态曝光姿态。每次只改变一项。

运行现有检查（工程根目录）：

```powershell
.\build\buff_pose_test.exe
.\build\buff_track_test.exe
.\build\buff_predict_test.exe
```

测试使用已知合成空间点验证PnP、动态手眼/IMU、跟踪及预测。通过测试不表示实机外参、机械尺寸或串口姿态方向已自动验证。`initialized`也不等于`ready`或允许发射；当前瞄准点还有恒速3s/正弦6s预热及待击板有效期约束。仅R或圆环校正不会无限刷新待击板丢失计时。

## 13. 延伸阅读

[OpenCV相机标定与三维重建](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)介绍针孔、畸变与PnP。本文的坐标链、数值例子及工程边界按本项目源码推导，没有把“教材常见坐标轴”直接当作本项目约定。
