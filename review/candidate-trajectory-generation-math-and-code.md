# JevPilot 候选轨迹集合：数学描述与源码对应

本文描述 JevPilot 如何从当前车辆、道路、交通和规则状态生成局部候选轨迹集合。组织顺序是：先给出数学模型，再把每一步对应到当前源码。

需要先区分三个集合：

- **原始采样池**：正常道路模式通常生成 55 个 maneuver；
- **内部规划集合**：为了排序、可视化和诊断，保留约 11 个移动 maneuver，并按需加入 1 个零速 maneuver，设计上最多 12 个；
- **Jev 可选集合**：再次应用道路、车道、行驶方向、紧迫碰撞和排队规则，只把合法移动候选的摘要发送给 Jev。完整轨迹点不发送给 Jev。

因此，源码中的 `plan.projections`、`plan.vectors`、`plan.eligible` 和 Jev 请求中的 `state.candidates` 不是同一个集合。

## 1. 符号与状态

令当前自车状态为

\[
q_0=(x_0,z_0,\theta_0,v_0,\delta_0),
\]

其中：

- \(x\) 为向东坐标；
- \(z\) 为向南坐标；
- \(\theta=0\) 表示朝北；
- \(v\) 为有符号速度，负值表示倒车；
- \(\delta\) 为转向控制轴值，而不是直接以弧度表示的前轮角。

令导航路线中心线为按弧长参数化的折线

\[
R(s)=(x_R(s),z_R(s),\theta_R(s)),
\]

其中 \(s\) 表示沿路线的累计距离。将一个姿态投影到路线后，可得到最近点弧长 \(s(q)\)、路线切向角 \(\theta_R(s(q))\) 和到路线的距离。

一个候选 maneuver 表示为

\[
m=(\delta^*,v^*,o^*,\ell^*,\sigma),
\]

其中：

- \(\delta^*\)：采样的转向值；
- \(v^*\)：目标速度；
- \(o^*\)：相对路线中心线的目标横向偏移；
- \(\ell^*\)：路线跟踪前视距离；
- \(\sigma\)：可选的停止线控制参数。

当 \(o^*=\varnothing\) 时，候选在 rollout 中使用固定的采样转向；当 \(o^*\) 有限时，每一个仿真步都会根据路线和偏移目标重新计算转向。

## 2. 规划上下文与模式判定

### 2.1 局部道路和路线投影

规划器首先收集自车附近的道路面，搜索半径为

\[
r_{road}=\max(100,4|v_0|).
\]

随后计算当前车身是否完整位于道路并投影到导航路线。

定义恢复模式标志

\[
recovering =
\neg onRoad(q_0)
\lor d(q_0,R)>6
\lor |wrap(\theta_R-\theta_0)|>1.2.
\]

也就是说，只要车身离开道路、距离路线超过 6 m，或者航向误差超过 1.2 rad（约 68.8°），规划器就进入低速恢复模式。

### 2.2 速度上界

外层 `Simulation.decisionState()` 首先计算交通、终点、弯道和 U-turn 等约束，形成速度上界

\[
v_{ceiling}=\min(v_{traffic},v_{destination},v_{uturn},v_{turn}).
\]

在 `createDrivingPlan()` 中进一步定义

\[
v_{max}=
\begin{cases}
2, & recovering,\\
\min(v_{ceiling},v_{stopApproach}), & requiredStop,\\
v_{ceiling}, & otherwise.
\end{cases}
\]

如果正在接近必须停车的停止线，令保险杠到停止线并扣除 0.5 m 净空后的距离为

\[
d_{room}=\max(0,d_{line}-0.5).
\]

停止线接近速度使用

\[
v_{stopApproach}
=\max\left(0.6,
\sqrt{(a\tau)^2+2ad_{room}}-a\tau
\right),
\]

其中 \(a=4.5\,m/s^2\)，响应预留时间 \(\tau=0.45\,s\)。

### 2.3 恢复目标

恢复模式沿路线尝试以下相对弧长位置：

\[
\Delta s\in\{8,14,20,4,0,-6,28,-12,40\}\;m.
\]

候选目标必须位于道路上。如果自车到目标的直线段不穿过建筑膨胀后的矩形，就采用第一个可见目标。

如果所有直线均受阻，则用“自车 + 路线目标 + 最多 12 个邻近建筑的四个膨胀角点”构造可见图。边权为欧氏距离：

\[
w(i,j)=\|p_i-p_j\|_2,
\]

只连接不穿过建筑的节点，再以最短路径的第一个节点作为恢复 waypoint。

## 3. 路线引导转向

正常模式先选择路线前方目标点

\[
p_g=R\left(s(q_0)+\max(5,1.2|v_0|)\right).
\]

令目标相对航向误差为

\[
\alpha=wrap(heading(q_0,p_g)-\theta_0),
\]

则 Pure-Pursuit 风格的引导曲率为

\[
\kappa_g=\frac{2\sin\alpha}{\max(4,\|p_g-p_0\|_2)}.
\]

代码没有把转向轴直接当作前轮角，而是使用随速度变化的缩放

\[
g(v)=0.58+0.37\left(1-clamp\left(\frac{|v|-3}{5},0,1\right)\right).
\]

曲率和控制轴之间的转换为

\[
\kappa(\delta,v)=\frac{\tan(\delta g(v))}{L},
\qquad
\delta(\kappa,v)=clamp\left(\frac{\arctan(L\kappa)}{g(v)},-0.85,0.85\right),
\]

其中轴距 \(L=2.7\,m\)。

正常模式的转向采样上界会随车速缩小：

\[
\delta_{lim}=0.85\min\left(1,\frac{9}{\max(5,|v_0|)}\right).
\]

恢复模式固定使用 \(\delta_{lim}=0.85\)。

## 4. 原始候选采样

令每次调用伪随机数生成器得到独立的 \(U\sim Uniform(0,1)\)。项目使用 `rng(seed + 9173)` 初始化规划随机序列：相同 seed 的完整运行可复现，但同一次运行连续规划批次会继续推进随机序列，因此候选会变化。

### 4.1 正常模式：55 个候选

正常模式令 \(N=55\)。第 \(i\) 个候选的转向采样为

\[
\delta_i=
\begin{cases}
(2U_i-1)\delta_{lim}, & i\bmod 3=0,\\
clamp\left(\delta_g+(2U_i-1)\max(0.015,0.25\delta_{lim}),
-\delta_{lim},\delta_{lim}\right), & otherwise.
\end{cases}
\]

所以约三分之一候选覆盖整个转向范围，约三分之二候选集中在路线引导值附近。

横向偏移按分层范围采样：

\[
o_i=
\begin{cases}
(2U_i-1)0.10, & 0\le i<14,\\
(2U_i-1)0.65, & 14\le i<30,\\
(2U_i-1)1.35, & 30\le i<44,\\
\varnothing, & 44\le i<55.
\end{cases}
\]

前 44 个是路线相对轨迹，最后 11 个是固定转向探索轨迹。

前视距离为

\[
\ell_i=round\left(
clamp(3.5+0.36\max(v_0,v_{max})+0.8U_i,4,10),2
\right).
\]

速度采样是场景相关的。普通道路大部分候选位于

\[
v_i^*\in[0.94v_{max},v_{max}],
\]

每五个候选中的一个位于

\[
v_i^*\in[0.78v_{max},0.90v_{max}].
\]

存在前车、停止要求或合流交通时，会额外生成约

\[
v_i^*\in[0.25v_{max},0.55v_{max}]
\]

的低速候选。无实际合流冲突时，低于 \(0.7v_{max}\) 的合流候选会被删除，避免车辆仅因旁边存在交通就不必要地爬行。

### 4.2 停止线候选

如果当前控制要求停车，前 8 个候选附带

\[
\sigma_i=(p_{line},0.5,a_i),
\qquad a_i\in[4,4.8]\,m/s^2.
\]

它表示“继续向停止线行驶，并在停止线前 0.5 m 停下”，而不是把目标速度立即设为零。

在 rollout 的每一步，可用距离为

\[
d=\max(0,d_{line}-0.5),
\]

停车线速度上限为

\[
v_{line}=\min\left(\sqrt{2a_i d},2.5d\right).
\]

### 4.3 恢复模式：11 个候选

恢复模式生成 \(N=CANDIDATE\_COUNT-1=11\) 个移动候选。转向采用分层随机采样：

\[
\delta_i=-\delta_{lim}+2\delta_{lim}\frac{i+U_i}{N}.
\]

速度在前进和倒车之间交替：

\[
v_i^*=(-1)^i\,v_{max}(0.6+0.4U_i),
\]

符号的具体起点由 `i % 2` 决定；速度绝对值不超过 2 m/s。恢复候选不使用 lane offset 和 lookahead 路线控制器。

## 5. 轨迹前向仿真

每个 maneuver 都从真实车辆状态复制一个 ghost：

\[
q^{(m)}_0=q_0.
\]

预测时域和步长为

\[
T=3s,\qquad K=60,\qquad \Delta t=T/K=0.05s.
\]

### 5.1 速度和方向盘动态

令最大加速度 \(a_+=5\,m/s^2\)，最大制动减速度 \(a_-=8\,m/s^2\)。速度更新为

\[
v_{k+1}=v_k+clamp(v^*_{effective,k}-v_k,-a_-\Delta t,a_+\Delta t).
\]

方向盘不能瞬时跳到目标值，其变化率受限为 1.8/s：

\[
\delta^{wheel}_{k+1}
=\delta^{wheel}_k
+clamp(\delta^*_{k}-\delta^{wheel}_k,-1.8\Delta t,1.8\Delta t).
\]

### 5.2 路线偏移候选的闭环转向

如果 \(o^*\ne\varnothing\)，在每个仿真步：

1. 把 ghost 投影到路线，得到 \(s_k\)；
2. 取中心线前视点 \(c_k=R(s_k+\ell^*)\)；
3. 沿路线右法向移动 \(o^*\)，得到目标点 \(g_k\)；
4. 计算目标曲率

\[
\kappa_k=
\frac{2\sin(wrap(heading(q_k,g_k)-\theta_k))}
{\max(2,\|g_k-p_k\|_2)};
\]

5. 将曲率转换为该步转向 \(\delta_k^*=\delta(\kappa_k,v_k)\)。

这使路线候选在弯道中持续转向，并在弯道出口回正，而不是保持固定方向盘角度。

### 5.3 简化自行车模型

每一步先更新航向：

\[
\theta_{k+1}=wrap\left(
\theta_k+v_{k+1}\kappa(\delta^{wheel}_{k+1},v_{k+1})\Delta t
\right),
\]

再更新位置：

\[
x_{k+1}=x_k+\sin(\theta_{k+1})v_{k+1}\Delta t,
\]

\[
z_{k+1}=z_k-\cos(\theta_{k+1})v_{k+1}\Delta t.
\]

由此得到完整离散轨迹

\[
P_m=(q_0,q_1,\ldots,q_{60}).
\]

第 24 个积分步，即约 1.2 s 处的状态，还会作为短期路线跟踪评价点。

## 6. 前车和障碍物预测

### 6.1 前车速度约束

对于同向前车，令车身间隙为 \(g\)，期望跟车间隙为 \(g_{desired}\)，则可用空间

\[
r=g-g_{desired}.
\]

代码使用减速度 \(5\,m/s^2\) 和反应时间 \(0.35s\) 计算制动速度：

\[
v_b(d)=\max\left(0,
\sqrt{(5\times0.35)^2+10\max(0,d)}-5\times0.35
\right).
\]

考虑前车自身速度 \(v_l\) 后，候选自车速度被限制为

\[
v_{follow}=\min\left(
v_b\left(r+\frac{v_l^2}{10}\right),
\max(0,v_l+1.5r)
\right).
\]

该约束在 rollout 的每一步重新计算，而不是只在轨迹开始时计算一次。

### 6.2 障碍物预测

候选评估只选择可能在 3 s 内相关的障碍物。近似筛选距离为

\[
d(o,q_0)<
(\max(|v_0|,v_{max})+|v_o|)\times3
+\frac{\sqrt{w_o^2+l_o^2}}{2}+6.
\]

之后按对象类型预测：

- 建筑物保持静止；
- 有路线和弧长状态的车辆沿自身路线前进；
- 普通运动对象按当前航向和速度外推；
- 行人沿 crossing 或 walk path 前进；
- 自车后方跟随车辆根据预测中的自车运动和跟车控制更新速度。

## 7. 道路、车道和碰撞评价

轨迹每隔两个积分点评价一次，即每 0.1 s 一次，共 31 个评价姿态。

### 7.1 车道横向量

对轨迹姿态 \(q_k\) 投影到局部路线，令最近点为 \(R(s_k)\)，横向偏移为

\[
e_k=(x_k-x_R)\cos\theta_R+(z_k-z_R)\sin\theta_R.
\]

令航向误差

\[
\Delta\theta_k=wrap(\theta_k-\theta_R).
\]

旋转车身在路线横向上的投影半径为

\[
\rho_k=
\frac{|\cos\Delta\theta_k|w}{2}
+\frac{|\sin\Delta\theta_k|l}{2}.
\]

相对车道半宽 \(h_k\) 的超出量为

\[
excess_k=\max(0,|e_k|+\rho_k-h_k).
\]

由此定义：

\[
staysInLane \iff \max_k(excess_k)<0.12,
\]

以及

\[
returningToLane \iff
\max_k(excess_k)\le excess_0+0.15
\land |e_{60}|<|e_0|.
\]

### 7.2 道路占用

`roadOccupancy()` 计算完整旋转车身 footprint 位于道路多边形并集之外的面积比例 \(f_k\)，而不是只检查车辆中心点。

定义

\[
f_{max}=\max_k f_k,
\qquad
\bar f=\frac{1}{31}\sum_k f_k,
\]

则

\[
staysOnRoad \iff f_{max}<10^{-5}.
\]

### 7.3 扫掠碰撞

车辆和障碍物均表示为有向矩形。相邻的两个 0.1 s 评价姿态之间，不仅检查端点重叠，还同时插值双方的位置与旋转角。

扫掠段的细分数为

\[
n=max\left(1,
\left\lceil
\frac{d_{ego}+d_{obj}+|\Delta\theta_{ego}|r_{ego}+|\Delta\theta_{obj}|r_{obj}}
{0.12}
\right\rceil
\right).
\]

发现首次矩形重叠后，再进行最多 12 次二分细化，得到首次接触比例和碰撞时间 \(t_c\)。

紧迫碰撞定义为

\[
collisionImminent \iff
t_c\le\max\left(0.75,\frac{|v_0|}{8}+0.3\right).
\]

因此 `collision_predicted=true` 但 `collision_imminent=false` 的较晚冲突可以保留给 Jev 比较；紧迫冲突才作为本地硬过滤条件。

## 8. 轨迹评价函数

### 8.1 正常模式

令：

- \(I_{imm}\)：是否存在紧迫碰撞；
- \(I_{col}\)：是否存在非紧迫预测碰撞；
- \(f_{max}\)：最大车身越界比例；
- \(e_{lane}^{max}\)：最大车道超出量；
- \(\bar e_{lane}\)：31 个评价姿态的平均绝对横向误差；
- \(e_{track}\)：1.2 s 处的位置误差与航向误差组合；
- \(d_{route,end}\)：3 s 末端到路线距离；
- \(\delta_{current}\)：当前真实方向盘/转向状态。

正常模式分数为

\[
\begin{aligned}
J(m)={}&10000I_{imm}+20I_{col}
+1000f_{max}
+30e_{lane}^{max}\\
&+8\bar e_{lane}
+e_{track}
+2d_{route,end}
+0.3|\delta_m-\delta_{current}|.
\end{aligned}
\]

分数越小越好。权重体现的优先级是：紧迫碰撞、道路边界、车道保持、路线跟踪、转向平滑。

短期跟踪误差具体为

\[
e_{track}=d(q_{1.2s},R)
+3|wrap(\theta_R(q_{1.2s})-\theta_{1.2s})|.
\]

### 8.2 恢复模式

恢复模式分数为

\[
J_{recovery}(m)=
10000I_{imm}+20I_{col}
+d(q_{3s},p_{goal})
+0.035e_{heading,deg}
+0.5d(q_{3s},Road).
\]

如果不存在恢复目标，则第一项恢复距离以常数 100 代替。

## 9. 从原始池收缩为内部规划集合

正常模式先定义优先安全集

\[
S=\{m\mid
staysOnRoad
\land\neg collisionImminent
\land directionOK
\land(staysInLane\lor returningToLane)
\}.
\]

高速公路上的 `directionOK` 额外要求：

\[
v_m^*\ge0,
\quad \max_k|\Delta\theta_k|<\pi/2,
\quad s(q_{60})\ge s(q_0)-0.1.
\]

如果处于正常交通队列中，还要求

\[
queueCompatible \iff o^*\ne\varnothing\land |o^*|\le0.2.
\]

候选按 \(J(m)\) 从小到大排序，但最终集合不只是取 top-k，而是保留轨迹多样性：

1. 先保留最多 3 个高质量候选；接近停止线或存在真实合流冲突时，优先覆盖快、慢速度层；
2. 对剩余路线偏移候选按 \(o^*\) 排序，在序列的 0、1/3、2/3、1 位置取 4 个；
3. 对固定转向探索候选按 \(\delta^*\) 排序，同样均匀取 4 个；
4. 追加一个目标速度为零的 stop maneuver。

设计规模为

\[
3+4+4+1=12.
\]

需要注意：第 2、3 步是从原始 pool 中取样，因此内部集合可能故意保留越界或偏离车道的探索轨迹。这些轨迹可用于画面展示和诊断，但还不等于 Jev 可选轨迹。

## 10. Jev 可选集合

对内部 `state.vectors`，首先删除：

- 目标速度为零的项（零速由独立的 `motion=stop` 问题表达）；
- 紧迫碰撞项；
- 正常模式中不在道路上的项；
- 高速公路阶段不沿路线正方向行驶的项；
- 排队场景中不满足 `queue_compatible` 的项。

在剩余集合中采用优先级：

\[
C_{Jev}=
\begin{cases}
C_{inLane}, & C_{inLane}\ne\varnothing,\\
C_{returning}, & C_{inLane}=\varnothing\land C_{returning}\ne\varnothing,\\
C_{safe}, & 非高速恢复允许回退,\\
\varnothing, & otherwise.
\end{cases}
\]

只有满足以下任一条件，`motion=stop` 才可用：

\[
\begin{aligned}
stopAvailable \iff{}& blockerDistance\le2.5\\
&\lor requiredLineDistance\le2.5\\
&\lor destinationDistance\le2.5\\
&\lor C_{Jev}=\varnothing.
\end{aligned}
\]

Jev 请求不会携带 \(P_m=(q_0,\ldots,q_{60})\) 的完整点列，而只发送每个合法候选的目标速度、末端速度、路线进度、路线误差、车道误差、航向误差、末端相对位置、道路状态、停止线状态和预测冲突摘要。

## 11. 数学步骤与源码对应

| 数学步骤 | 主要源码位置 |
| --- | --- |
| 规划状态、速度 ceiling、障碍物集合 | `src/simulation.js:1120-1170` |
| 恢复模式判定、停止线、前车、局部障碍物 | `src/driving-plan.js:146-247` |
| 恢复目标和建筑可见图 | `src/driving-plan.js:40-143` |
| 引导曲率和转向采样上界 | `src/driving-plan.js:248-255` |
| 55 个正常候选和 11 个恢复候选的参数采样 | `src/driving-plan.js:417-475` |
| 三秒、60 步 rollout | `src/planning.js:219-257` |
| 自行车运动学、加速度和转向变化率 | `src/planning.js:45-68` |
| 路线偏移闭环转向 | `src/planning.js:199-217` |
| 跟车速度和障碍物未来姿态 | `src/traffic-safety.js:131-294` |
| 道路、车道、停止线和碰撞指标 | `src/driving-plan.js:257-400` |
| 扫掠有向矩形碰撞 | `src/collisions.js:27-105` |
| 评分公式 | `src/driving-plan.js:401-414` |
| 55 条收缩到内部最多 12 条 | `src/driving-plan.js:476-563` |
| Jev 合法候选的二次过滤 | `src/planning.js:300-401` |
| 候选摘要表和 Jev 问题构造 | `src/jev-request.js:78-115, 240-334` |
| 响应 batch/context/控制值校验与应用 | `src/main.js:712-749` |

## 12. 关键源码片段

以下片段按上述数学过程排列，省略与该步骤无关的字段。

### 12.1 规划模式和速度上限

```js
const occupancy = roadOccupancy(car, surfaces),
  near = nearestOnPath(car, car.route.points);
const recovering =
  !occupancy.on_road ||
  near.distance > 6 ||
  Math.abs(angle(near.heading - car.heading)) > 1.2;

const maxSpeed = recovering
  ? 2
  : round(
      Math.min(
        ceiling,
        requiresStop ? stopApproachSpeed(car, stopLine) : Infinity,
      ),
      2,
    );
```

对应 `src/driving-plan.js:160-202`。

### 12.2 曲率与车辆物理

```js
export function steeringCurvature(steering, speed) {
  return Math.tan(steering * steeringAngleScale(speed)) / WHEELBASE;
}

export function physics(car, steer, target, dt) {
  car.speed += clamp(target - car.speed, -BRAKING * dt, ACCELERATION * dt);
  const actual = car.wheelSteering ?? car.steering ?? 0;
  car.wheelSteering = actual + clamp(steer - actual, -1.8 * dt, 1.8 * dt);
  integratePose(car, car.wheelSteering, dt);
  car.steering = steer;
}
```

对应 `src/planning.js:45-69`。

### 12.3 路线偏移闭环控制

```js
export function maneuverSteering(car, candidate) {
  if (!Number.isFinite(candidate?.lane_offset_m) || !car.route?.points?.length)
    return candidate?.steering ?? car.steering ?? 0;
  const near = nearestOnPath(car, car.route.points);
  const center = pointAt(car.route.points, near.s + candidate.lookahead_m);
  const tangent = center.heading ?? near.heading;
  const goal = move(center, tangent + Math.PI / 2, candidate.lane_offset_m);
  return steeringForCurvature(
    (2 * Math.sin(angle(heading(car, goal) - car.heading))) /
      Math.max(2, dist(car, goal)),
    car.speed,
  );
}
```

对应 `src/planning.js:199-217`。实际代码还对 U-turn 前视距离做了限制。

### 12.4 轨迹 rollout

```js
export function projectVector(car, steering, velocity, candidate = null, speedLimit = null) {
  const ghost = { ...car };
  const points = [{ ...ghost }];
  for (let i = 0; i < VECTOR_STEPS; i++) {
    physics(
      ghost,
      candidate ? maneuverSteering(ghost, candidate) : steering,
      Math.min(
        maneuverVelocity(ghost, candidate, velocity),
        speedLimit
          ? speedLimit(ghost, (i * VECTOR_HORIZON) / VECTOR_STEPS)
          : Infinity,
      ),
      VECTOR_HORIZON / VECTOR_STEPS,
    );
    points.push({
      x: ghost.x,
      z: ghost.z,
      heading: ghost.heading,
      speed: ghost.speed,
    });
  }
  return { axis: steering, velocity, points };
}
```

对应 `src/planning.js:219-257`。

### 12.5 正常/恢复采样

```js
const count = recovering ? CANDIDATE_COUNT - 1 : 55;
for (let i = 0; i < count; i++) {
  const steering = recovering
    ? -limit + 2 * limit * ((i + random()) / count)
    : i % 3 === 0
      ? (random() * 2 - 1) * limit
      : clamp(
          guide + (random() * 2 - 1) * Math.max(0.015, limit * 0.25),
          -limit,
          limit,
        );

  const laneOffset =
    recovering || i >= 44
      ? null
      : round((random() * 2 - 1) * (i < 14 ? 0.1 : i < 30 ? 0.65 : 1.35), 3);

  pool.push(evaluate(steering, velocity, laneOffset, lookahead, stopAtLine));
}
```

对应 `src/driving-plan.js:417-475`。`velocity`、`lookahead` 和 `stopAtLine` 的完整分支位于同一循环中。

### 12.6 轨迹逐点评价与扫掠碰撞

```js
for (let i = 0; i < projection.points.length; i += 2) {
  const pose = { ...car, ...projection.points[i] };
  const status = roadOccupancy(pose, surfaces);
  const lane = laneMeasure(pose);

  laneError += Math.abs(lane.offset);
  laneExcess = Math.max(laneExcess, lane.excess);
  outside += status.outside_fraction;
  maxOutside = Math.max(maxOutside, status.outside_fraction);

  if (!collision && nearby.length) {
    const hit = firstCollision(
      previous,
      pose,
      predictObstacles(i * 0.05, pose),
    );
    collision = !!hit;
  }
  previous = pose;
}
```

对应 `src/driving-plan.js:317-348`。

### 12.7 评分

```js
const score =
  imminentCollision * 10000 +
  (collision && !imminentCollision ? 20 : 0) +
  (recovering
    ? (recovery ? dist(end, goal) : 100) +
      headingError * 0.035 +
      data.road_distance_after_m * 0.5
    : maxOutside * 1000 +
      laneExcess * 30 +
      (laneError / 31) * 8 +
      tracking +
      routeEnd.distance * 2 +
      Math.abs(steering - (car.wheelSteering ?? car.steering)) * 0.3);
```

对应 `src/driving-plan.js:401-414`。

### 12.8 优先安全集和多样性保留

```js
const safe = pool.filter(
  (p) =>
    p.data.stays_on_road &&
    !p.data.collision_imminent &&
    (world.type !== "highway" || p.data.follows_route_direction) &&
    (p.data.stays_in_lane || p.data.returning_to_lane),
);

const eligible = queue ? safe.filter((p) => p.data.queue_compatible) : safe;
const ranked = [
  ...eligible.sort(rank),
  ...pool.filter((p) => !eligible.includes(p)).sort(rank),
];
```

之后代码取最多 3 个优先项、4 个横向分布项和 4 个转向探索项，并追加 stop，见 `src/driving-plan.js:495-563`。

### 12.9 Jev 可选集合过滤

```js
const moving = entries.filter(
  ([, v]) =>
    v.velocity_mps !== 0 && !(v.collision_imminent ?? v.collision_predicted),
);

const roadSafe = state.recovery?.active
  ? moving
  : moving.filter(
      ([, v]) =>
        v.stays_on_road && (!forwardOnly || v.follows_route_direction),
    );

const safe =
  state.traffic?.queue && !state.recovery?.active
    ? roadSafe.filter(([, v]) => v.queue_compatible)
    : roadSafe;

const inLane = safe.filter(([, v]) => v.stays_in_lane);
const returning = safe.filter(([, v]) => v.returning_to_lane);
```

对应 `src/planning.js:300-335`。

由于当前 `createDrivingPlan()` 总会生成布尔型 `collision_imminent`，空值合并表达式实际优先使用该字段；所以较晚的 `collision_predicted=true` 不会仅凭这一点被删除，而会在 Jev 请求的 `conflicts` 中显式呈现。

### 12.10 Jev 请求只发送摘要

```js
state.candidates = {
  horizon_s: 3,
  ...(needsRoadStatus ? {} : { all_on_road: true }),
  ...(needsLaneStatus ? {} : { all_in_lane: true }),
  ...table(columns, rows),
};

const conflicts = Object.fromEntries(
  Object.entries(vectors)
    .filter(([, v]) => v.collision_predicted)
    .map(([id, v]) => [
      id,
      { object: v.collision_object_id, in_s: v.collision_in_s },
    ]),
);
```

对应 `src/jev-request.js:240-296`。

## 13. 当前实现的一个接口注意点

当前实现明确保留两层语义：

- `state.vectors` 可能包含为了展示而保留、但不能由 Jev 选择的探索轨迹；
- `plan.eligible` 和 `prepareJevRequest()` 内部的 `moving` 才表示经过二次过滤的选择集合。

当前 `tests/road-recovery.test.js` 中“所有 `state.vectors` 移动项都在道路上”的断言，与这种两层集合设计不一致。以 seed 42 的 town 初始状态为例，内部状态有 11 个移动向量，最终只有 7 个进入 `candidateChoices()`；其余是越界或偏离车道的探索项。

因此，后续如果调整接口，建议二选一：

1. 保持现有实现，并把测试和字段命名改为明确区分 `sampled/display` 与 `eligible`；
2. 让 `state.vectors` 只暴露合法项，把探索项仅保留在 `plan.projections` 中。

在当前源码语义下，判断“Jev 能否选择某条轨迹”时，应以 `candidateChoices(state)` 或 `prepareJevRequest(state)` 的输出为准，而不能仅检查该轨迹是否存在于 `state.vectors`。
