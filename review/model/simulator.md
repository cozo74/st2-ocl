# JevPilot 模拟器模型

本文档整理 JevPilot 模拟器中的动态实体、运行状态、状态更新和实体交互。地图、道路、静态设施及任务路线的静态表示见 `review/model/mapsystem.md`。

后续按照问题逐项补充，并在内容增多后按模型相关性组织章节。

## 目录

1. [模拟器管理的模型总览](#1-模拟器管理的模型总览)
2. [Ego 车辆模型](#2-ego-车辆模型)
3. [NPC 交通车辆模型](#3-npc-交通车辆模型)
4. [行人模型](#4-行人模型)
5. [共享辅助状态模型](#5-共享辅助状态模型)
6. [模型关系总结](#6-模型关系总结)
7. [动态行为与更新顺序](#7-动态行为与更新顺序)

## 1. 模拟器管理的模型总览

`Simulation` 是一次驾驶模拟的总容器。它直接管理动态实体，同时引用地图和路线，并保存交通规则协调、感知、规划与驾驶结果等运行状态。

代码中并没有分别定义 `Car`、`Motorcycle`、`Pedestrian` 类。除 `Simulation` 本身外，这些模型主要都是通过 `type` 和字段组合区分的普通 JavaScript 对象。

```text
Simulation
├── 静态环境
│   └── world
├── 动态实体
│   ├── player / ego
│   ├── traffic：NPC 汽车和摩托车
│   └── pedestrians：行人
├── 路线与导航
│   ├── 各车辆的 route 和 s
│   ├── destinationPoint
│   └── routeChoices / routeVersion
├── 路口协调与交通规则状态
│   ├── stops / intersectionMemory
│   ├── locks
│   └── courtesy
├── 感知与驾驶决策
│   ├── perception / discovered
│   ├── lastPlan
│   └── lastDecisionState
└── 模拟过程与结果
    ├── time / paused / complete
    ├── collisions / crash / contacts
    ├── violations / distance
    └── events
```

### 1.1 静态环境模型 `world`

模拟器在 `reset()` 时调用 `generateWorld(seed,type)` 创建 `world`，并在整个模拟过程中引用它。

`world` 提供：

- 地图范围、道路节点和道路边；
- 道路几何的源数据；
- 建筑、交通灯和停车标志等静态对象；
- 初始任务路线和目的地。

`world` 已在地图系统文档中说明。对模拟器来说，它主要是所有动态实体运动和交通规则判断的静态环境。

### 1.2 动态实体覆盖范围

源码审计后，模拟器中的核心物理动态实体已经全部覆盖：

| 容器 | 实体类型 |
|---|---|
| `player` | ego 汽车 |
| `traffic` | NPC 汽车和摩托车 |
| `pedestrians` | 普通人行道行人和过街行人 |

项目没有独立的自行车、动物或其他动态实体模型。建筑、停车标志和交通灯等属于 `world.objects` 静态地图对象；交通灯颜色由 node 和模拟时间即时计算，不存在单独持续更新的“信号灯状态实体”。

除物理实体外，模拟器还保存路线、路口协调、感知、规划和事故结果等辅助状态，它们在第五章归纳。

## 2. Ego 车辆模型

### 2.1 `player` 的基础状态字段

`player` 表示本次驾驶任务中的 ego 车辆。初始化时各字段的含义如下：

```js
{
  id: "ego",              // 实体唯一标识；ego 固定使用 "ego"
  type: "car",            // 类型标记，使其按汽车参与感知和碰撞等逻辑

  x: p.x,                  // 车辆中心在世界坐标系中的 x 坐标，单位 m
  z: p.z,                  // 车辆中心在世界坐标系中的 z 坐标，单位 m
  heading: h,              // 当前航向，单位 rad；0 向北，正方向顺时针

  speed: 0,                // 当前实际纵向速度，单位 m/s；负值表示倒车
  steering: 0,             // 车辆转向字段；人工/自动模式下的具体语义见下文
  steeringProgress: 0,     // 人工驾驶时逐渐建立/回正的虚拟方向输入进度
  target: 0,               // 当前请求的目标纵向速度，单位 m/s

  route: world.route,      // ego 当前应跟随的导航路线
  s: 0,                    // ego 在 route 上的累计里程位置，单位 m

  stops: {},               // 按路口 nodeId 保存的停车、放行和通过记录
  intersectionMemory: null,// 当前驶入路口方向上的详细停车记忆

  width: 1.9,              // 车辆横向宽度，单位 m
  depth: 4.75              // 车辆前后方向长度，单位 m
}
```

其中 `x,z,heading` 是车辆当前实际位姿；`route,s` 是相对于导航路线的定位。`s` 可以根据当前 `(x,z)` 投影到 `route.points` 后重新计算，因此它不是另一个独立的世界坐标。

可以把初始化字段分为四组：

| 字段组 | 示例 | 作用 |
|---|---|---|
| 空间状态 | `x,z,heading` | 表示车辆当前位姿 |
| 运动与控制 | `speed,steering,target` | 表示当前运动及控制目标 |
| 导航状态 | `route,s` | 表示当前路线和路线进度 |
| 规则与尺寸 | `stops,intersectionMemory,width,depth` | 表示停车记录和车辆占用尺寸 |

### 2.2 运行过程中追加的字段

JavaScript 对象可以在运行期间增加属性。ego 开始运行后还可能出现以下顶层字段：

| 字段 | 含义 |
|---|---|
| `wheelSteering` | 经过变化速率限制或人工输入整形后，当前实际用于运动学积分的转向控制值 |
| `maneuver` | Jev 最近选择的驾驶方案，例如目标速度、转向、车道偏移或停车线接近方案；人工驾驶或重新规划时可为 `null` |
| `appliedTarget` | 经过安全限速、目的地停车等约束后，本次更新真正交给运动学模型的目标速度 |
| `amber` | 当前黄灯周期内“停车还是继续通过”的已固定判断，避免同一黄灯期间反复改变决定 |
| `waitingSince` | 车辆本次持续静止开始时的模拟时间；车辆重新运动后变为 `null` |

`target` 和 `appliedTarget` 需要区分：

```text
target
  驾驶员或自动驾驶请求的目标速度
        ↓ 安全限制、目的地限制等
appliedTarget
  当前更新实际采用的目标速度
        ↓ 加减速约束
speed
  车辆当前已经达到的真实速度
```

转向相关字段比速度字段多一层，而且 `player.steering` 在人工与自动模式下的赋值方式略有不同，不能简单地把它理解成只取 `-1/+1` 的按键值。

### 2.3 `steeringProgress` 与人工转向平滑

`steeringProgress` 只用于人工键盘驾驶，可以理解为一个取值约在 `[-1,1]` 之间的“虚拟方向杆当前位置”：

```text
-1             0             +1
向左打满       回正          向右打满
```

键盘输入本身只有离散状态：

```text
A / 左方向键  → steeringInput = -1
没有按键      → steeringInput = 0
D / 右方向键  → steeringInput = +1
```

如果直接把这个值用于转向，按下按键的一瞬间就会从回正跳到打满。`steeringProgress` 在两者之间增加一个渐变过程：

- 持续按住方向键时，以约 `1.6/s` 的速度逐渐趋近 `-1` 或 `+1`，约 0.63 s 达到打满；
- 松开方向键时，以约 `5/s` 的速度快速回到 `0`；
- 从左转直接切换到右转时，先回到中心，再向另一侧增加。

完整的平滑更新规则是：

```js
steer = clamp(steeringInput, -1, 1);
current = steeringProgress;

reversing = steer * current < 0;
goal = reversing ? 0 : steer;
returning = reversing || Math.abs(goal) < Math.abs(current);
rate = returning ? 5 : 1.6;

steeringProgress =
  current + clamp(goal - current, -rate * dt, rate * dt);
```

也就是：

| 情况 | 临时目标 `goal` | 变化率 |
|---|---:|---:|
| 从中心向左或向右打方向 | 输入值 | `1.6/s` |
| 保持同方向并继续增加 | 输入值 | `1.6/s` |
| 松开或减小输入 | 输入值，通常为 0 | `5/s` |
| 输入突然变为相反方向 | 先临时设为 0 | `5/s` 回正 |
| 已回到 0 后继续按相反方向 | 新输入值 | `1.6/s` |

这里的 `1.6/s` 表示每秒最多变化 1.6，而不是每 0.1 秒变化 1.6。若 `dt=0.1 s`：

```text
向某侧增加：1.6 × 0.1 = 0.16
快速回正：  5.0 × 0.1 = 0.50
```

例如从回正状态持续按 D：

```text
时间       steeringProgress
0.0 s      0
0.1 s      0.16
0.2 s      0.32
0.3 s      0.48
0.4 s      0.64
0.5 s      0.80
0.6 s      0.96
0.625 s    1.00
```

这些数值是无量纲的虚拟方向输入，不是弧度或角度。实际程序通常会把一帧时间进一步拆成不超过约 0.025 s 的物理子步，因此单个子步的变化更小，但相同总时间内的累计变化率仍分别是 `1.6/s` 和 `5/s`。

最后一种情况是代码自动完成的。假设当前：

```text
steeringProgress = -1   左侧打满
```

此时直接从 A 切换到 D，新的输入为 `+1`。由于输入与当前进度异号：

```js
reversing = steer * current < 0; // true
goal = reversing ? 0 : steer;    // 临时目标为 0
step = 5 * dt;                   // 使用快速回正速度
```

所以完整过程是：

```text
-1 ──以 5/s 回正──→ 0 ──以 1.6/s 向右──→ +1
```

而不是以 `1.6/s` 从 `-1` 一直移动到 `+1`。理论时间约为：

\[
\frac{1}{5}+\frac{1}{1.6}=0.2+0.625=0.825\text{ s}
\]

先松开 A、等待回正，再按 D，执行的也是同样两段：

```text
-1 ──以 5/s 回正──→ 0
                       用户按下 D
                    0 ──以 1.6/s──→ +1
```

如果在刚好回到 0 时立即按 D，理论时间同样约为 0.825 s；如果人工操作中间有停顿，反而会更慢。因此第二种方式并不会因为代码而更快。直接切换到 D 已经包含快速回正逻辑。

如果 A 和 D 同时按住，键盘输入会相互抵消为 `0`，车辆同样以 `5/s` 回正；随后松开 A、只保留 D，才开始以 `1.6/s` 向右增加。

随后系统还会对它进行非线性整形并根据车速限制转向，得到实际用于运动学计算的 `wheelSteering`：

```text
键盘 A/D
   ↓
steeringInput          离散的 -1 / 0 / +1
   ↓ 渐进与回正
steeringProgress       虚拟方向杆位置
   ↓ 非线性整形、车速限制
wheelSteering          本次运动学更新采用的转向值
```

所以它虽然叫 `Progress`，但并不是完成百分比或累计时间，而是人工转向输入经过平滑后的中间状态。自动驾驶走另一套转向变化限制，通常不依赖这个字段。

这里需要区分“平滑”和“整形”：

- `steeringProgress` 已经过随时间逐渐打方向和快速回正的**平滑**；
- 它还没有经过非线性幅值变换和车速限制的**整形**；
- 它的正负号表示当前转向方向，绝对值表示虚拟方向杆偏离中心的程度。

非线性整形在下一步临时计算：

```js
shaped = steeringProgress * (0.2 + 0.8 * Math.abs(steeringProgress));
wheelSteering = shaped * speedLimit;
```

例如 `steeringProgress=0.5` 时：

```text
shaped = 0.5 × (0.2 + 0.8 × 0.5)
       = 0.3
```

随后还要乘以随车速变化的限制系数，才得到 `wheelSteering`。所以它可以称为“当前平滑后的 steering 输入方向和幅度”，但不能称为“最终整形后的 steering”。

### 2.4 `steering`、`steeringProgress` 与实际转角

结论是：

- `steering` 不只取 `-1` 或 `+1`；
- `steeringProgress` 不是实际转向角；
- 运动学模型实际使用的前轮转角没有作为一个独立字段直接保存。

人工键盘驾驶时，原始输入 `Simulation.steeringInput` 才通常是：

```text
-1  按住左键
 0  没有转向按键，或者左右同时按下
+1  按住右键
```

触摸摇杆还可以给出 `-1` 到 `+1` 之间的连续值。人工驾驶的数据流为：

```text
Simulation.steeringInput
        ↓ 平滑
player.steeringProgress
        ↓ 非线性整形和车速限制
player.wheelSteering
        ↓ 乘以随车速变化的角度尺度
实际进入自行车模型的前轮转角 δ
```

前轮转角在计算时得到：

\[
\delta=wheelSteering\times steeringAngleScale(speed)
\]

随后曲率为：

\[
\kappa=\frac{\tan\delta}{wheelbase}
\]

所以 `steeringProgress=0.5` 只表示虚拟方向杆大约打到右侧一半，不能解释为 `0.5 rad` 或某个固定度数。经过非线性整形、车速限制和角度尺度以后，最终的 \(\delta\) 才是运动学意义上的转向角。

`player.steering` 是一个供其他代码查看的转向状态字段：自动驾驶时，它保存 Jev 请求的连续转向值；人工驾驶完成一次更新后，它会被写成这次实际采用的 `wheelSteering`。因此它也不是固定的三值按键枚举。

### 2.5 目标速度 `target`

`target` 是车辆希望达到的**目标纵向速度**，不是当前速度、油门大小或加速度。

例如：

```text
speed  = 4 m/s    当前实际速度
target = 10 m/s   希望加速到的速度
```

运动学更新不会立即把 `speed` 改成 10 m/s，而是按照加减速上限逐渐接近：

```text
speed += clamp(
  target - speed,
  -8 × dt,   // 最大减速变化
  +5 × dt    // 最大加速变化
)
```

若 `dt=0.1 s`，从 `4 m/s` 朝 `10 m/s` 加速时，一次更新最多增加：

\[
5\times0.1=0.5\text{ m/s}
\]

所以本次更新后是 `4.5 m/s`，而不是直接变成 `10 m/s`。

在自动驾驶模式下，`target` 通常来自 Jev 选择的速度；之后安全限制可能把它降低为 `appliedTarget`，实际 `speed` 再逐渐逼近 `appliedTarget`。负数目标速度表示倒车。

在人工踏板模式下，车辆速度主要由油门、刹车、阻力和加速度直接更新，`pedalPhysics()` 并不使用 `target` 驱动车辆。因此 `target` 主要是自动驾驶目标速度字段。

### 2.6 人工驾驶与自动驾驶的有效字段

模拟器通过 `Simulation.autopilot` 选择两条控制路径。两种模式最终都会更新 `speed、x、z、heading`，但输入字段不同。

| 内容 | 人工驾驶 | 自动驾驶 |
|---|---|---|
| 模式 | `autopilot=false` | `autopilot=true` |
| 原始转向输入 | `Simulation.steeringInput` | `player.maneuver`，没有 maneuver 时回退到 `player.steering` |
| 纵向输入 | `Simulation.pedals.throttle/brake` | `player.target` 和 `player.maneuver` |
| 转向中间状态 | `player.steeringProgress` | 不使用 `steeringProgress` |
| 实际转向控制 | 计算得到 `player.wheelSteering` | 以最大 `1.8/s` 逐渐逼近规划转向并写入 `wheelSteering` |
| 实际速度目标 | 不使用速度目标，直接按踏板更新 `speed` | 安全限制后写入 `player.appliedTarget` |
| 运动更新函数 | `pedalPhysics()` | `physics()` |

人工驾驶的数据流是：

```text
steeringInput ─→ steeringProgress ─→ wheelSteering ─┐
                                                     ├→ pedalPhysics → 位姿
pedals.throttle / pedals.brake ──────────────→ speed ┘
```

此时：

- `steeringInput`、`pedals.throttle`、`pedals.brake` 是有效的外部输入；
- `steeringProgress`、`wheelSteering` 是有效的内部转向状态；
- `player.target` 每帧被设为 0，但 `pedalPhysics()` 不读取它来控制速度；
- `player.maneuver` 不参与人工驾驶；
- `appliedTarget` 虽可能显示为 0，但不是人工踏板实际采用的速度目标。

自动驾驶的数据流是：

```text
Jev 选择 candidate
        ↓
player.maneuver
  ├── maneuverSteering(...) ─→ wheelSteering ─┐
  └── maneuverVelocity(...)                   ├→ physics → 位姿
player.target ───────────────→ 安全限速 ─→ appliedTarget ─→ speed
```

此时：

- `maneuver` 是主要驾驶方案，包含路线跟踪、车道偏移、前视距离、候选速度或停车线接近信息；
- `target` 是 Jev 选择的基础目标速度；
- `maneuverVelocity()` 会进一步应用路线限速、掉头限速和停车线速度曲线；
- 安全保护还可以继续降低速度，最终得到 `appliedTarget`；
- `maneuverSteering()` 根据当前位姿和路线持续重新计算转向，`wheelSteering` 再以有限速率跟随它；
- `Simulation.steeringInput`、`pedals` 和 `steeringProgress` 不参与自动驾驶更新。

两种模式都始终使用的车辆状态包括：

```text
x、z、heading、speed
width、depth
route、s
stops、intersectionMemory
```

它们是车辆当前状态、尺寸和导航/规则状态，而不是某一种控制模式专属的输入。

### 2.7 人工踏板与 `speed`

人工驾驶使用 `pedalPhysics()`。输入范围为：

```text
throttle ∈ [-1,1]   正值前进，负值倒车
brake    ∈ [0,1]    0 不刹车，1 最大刹车
```

键盘中，W 对应 `throttle=1`，S 对应 `throttle=-1`，空格对应 `brake=1`；触摸输入可以提供中间值。

#### 第一步：计算阻力

令当前速度大小为：

\[
v=|speed|
\]

阻力减速度为：

\[
r=0.18+0.0017v^2+
\begin{cases}
0.32,& |throttle|<0.01\\
0,& \text{其他情况}
\end{cases}
\]

三部分分别表示：

- `0.18`：始终存在的基础滚动阻力；
- `0.0017v²`：随速度平方增加的空气阻力；
- `0.32`：松开油门时额外加入的滑行/发动机制动。

#### 第二步：判断踏板是否与运动方向相反

```js
opposingPedal = throttle * speed < -0.01;
```

例如车辆正在前进而按 S，或者正在倒车而按 W，踏板方向与当前运动相反。此时踏板先作为减速制动，不会立刻让速度穿过 0 并反向。

总制动减速度为：

\[
b=11\times brake+
\begin{cases}
8\times|throttle|,& \text{踏板与运动方向相反}\\
0,& \text{其他情况}
\end{cases}
\]

#### 第三步：选择速度更新分支

如果正在刹车、踩下反向踏板，或者完全松开油门：

\[
|speed_{new}|=\max(0,|speed|-(r+b)\Delta t)
\]

速度方向保持不变，最小减到 0，不会在同一次更新中越过 0。

如果油门方向与当前运动方向一致，或者车辆静止后开始前进/倒车，则先计算发动机加速度：

\[
a=
\begin{cases}
5\times throttle,& throttle>0\\
2.5\times throttle,& throttle<0
\end{cases}
\]

然后更新：

\[
speed_{new}=speed+(a-\operatorname{sign}(speed\ \text{或}\ throttle)\times r)\Delta t
\]

最后把速度限制在：

```text
-3 m/s ≤ speed ≤ 34 m/s
```

即最大倒车速度约 `3 m/s`，最大前进速度约 `34 m/s`。

#### 简单示例

若车辆以 `10 m/s` 前进并完全松开油门：

\[
r=0.18+0.0017\times10^2+0.32=0.67\text{ m/s}^2
\]

当 `dt=0.1s` 时：

```text
speed_new = 10 - 0.67 × 0.1
          = 9.933 m/s
```

若以 `10 m/s` 前进时踩满刹车，制动项为 `11 m/s²`，加上阻力约为 `11.67 m/s²`：

```text
speed_new = 10 - 11.67 × 0.1
          = 8.833 m/s
```

若前进时直接按 S，则 S 先提供最高约 `8 m/s²` 的反向踏板制动；车辆减速到 0 后，继续按 S 才开始以倒车加速度向负速度移动。

如果前进时同时踩空格刹车和反向踏板 S，两项制动力会直接相加：

\[
b=11\times brake+8\times|throttle|
\]

当二者都为 1 时：

\[
b=11+8=19\text{ m/s}^2
\]

再加上当前速度对应的阻力。例如 `speed=10 m/s`、`brake=1`、`throttle=-1` 时，由于油门不为 0，不添加 `0.32` 的松油门项：

\[
r=0.18+0.0017\times10^2=0.35\text{ m/s}^2
\]

若 `dt=0.1s`：

```text
speed_new = 10 - (19 + 0.35) × 0.1
          = 8.065 m/s
```

速度仍然只会减到 0，不会在一次更新中穿过 0。并且，只要空格刹车仍然按住，`brake` 分支就一直生效：即使 S 仍然按着，车辆到 0 后也会保持静止。松开空格但继续按 S 后，车辆才开始倒车。

同样，如果油门与当前运动方向相同但同时踩着刹车，代码仍优先进入刹车分支；这时油门不会提供发动机驱动力，只有刹车和阻力使车辆减速。

因此人工纵向控制可以概括为：

```text
同方向油门 → 发动机加速度减去阻力
松开油门   → 阻力使车辆滑行减速
空格刹车   → 制动力加阻力
反向踏板   → 先减速到 0，再向相反方向起步
```

### 2.8 `stops` 中的路口记录

`stops` 是每辆车都有的简化交通规则状态，以路口节点 ID 为键：

```js
stops[nodeId] = {
  arrived,          // 首次到达该路口停车区域的模拟时间
  served,           // 是否已经满足规定的停车时长
  stationarySince, // 当前这次静止从何时开始；未持续静止时为 null
  passed            // 是否已经通过该路口
}
```

它主要回答三个问题：

```text
这辆车何时到达路口？
是否已经完成规定停车？
是否已经通过路口？
```

当车辆满足以下条件时，模拟器创建或更新记录：

```text
停车线位于车辆路线进度前 5.5 m 以内
车辆尚未越过停车线超过 0.7 m
|speed| < 0.2 m/s
```

车辆连续静止达到规定时间后，`served=true`：

```text
ego：0.6 s
NPC：1.2 s
```

越过停车线以后，`passed=true`。这些状态用于：

- 停车标志前，`served=false` 时继续要求停车；
- 多辆车同时到达停车标志时，根据 `arrived` 决定先到先行；
- 判断停车标志违规：未 `served` 就越过 `stopS` 算违规；
- 路口占用、礼让和 NPC 通行协调；
- 向自动驾驶提供 `stopCompleted`。

所以 `stops` 是会直接影响交通规则执行的紧凑状态。它并非仅供显示，而且 ego、NPC 汽车和摩托车都会使用。

并非所有字段都会在记录创建时同时出现；它们会随着到达、停车和通过逐步补充。

### 2.9 `intersectionMemory` 中的停车记忆

`intersectionMemory` 只属于 ego，专门描述当前路线、当前路口、当前驶入方向上的停车过程。它的标识不是单纯的 `nodeId`，而是：

```text
routeVersion + nodeId + stopS + approach
```

因此，同一个路口从不同方向驶入，或者重新规划路线后再次到达，会被看作不同的驶入过程。主要字段包括：

| 字段 | 含义 |
|---|---|
| `key` | 由路线版本、路口、停车线和驶入航向组成的本次驶入标识 |
| `nodeId` | 当前路口节点 ID |
| `stopCount` | 在这个驶入方向上已确认的完整停车次数 |
| `stationarySince` | 当前连续静止的开始时间 |
| `currentStopRecorded` | 当前这次静止是否已经计入 `stopCount` |
| `lastStop` | 最近一次有效停车的位置、路线进度、距停车线距离、当时信号及持续时间 |

它会在 ego 位于路线附近、航向与驶入方向大致一致、尚未明显越过停车线且速度低于 `0.2 m/s` 时累计静止时间。持续约 `0.6 s` 后：

```text
stopCount += 1
currentStopRecorded = true
lastStop = 本次停车的详细信息
```

`currentStopRecorded` 防止车辆保持静止时每一帧都重复增加 `stopCount`。车辆重新运动后，下一次完整停车才可能再次计数。

与 `stops` 不同，`intersectionMemory` 可以记住在离停车线较远处发生的提前停车。这样的停车可能尚未满足停车标志的正式停车位置要求，但 Jev 可以知道“刚才已经停过一次，现在应该继续靠近停车线”，避免原地反复选择停车。

其中 `lastStop` 还会记录：

```text
position       停车时的世界坐标
progress       停车时的路线里程 s
lineDistance   当时车头距停车线的距离
signal         当时的信号灯状态
confirmedAt    停车得到确认的时间
duration       连续停车时长
```

这些信息会进入自动驾驶决策上下文；停车次数、信号状态或停车完成状态变化时，也可以触发 Jev 重新决策。

二者的区别可以归纳为：

| 对比 | `stops` | `intersectionMemory` |
|---|---|---|
| 所属对象 | ego 和所有 NPC 车辆 | 仅 ego |
| 索引范围 | 按 `nodeId` 保存多个路口记录 | 只描述当前驶入过程 |
| 主要问题 | 规则是否已经满足、谁先到、是否通过 | ego 在哪里停过、停过几次、当时信号如何 |
| 是否影响规则执行 | 是 | 主要用于 Jev 决策记忆 |
| 是否记录详细位置 | 否 | 是 |
| 提前停车 | 通常不会被标记为 `served` | 可以被记住并告知 Jev |

一次正常停车的大致更新过程为：

```text
接近路口
   ↓
intersectionMemory 开始跟踪当前驶入方向
   ↓
在停车线附近静止
   ├── stops[nodeId] 记录 arrived / stationarySince
   └── intersectionMemory 记录更详细的静止过程
   ↓ 持续 0.6 s
stops.served = true
intersectionMemory.stopCount += 1，并保存 lastStop
   ↓ 越过停车线
stops.passed = true
```

ego 既可以由人工输入控制，也可以由 Jev 自动驾驶选择的 maneuver 控制。其几何占用、碰撞、越线、路线进度和任务完成状态都由模拟器更新。

## 3. NPC 交通车辆模型

### 3.1 `traffic` 中的车辆状态字段

`traffic` 是 NPC 车辆数组，元素的 `type` 为：

```text
car
motorcycle
```

NPC 车辆与 ego 共享大部分基础表示：

```js
{
  id,
  type,
  x,
  z,
  heading,
  speed,
  s,
  route,
  stops,
  width,
  depth,
  color
}
```

各字段含义为：

| 字段 | 含义 |
|---|---|
| `id` | NPC 唯一标识，格式为 `vehicle-i` |
| `type` | `car` 或 `motorcycle`；当前每五辆中有一辆摩托车 |
| `x,z` | 车辆中心的当前世界坐标，单位 m |
| `heading` | 当前航向，单位 rad |
| `speed` | 当前沿自身 route 前进的速度，单位 m/s |
| `s` | 在自身 `route.points` 上的累计里程位置，单位 m |
| `route` | 该 NPC 独立的行驶路线对象 |
| `stops` | 按路口保存的到达、停车完成和通过状态 |
| `width,depth` | 二维碰撞矩形的宽度和前后长度 |
| `color` | 车辆外观颜色，不参与驾驶逻辑 |

汽车尺寸为 `1.9 m × 4.2 m`，摩托车为 `0.8 m × 2.3 m`。运行过程中 NPC 还可能增加：

| 字段 | 含义 |
|---|---|
| `amber` | 当前黄灯周期内是否继续通过的固定判断 |
| `waitingSince` | 当前连续静止开始的模拟时间 |

NPC 没有 ego 的 `steeringProgress、wheelSteering、target、maneuver` 等控制字段，因为它不使用 ego 的转向运动学控制，而是通过 `s` 直接沿 route 推进。

每辆 NPC 有自己的路线和路线进度，不是所有车辆都跟随 `world.route`。它们参与：

- 跟车和前车距离判断；
- 路口先后顺序及让行；
- ego 的感知和冲突预测；
- 候选行驶方案的碰撞预测；
- 真实碰撞判定。

### 3.2 NPC 的 route 何时生成

模拟器执行 `reset()` 时，会先生成 `world` 和 ego，然后按照地图主题中的交通数量多次调用 `spawnTraffic(i)`。每创建一辆 NPC，都会同时为它单独生成 route：

```text
Simulation.reset()
    ↓
生成 world 和 player
    ↓
循环调用 spawnTraffic(i)
    ├── 选择 NPC 的路径节点
    ├── makeRoute(world, ids)
    ├── 随机选择初始路线进度 s
    └── 根据 route.points[s] 放置 NPC
```

所以 NPC route 是 NPC 出生时生成的，不是它开始行驶以后逐段记录出来的。

城镇和城市中，每辆 NPC 会：

1. 随机选择两个相距超过 100 m 的节点 `a、b`；
2. 使用 `shortestPath(world,a.id,b.id)` 得到节点序列；
3. 调用 `makeRoute(world,ids)` 生成完整几何路线。

高速地图中，NPC 使用高速节点 `h0、h1...` 的正向或反向序列，从而形成两个行驶方向；不同车辆还会采用 `4.5 m` 或默认 `9 m` 的中心线偏移，分布到不同车道位置。

NPC 并不一定从 route 起点出生。代码会随机选择：

```js
s = random * route.length;
```

再通过：

```js
p    = pointAt(route.points, s);
next = pointAt(route.points, s + 1);
```

确定其初始 `(x,z)` 和 `heading`。如果位置离 ego 太近或与已有 NPC 太近，就放弃这个位置并重新生成。

### 3.3 NPC route 如何表示

NPC route 与 ego route 使用相同的基本结构：

```js
{
  ids,        // 经过的 node ID 序列
  points,     // 连续车道参考线采样点 {x,z,s}
  crossings,  // 沿途路口及其 approach、exit、stopS
  sections,   // 高速路线分段；城镇 route 没有
  length      // 路线总长度
}
```

`vehicle.s` 是该 NPC 在自己 route 上的累计里程。NPC 更新时并不是通过转向运动学自由跟踪路线，而是直接沿路线推进：

```text
根据交通规则更新 speed
          ↓
s = s + speed × dt
          ↓
pointAt(route.points, s)
          ↓
更新 x、z
          ↓
用前方 0.5 m 的路线点更新 heading
```

因此 NPC 通常会严格落在自己的 `route.points` 参考线上；ego 才使用转向和车辆运动学去跟踪路线。

### 3.4 NPC route 后续是否变化

在城镇和城市中，当 NPC 距当前 route 终点不足约 75 m 时，`continueTraffic()` 会：

1. 保留当前 route 最后两个节点；
2. 再随机选择五个后续相邻节点，尽量不立即原路返回；
3. 用新的节点序列重新执行 `makeRoute()`；
4. 把车辆当前位置投影到新 route，更新 `route` 和 `s`；
5. 清空旧路线对应的 `stops` 和黄灯决定。

这相当于在车辆接近终点前为它续接新的行驶路线。

高速 NPC 不使用这种五节点续接。它到达高速 route 末端后会继续沿当前航向驶出地图；等距离 ego 足够远时，再调用 `spawnTraffic()` 回收并重新生成其位置和 route，避免在可见范围内瞬移。

## 4. 行人模型

### 4.1 `pedestrians` 中的行人状态字段

`pedestrians` 是行人数组。行人对象主要包含：

```js
{
  id,
  type: "pedestrian",
  nodeId,
  x,
  z,
  heading,
  speed,
  width,
  depth,
  height,
  walkPath,
  progress,
  direction,
  crossing,
  walking
}
```

各字段含义为：

| 字段 | 含义 |
|---|---|
| `id` | 行人唯一标识，格式为 `pedestrian-i` |
| `type` | 固定为 `pedestrian` |
| `nodeId` | 生成或过街所关联的路口节点 ID |
| `x,z` | 行人中心的当前世界坐标，单位 m |
| `heading` | 当前行走朝向；初始化对象中尚未写入，首次状态更新时计算 |
| `speed` | 当前行走速度；等待时为 0 |
| `width,depth` | 碰撞占用尺寸，均为 `0.6 m` |
| `height` | 行人高度 `1.7 m`；不参与二维碰撞 |
| `walkPath` | 普通人行道直线路径 `{start,heading,length}` |
| `progress` | 沿当前步行路径的距离进度，单位 m |
| `direction` | `+1` 或 `-1`，表示沿路径正向或反向移动 |
| `crossing` | 是否为受信号控制的过街行人 |
| `walking` | 当前是否正在行走；过街等待时为 `false` |

行人没有车辆的 `route、s、stops` 字段，也没有转向和目标速度字段。其当前位置由简单步行路径、`progress` 和 `direction` 决定。

行人分为沿人行道移动和穿越路口两类。`walkPath/progress/direction` 描述其步行路径及进度，`crossing/walking` 描述是否属于过街行人以及当前是否正在行走。

行人会进入 ego 的感知、轨迹冲突预测、路口让行和碰撞判定。

### 4.2 行人路径的总体表示

行人不像车辆那样使用包含 `ids、points、crossings` 的 route。其路径是更简单的直线参数模型：

```text
路径起点 + 路径方向 + 路径长度 + 当前进度
```

其中 `progress` 的单位是米，表示沿该直线路径从当前定义的起点前进了多远。当前 `(x,z)` 根据路径和 `progress` 计算，并保存在行人对象中供感知和碰撞使用。

普通人行道行人与过街行人的具体路径定义不同。

### 4.3 普通人行道行人的 `walkPath`

普通行人使用：

```js
walkPath: {
  start: { x, z }, // 直线路径起点
  heading,         // 从当前 node 沿道路驶向相邻 node 的方向
  length           // 可步行直线的总长度
},
progress,          // 当前沿路径走过的距离
direction          // +1 正向，-1 反向
```

生成时先随机选择一个 node 和它的一个相邻节点 `other`：

```text
node ───────────────────────── other
```

路径方向为：

```js
walkHeading = heading(node, other);
```

路径起点位于：

- 从 node 沿道路方向前进 14 m；
- 再向道路一侧偏移 7.05 m；
- 左侧还是右侧由随机的 `side=±1` 决定。

路径长度为：

\[
walkPath.length=distance(node,other)-28
\]

减去的 28 m 对应两端各留出 14 m，所以行人只在两个路口之间的直线人行道区段往返：

```text
node ──14 m── [walkPath.start ───────── walkPath.end] ──14 m── other
                       距道路中心线侧向 7.05 m
```

每次更新：

\[
progress_{new}=progress+0.9\times direction\times\Delta t
\]

然后根据：

```js
position = move(walkPath.start, walkPath.heading, progress);
```

计算 `(x,z)`。当 `progress` 到达 `0` 或 `walkPath.length` 时，取值被限制在端点，`direction` 反号，于是行人掉头继续走。

```text
direction = +1：progress 从 0 增长到 length
direction = -1：progress 从 length 减小到 0
```

普通行人的速度固定为 `0.9 m/s`，`heading` 在反向行走时会相对 `walkPath.heading` 增加 `π`。

### 4.4 过街行人的路径和 `progress`

`crossing=true` 的行人不使用上述 `walkPath` 更新位置。虽然初始化对象里仍带有 `walkPath` 字段，但过街分支实际使用一条固定的 16 m 横向直线：

```text
(node.x - 8, node.z - 7.8)
              ←──── 16 m ────→
(node.x + 8, node.z - 7.8)
```

所以它位于路口 node 北侧约 7.8 m，并沿东西方向穿过道路。其位置计算为：

```js
// 从西向东
x = node.x - 8 + progress;

// 从东向西
x = node.x + 8 - progress;

z = node.z - 7.8;
```

这里 `progress` 始终从 `0` 增长到 `16`；行走方向不通过让 progress 递减表示，而是由 `direction` 决定使用哪一个位置公式。

过街行人只有在以下条件成立时才开始：

- 当前信号周期允许行人通行；
- `progress=0`，即位于一侧起点；
- 路口 node 周围 13 m 内没有车辆。

开始后：

\[
progress_{new}=progress+3.8\Delta t
\]

其速度为 `3.8 m/s`。到达 `progress=16` 后，代码将：

```text
progress = 0
walking = false
direction *= -1
```

行人因此停在对面等待；下一次允许通行时，再从对面沿相反方向穿越。

两种路径的差别可以归纳为：

| 对比 | 普通人行道行人 | 过街行人 |
|---|---|---|
| 路径 | `walkPath={start,heading,length}` | node 北侧固定 16 m 横向线 |
| `progress` 范围 | `0～walkPath.length` | `0～16` |
| 反向方式 | progress 反向增减 | progress 重置为 0，切换位置公式 |
| 速度 | `0.9 m/s` | `3.8 m/s` |
| 是否等待信号 | 否 | 是 |
| 高速地图 | 不生成行人 | 不生成行人 |

## 5. 共享辅助状态模型

### 5.1 路线与导航状态

路线虽然来源于地图，但在模拟器中还承担动态导航状态：

- `player.route`：ego 当前使用的路线；
- `vehicle.route`：每辆 NPC 自己的路线；
- `s`：相应实体在自己路线上的当前进度；
- `destinationPoint`：ego 原任务目的地的几何位置；
- `destinationApproach`：到达目的地前的最后两个节点；
- `routeChoices`：车辆偏航后可选择的重规划路线；
- `routeVersion`：ego 路线被替换的版本号。

因此地图中的初始 `world.route` 进入模拟器后，会成为 ego 的当前导航路线，并可能在偏航重规划后被新 route 替换。

### 5.2 路口规则与协调状态

模拟器还管理一些不是物理实体、但会影响实体行为的状态模型：

| 状态 | 所属范围 | 作用 |
|---|---|---|
| `vehicle.stops` | 每辆车 | 记录车辆在各路口的到达、停车完成和通过情况 |
| `player.intersectionMemory` | ego | 记录当前驶入方向上的停车次数和最近一次有效停车 |
| `locks` | 模拟器全局 | 记录当前由哪辆车占用或获准通过某路口 |
| `courtesy` | 模拟器全局 | 处理多车长期等待时的礼让放行 |

交通灯当前颜色没有作为一个持续修改的对象字段保存，而是根据 `node`、模拟时间和驶入方向调用 `signalState(...)` 即时计算。

### 5.3 感知、规划与决策状态

以下对象由动态实体和地图状态派生出来：

| 模型 | 作用 |
|---|---|
| `perception` | 当前扫描到的车辆、摩托车、行人、建筑和交通设施摘要 |
| `discovered` | 已被发现对象的首次与最近可见时间 |
| `lastPlan` | 最近一次驾驶规划，包括道路状态、候选方案和预测结果 |
| `lastDecisionState` | 最近一次提供给 Jev 决策的结构化状态 |

它们不是新的物理实体，而是模拟器根据 `world + player + traffic + pedestrians` 计算出的观测或规划结果。

### 5.4 模拟过程与结果状态

模拟器还保存整次驾驶的过程状态和统计结果：

```text
time、paused                   时间与暂停状态
autopilot、safety             控制模式与安全保护
pedals、steeringInput         人工输入
complete、freeExplore         任务完成和自由驾驶模式
collisions、crash、contacts   碰撞结果
violations                    交通违规次数
distance                      累计行驶距离
events                        事件记录
```

这些不是独立的道路参与者，但共同构成一次 `Simulation` 的完整运行状态。

## 6. 模型关系总结

可以把核心关系简化为：

```text
world 提供地图、静态障碍和初始路线
                    │
                    ▼
player + traffic + pedestrians
        动态实体在地图中运动
                    │
                    ▼
规则协调 + 感知 + 冲突/碰撞预测
                    │
                    ▼
驾驶决策、状态更新和结果统计
```

从实体角度，后续最需要详细讨论的是：

1. ego 车辆；
2. NPC 汽车和摩托车；
3. 行人。

从运行机制角度，则还需要讨论路线进度、路口规则、感知、驾驶规划、运动学更新以及碰撞和违规判定。

## 7. 动态行为与更新顺序

动态行为关注的是：给定上一时刻的地图和实体状态，模拟器如何在时间步长 `dt` 内计算下一时刻状态。

本章将已经确认的更新时间、对象调度和信号灯行为写成完整说明；其余主题暂时保留为后续讨论大纲。

### 7.1 模拟器主更新循环

#### 7.1.1 渲染帧时间与车辆动力学时间步

项目没有设置固定的目标帧率或 FPS limiter。浏览器通过 `requestAnimationFrame(animate)` 驱动主循环，实际渲染帧率由显示器刷新率、浏览器调度、CPU/GPU 性能和当前负载共同决定。每个渲染帧首先根据相邻两次回调的时间戳计算：

```js
const dt = Math.min((now - lastNow) / 1000, 0.2);
```

源码变量名是 `dt`；本节记作 `frameDt`，以便与传入 `Simulation.step()` 的物理子步区分。`now` 的单位是毫秒，除以 1000 后得到以秒为单位的 `frameDt`。单帧最多只计入 `0.2 s`：如果页面或主线程停顿超过 0.2 秒，超过部分不会在恢复后全部补算。

帧率只改变实际得到的 `frameDt`，不会改变车辆动力学公式。例如显示器和浏览器以约 60 FPS 运行时，通常有：

```text
frameDt ≈ 1 / 60 ≈ 0.0167 s
```

以约 120 FPS 运行时：

```text
frameDt ≈ 1 / 120 ≈ 0.0083 s
```

为了避免低帧率时用一个过大的时间步直接更新车辆，主循环把每个渲染帧进一步拆成若干物理子步：

```js
const steps = Math.max(1, Math.ceil(dt / 0.025));
for (let i = 0; i < steps; i++) {
  sim.step(dt / steps);
}
```

因此真正传入车辆和其他动态实体更新的时间步为：

\[
\Delta t_{physics}=
\frac{\Delta t_{frame}}
{\left\lceil\Delta t_{frame}/0.025\right\rceil}
\le 0.025\text{ s}
\]

典型情况为：

| 实际渲染帧率 | `frameDt` | 物理子步数 | 每个 `Simulation.step()` 的 `dt` |
|---|---:|---:|---:|
| 120 FPS | 约 0.0083 s | 1 | 约 0.0083 s |
| 60 FPS | 约 0.0167 s | 1 | 约 0.0167 s |
| 30 FPS | 约 0.0333 s | 2 | 约 0.0167 s |
| 单帧达到上限 | 0.2000 s | 8 | 0.0250 s |

所以真实模拟采用的是**有上限的可变时间步**，不是严格固定为 0.025 秒；`0.025 s` 只是主循环允许的最大物理子步。`Simulation.step(dt)` 内部还执行：

```js
dt = Math.min(dt, 0.05);
```

这是对直接调用者的第二层防御。正常主循环已经保证 `dt≤0.025 s`，所以运行时通常不会触发这个 0.05 秒上限。

不同帧率下使用的更新公式保持不变，例如位移仍然是速度乘以 `dt`。相同现实时间内，高帧率执行更多次较小更新，低帧率执行较少次但可能拆成多个子步。由于当前实现是离散数值积分，不同子步序列可能产生少量数值差异；限制最大子步可以降低这种误差，并改善高速运动和碰撞检测的稳定性。

这里的真实物理 `dt` 还应与候选轨迹的预测步长区分。候选轨迹在 `planning.js` 中固定预测 3 秒、60 步，即 0.05 秒一步；它不是实际 `Simulation.step()` 的车辆更新时间步。

#### 7.1.2 从渲染帧进入物理子步

每个渲染帧先读取人工驾驶的键盘或触摸输入，然后连续执行本帧需要的全部 `Simulation.step(dt)` 子步。因此，一帧被拆成多个物理子步时，这些子步使用同一组人工输入，但车辆速度、位置以及周围实体状态会在每个子步中继续演化。

暂停、事故、页面隐藏或资源仍在加载时不会推进物理模拟：

- 页面隐藏或仍在加载时，主循环在调用 `Simulation.step()` 前返回；
- `paused=true` 或已经发生 `crash` 时，主循环不执行物理子步；
- `Simulation.step()` 自身也再次检查暂停和事故状态；
- 暂停和加载期间，只要动画回调仍在执行，`lastNow` 仍会逐帧更新，因此恢复后不会补算这段时间；如果浏览器在后台完全暂停了 `requestAnimationFrame`，恢复后的第一帧最多只会因 0.2 秒上限补算 0.2 秒，而不会追赶全部后台时间。

任务到达 `complete=true` 与事故不同：模拟器仍可继续进入时间步，但会把 ego 的目标速度设为 0，使其减速停车，并关闭自动驾驶。

#### 7.1.3 手动驾驶与自动驾驶的合并流程

`Simulation.step(dt)` 并不只属于手动驾驶。行人、NPC、时间、碰撞、规则和渲染等大部分流程由两种模式共享；主要差异是 ego 控制量从哪里产生。自动驾驶还在物理循环之外运行一个异步决策闭环，周期性地更新后续物理子步要使用的 `player.maneuver、steering、target`。

两条流程合并后如下：

```text
                              浏览器 requestAnimationFrame
                                           │
                                           ▼
                         计算 frame dt，并拆分 physics dt
                                           │
                                           ▼
                                      判断控制模式
                         ┌─────────────────┴─────────────────┐
                         │                                   │
                         ▼                                   ▼
                  手动驾驶（左分支）                  自动驾驶（右分支）
                         │                                   │
                         ▼                                   ▼
                  读取键盘/触摸输入                 读取当前已应用的控制状态
                         │                          maneuver / steering / target
                         ▼                                   │
              steeringInput / throttle / brake              ▼
                         │                         若决策超过 1.8 s 未更新
                         │                              令 target = 0
                         │                                   │
                         │                                   ▼
                         │                         当前控制立即供本帧使用
                         │                                   ├····························┐
                         │                                   │                              │
                         │                                   │                    异步决策旁路
                         │                                   │                    不阻塞物理主线
                         │                                   │                              │
                         │                                   │                              ▼
                         │                                   │                    到期触发 decide()
                         │                                   │                              │
                         │                                   │                              ▼
                         │                                   │                 worker 复制 Simulation 快照
                         │                                   │                              │
                         │                                   │                              ▼
                         │                                   │              decisionState() / createDrivingPlan()
                         │                                   │                              │
                         │                                   │                              ▼
                         │                                   │                    candidateChoices()
                         │                                   │                              │
                         │                                   │                              ▼
                         │                                   │                    POST /api/decide
                         │                                   │                              │
                         │                                   │                              ▼
                         │                                   │              唯一答案本地确定 / 请求外部 Jev
                         │                                   │                              │
                         │                                   │                              ▼
                         │                                   │                  服务器 + 浏览器两级校验
                         │                                   │                              │
                         │                                   └··有效结果只写回后续帧控制状态··┘
                         │                                   │
                         └─────────────────┬─────────────────┘
                                           │
                                           ▼
                        对每个物理子步执行 Simulation.step(dt)
                                           │
                                           ▼
                         保存动态对象旧位姿并推进模拟时间
                                           │
                                           ▼
                          路线/锁维护 → 行人 → 礼让 → NPC
                                           │
                                           ▼
                                  必要时刷新场景感知
                                           │
                                           ▼
                                     ego 更新模式分支
                         ┌─────────────────┴─────────────────┐
                         │                                   │
                         ▼                                   ▼
                  手动驾驶（左分支）                  自动驾驶（右分支）
                         │                                   │
                         ▼                                   ▼
                  steeringProgress                  maneuverVelocity()
                         │                          maneuverSteering()
                         ▼                                   │
             非线性整形与车速转向限制                        ▼
                         │                         实时 safety / recovery
                         ▼                                   │
                  pedalPhysics(...)                         ▼
                         │                     physics(steering, appliedTarget, dt)
                         │                                   │
                         └─────────────────┬─────────────────┘
                                           │
                                           ▼
                         世界边界限制 → 连续碰撞检测
                                           │
                                           ▼
                       距离/路线进度 → 违规 → 到达状态
                                           │
                                           ▼
                              重复至本帧所有物理子步完成
                                           │
                                           ▼
                               scene.render(frameDt)
                                           │
                                           ▼
                                      下一渲染帧
```

因此自动驾驶不是在每个物理子步中同步调用 Jev。物理循环会继续使用最近一次已接受的 maneuver；后台 worker 规划和 HTTP/Jev 请求都不会暂停车辆、NPC 或行人更新。新响应只有通过完整的新鲜度与一致性检查后，才替换控制状态。

两种模式的共同部分和差异可以归纳为：

| 阶段 | 手动驾驶 | 自动驾驶 |
|---|---|---|
| 世界时间、行人、礼让、NPC | 共同执行 | 共同执行 |
| 路线偏离检测与重规划 | 共同执行 | 共同执行 |
| ego 原始控制来源 | 键盘或触摸输入 | 最近一次有效 Jev/本地唯一候选决策 |
| 本地候选轨迹规划 | 仅开启候选预览或检查器时运行，不控制车辆 | 每次自动决策前运行 |
| 外部 Jev API 请求 | 不发送 | 本地服务器发现仍有多个合法答案时发送；浏览器每轮仍会调用本地 `/api/decide` |
| ego 运动学 | `pedalPhysics()` | `maneuverVelocity/Steering + physics()` |
| 实时安全限速 | 不走自动驾驶 safety 分支 | 在每个物理子步重新检查 |
| 边界、碰撞、路线进度、违规 | 共同执行 | 共同执行 |
| 3D 场景和 UI | 共同更新 | 共同更新，并显示候选与决策 |

这里存在两种不同的“路径规划”，不能混为一谈：

1. **导航路线规划/重规划**：在 ego 偏离当前 route 时，通过 `routeFromLocation()` 寻找回到目的地的路线；手动和自动驾驶都可能触发。
2. **局部候选轨迹规划**：`createDrivingPlan()` 从当前位姿生成未来 3 秒的候选 maneuver，供安全评估、可视化和 Jev 选择；自动驾驶决策每轮都会执行。

手动模式开启“显示候选轨迹”时也会运行第二种本地规划，但结果只进入 `scene.vectors` 做预览，不会写入 `player.maneuver`，也不会向 `/api/decide` 发送请求。

#### 7.1.4 一个物理子步的宏观对象更新顺序

一次 `Simulation.step(dt)` 的实际顺序如下：

```text
检查暂停或事故状态
        ↓
保存 NPC 车辆和行人的旧碰撞位姿
        ↓
推进模拟时间
        ↓
检查 ego 是否需要重新规划路线
        ↓
清理失效的路口锁
        ↓
更新行人
        ↓
更新礼让和路口通行协调状态
        ↓
按数组顺序逐辆更新 NPC 车辆
        ↓
必要时刷新场景感知
        ↓
计算并更新 ego 车辆
        ↓
执行连续碰撞检测
        ↓
更新距离、路线进度、违规和到达状态
```

各阶段的作用为：

1. **保存旧位姿**：时间步开始时保存全部 NPC 和行人的碰撞位姿及 route 引用，供时间步末的动态扫掠碰撞检测使用。
2. **推进全局时间**：先执行 `this.time += dt`，此后本子步中的信号灯、等待时间和规则判断都使用新时间。
3. **路线与锁维护**：检查 ego 是否需要偏航重规划，并删除车辆已经离开、超时或对象已消失的路口锁。
4. **更新行人**：更新所有普通行人和过街行人的 `progress、walking、speed、heading、x、z`。
5. **更新礼让协调**：根据此时的玩家和 NPC 等待状态、已经更新后的行人位置，维护 `courtesy` 与 `locks`。
6. **更新 NPC**：每辆 NPC 依次计算交通规则、目标速度、路线进度和新位姿；必要时续接或回收路线。
7. **刷新感知**：当 `this.time >= nextScan` 时调用 `scanScene()`，随后把下一次周期扫描安排到 0.2 秒之后。
8. **更新 ego**：根据人工踏板或自动驾驶 maneuver、安全限制和道路规则更新 ego 的速度、转向、航向与位置。
9. **碰撞与后处理**：检测 ego 从旧位姿到新位姿的扫掠轨迹；无碰撞时再更新行驶距离、route 上的 `s`、违规和任务完成状态。

可以把核心动态对象顺序概括为：

\[
\boxed{
\text{行人}
\rightarrow
\text{礼让协调}
\rightarrow
\text{NPC车辆}
\rightarrow
\text{感知}
\rightarrow
\text{ego}
\rightarrow
\text{碰撞与规则后处理}
}
\]

#### 7.1.5 更新顺序带来的语义

这些更新不是一次统一的“读取旧状态、计算全部新状态、最后同时提交”。它是有明确先后顺序的原地更新，因此需要注意：

- 行人判断是否开始过街时，车辆仍是本子步开始时的位置；
- 礼让协调看到的是已更新的行人，但车辆仍未进行本子步的移动；
- NPC 使用 `for...of` 逐辆原地更新。靠后的 NPC 可能看到靠前 NPC 的新状态，而靠前 NPC 看到靠后 NPC 的旧状态，因此存在数组遍历顺序依赖；
- 周期性 `scanScene()` 位于 NPC 更新之后、ego 更新之前，所以这次周期扫描使用 NPC/行人的新状态和 ego 的旧状态；生成 Jev `decisionState()` 时还会主动重新扫描，以获得调用时的当前状态；
- ego 最后更新，所以其实时安全限制与碰撞检测可以使用本子步中已经移动后的 NPC 和行人状态；
- 动态碰撞检测同时保留 NPC/行人的旧位姿和新位姿，因此检查的是双方在整个子步内的运动，不只是终点是否重叠。

#### 7.1.6 物理状态与 3D 场景的先后关系

`Simulation` 是权威物理状态，Three.js 场景只是它的显示镜像。一个渲染帧内若包含多个物理子步，程序会先完成全部 `Simulation.step()`，随后只调用一次 `scene.render(frameDt)`。因此 3D mesh 不会在每个物理子步之间单独绘制，而是在本帧全部子步完成后同步到最终模拟状态。

Jev 决策也不是 `Simulation.step()` 中每个子步末尾固定执行的一部分。它由主循环之外的决策调度触发，读取当时的 `Simulation` 状态，再异步产生新的 maneuver；已经选定的 maneuver 则在后续物理子步中持续执行。

#### 7.1.7 本节对应的主要源码

- `src/main.js`：`requestAnimationFrame`、`frameDt`、物理子步拆分和渲染调用；
- `src/simulation.js`：`Simulation.step(dt)` 的世界级更新顺序；
- `src/planning.js`：ego 的速度、转向和位姿积分；
- `src/collisions.js`：时间步内的扫掠碰撞检测；
- `src/background-planner.js` 与 `src/planner.worker.js`：异步状态快照和本地候选规划；
- `src/jev-request.js` 与 `server/jev.js`：Jev 请求压缩、调用、答案展开和服务器校验。

### 7.2 Ego 车辆动态

讨论 ego 从控制输入到新位姿的完整过程：

1. 人工驾驶的键盘、触摸、油门和刹车输入；
2. 自动驾驶的 `maneuver、steering、target`；
3. 安全限制如何得到 `appliedTarget`；
4. 加速度和制动如何更新 `speed`；
5. 简化自行车模型如何更新 `heading、x、z`；
6. 如何把新位置投影回 route 并更新 `s`；
7. 越界限制、停车和任务完成后的处理。

### 7.3 NPC 汽车与摩托车动态

NPC 不使用 ego 的转向积分模型，而是按数组顺序逐辆执行“规则与限速 → 速度积分 → route 进度 → 折线位姿”的更新。核心循环为：

```text
必要时续接 route
        ↓
rule(v,true) 更新交通规则状态
        ↓
speedEnvelope(v) 计算最高允许速度
        ↓
弯道速度限制
        ↓
加速/制动更新 speed
        ↓
停止线越界保护
        ↓
s += speed × dt
        ↓
pointAt(route,s) 更新位置和航向
```

#### 7.3.1 路线续接

城镇 NPC 距 route 末端不足 `75 m` 时调用 `continueTraffic(v)`。该函数保留当前 route 最后两个节点，再随机选择 5 个后续节点；存在其他出口时避免立即返回上一节点。`makeRoute()` 生成新 route 后，只有当前车辆到新 route 的投影距离不超过 `0.5 m` 才安装，并同步更新 `s`、清空 `stops` 和 `amber`。

高速公路 NPC 不执行这种城镇路线续接。它们到达 route 末端后先沿当前航向继续驶出；距 ego 超过 `1300 m` 后才重新生成，避免在玩家视野内发生明显传送。

#### 7.3.2 路口规则更新

每辆 NPC 首先调用：

```js
const rule = this.rule(v, true);
```

该调用读取或更新：

- 红灯、绿灯和一次性黄灯决策；
- 停车标志的到达时间、停车持续时间和 `served/passed`；
- 停车标志的先到先行；
- `locks` 和 `courtesy`；
- 已经在本子步中更新完成的过街行人状态。

它可能返回 `mustStop、distance、reason、color、nodeId、stopCompleted、walk`，并在允许接近停止线或已经进入路口时取得/刷新锁。具体路口协调规则见 7.5。

#### 7.3.3 基础速度与路口停车速度

NPC 的基础最高速度是：

```js
min(world.theme.limit, routeSpeedLimit(v, v.s))
```

即取场景速度上限和当前 route 区段速度上限的较小值。若 NPC 必须在前方路口停车，则进一步使用：

\[
v_{\max}
=
\sqrt{
2\cdot5\cdot
\max\left(0,\,
d-\frac{L}{2}-0.2
\right)
}
\]

其中 \(d\) 是车辆中心到停止线的 route 距离，\(L/2\) 是车辆前半长度，`0.2 m` 是停止余量，`5 m/s²` 是速度包络采用的减速度。随着可用距离减少，允许速度逐渐降低到 0。

#### 7.3.4 前车识别和跟车限速

`leadVehicle()` 将其他车辆中心转换到当前 NPC 的局部坐标：

```text
forward：沿当前车头方向的纵向距离
right：相对当前车头方向的横向距离
```

只有位于前方、横向车身范围重叠且航向大致一致的车辆才会被识别为前车。代码要求：

```js
Math.cos(other.heading - vehicle.heading) >= 0.5
```

因此航向差超过约 `60°` 的对向或横穿车辆不会被当作同车道前车。确定最近前车后，`followingSpeed()` 根据净间距、随速度变化的期望间距、前车沿本车方向的速度和 `5 m/s²` 制动模型降低最高速度。

这意味着横向车辆不能依靠普通跟车逻辑阻止 NPC；横向路口协调主要依靠信号、路口锁和行人规则。NPC 也不调用只为 ego 提供的 `predictTrafficConflict()`。

#### 7.3.5 礼让与弯道限速

获得 `courtesy` 的 NPC 在允许通行时被限制为：

```js
maxSpeed <= 1.5 m/s
```

此外，系统取得 route 前方 `9 m` 的点，并比较从当前位置指向该点的方向与当前航向。如果差值超过 `0.2 rad`，约 `11.5°`，则：

```js
target = Math.min(target, 6.5);
```

所以明显弯道前的 NPC 最高以 `6.5 m/s` 行驶。

#### 7.3.6 速度积分与停止线保护

速度使用受限的一阶变化：

```js
v.speed += clamp(target - v.speed, -7 * dt, 2.8 * dt);
```

即最大加速度为 `2.8 m/s²`，最大减速度为 `7 m/s²`：

\[
v_{new}
=
v_{old}
+
\operatorname{clamp}
\left(
v_{target}-v_{old},
-7\Delta t,
2.8\Delta t
\right)
\]

若单纯使用速度积分会使必须停车的 NPC 在本子步越过停止线，代码还会把速度截断为：

```text
availableRoom / dt
```

其中：

```text
availableRoom =
max(0, rule.distance - vehicle.depth/2 - 0.2)
```

因此本子步位移不会超过停止线前剩余空间。

#### 7.3.7 route 进度、位置和航向

速度更新后，NPC 的 route 弧长进度变为：

```js
v.s += v.speed * dt;
```

尚未到 route 末端时，代码取得：

```js
const p = pointAt(v.route.points, v.s);
const ahead = pointAt(v.route.points, v.s + 0.5);
```

`p` 是 route 上当前进度对应的位置；`ahead` 是沿同一 route 向前 `0.5 m` 的位置。随后：

```js
v.x = p.x;
v.z = p.z;
v.heading = heading(p, ahead);
```

所以“前方 0.5 米”只用于用一段短弦近似 route 在当前位置的切线方向：

\[
heading
=
\operatorname{atan2}
\left(
ahead_x-p_x,\,
p_z-ahead_z
\right)
\]

它不是额外移动 `0.5 m`、不是传感器距离，也不是预测 `0.5 s` 后的状态。NPC 的真实新位置仍是 `p`。这种更新会把 NPC 每个子步直接放回 route 折线，并直接重设航向，而不是像 ego 一样根据转向角和轴距积分横摆运动。

#### 7.3.8 顺序依赖与碰撞边界

NPC 使用 `for...of` 原地更新。靠后的 NPC 会看到靠前 NPC 的新位置、新速度和新路口锁；靠前 NPC 看到靠后 NPC 的仍是本子步旧状态。因此同时到达路口和跟车时可能存在数组顺序依赖。

时间步开始时会保存全部 NPC 和行人的旧位姿，时间步末再将旧、新位姿用于 ego 的扫掠碰撞检测。这能检测 NPC/行人与 ego 在整个子步中的碰撞，但它是事故检测和记录，不是 NPC 移动前的通用横向避碰控制，也没有建立完整的 NPC–NPC 碰撞响应模型。

### 7.4 行人动态

行人在 NPC 之前更新。模拟器根据 `crossing` 将其分为过街行人和普通人行道行人，两类对象都直接更新 `progress、direction、walking、speed、heading、x、z`，不使用车辆动力学。

#### 7.4.1 过街行人开始移动的条件

每个子步首先调用：

```js
signalState(node, simulation.time, 0).walk
```

得到所在路口当前是否允许行人通行。等待中的过街行人只有同时满足以下条件才开始移动：

```text
walk == true
walking == false
progress == 0
路口节点中心13米内没有车辆中心
```

最后一个条件的代码为：

```js
const anyCar = [this.player, ...this.traffic].some(
  v => dist(v, node) < 13
);
```

其中：

\[
dist(v,node)
=
\sqrt{
(v_x-node_x)^2+
(v_z-node_z)^2
}
\]

它以路口节点中心为圆心、`13 m` 为半径，检查 ego 以及全部 NPC 汽车和摩托车的中心点。它不考虑车身长宽、车道、航向、速度、是否真正与人行横道冲突或建筑遮挡。因此它是保守的节点圆形占用检查，不是车辆 OBB 与人行横道的几何相交检查。

这项判断发生在 NPC 和 ego 进行本子步移动之前，所以使用车辆在本子步开始时的位置。它也只决定能否开始过街；一旦 `walking == true`，后续子步不会重新执行 13 米车辆检查来中途暂停行人。

#### 7.4.2 过街运动与到达

过街行人的进度以固定速度更新：

```js
p.progress += dt * 3.8;
```

即：

\[
progress_{new}=progress_{old}+3.8\Delta t
\]

当前位置为：

```js
p.x = node.x + (
  p.direction > 0
    ? -8 + p.progress
    :  8 - p.progress
);
p.z = node.z - 7.8;
```

因此当前实现把过街路径表示为长度 `16 m`、`z` 不变的固定横向线段。`direction` 决定从 `node.x-8` 走向 `node.x+8`，还是反向行走；速度为 `3.8 m/s`，航向相应设为东向或西向。

一旦开始过街，即使 `walk` 在途中变为禁止，行人也会继续走到另一侧，不会停在道路中间。当：

```js
p.progress >= 16
```

代码将：

```js
p.progress = 0;
p.walking = false;
p.direction *= -1;
```

行人从另一侧等待后续允许通行相位，并在下一次通过时沿反方向行走。

#### 7.4.3 普通人行道行人

普通行人不读取交通信号、车辆或路口锁，而是以固定 `0.9 m/s` 沿 `walkPath` 往返：

```js
p.progress += dt * 0.9 * p.direction;
```

到达 `[0, walkPath.length]` 的任一端点时，先将进度夹紧到端点，再令：

```js
p.direction *= -1;
```

位置通过：

```js
move(walkPath.start, walkPath.heading, progress)
```

计算，航向在反向行走时增加 \(\pi\)。这类行人始终设置 `walking=true、speed=0.9`。

#### 7.4.4 行人状态对后续阶段的影响

全部行人完成本子步更新后才调用 `updateCourtesy()`，所以礼让协调看到的是行人新状态。若某个行人在本子步刚开始过街，停车标志路口不会再向车辆授予新的 `courtesy`。

随后 NPC 执行 `rule(v,true)`，也会读取更新后的行人状态。只要存在：

```text
crossing == true
walking == true
pedestrian.nodeId == 当前路口nodeId
```

NPC 就得到 `mustStop=true` 和 `Yield to pedestrian`。这里同样按 `nodeId` 协调，不进一步判断行人位于路口哪一条横道。ego 不由这条脚本规则强制停车：自动驾驶通过候选轨迹、感知与实时冲突预测处理，手动驾驶由驾驶员负责。

时间步末，行人的旧、新位姿会与 ego 的运动一起进入扫掠碰撞检测。因此行人与 ego 在子步中间相交也可被检测，而不只检查最终位置是否重叠。

### 7.5 交通信号与路口协调

#### 7.5.1 信号灯是由时间派生的状态

交通灯对象本身不保存一个随时间不断修改的 `color` 字段。需要使用灯色时，代码调用：

```js
signalState(node, time, approach)
```

该函数根据路口偏移、当前模拟时间和驶入方向即时返回：

```js
{
  color: "green" | "amber" | "red",
  walk: boolean,
  remaining: number
}
```

信号周期为 24 秒：

\[
phase=(time+node.offset)\bmod 24
\]

`approach` 被归类为南北或东西方向，因此同一个路口在同一时刻可能对不同驶入方向返回不同灯色。`node.offset` 则使不同路口不必同时切换。

#### 7.5.2 一个物理子步中何时求值

信号灯不是在时间步开始时统一遍历并保存，而是在消费者需要时按需调用。`Simulation.step(dt)` 先推进 `this.time`，因此随后所有调用都使用本子步的新时间。主要调用顺序为：

| 顺序 | 使用者 | 用途 |
|---:|---|---|
| 1 | 行人更新 | 判断当前是否为允许过街的 `walk` 阶段 |
| 2 | NPC 规则与限速 | 判断下一路口灯色、是否停车以及黄灯决策 |
| 3 | ego 规则与安全限制 | 得到当前控制点状态和实际速度限制 |
| 4 | 越线后处理 | ego 跨过 `stopS` 时判断是否闯红灯 |

一次时间步内同一信号可能被多次查询。例如 NPC 先通过 `rule(v,true)` 更新规则状态，随后 `speedEnvelope(v)` 又可能调用 `rule(v)`；ego 的规则与安全限制也可能分别查询。由于 `signalState()` 是无副作用的纯计算，只要 `node、this.time、approach` 相同，重复调用会得到完全相同的结果。

如果某个子步把时间从 7.99 秒推进到 8.01 秒，而且 8 秒恰好是该路口的相位边界，那么这个子步中后续的行人、车辆和违规判断都会直接看到 8.01 秒对应的新状态。

#### 7.5.3 为什么没有统一预计算全部信号灯

当前设计采用按需计算主要有以下原因：

- `signalState()` 只包含取模、方向分类和少量条件判断，单次成本很低；
- 状态不仅按 `node` 区分，还依赖驶入方向，不能只用一个 `nodeId -> color` 表示；
- 每个车辆通常只关心路线上的下一个控制点，没有必要每个物理子步计算世界中的全部信号灯；
- 渲染、感知、Jev 状态生成和完整 world 导出也会在 `Simulation.step()` 之外查询灯色；
- 直接从 `this.time` 推导可以避免初始化、暂停、重置、子步切换和异步读取中的缓存失效问题。

如果以后世界规模扩大且性能分析证明这里成为热点，更合适的优化是“每个物理子步的按需缓存”：以 `nodeId + 南北/东西方向` 为键，第一次查询时计算，并在 `this.time` 推进后清空，而不是预先计算全部路口。当前规模下，缓存查询和维护成本未必低于直接计算。

#### 7.5.4 3D 场景中的信号灯更新范围

这里需要区分 CPU 上的灯色/材质更新和 GPU 最终绘制范围。

场景初始化时，`DriveScene` 会为 `world.objects` 中的全部 `traffic_light` 和 `stop_sign` 创建 Three.js Group。每个交通灯创建红、黄、绿三个独立灯泡 mesh，并把三个灯泡都加入 `this.lights`。

每次 `scene.render()` 都遍历完整的 `this.lights`：

```js
for (const l of this.lights) {
  const color = signalState(...).color;
  // 更新当前灯泡的 color、emissive 和 emissiveIntensity
}
```

这里没有距离或可见性判断。因此每个渲染帧都会计算并更新世界中所有交通灯的灯泡材质；由于每个交通灯有三个灯泡，同一路口同一方向的 `signalState()` 当前会重复调用三次。

但交通控制设施是否提交给渲染器，受到单独的距离条件限制。每帧先根据控制设施 Group 到 ego 的水平距离设置：

```js
control.visible = distance(control, player) < 170;
```

所以超过 ego 170 米的交通灯和停车标志不会实际绘制。170 米以内的对象还会接受 Three.js 的摄像机视锥裁剪；位于相机后方或视野之外的 mesh 通常也不会产生最终可见图像。建筑遮挡则主要依赖深度测试，它会让灯不可见，但不等同于预先跳过相应的 draw call。

可将各层范围归纳为：

| 操作 | 范围 |
|---|---|
| 创建交通控制设施 mesh | 世界中全部交通灯和停车标志 |
| 调用 `signalState()` 并更新灯泡材质 | 每个渲染帧、全部交通灯的三个灯泡 |
| 设置控制设施 Group 可见 | 仅 ego 周围 170 米内 |
| 最终屏幕可见 | 170 米内、位于摄像机视锥中且未被前景深度遮挡 |

因此，远处不可见的信号灯目前仍会计算灯色并更新材质，只是不实际绘制。这个范围与 `scanScene()` 的感知范围不是同一概念：视觉 mesh 的 170 米显示距离不会直接决定 ego 感知或车辆规则判断。

#### 7.5.5 路口锁的数据结构与粒度

`locks` 是一个以路口节点 ID 为键的 `Map`：

```text
Map<nodeId, { id: vehicleId, at: simulationTime }>
```

`id` 是当前记录的占用车辆，`at` 是该车辆最近一次写入或刷新锁的仿真时间，而不是首次获得锁的时间。锁是交通模型中的路口占用记录，不是线程同步意义上的互斥锁。

当前锁的粒度是整个 `nodeId`，它不区分进口方向、车道、直行/转弯动作或实际轨迹冲突区。因此对遵守锁的 NPC 而言，即使两条绿灯轨迹并不冲突，也可能因共用同一个节点锁而不能同时通行。这是一种保守但较粗粒度的单占用者模型。

#### 7.5.6 车辆如何获得和刷新锁

`rule(v, true)` 先定义车辆到停止线的有符号距离：

```text
delta = crossing.stopS - vehicle.s
```

锁有两条常规写入路径：

1. **接近停止线**：当 `delta < 3 m` 且当前 `mustStop == false` 时，车辆写入锁。对停车标志路口，这意味着已经完成停车且没有锁、先到车辆或礼让授权要求继续等待；对信号灯路口，通常意味着当前是绿灯或已经决定继续通过黄灯。
2. **已经进入路口**：当 `delta < -0.7 m` 时，车辆被视为已在路口内，每次执行 `rule(v,true)` 都直接写入并刷新锁。

```js
this.locks.set(node.id, { id: v.id, at: this.time });
```

第二条路径没有比较现有持锁者，因此一辆已被判定为 `inside` 的车可以覆盖当前锁。所以这不具有原子“申请—批准—释放”语义，更接近“最近写入的路口占用者”。

每个物理子步中，NPC 先按数组顺序逐辆执行 `rule(v,true)`，ego 在此后执行。因此多车同时到达时存在遍历顺序依赖，而已经进入路口的后更新车辆还可能再次覆盖锁。

#### 7.5.7 锁的清理、超时与路口内停车缺口

每个物理子步在更新行人和车辆之前遍历 `locks`。任一条件成立就删除锁：

- 持锁车辆已不存在；
- 持锁车辆中心到路口节点的二维距离大于 `17 m`；
- `simulation.time - lock.at > 7 s`。

七秒指最后一次刷新之后的仿真时间，而不是持锁总时间。只要车辆仍被 `crossingFor(v)` 关联到该路口，并在每个子步刷新，锁可以持续超过七秒。仿真暂停时 `simulation.time` 不增长，锁也不会因现实时间流逝而超时。

当前实现有一个重要安全边界：

```js
crossingFor(v) = route.crossings.find(c => c.stopS - v.s > -19)
```

车辆越过停止线 `19 m` 后，该路口就不再是它的当前 crossing。即使车身在视觉或几何上仍占用路口，`rule(v,true)` 也不再刷新原锁。若距离条件没有先触发，原锁会在最后刷新后七个仿真秒被删除。

锁清理又发生在 NPC 更新之前，因此可能出现：

```text
ego 进入路口并获得锁
        ↓
ego 越过 stopS + 19m 后停住
        ↓
原路口不再刷新，7s 后锁被清理
        ↓
NPC 在本子步看到“无锁”并进入
        ↓
横向 NPC 的普通跟车逻辑不把 ego 当作同车道前车
        ↓
时间步末的扫掠检测记录实际碰撞
```

根本原因是锁的续期依赖 route 上的 `s/stopS` 范围，而不是“车辆 OBB 是否仍与路口冲突区重叠”。七秒超时本意可以作为清理僵尸锁的租约保护，但目前也可能删除仍在几何上占用路口的车辆之锁。

#### 7.5.8 停车标志的停车确认与先到先行

停车标志路口先通过 `stops[nodeId]` 确认车辆已停稳。车辆需要位于停止线附近 `-0.7 m < delta < 5.5 m`，且 `|speed| < 0.2 m/s`。ego 持续 `0.6 s`、NPC 持续 `1.2 s` 后将 `served` 设为 `true`。

已完成停车不等于立即放行。车辆还会查找同一路口中 `arrived` 时间更早、尚未 `passed` 的车辆。存在更早到达者时，当前车继续以 `Yield to first arrival` 等待。此外还要遵守其他车辆的路口锁、已授予其他车辆的 `courtesy`，NPC 还要对正在通过该节点的行人让行。

#### 7.5.9 `courtesy` 长时间僵持解锁机制

`courtesy` 只处理 `node.control === "stop"` 的停车标志路口，不用于红绿灯路口。每个子步都根据 `|speed| < 0.2 m/s` 维护各车的 `waitingSince`，但礼让选择最快每 `0.5 s` 执行一次。

一个路口需要同时满足：

- 附近 `24 m` 内至少有两辆车；
- 这些附近车辆全部处于 `|speed| <= 0.2 m/s` 的静止状态；
- 没有与该节点距离小于 `13 m` 且正在过街的行人；
- 当前没有未失效的礼让授权。

候选车辆还必须：

- 已连续等待至少 `4 s`；
- 已完成该路口停车，即 `stops[nodeId].served == true`；
- `crossingFor(v)` 仍指向该路口；
- 通过 `clearCreep()` 的短距离安全检查；
- NPC 始终可以参与，ego 只有在自动驾驶模式下才能成为候选者。

候选者先按 `stops[nodeId].arrived` 从早到晚排列，到达时间相同时再按车辆 ID 排列。`clearCreep()` 将虚拟车身的宽度和长度各增加 `0.5 m`，对约 `3 s`/30 步、目标速度约 `1.5 m/s`、前方约 `4.5 m` 的慢行进行扫掠碰撞检查。第一辆检查通过的候选车成为获胜者。

获胜者同时获得 `courtesy` 和路口锁；它可忽略外来锁和“更早到达车辆”约束，其他车则以 `Letting stopped traffic clear` 继续停车。NPC 获胜者的速度被限制在 `1.5 m/s` 以内；对自动驾驶 ego，当前代码会生成“以步行速度通过”事件，但这个 `1.5 m/s` 上限没有在 `courtesy` 分支中直接强制应用给 ego。

礼让授权在以下任一情况下删除：获胜车辆不存在，授权超过 `12 s`，车辆距节点大于 `22 m`，或重新执行 `clearCreep()` 时已不安全。

#### 7.5.10 红绿灯路口的信号与锁

红绿灯决定某个驶入方向原则上是否可通行，路口锁则是叠加的占用协调。黄灯出现时，代码根据当前速度与到停止线的剩余距离一次性记录本相位是停车还是继续通过，避免在同一黄灯相位中反复改变决定。

NPC 在红绿灯路口也遵守其他车辆的锁。因此即使是绿灯，只要其他车辆仍持有该 `nodeId` 的锁，NPC 仍会以 `Yield to crossing traffic` 在停止线前等待。由于锁不区分冲突轨迹，NPC 的实际效果近似于“同一路口一次一辆持锁车通行”，但由于覆盖、超时和更新顺序，它并不提供严格的单车互斥保证。

ego 是特例。对信号灯路口，`reservationRequired` 对 ego 为 `false`，因此 NPC 持有的节点锁不会在 `rule()` 中将 ego 强制设为 `mustStop`：

- 手动驾驶依赖驾驶员观察并让行；
- 自动驾驶依赖感知、候选轨迹碰撞预测、Jev 决策和 ego 实时冲突限速；
- NPC 则由脚本规则同时遵守信号灯和节点锁。

不过，一旦 ego 越过停止线 `0.7 m`，它也会作为 `inside` 车辆直接写入锁；这包括手动驾驶违反信号进入路口的情况。所以“持锁”只表示占用记录，不证明车辆是合法进入。

### 7.6 感知更新

`scanScene()` 不读取摄像机画面，而是从 `Simulation` 的真实对象状态构造一个经过距离、视角和遮挡限制的 ego 局部观测层。周期扫描条件为：

```js
if (simulation.time >= nextScan) {
  scanScene();
  nextScan = simulation.time + 0.2;
}
```

因此普通感知约每 `0.2` 个仿真秒刷新一次，即约 `5 Hz`。它位于 NPC 更新之后、ego 更新之前，所以周期扫描读取本子步已经更新的行人和 NPC，但 ego 仍是本子步移动前的位姿。`decisionState()` 在生成自动驾驶决策状态前还会强制执行一次 `scanScene()`，使用 worker 当前快照，而不直接沿用上一个周期感知结果。

#### 7.6.1 对象范围和三层筛选

扫描候选来自 `traffic、pedestrians、world.objects`，但只保留六类：

| 分类 | 类型 |
|---|---|
| 动态对象 | `car、motorcycle、pedestrian` |
| 静态对象 | `building、stop_sign、traffic_light` |

ego 本身不在候选集合中。对每个候选对象依次执行：

```text
对象类型白名单
        ↓
1. 感知距离
        ↓
2. 静态对象前向 ±65° 视场
   动态对象跳过角度限制
        ↓
3. 对所有剩余对象执行建筑遮挡检查
        ↓
加入当前 perception
```

感知距离为：

\[
range=\max(80,\;6|v_{ego}|)
\]

距离使用 ego 与目标对象 `x、z` 参考点之间的二维欧氏距离，不考虑对象尺寸。动态对象在距离范围内使用 `360°` 感知；静态对象还必须位于车头左右各 `65°` 的前向视场内。

建筑遮挡同样应用于动态和静态目标，但能够充当遮挡体的只有 `building`。`blockedByBuilding()` 使用“ego 参考点到目标参考点”的二维线段，检查它是否与其他建筑的轴对齐矩形相交。目标本身若为建筑，会按 ID 排除，避免自己遮挡自己；其他建筑仍可遮挡它。该方法不计算目标轮廓的部分可见面积，也不考虑高度。

#### 7.6.2 感知结果

通过筛选的对象转换为 ego 局部坐标：

```text
ahead_m > 0：ego前方
ahead_m < 0：ego后方
right_m > 0：ego右侧
right_m < 0：ego左侧
```

公共字段包括 `id、type、ahead_m、right_m、speed_mps`。动态对象还包含相对位置、相对航向和纵向相对速度；交通灯附带按当前仿真时间即时计算的 `signal`；行人的 `crossing` 只有在它属于过街行人且当前正在行走时才为 `true`。结果按到 ego 的距离从近到远排序，并保存到：

```js
simulation.perception
```

每次成功看见对象还会更新：

```js
simulation.discovered[id] = {
  first_seen_s,
  last_seen_s,
  type
}
```

`perception` 表示本次扫描当前可见的对象，对象离开范围或被遮挡后会消失；`discovered` 是任务期间曾经看见过的历史记录，不会因暂时不可见而删除。

#### 7.6.3 感知服务的对象

场景感知主要用于：

1. 在 `decisionState()` 中生成面向 Jev 的局部交通描述；感知到的动态对象经过风险优先排序后，最多取 10 个写入 `scene.nearby`。
2. 判断当前 route 控制点的交通灯或停车标志是否可见；设施被遮挡时，Jev 仍可知道前方有路口，但 `visible=false、signal=null`。
3. 只用已经观测到的车辆计算后方交通压力。
4. 为 JSON/Perception 检查器提供 `live_nearby`，并维护 `discovered_objects` 历史。

它不直接驱动 NPC；NPC 使用真实信号、车辆、行人和锁状态。它也不是 ego 本地安全的唯一输入：`createDrivingPlan()`、跟车包络和实时冲突预测仍直接读取完整建筑、NPC和行人集合。因此，被建筑遮挡而没有进入 Jev 可读感知的对象，仍可能参与本地候选轨迹碰撞检查或实时安全限速。

逻辑感知还独立于 Three.js 渲染。调试用传感器扇形只是显示辅助，其固定几何范围不等于 `scanScene()` 的实际动态范围和动态对象 `360°` 感知。

### 7.7 自动驾驶规划与 Jev 决策

7.1.3 已把手动和自动驾驶放在同一张流程图中。本节进一步说明自动驾驶分支如何产生、校验并应用一次新控制决策。

#### 7.7.1 决策触发与物理循环解耦

主线程使用：

```js
setInterval(decide, 25);
```

每 25 毫秒检查一次是否需要决策，但这只是轮询频率，并不表示每 25 毫秒都会请求 Jev。以下情况直接跳过：

- 正在加载或已有决策请求处于 `busy`；
- 未开启 autopilot；
- 模拟已暂停、完成或发生事故；
- 页面位于后台；
- 尚未到达 `nextDecision`，并且信号灯、停车记忆等上下文没有提前变化。

正常环境的下一次决策间隔约为 650 毫秒；恢复、明显弯道、匝道/合流、路口、跟车或附近风险环境中缩短为约 250 毫秒。路口灯色、停车完成状态等上下文变化可以触发提前重规划，但不会突破应用上一决策后最初约 250 毫秒的保护间隔。

`decide()` 是异步函数。等待 worker 或 Jev 响应期间，`requestAnimationFrame` 和 `Simulation.step(dt)` 继续运行，不会阻塞世界更新。

#### 7.7.2 后台本地规划

需要新决策时，`refreshPlan()` 调用 `BackgroundPlanner.run("plan",sim)`。主线程只把当前权威状态的结构化快照发送给 worker，包括：

- 模拟时间、ego、NPC 和行人；
- 当前感知、传感器距离；
- `locks、courtesy`；
- 路线版本、目的地和偏航状态；
- 自动驾驶、任务完成等模式状态。

worker 内维护一个独立 `Simulation` 实例，把快照覆盖到该实例后执行 `decisionState()`。该过程会：

1. 重新调用 `scanScene()`，避免只使用上一个 0.2 秒周期扫描；
2. 计算导航、交通规则、速度上限和当前冲突；
3. 调用 `createDrivingPlan()` 生成并投影局部候选轨迹；
4. 评估道路/车道占用、路线误差、速度、停车线和预测碰撞；
5. 通过 `candidateChoices(state)` 得到真正允许提供给 Jev 的候选；
6. 返回 `state、plan` 以及可能的 route choices。

内部 `plan.vectors/plan.projections` 可以包含为了显示和诊断而保留的探索轨迹，其中并非每一条都安全合法。`plan.eligible` 和请求准备阶段再次得到的 `candidateChoices(state)` 才代表 Jev 可以选择的集合。

worker 返回结果时，浏览器先检查 `generation`、`routeVersion` 和事故状态，防止重置、切换模式、改路线或事故之前启动的旧规划覆盖当前状态。

#### 7.7.3 浏览器到服务器的请求

本地规划通过后，浏览器发送：

```text
POST /api/decide
body = { state, request_id }
```

这里发送给本地 JevPilot 服务器的是完整 decision state，而不是已经压缩的外部 Jev payload。服务器先用 `validState()` 检查批次、候选数量、速度范围、转向范围、道路状态和候选字段。

随后服务器调用 `prepareJevRequest(state)`：

- 再次筛出合法候选；
- 把内部候选 ID 匿名映射为 `v0、v1...`；
- 将候选压缩为速度、终点、路线/车道/航向误差和冲突摘要；
- 组织 `motion、vector` 以及必要时的 `route` 选择问题；
- 不发送本地完整的 61 点轨迹。

如果某个问题只有一个合法答案，`prepareJevRequest()` 把它放入 `fixed`。若全部问题都能唯一确定，服务器直接在本地形成答案，`decision_source="only_eligible_action"`，不会调用外部 Jev。只有仍存在多个选择时，服务器才向 Jev API 发送压缩后的请求。

#### 7.7.4 服务器端决策校验

收到 Jev 答案后，服务器依次：

1. 用 `expandJevAnswers()` 把 `v0、v1...` 恢复为本地候选 ID，并合并唯一答案；
2. 用 `decisionSelection()` 重新计算最终候选及概率；
3. 校验 route 选择属于本次提供的选项；
4. 拒绝选择非零速度且预测会发生碰撞的候选；
5. 从选中候选中提取 `steering` 和 `velocity_mps`；
6. 返回 `batch_id、answers、selection、controls` 以及用量、延迟和费用信息。

因此 Jev 返回的是选择和概率；真正交给车辆的控制值仍从本地候选数据中提取，而不是信任模型自由生成任意转向或速度。

#### 7.7.5 浏览器端再次校验并应用

响应回到浏览器以后不会立即写入车辆。主线程先检查：

- `generation` 仍与请求开始时一致；
- `state.route_version` 仍等于当前路线版本；
- autopilot 仍开启，且没有暂停或事故；
- 从开始规划到响应到达没有超过 1.8 秒；
- 信号灯、停车完成状态、是否进入路口等决策上下文没有改变；
- `decisionControls()` 能从原始 state 和返回 answers 重算出完全一致的 candidate、概率、batch 和 controls。

校验通过后有两个分支：

```text
Jev 选择新的 route
    → chooseRoute()
    → 安装路线并把 nextDecision 置 0
    → 下一轮重新生成局部候选

Jev 选择行驶候选
    → player.maneuver = 被选中的完整本地候选
    → player.steering = controls.steering
    → player.target = controls.velocity
    → 后续 Simulation.step() 连续执行该 maneuver
```

一次有效响应不会直接改变 ego 的 `x、z、heading、speed`。它只更新控制目标；车辆仍通过后续多个物理子步逐渐改变速度、转向和位姿。

#### 7.7.6 等待、过期和失败时的行为

当新决策仍在计算时，车辆继续执行最近一次有效 maneuver，而不是冻结整个模拟。如果超过 1.8 秒仍没有新决策，动画循环会把 `player.target` 设为 0，使车辆按正常制动能力逐渐停车；迟到超过 1.8 秒的响应也会被拒绝。

请求失败时同样把目标速度设为 0，清除候选显示，并按指数退避等待重试。连续失败三次后关闭 autopilot，要求用户重新开启。

完整控制闭环可以简化为：

```text
Simulation 当前权威状态
        ↓
worker 快照 + decisionState()
        ↓
本地 createDrivingPlan() + candidateChoices()
        ↓
POST /api/decide
        ↓
服务器 prepareJevRequest()
        ↓
唯一答案本地确定 / 多答案交给 Jev
        ↓
服务器与浏览器两级校验
        ↓
写入 player.maneuver / steering / target
        ↓
后续物理子步执行并形成新的 Simulation 状态
        └──────────────────────────────→ 下一轮决策
```

### 7.8 安全、冲突与碰撞

讨论三层不同机制：

1. **规划期预测**：候选轨迹是否将与车辆、行人或建筑冲突；
2. **实时安全限制**：已选 maneuver 执行时是否需要进一步限速或停车；
3. **真实碰撞检测**：ego 从旧位姿移动到新位姿时，扫掠矩形是否发生重叠。

还需要解释：

- OBB/车辆 footprint 的使用；
- 静态建筑和动态实体的区别；
- `contacts、collisions、crash` 的更新；
- 碰撞后为什么停止驾驶。

### 7.9 违规判定

单独讨论规则违反与物理碰撞的区别：

- 未完成停车就越过 `stopS`；
- 红灯状态下越过 `stopS`；
- `violations` 何时增加；
- 为什么交通标志对象本身不参与越线判断。

### 7.10 偏航、重规划与任务生命周期

#### 7.10.1 route 最近距离

每隔最多 `0.75` 个仿真秒，`rerouteIfNeeded()` 调用 `nearestOnPath(player,route.points)`，计算 ego 中心到当前 route 折线的最短二维距离。这里没有使用外部 geometry 库，而是使用 `src/math.js` 中的手写线段投影。

对折线的每个线段 \(AB\) 和车辆中心 \(P\)，计算：

\[
t=\operatorname{clamp}\left(
\frac{(P-A)\cdot(B-A)}{\lVert B-A\rVert^2},0,1
\right)
\]

然后得到线段上的最近点及距离：

\[
C=A+t(B-A),\qquad d=\lVert P-C\rVert
\]

遍历后取全局最小的 \(d\)，并返回线段索引、`t`、投影点、方向以及 route 弧长进度 `s`。偏航检测没有传入 hint，所以扫描当前 route 的全部线段，复杂度为 \(O(N)\)。该距离是车辆中心到 route 中心折线的距离，不是车辆 OBB 到道路边界的间隙。

#### 7.10.2 偏航判定与触发条件

完成任务、自由驾驶或已发生事故时不检查重规划。其余情况下：

- 最近距离 `<= 30 m` 时视为仍在 route 范围，立即清除 `offRouteSince` 和旧 `routeChoices`；
- 第一次超过 `30 m` 时只记录 `offRouteSince = simulation.time`；
- 需要连续偏离至少 `6 s` 才能进入重规划；
- 与上一次安装新 route 必须间隔至少 `30 s`；
- 必须已超过 `routeHoldUntil`，且触发时再次确认距离仍大于 `30 m`。

每个子步末，ego 新位置还会重新投影到 route。只要距离恢复到 `30 m` 内，即使后台 worker 正在计算，也会立即取消偏航状态。

#### 7.10.3 新 route 的生成与选择

`routesFromLocation()` 保留原任务的目的地驶入边和精确终点，然后：

1. 计算车辆到每条道路边折线的最近距离，只取最近的 6 条边；
2. 遵守单行方向，从可能的驶入方向寻找回到原目的地驶入边的最短路径及若干绕行路径；
3. 对每个候选 route 计算：

```text
score = 接入距离 × 5
      + |相对航向角| × 9
      + 剩余路线长度 × 0.025
      + 建筑阻挡惩罚 80
```

4. 从最近接入点前约 `3 m` 处裁剪 route 开头，并同步重定义 `points.s`、`crossings.stopS`、`sections` 和 route 长度；
5. 按 score 从小到大返回。

普通自动重规划使用得分最低的候选。导航观测还可向 Jev 暴露最多 3 条首段道路或转向确实不同的 `routeChoices`。车辆正以较高速度通过路口附近时不提供中途换路。Jev 选择的 route 只有在车辆距候选生成原点不超过 `12 m`、且不在高速通过路口时才能安装；选择后设置 `20 s` 的 route hold，避免立即反复换路。

#### 7.10.4 异步计算和安装校验

浏览器中的重规划由 planner worker 异步执行，主仿真在此期间继续更新；为避免重复请求，下一次完整路线检查被推迟约 `4 s`。worker 返回后还要验证：

- 页面 generation 未改变；
- `routeVersion` 与请求时相同；
- 尚未发生事故；
- 当前仍满足 `routeChoiceNeeded()`；
- 新旧 route 的节点 ID 序列并非完全相同。

这些检查防止车辆已经重新回到原 route、页面已重置或已发生事故后，旧 worker 结果仍覆盖当前状态。

#### 7.10.5 安装新 route 时的状态清理

`installRoute()` 同时替换 `player.route` 和 `world.route`，把 ego 的 `s` 设为新 route 上的接入进度，并清理与旧 route 绑定的状态：

- 重置 `stops`；若新旧 route 的当前路口和驶入方向一致，则保留该路口已有的停车记录；
- 清除 `amber、maneuver`，并将 `target` 设为 0；
- 删除 ego 持有的锁和礼让授权；
- 清除旧 `lastPlan/lastDecisionState`；
- 记录 `lastReroute`，清除偏航和候选状态；
- 执行 `routeVersion++`，令旧规划和决策结果失效。

重规划与局部候选轨迹规划是两个不同层次：前者替换前往原目的地的全局 route，后者在当前 route 附近预测未来 3 秒的驾驶 maneuver。

### 7.11 输出、事件与观测

最后讨论模拟器如何对外提供结果：

- `event` 事件记录；
- `observation()` 的 ego、导航、道路规则、传感器和遥测结构；
- `decisionState()` 与普通 observation 的区别；
- 完整 world 快照与精简自动驾驶输入的区别。

整体依赖关系可以概括为：

```text
时间步进
  ↓
实体运动 ──→ 路口交互
  ↓             ↓
感知更新 ──→ 驾驶规划与 Jev 决策
  ↓             ↓
冲突预测与安全限制
  ↓
真实碰撞、违规、重规划和任务结果
```

目前 **7.1 模拟器主更新循环**、**7.3 NPC 动态**、**7.4 行人动态**、**7.5 交通信号、路口锁与礼让协调**、**7.6 感知更新**、**7.7 自动驾驶规划与 Jev 决策** 以及 **7.10 偏航与重规划** 已完成展开。后续可继续讨论 7.2 的 ego 动态，或按问题直接进入其他小节。
