# JevPilot 场景表示模型

本文介绍 JevPilot 在仿真、地图、交通参与者和决策接口中使用的场景表示。这里的“场景模型”不是一份单独的 JSON Schema，也不是 Three.js 场景树；它是由普通 JavaScript 对象组成的一组相互关联的状态。`Simulation` 持有权威仿真状态，`DriveScene` 只把这些状态同步为可视化对象，感知、规划和 Jev 请求则从权威状态派生出不同粒度的视图。

## 1. 总体结构

场景可以概括为四层：

```text
程序化世界 World
  ├─ 地图拓扑：nodes + edges + byId
  ├─ 连续道路：road surfaces + route.points + route.sections
  └─ 静态对象：objects（建筑、交通设施、植被等）
             │
             ▼
权威动态状态 Simulation
  ├─ ego：player
  ├─ NPC：traffic
  ├─ 行人：pedestrians
  └─ 时间、控制、路权记忆、碰撞与统计状态
             │
             ├─► Three.js 渲染镜像 DriveScene
             ├─► observation(full)：调试/界面视图
             └─► decisionState → prepareJevRequest：规划与 Jev 视图
```

这一区分很重要：

- `world`、`player`、`traffic` 和 `pedestrians` 是仿真真值；
- Three.js 中的 mesh 不参与运动决策，位置和旋转每帧从仿真状态复制；
- `perception` 不是全部世界对象，而是经过距离、视场和遮挡过滤后的观测；
- Jev 收到的又是进一步裁剪、转换和压缩后的场景快照，不等于完整场景模型。

主要入口位于 `src/simulation.js`，城市/城镇地图生成位于 `src/world.js`，高速场景位于 `src/highway.js`，可行驶表面位于 `src/road-geometry.js`，坐标运算位于 `src/math.js`。

## 2. 空间与坐标表示

### 2.1 2.5D 世界空间

驾驶逻辑主要工作在水平的 `x-z` 平面上，长度和速度的单位分别是米和米每秒：

- `+x`：向东；
- `+z`：向南；
- `y`：主要用于渲染高度，不参与道路寻路和车辆平面运动；
- `heading = 0`：向北，即世界空间的 `-z`；
- `heading = +π/2`：向东，即 `+x`；
- 航向通常归一化到 `[-π, π]`。

给定位置 `p=(x,z)`、航向 `h` 和前进距离 `d`，项目采用：

```text
x' = x + sin(h) · d
z' = z - cos(h) · d
```

因此，车辆的前向单位向量是 `(sin(h), -cos(h))`，右向单位向量是 `(cos(h), sin(h))`。

### 2.2 世界坐标、路线坐标与自车坐标

项目同时使用三种互补的空间表示：

| 表示 | 典型字段 | 用途 |
| --- | --- | --- |
| 世界坐标 | `(x, z, heading)` | 保存地图和实体的绝对位置，执行物理、碰撞与渲染 |
| 路线坐标 | `s` | 表示沿某条连续路线的累计弧长，便于跟踪进度、停止线和路段 |
| 自车坐标 | `(right, ahead)` | 表示道路边界、障碍物和目标相对 ego 的位置，供感知和决策使用 |

世界点 `q` 转到 ego 局部坐标时，先计算 `dx=q.x-ego.x`、`dz=q.z-ego.z`：

```text
right = dx · cos(h) + dz · sin(h)
ahead = dx · sin(h) - dz · cos(h)
```

`right > 0` 表示位于车右侧，`ahead > 0` 表示位于车前方。发送给 Jev 的点通常编码为 `[right, ahead]`，而不是绝对的 `[x, z]`。

### 2.3 实体占用空间

车辆、摩托车、行人和建筑等实体至少用中心点、航向、宽度和长度描述。碰撞和道路占用检查不会只检查中心点，而是把对象转换为随航向旋转的矩形 footprint：

```text
中心：(x, z)
方向：heading
横向范围：[-width/2, +width/2]
纵向范围：[-depth/2, +depth/2]
```

建筑的 `rotation` 在转换为碰撞姿态时也会参与计算。车辆运动过程中使用连续扫掠碰撞检查，以降低两个离散帧之间发生高速穿透的可能。

## 3. 整个世界 `world`

`generateWorld(seed, type)` 根据随机种子和环境类型生成场景。当前环境类型包括 `town`、`city` 和 `highway`。同一 `seed + type` 会得到可复现的地图布局和初始对象分布。

典型的 `world` 结构如下：

```js
{
  seed, type, theme,
  nodes, byId, edges,
  objects,
  bounds,
  startNode, nextNode, destination,
  route,

  // 高速场景还可能包含：
  roadSamples, connectorRoads, shoulderOpenings,
  destinationStopLine
}
```

关键字段含义：

| 字段 | 含义 |
| --- | --- |
| `seed` | 程序化生成的随机种子 |
| `type` | `town`、`city` 或 `highway` |
| `theme` | 场景名、默认限速、交通密度和生成参数 |
| `nodes` | 道路图中的路口或关键连接点 |
| `byId` | 从节点 ID 到节点对象的索引 |
| `edges` | 节点之间的道路连接 |
| `objects` | 静态场景对象集合 |
| `bounds` | ego 可活动的世界坐标包围盒 |
| `route` | ego 当前规划路线 |

`world` 主要是相对静态的，但 `world.route` 可能在重新规划或路线选择后被替换。交通灯颜色没有固化在 `objects` 中，而是由节点相位、仿真时间和驶入方向动态计算。

## 4. 地图表示

地图不是单一格式，而是同时保存“可达关系”“可跟随曲线”和“实际沥青表面”。三者服务于不同计算，不能互相替代。

### 4.1 道路拓扑图

节点描述路口或连接点：

```js
{
  id: "j2-1",
  x, z,
  neighbors: [/* 相邻节点 ID */],
  control: "stop" | "signal" | "none",
  offset // 信号周期相位偏移
}
```

城镇和城市节点还带网格索引 `i/j`；高速场景中的部分节点带 `townJunction` 等场景语义。

边描述节点之间的道路：

```js
{
  id,
  a, b,                 // 起止节点 ID
  length,
  width,
  speedLimit,
  name,
  oneWay,               // 高速及匝道中使用
  kind,                 // local/ramp_turn/onramp/merge/interstate/exit/...
  path, centerline, laneOffset // 复杂高速连接中按需出现
}
```

城镇/城市使用带随机缺边的规则网格，但保证连通性；高速场景显式构造本地街道、入口匝道、加速/合流段、州际公路、出口和目的地城镇。

### 4.2 连续路线 `route`

拓扑路径只是节点 ID 序列，车辆实际跟随的是从它生成的连续路线：

```js
{
  ids: [/* 节点 ID */],
  points: [{ x, z, s }, ...],
  crossings: [{ nodeId, x, z, approach, exit, stopS }, ...],
  sections: [{ kind, name, startS, endS, speedLimit, laneHalfWidth }, ...],
  length
}
```

- `points` 是经过采样的路线中心轨迹；`s` 是从路线起点累计的距离；
- `crossings` 把路口、驶入/驶出航向和停止线弧长关联起来；
- `sections` 主要用于高速复合路线，描述每个连续路段的类型、限速和车道半宽；
- `nearestOnPath()` 把任意世界位置投影到路线，得到最近点、距离、航向和 `s`；
- `pointAt(points, s)` 按路线弧长插值得到世界位置和切线航向。

每辆 NPC 可以持有自己的 `route`。因此 `world.route` 是 ego 的当前路线，不是所有车辆共享的唯一轨迹。

### 4.3 可行驶道路表面

规划和越界检查使用 `roadGeometry(world)` 生成的多边形集合，而不是仅使用道路中心线：

- 城镇/城市：每条 `edge` 生成一个矩形道路面，每个路口节点补充一个路口方形面；
- 高速：沿 `roadSamples` 生成左右分隔的道路带，中央隔离区明确不可行驶；
- 匝道和连接路：沿 `connectorRoads` 的采样曲线生成带状多边形；
- 草地、人行道和高速中央隔离区不属于可行驶表面。

`roadOccupancy()` 计算完整车身 footprint 落在道路多边形并集之外的面积比例。`roadState()` 还会沿 ego 路线取横截面，得到真实沥青左右边界、车道边界和车身边缘净空。

### 4.4 静态场景对象

`world.objects` 是带 `type` 标签的异构对象数组。所有对象通常都有 `id/type/x/z`，其余字段随类型变化：

| 类型 | 主要附加字段 | 是否直接影响驾驶语义 |
| --- | --- | --- |
| `building` | `width/depth/height/rotation/style/color` | 是：遮挡、碰撞、本地安全检查 |
| `stop_sign` | `nodeId/approach/height` | 是：关联路口和驶入方向 |
| `traffic_light` | `nodeId/approach/height` | 是：颜色由时间动态求得 |
| `parcel` | `width/depth/park` | 主要用于场景布局 |
| `tree`、`bench` | 高度、种类或旋转 | 主要用于环境与渲染 |
| 高速标牌、路灯、山体、高架桥 | 文本、尺寸、方向等 | 主要用于高速环境与渲染 |

不是所有可见物体都会作为碰撞体或进入感知。例如当前车辆碰撞检查把建筑作为静态障碍，但树木主要是场景装饰。

## 5. Ego 表示

Ego 在 `Simulation.player` 中表示。它和普通汽车共享位置、姿态和尺寸字段，同时保存输入、路线跟踪和决策执行状态：

```js
{
  id: "ego",
  type: "car",
  x, z,
  heading,
  speed,
  width: 1.9,
  depth: 4.75,

  route,
  s,
  stops,
  intersectionMemory,

  steering,
  steeringProgress,
  wheelSteering,       // 首次积分后出现
  target,
  appliedTarget,       // 执行速度约束后出现
  maneuver             // 自动驾驶安装候选后出现
}
```

字段可分为四组：

- 几何状态：`x/z/heading/width/depth`；
- 运动状态：`speed/steering/wheelSteering`；
- 导航状态：`route/s`；
- 规则与控制记忆：`stops/intersectionMemory/target/maneuver`。

`s` 是车辆在自己路线上的进度，但真实位置始终是 `x/z`。手动驾驶时 ego 可以偏离路线，因此系统会用实际位置重新投影并在必要时重规划，不能用 `s` 单独恢复车辆姿态。

运动采用简化自行车模型。目标速度先受加速/制动能力约束，方向盘输入转换为曲率，再积分更新 `heading/x/z`。手动模式从踏板输入计算速度，自动驾驶模式从已选候选 `maneuver` 计算目标转向和速度；二者最终更新的是同一个 ego 状态。

## 6. 其他车辆和摩托车

动态机动车保存在 `Simulation.traffic` 数组中。汽车和摩托车复用同一种数据结构，以 `type` 和尺寸区分：

```js
{
  id: "vehicle-3",
  type: "car" | "motorcycle",
  x, z, heading, speed,
  width, depth,
  color,
  route,
  s,
  stops
}
```

当前默认尺寸是：普通 NPC 汽车约 `1.9 m × 4.2 m`，摩托车约 `0.8 m × 2.3 m`。这些尺寸参与道路净空、跟车距离和碰撞计算，不只是渲染比例。

NPC 的位置主要由自身路线坐标 `s` 推进：系统根据限速、前车、行人、路口控制和礼让关系求目标速度，然后令 `s += speed·dt`，再通过 `pointAt(route.points, s)` 更新 `x/z/heading`。城镇车辆接近路线末端时会延长路线；高速车辆驶出远处后才被回收，避免可见范围内瞬移。

运行时还可能增加 `waitingSince`、`amber` 等临时字段，用于路口等待和黄灯规则。它们属于动态行为记忆，不是地图属性。

## 7. 行人表示

行人保存在 `Simulation.pedestrians` 数组中。基础结构为：

```js
{
  id: "pedestrian-5",
  type: "pedestrian",
  nodeId,
  x, z, heading, speed, // heading 在首次状态更新时确定
  width: 0.6,
  depth: 0.6,
  height: 1.7,

  walkPath: { start: {x, z}, heading, length },
  progress,
  direction,
  crossing,
  walking
}
```

项目有两种行人运动模式：

1. 普通沿街行人沿 `walkPath` 前后行走，`progress` 到达端点后反向；默认速度约为 `0.9 m/s`。
2. `crossing=true` 的行人在信号允许且路口附近没有车辆时开始横穿，横穿时默认速度约为 `3.8 m/s`，到达另一侧后停止并反向等待下一次机会。

`walkPath + progress + direction` 是行人的路径状态，`x/z/heading/speed` 是当前运动状态。行人没有机动车的 `route/s/stops`，但会进入感知、预测冲突、候选安全评价和实际碰撞检查。

当前 `highway` 环境初始化时不生成普通行人；`town` 和 `city` 会生成行人，其中部分绑定到信号控制路口的横穿逻辑。

## 8. 路口与交通控制

交通控制由地图节点、静态设施和运行时规则状态共同表示：

- 节点的 `control` 指明 `stop`、`signal` 或 `none`；
- `stop_sign`/`traffic_light` 对象给出设施的空间位置、所属 `nodeId` 和对应驶入方向 `approach`；
- `route.crossings` 给出车辆路线经过路口时的 `approach/exit/stopS`；
- `signalState(node, time, approach)` 根据仿真时间、节点相位和驶入方向计算灯色、行人通行状态与剩余时间；
- 每辆车的 `stops` 以及 ego 的 `intersectionMemory` 保存到达、完成停车和通过等运行时记忆。

因此，交通灯并不是一个只含当前 `color` 的对象；颜色是从静态地图语义和动态时间计算得到的派生状态。

## 9. 从完整场景到感知与决策视图

### 9.1 原始感知

`scanScene()` 从 `traffic + pedestrians + world.objects` 中扫描支持的对象类型。当前规则包括：

- 动态对象：汽车、摩托车、行人，采用 360° 视场；
- 静态驾驶相关对象：建筑、停车标志、交通灯，只取 ego 前方约 `±65°`；
- 感知距离为 `max(80, |ego.speed| × 6)` 米；
- 建筑 footprint 会执行视线遮挡过滤。

观测对象会转换为相对 ego 的 `ahead/right`、相对航向和相对纵向速度。该结果保存在 `Simulation.perception` 中，并记录可见时间、信号灯状态或行人横穿状态等派生信息。

### 9.2 三种输出视图

项目对同一场景提供三类不同目的的视图，其中 `observation()` 有简略和完整两种模式：

| 视图 | 面向对象 | 特点 |
| --- | --- | --- |
| `observation(false)` | Web 状态面板和调试 | ego、导航、道路规则、可见对象、候选和遥测 |
| `observation(true)` | 完整调试/导出 | 额外包含地图、静态对象、全部 NPC、行人和规划路线 |
| `decisionState()` | 本地候选规划与决策 | 强调局部道路、路口、交通、速度约束和候选轨迹 |
| `prepareJevRequest()` | Jev 模型 | 转为自车坐标并裁剪、表格化、匿名化候选，尽量减少 token |

其中 Jev 请求中的 `traffic` 最多只保留经过优先级排序的少量动态对象，本地候选安全检查则可以直接使用更完整的车辆、行人、建筑和道路真值。因此，“Jev 看见的场景”只是完整场景模型的一种受限投影。

## 10. 状态更新与渲染关系

`Simulation.step(dt)` 是权威状态推进过程，`dt` 被限制为不超过 `0.05 s`。一次更新大致执行：

```text
仿真时间/重规划
  → 路口锁与行人
  → 礼让状态与 NPC
  → 周期性感知扫描
  → ego 实时规则与安全限制
  → ego 车辆动力学
  → 连续碰撞检查
  → 路线进度、违规和到达状态
```

随后 `DriveScene.update()` 才把 `player`、`traffic` 和 `pedestrians` 的 `x/z/heading` 写入 Three.js 对象。渲染坐标使用 `(x, y, z)`，车辆模型的 `rotation.y = -heading`；这种符号转换是 Three.js 模型朝向约定，不改变仿真中的航向定义。

## 11. 一个最小关系示例

下面的示例只展示实体间的引用关系，不是完整运行快照：

```js
world.nodes = [
  { id: "j1", x: 0, z: 0, control: "stop", neighbors: ["j2"] },
  { id: "j2", x: 0, z: -100, control: "signal", neighbors: ["j1"] }
];

world.edges = [
  { id: "road-0", a: "j1", b: "j2", width: 12, speedLimit: 14 }
];

world.route = {
  ids: ["j1", "j2"],
  points: [{ x: 3, z: -14, s: 0 }, /* ... */],
  crossings: [],
  length: 86
};

simulation.player = {
  id: "ego",
  x: 3,
  z: -14,
  heading: 0,
  speed: 0,
  route: world.route,
  s: 0,
  width: 1.9,
  depth: 4.75
};
```

这里 `nodes/edges` 回答“道路如何连通”，`route.points` 回答“车辆应沿哪条连续曲线行驶”，道路多边形回答“车身是否仍在沥青上”，`player` 则保存 ego 在当前时刻的真实姿态和运动状态。

## 12. 源码索引

- `src/simulation.js`：`Simulation`、ego/NPC/行人初始化、每帧状态更新、感知和输出视图；
- `src/world.js`：城镇/城市世界、路网、静态对象、路线与信号状态；
- `src/highway.js`：高速主路、匝道、合流/出口和高速路线分段；
- `src/math.js`：世界坐标运动、航向、路线投影和弧长采样；
- `src/road-geometry.js`：车辆 footprint、可行驶表面、道路占用和局部道路边界；
- `src/planning.js`：车辆运动学、路线段限速和路径跟踪；
- `src/collisions.js`：动态实体的连续扫掠碰撞检测；
- `src/scene.js`：将权威仿真状态镜像到 Three.js；
- `src/jev-request.js`：把规划状态压缩成实际 Jev 请求。

相关的实际 Jev 输入示例见 `review/jev-input-example.json`，字段说明见 `review/jev-input-example.md`。两者展示的是本场景模型经过感知、规划和请求压缩后的最终模型输入，而不是完整世界真值。
