# Jev 输入 JSON 示例说明

`jev-input-example.json` 是使用当前项目源码实际生成的一次 Jev 请求体，不是手工编写的伪数据。生成条件为：

- 使用 `new Simulation(42)` 创建确定性场景；
- 模拟器处于初始时刻，车辆速度为 `0 m/s`；
- 调用 `Simulation.decisionState()` 生成详细驾驶状态；
- 调用 `prepareJevRequest()` 将详细状态压缩为实际发送给 Jev 的请求格式。

## 复现命令

在项目根目录执行：

```bash
node --input-type=module -e '
import { Simulation } from "./src/simulation.js";
import { prepareJevRequest } from "./src/jev-request.js";

const state = new Simulation(42).decisionState();
console.log(JSON.stringify(prepareJevRequest(state).request, null, 2));
'
```

生成请求的主要代码位于：

- `src/simulation.js`：采集当前仿真状态并生成候选轨迹；
- `src/jev-request.js`：压缩状态、匿名化候选轨迹并构造 Jev 问题；
- `server/jev.js`：将请求体发送到 `https://api.typesafe.ai/v1/systemone`。

## 顶层结构

```text
model       要调用的 Jev 模型名称
state       当前车辆、道路、导航、交通和候选轨迹
questions   Jev 本次需要回答的选择题
```

该请求不包含摄像头图像，也不包含完整仿真世界。它是经过感知、规划和压缩后的结构化驾驶快照。

## `state` 字段

### 驾驶策略和单位

- `driving_style`：要求 Jev 遵循的驾驶风格。本例强调积极前进、靠右行驶，并只在必要时停车。
- `units`：数值单位和坐标系说明。位置点采用 `[right, ahead]`，向右和向前为正。

### 当前车辆和导航

- `speed`：当前速度，单位为 `m/s`，本例为 `0`。
- `limit`：当前道路限速，单位为 `m/s`，本例为 `14`。
- `nav.turn`：下一导航动作，本例为左转。
- `nav.in_m`：距下一导航动作的距离。
- `nav.remaining_m`：到目的地的剩余路线长度。
- `nav.destination`：目的地在自车坐标系中的 `[right, ahead]` 位置。

### 道路

- `road.on_road`：车辆当前是否位于可行驶道路上。
- `lane_offset`：车辆相对当前车道中心的横向偏移。
- `lane_half_width`：车道半宽。
- `body_width_length`：车辆的宽度和长度。
- `boundaries`：规划路线采样点的相对位置信息，前方路线中心及左右沥青边界距离。
- `body_edge_clearance_rear_center_front`：车辆后部、中心和前部到左右道路边缘的净空。

`boundaries.rows` 中每行的含义由 `boundaries.columns` 定义。这样可以减少重复字段名，降低请求 token 数量。

### 路口

- `intersection.control`：路口控制类型，本例为停车标志 `stop`。
- `bumper_to_line`：前保险杠到停止线的距离，本例为 `119.5 m`。
- `entered`：车辆是否已经进入路口。
- `stop_completed`：是否已经满足停车要求。

### 周边交通参与者

`traffic.rows` 是紧凑表格，每行按照 `traffic.columns` 解释。本例的列为：

```text
id, type, right, ahead, heading
```

`traffic.shared` 表示所有行共有的值。本例中三个目标的速度和相对前向速度都是 `0`，因此只记录一次。

例如：

```json
["pedestrian-9", "pedestrian", -10, 44.7, -180]
```

表示行人位于车辆左侧 `10 m`、前方 `44.7 m`，其相对航向约为 `-180°`。

## 候选轨迹

`candidates` 是本地规划器生成并预测的轨迹集合：

- `horizon_s: 3`：每条候选轨迹预测未来三秒；
- `all_on_road: true`：所有候选轨迹都保持在道路内；
- `all_in_lane: true`：所有候选轨迹都保持在车道内；
- `v0` 至 `v6`：匿名化后的候选轨迹编号。

每条候选数组按 `candidates.columns` 解释：

| 列 | 含义 |
| --- | --- |
| `speed` | 候选目标速度 |
| `end_speed` | 三秒预测结束时的速度 |
| `progress` | 沿路线前进的距离 |
| `route_error` | 相对导航路线的误差 |
| `lane_error` | 相对车道的误差 |
| `heading_error` | 航向误差，单位为度 |
| `end_right` | 预测终点相对当前位置的右向距离 |
| `end_ahead` | 预测终点相对当前位置的前向距离 |
| `line_after` | 预测结束时停止线仍在车辆前方的距离 |

例如 `v3`：

```json
"v3": [13.2, 13.2, 22.5, 0.3, 0.4, 0.9, -1.1, 22.5, 97]
```

表示该候选的目标和终点速度均为 `13.2 m/s`，预计前进 `22.5 m`，终点位于当前车辆左侧 `1.1 m`、前方 `22.5 m`，结束时距停止线约 `97 m`。

## Jev需要回答的问题

本例的 `questions` 只有 `vector`：Jev需要从 `v0` 至 `v6` 中选择一条候选轨迹，并返回各候选项的概率。

此时继续驾驶是唯一合法的运动类别，因此 `motion=drive` 已由 `prepareJevRequest()` 在本地解析为固定答案，没有放进实际请求体。如果驾驶和立即停车都合法，请求中还会出现 `motion` 问题；如果存在多个有效导航路线，还可能出现 `route` 问题。

Jev只负责选择候选项，不直接生成方向盘角度。本地服务器会将 Jev 选择的 `vN` 映射回原始候选轨迹，再取得其中的 `steering` 和 `velocity_mps` 作为模拟器控制量。
