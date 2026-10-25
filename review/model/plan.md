# JevPilot 局部规划的形式化算法

## 文档结构与阅读顺序

本文分为两层：第 0 节给出一次局部规划调用的端到端摘要；第 1—20 节给出该摘要中各步骤的源码对应定义、公式和边界条件。第 0 节中的符号和流程图是导航，后续同名章节是规范性的详细定义。

一次规划应按以下依赖顺序阅读：

\[
\underbrace{\Sigma_{main}\rightarrow S_{msg}\rightarrow\Sigma_w}_{\text{快照与 worker 状态}}
\rightarrow
\underbrace{\mathcal R,n_0,\mu,\Gamma}_{\text{路线与环境上下文}}
\rightarrow
\underbrace{u_i}_{\text{maneuver 参数采样}}
\rightarrow
\underbrace{\Pi_i}_{\text{离散 rollout}}
\rightarrow
\underbrace{a_i,\sigma_i}_{\text{几何评估与评分}}
\rightarrow
\underbrace{\mathcal V,\mathcal E}_{\text{展示集合与 Jev 合法集合}}
\rightarrow
\text{Jev 选择与客户端执行检查}.
\]

对应的详细章节为：

| 阶段 | 详细章节 | 主要产物 |
|---|---|---|
| 快照、路线和 worker 状态 | 第 1—2 节 | \(S_{msg},\Sigma_w,\mathcal R\) |
| 速度、道路和路口环境 | 第 3—6 节 | \(v_{ceil},v_{max},\mu,\Gamma\) |
| 控制参数和原始采样 | 第 7—9 节 | \(u_i\)，原始候选池 |
| 动力学 rollout 与障碍物预测 | 第 10—11 节 | \(\Pi_i\) |
| 几何指标和候选评分 | 第 12—13 节 | \(a_i,\sigma_i\) |
| 安全过滤、展示缩减和停止候选 | 第 14—16 节 | \(\mathcal V\) |
| Jev 合法集合、表示和执行 | 第 17—18 节 | \(\mathcal E\)、候选表示 |
| 完整状态变换与源码索引 | 第 19—20 节 | 端到端复核 |

其中，第 0 节的 0.3—0.4 小节将环境构造、rollout 和评估压缩在一张流程中；不要将该摘要中的同名符号理解为额外的一套计算。凡是摘要与详细章节同时出现的公式，以详细章节的源码映射和边界条件为准。

## 目录

- [文档结构与阅读顺序](#文档结构与阅读顺序)
- [0. 从 maneuver 采样到控制执行的全局流程](#0-从-maneuver-采样到控制执行的全局流程)
- [1. 符号与坐标约定](#1-符号与坐标约定)
- [2. 主线程快照与 worker 状态重建](#2-主线程快照与-worker-状态重建)
- [3. 规划入口的速度上限](#3-规划入口的速度上限)
- [4. 局部几何、恢复模式与目标点](#4-局部几何恢复模式与目标点)
- [5. 路口控制、停车线与本轮最高速度](#5-路口控制停车线与本轮最高速度)
- [6. 前车、队列、合流交通与附近障碍物](#6-前车队列合流交通与附近障碍物)
- [7. 控制参数化](#7-控制参数化)
- [8. 原始候选池的精确采样](#8-原始候选池的精确采样)
- [9. 候选目标速度的逐步约束](#9-候选目标速度的逐步约束)
- [10. 轨迹 rollout 的离散动力学](#10-轨迹-rollout-的离散动力学)
- [11. 障碍物运动预测与连续碰撞检测](#11-障碍物运动预测与连续碰撞检测)
- [12. rollout 轨迹的道路、车道与 route 指标](#12-rollout-轨迹的道路车道与-route-指标)
- [13. 候选评分函数](#13-候选评分函数)
- [14. 原始池内的安全集合与合流预过滤](#14-原始池内的安全集合与合流预过滤)
- [15. 从原始池缩减为展示候选集](#15-从原始池缩减为展示候选集)
- [16. 停止候选是否进入决策状态](#16-停止候选是否进入决策状态)
- [17. 最终 Jev 合法候选集合](#17-最终-jev-合法候选集合)
- [18. 候选的四种表示](#18-候选的四种表示)
- [19. 一次规划的完整状态变换](#19-一次规划的完整状态变换)
- [20. 实现依据](#20-实现依据)

本文精确定义自动驾驶模式下，一次规划请求如何从主线程仿真状态产生候选轨迹，以及哪些候选最终可以交给 Jev 选择。

局部规划采用“主线程决定何时规划，worker 只响应请求”的工作模式。主线程持续执行物理更新和渲染，不等待规划；后台只有一个规划 worker，它不会自行按仿真时间运行，也不会观察主线程状态变化。只有主线程显式发送一份状态快照后，worker 才针对该快照执行一次完整的局部规划。

主线程以 25 ms 的墙钟周期调用 `decide()`，但这只是轮询是否应该开始新一轮决策，并不意味着每 25 ms 规划一次。只有同时满足以下运行条件时，主线程才继续调度：自动驾驶已开启，系统不在加载或处理上一轮决策，仿真未暂停，页面可见，行程未完成，车辆也未碰撞。满足运行条件后，还必须满足下列两个触发条件之一：

1. 已到达常规规划期限 `nextDecision`；
2. 尚未到期，但 crossing 上下文已经变化，需要提前规划。

这里“没有处理上一轮决策”由主线程变量 `busy` 表示，但 `busy` 不是“worker 是否正在运行”的标志。一次 `decide()` 确定可以启动新决策后，会在调用 `refreshPlan()` 之前设置 `busy=true`；此后它覆盖整条异步决策链：

\[
\texttt{busy=true}
\rightarrow\text{等待 worker 局部规划}
\rightarrow\text{等待 Jev 响应}
\rightarrow\text{主线程检查并应用或丢弃结果}.
\]

worker 返回 \((\mathcal D_0,\mathcal L_0)\) 时不会立即清除 `busy`，因为 Jev 请求和结果验收尚未完成。只有本轮 `decide()` 离开其 `try/catch/finally` 结构时，`finally` 才统一执行 `busy=false`。因此无论成功应用 maneuver、安装新 route、发现结果过期、上下文变化，还是 worker/Jev 报错，本轮结束时都会清除 `busy`。在 `busy=true` 期间，25 ms 定时器仍会调用 `decide()`，但入口检查会立即返回，不再启动另一轮 worker 规划或 Jev 请求；物理仿真和渲染则继续运行。

一次决策成功应用后，主线程根据刚才那份决策状态确定常规规划期限：

\[
t_{next}=t_{requestStart}+\Delta t_{plan},
\qquad
\Delta t_{plan}=\begin{cases}
250\text{ ms},&\chi_{fast},\\
650\text{ ms},&\neg\chi_{fast}.
\end{cases}
\]

其中 \(t_{requestStart}\) 是主线程开始本轮 `decide()` 的墙钟时间，而不是 worker 返回结果的时间。快速规划谓词 \(\chi_{fast}\) 在恢复模式、route 弯曲角大于 \(12^\circ\)、`merge` 或 `ramp_turn` 阶段、存在当前 hazard，或者满足以下任一邻近条件时成立：

\[
stopLineAhead<d_{near},
\quad followingGap<d_{near},
\]

\[
\exists o\in nearby:
|o.right|<8\land|o.ahead|<d_{near},
\qquad
d_{near}=\max(18,3|v|)\text{ m}.
\]

除常规期限外，主线程还负责事件触发。开启自动驾驶、从暂停中恢复、检测到 route 版本变化或成功安装新的全局 route 时，主线程令 `nextDecision=0`，所以下一次 25 ms 调度检查会立即启动规划。在尚未到达常规期限时，只要上一决策已经应用至少 250 ms、当前没有错误退避，主线程就每 100 ms 比较一次旧决策状态与当前 crossing 上下文；crossing ID、可见信号灯颜色、停车完成状态、是否已进入路口或当前 approach 的停车次数任一变化，都会提前启动规划。

当触发条件成立时，主线程调用 `refreshPlan()`。如果已经存在 `planningJob`，新的调用直接复用该 Promise；否则主线程记录当前 `generation` 和 `routeVersion`，从权威仿真状态 \(\Sigma_{main}\) 构造快照 \(S_{msg}\)，然后通过 `postMessage` 发送给 worker。发送完成后主线程继续推进物理仿真和渲染，因此车辆状态可能在 worker 计算期间继续变化。

worker 收到消息后，先按 `epoch:worldType:seed` 选择或创建自己的缓存世界，再把 \(S_{msg}\) 恢复为 worker 状态；随后重新执行场景感知、约束计算、maneuver 采样、rollout、候选评估和合法集合计算。返回结果的主体是决策状态 \(\mathcal D_0\) 与规划产物 \(\mathcal L_0\)，同时附带 worker 更新后的 `routeChoices` 分支记忆。worker 的计算对象始终是快照时刻的状态，不是主线程此刻正在变化的实时状态。

主线程收到 worker 结果后首先检查 `generation`、`routeVersion` 和碰撞状态。若规划期间发生重置、驾驶模式切换或 route 变化，结果直接丢弃，不会提交给 Jev。通过检查后，主线程才把 \(\mathcal D_0\) 发送给 `/api/decide`；\(\mathcal L_0\) 留在客户端用于候选显示。等待 Jev 时主线程仍继续物理更新。Jev 响应到达后还要再次检查驾驶模式、暂停/碰撞状态、route 版本、1800 ms 时效、候选批次以及 crossing 上下文；全部成立才把所选 maneuver 应用于车辆。由此形成

\[
\text{主线程触发}
\rightarrow\text{发送快照}
\rightarrow\text{worker 规划}
\rightarrow\text{主线程验收}
\rightarrow\text{Jev 决策}
\rightarrow\text{主线程再次验收并执行}.
\]

如果任一决策过程失败，主线程将目标速度置零，并按 \(\min(15,2^e)\) 秒退避，其中 \(e\) 是连续失败次数；连续失败 3 次后关闭自动驾驶。手动驾驶不进入上述 Jev 闭环：只有打开底部的“显示候选”按钮时，主线程才至多每 500 ms 向同一个 worker 请求一次预览规划；预览结果只用于绘制候选，不提交 `/api/decide`，也不改变车辆控制。

## 0. 从 maneuver 采样到控制执行的全局流程

### 0.1 文档级统一符号

下列符号在全文保持同一语义；局部推导不得用它们表示其他对象。

| 符号 | 类型 | 全文统一语义 |
|---|---|---|
| \(\Sigma_{main}\) | 仿真状态 | 主线程在发起规划时的权威 `Simulation` 状态 |
| \(S_{msg}\) | 传输对象 | 主线程发送给 worker 的结构化克隆快照 |
| \(\Sigma_w\) | 仿真状态 | worker 将 \(S_{msg}\) 恢复到其缓存世界后、重新感知前的状态 |
| \(\Sigma_0\) | 仿真状态 | worker 恢复快照并重新感知后，用于本轮规划的状态 |
| \(\Sigma_{now}\) | 仿真状态 | Jev 响应到达时主线程的当前权威状态 |
| \(\mathcal D_0\) | 决策状态 | worker 从 \(\Sigma_0\) 生成的 `decisionState`；它是提交给本地 Jev 服务的完整 `state` |
| \(\mathcal L_0\) | 规划产物 | worker 与 \(\mathcal D_0\) 一并返回给主线程、但不提交给 Jev 服务的 `plan`；保存完整 projections |
| \(q_0\) | 车辆状态 | 本轮规划起点的 ego 状态 |
| \(\mathcal R\) | route | ego 当前全局参考路线 |
| \(\mathcal I_R\) | 节点序列 | `route.ids`，即 route 的道路图拓扑节点序列 |
| \(\mathcal P_R\) | 点列 | `route.points`，即 \(\mathcal R\) 的离散参考折线 |
| \(\mathcal C_R\) | 记录序列 | `route.crossings`，即 route 上受控 crossing 的弧长记录 |
| \(\mathcal S_R\) | 记录序列 | `route.sections`，即高速 route 的分段属性 |
| \(L_R\) | 米 | route 总弧长 `route.length` |
| \(R(s)\) | 点 | route \(\mathcal R\) 在弧长 \(s\) 处的线性插值点 |
| \(\mathcal A\) | 集合 | 局部可行驶道路面多边形集合 |
| \(\mathcal O\) | 集合 | 建筑、NPC 汽车、摩托车和行人的完整障碍物集合 |
| \(\mathcal O_{near}\) | 集合 | 进入本轮 3 秒碰撞预测范围的障碍物子集 |
| \(\Gamma\) | 上下文 | 本轮采样共享的规划上下文和谓词值 |
| \(U\) | 随机变量 | 对规划器伪随机数发生器的一次新调用，\(U\sim\mathcal U[0,1)\) |
| \(\mu\) | 枚举 | 规划模式，\(\mu\in\{normal,recovery\}\) |
| \(v_{ceil}\) | m/s | 主规划入口综合交通、route、目的地、转弯和 U 形弯约束后传给局部规划器的速度上限 |
| \(v_{max}\) | m/s | 局部规划器再应用停车接近或恢复模式规则后的本轮采样速度上限 |
| \(requiresStop\) | 布尔值 | 当前 crossing 的停车线约束是否尚待满足 |
| \(lead\) | 可空记录 | 同向前车及其保险杠净间距 |
| \(queue\) | 可空记录 | `lead` 位于 ego 前方受控 crossing 附近时形成的队列描述 |
| \(merging\) | 布尔值 | 当前 route section 是否属于 `onramp` 或 `merge` |
| \(mergeTraffic\) | 布尔值 | 合流局部 route 附近是否存在同向汽车或摩托车 |
| \(m_i\) | maneuver | 第 \(i\) 个参数化控制策略 \((\delta_i,l_i,L_i,\ell_{s,i})\) |
| \(v_i^*\) | m/s | 第 \(i\) 个候选的名义目标速度 |
| \(u_i\) | 控制候选 | 完整控制参数 \(u_i=(m_i,v_i^*)\) |
| \(\Pi_i\) | 状态序列 | 对 \(u_i\) 进行 3 秒 rollout 得到的 61 点预测轨迹 |
| \(a_i\) | 记录 | 从 \(\Pi_i\) 计算出的道路、车道、route、停车线和碰撞摘要 |
| \(\sigma_i\) | 实数 | 第 \(i\) 个候选的本地启发式评分，越小越优 |
| \(\gamma_i\) | 候选记录 | \((u_i,\Pi_i,a_i,\sigma_i)\) |
| \(N_\mu\) | 整数 | 规划模式 \(\mu\) 下原始移动候选池的长度：正常模式为 55，恢复模式为 11 |
| \(\mathcal P\) | 有序集合 | 原始移动候选池 \(\langle\gamma_0,\ldots,\gamma_{N_\mu-1}\rangle\)；不含之后单独生成的停车候选 \(\gamma_{stop}\) |
| \(\mathcal V_{move}\) | 有序集合 | 缩减后用于展示的移动候选 |
| \(\gamma_{stop}\) | 候选记录 | 缩减后单独生成的零目标速度候选 |
| \(\mathcal V^+\) | 有序集合 | \(\mathcal V_{move}\) 追加 \(\gamma_{stop}\) 后的临时集合 |
| \(\mathcal V\) | 有序集合 | 根据停止可用性保留或删除 \(\gamma_{stop}\) 后的 `state.vectors` 候选 |
| \(\mathcal E\) | 有序集合 | 最终允许 Jev 选择的合法候选集合 |
| \(\mathcal E_{move}\) | 有序集合 | \(\mathcal E\) 中目标速度非零的移动候选 |
| \(\chi_{stop}\) | 布尔值 | 当前是否允许把零速候选交给 Jev |
| \(\mathcal M\) | 集合 | 运动层选项，取自 \(\{drive,stop\}\) |
| \(X_J\) | 对象 | 发送给 Jev 的压缩场景与候选摘要状态 |
| \(\phi\) | 双射 | Jev 的短 vector ID 到本轮原候选 ID 的映射 |
| \(\mathcal Q\) | 集合 | 本轮适用的概念问题，取自 motion、vector、route |
| \(Q_J\) | 问题集合 | \(\mathcal Q\) 中真正发送给 Jev 的多选项问题 |
| \(Q_F\) | 问题集合 | \(\mathcal Q\) 中仅有一个选项、因而在本地固定回答的问题 |
| \(A_{raw}\) | 对象 | Jev 对 \(Q_J\) 返回的原始 `answers` |
| \(A_J\) | 对象 | 将 \(A_{raw}\) 与 \(Q_F\) 的固定答案合并并恢复候选原 ID 后的完整答案 |
| \(Resp_J\) | 对象 | Jev 原始响应包；无 API 调用时是本地构造的空答案、零 usage 响应 |
| \(Resp_S\) | 对象 | 本地服务验证后返回客户端的响应，包含 \(A_J\)、selection、controls 与批次信息 |
| \(c^*\) | 候选 ID | 两阶段选择合成后的最终局部候选 ID |
| \(\Psi_S\) | 布尔值 | 服务端响应验证谓词 |
| \(\Psi_C\) | 布尔值 | 客户端时效与上下文验证谓词 |

### 0.1.1 谓词表

下表汇总全文用来作布尔判断的主要谓词及其判断方式。集合筛选条件也列在相应行中；完整公式、边界条件和阶段顺序仍以引用的小节为准。

| 谓词 | 判断方法 |
|---|---|
| `onRoad(q)` | 计算车辆矩形足迹落在道路多边形外的面积比例；小于 `1e-5` 为真（§4.1）。 |
| `recovering` | 起点不在道路上，或离 route 大于 6 m，或与 route 航向差大于 1.2 rad，任一成立即为真（§4）。 |
| `requiresStop` | 非恢复模式下，当前受控 crossing 尚在适用距离内且规则有效；stop 标志未完成，或信号灯为红/黄（§5）。 |
| `queue != null` | 存在前车和受控 crossing，且 crossing 距离、前车间距满足排队阈值（§5、§6）。 |
| `merging` | 当前 route section 的类型为 `onramp` 或 `merge`（§6）。 |
| `mergeTraffic` | `merging` 成立，且附近存在汽车/摩托车：朝向大致与 route 同向，距局部 route 小于 8 m（§6）。 |
| `queueCompatible(γ)` | 候选横向偏移 `lane_offset_m` 非空且绝对值不超过 0.2 m（§7、§14）。 |
| `collisionPredicted(γ)` | 候选 3 秒 rollout 中，扫掠 OBB 检测到至少一次障碍物接触（§11）。 |
| `collisionImminent(γ)` | 有首次接触，且接触时间不超过 `max(0.75, |v₀|/8 + 0.3)` 秒（§11）。它比 `collisionPredicted` 条件更严格。 |
| `staysOnRoad(γ)` | rollout 的 31 个评估位姿中，车身道路外比例最大值小于 `1e-5`（§12）。 |
| `staysInLane(γ)` | 31 个评估位姿的最大车道越界量小于 0.12 m（§12）。 |
| `returningToLane(γ)` | 最大越界量不超过起点越界量加 0.15 m，且终点比起点更接近 route 中心（§12）。 |
| `followsRoute(γ)` | 目标速度非负、最大 route 航向差小于 `pi/2`，且终点 route 进度没有比起点倒退超过 0.1 m（§12）。 |
| `crosses_stop_line(γ)` | 起点在停车线前，且 rollout 的某个评估位姿到达或越过停车线平面（§12）。 |
| `mergeConflict` | `mergeTraffic` 成立，并且原始池中存在队列兼容、速度大于 `0.7 × v_max` 且 `collisionPredicted` 的候选（§14）。 |
| 合流慢速预过滤 | 仅在合流、非恢复、无需停车、`mergeConflict` 为假且 `v_max >= 0.15` 时，删除速度低于 `0.7 × v_max` 的候选（§14）。 |
| `forwardOnly` | 正常模式且 route phase 属于 `onramp/merge/interstate/exit/offramp`（§17）。 |
| 候选安全集 `P_safe` | 候选同时满足 `staysOnRoad`、非 `collisionImminent`，高速世界还须 `followsRoute`，并且 `staysInLane` 或 `returningToLane`（§14）。 |
| `P_eligible` | 有队列时从 `P_safe` 再筛出 `queueCompatible` 候选；无队列时等于 `P_safe`（§14）。 |
| `C₀`、`C_road`、`C_queue`、`C_in`、`C_return` | Jev 合法集合构造中的逐层筛选：先排除零速/紧迫碰撞，再按恢复/道路方向、队列兼容及车道内或回归车道条件筛选（§0.7）。 |
| `B_near`、`L_near`、`C_required`、`D_arrive`、`M_empty` | 分别判断阻塞物是否在 2.5 m 内、停车线是否临近且控制要求停车、当前停车/信号控制是否尚未满足、目的地是否在 2.5 m 内、合法移动候选是否为空（§0.6）。`C_required` 是 `L_near` 的子条件。 |
| `χ_stop` | `B_near ∨ L_near ∨ D_arrive ∨ M_empty`；为真才将零速候选加入最终展示/决策集合（§0.6）。 |
| `StateValid`、`VectorValid`、`StopFieldValid` | 服务端检查决策状态、每条候选字段及可选停车线字段的类型、范围和批次 ID 等约束（§0.10）。 |
| `WeightValid`、`AnswerValid`、`UsageValid`、`ChoiceSafe` | 分别检查概率与选项、答案结构、usage 数值，以及最终候选是否属于合法集合且可安全提交（§0.10）。 |
| `Ψ_S` | 服务端接受条件：`StateValid ∧ AnswerValid ∧ UsageValid ∧ ChoiceSafe`（§0.10）。 |
| `BatchConsistent`、`ContextUnchanged` | 分别核对响应批次/候选/控制与本地重算结果一致，以及 crossing、信号灯、停车记忆等决策上下文未改变（§0.11）。 |
| `Ψ_C` | 客户端接受条件：请求代次与 route 版本未变、仍在自动驾驶且未暂停/碰撞、响应未超时，并且 `BatchConsistent ∧ ContextUnchanged`（§0.11）。 |

候选集合谓词按阶段求值：`mergeConflict` 与合流预过滤作用于原始池；`P_safe` 用于本地代表候选筛选；最终 Jev 合法集合还会再次应用非零速度、紧迫碰撞、道路、队列和车道规则。它们的对象范围和目的不同，不能互相替代。

其中 maneuver 的分量统一为

\[
m_i=(\delta_i,l_i,L_i,\ell_{s,i}),
\]

分别表示基础转向、route 横向偏移、前视距离和可选停车线控制。完整控制候选还必须包含速度：

\[
u_i=(m_i,v_i^*).
\]

因此本文不会把 \(m_i\) 单独称为完整候选，也不会把 maneuver 与预测轨迹 \(\Pi_i\) 混为一体。

对候选记录 \(\gamma=(u,\Pi,a,\sigma)\)，下文使用字段投影简写：\(\gamma.v^*\)、\(\gamma.l\) 和 \(\gamma.\delta\) 表示从控制分量 \(u\) 读取相应值；\(staysOnRoad_\gamma\)、\(collisionImminent_\gamma\) 等表示从摘要分量 \(a\) 读取相应谓词值。它们不是 \(\gamma\) 之外的新变量。

全文中，对有序集合使用集合推导式时，结果继承原集合的相对顺序；\(\Vert\) 专门表示序列连接。因此筛选不会隐式重排候选，只有显式的 `sort` 才改变顺序。

### 0.2 总流程图

```text
主线程权威状态 Σ_main
        │  snapshot + structured clone
        ▼
worker 恢复状态并重新 scanScene
        │
        ▼
规划起点 Σ_0 = (q_0, 𝓡, 𝓐, 𝓞, traffic rules, navigation, ...)
        │
        ▼
计算共享上下文 Γ
  ├─ recovering / requiresStop
  ├─ v_ceil / v_max
  ├─ lead / queue
  └─ merging / mergeTraffic / nearby
        │
        ▼
判定规划模式 μ
        │
        ┌───────────────┴───────────────┐
        │                               │
   normal 分支                     recovery 分支
        │                               │
   采样 55 个 u_i                   采样 11 个 u_i
        │                               │
        └───────────────┬───────────────┘
                                     ▼
                     对每个 u_i 从同一 q_0 独立 rollout
                     Π_i = (q_i^0, ..., q_i^60)
                     T = 3 s, K = 60, h = 0.05 s
                                         │
                                         ▼
                         在 j = 0,2,...,60 上评估
                         a_i = Eval(Π_i, Σ_0)
                         σ_i = Score(a_i, Π_i, Σ_0)
                                         │
                                         ▼
                       原始移动候选池 𝓟 = ⟨γ_0,...,γ_(N_μ-1)⟩
                       γ_i = (u_i, Π_i, a_i, σ_i)
                                         │
                ┌──────── normal ────────┴────── recovery ────────┐
                │                                                  │
        排序 + 速度带 + 横向/转向分位                      保留全部 11 个
                │                                                  │
                └──────────────────┬───────────────────────────────┘
                                   ▼
                       移动展示集合 𝓥_move（通常 11 个）
                                   │
                                   ▼
                     单独 rollout 并评估 γ_stop（v*=0）
                                   │
                                   ▼
                  临时展示集合 𝓥⁺ = 𝓥_move ‖ ⟨γ_stop⟩（12 个）
                                   │
                                   ▼
                        计算停止可用谓词 χ_stop
                         ┌─────────┴─────────┐
                    χ_stop=true        χ_stop=false
                         │                   │
                    𝓥 = 𝓥⁺            𝓥 = 𝓥_move
                         └─────────┬─────────┘
                                   ▼
                  应用道路/车道/队列/碰撞合法集合算子
                         𝓔 = Legal(𝓥, Σ_0)
                                   │
                                   ▼
                    装配决策状态 𝓓_0 与规划产物 𝓛_0
                                   │
                                   ▼
                      服务端前置检查 StateValid(𝓓_0)
                         ┌─────────┴─────────┐
                      false                true
                        │                    │
                  拒绝并制动/重试       重新计算 𝓔 并拆分
                                      𝓔_move、γ_stop、𝓜
                                             │
                                             ▼
                   构造 Jev 输入 (X_J, Q_J)，不发送 Π_i
                    ┌──────────────┴──────────────┐
             某问题只有一个选项              某问题有多个选项
                    │                              │
          本地固定 choice, P_Q=1          调用 Jev choice 问题
                    └──────────────┬───────────────┘
                                   ▼
                 合并本地固定答案与 Jev 输出，得到 A_J
                                   │
                                   ▼
                      两阶段合成最终局部候选 c*
                      motion=drive → vector.choice
                      motion=stop  → γ_stop.id
                                   │
                                   ▼
                 服务端响应后检查 AnswerValid(A_J)
                    ∧ UsageValid(Resp_J) ∧ ChoiceSafe(c*)
                         ┌─────────┴─────────┐
                      false                true
                        │                    │
                  拒绝并制动/重试      返回 selection + controls
                                             │
                                             ▼
                            客户端检查 Ψ_C(Resp_S, Σ_now)
                              ┌──────────────┴──────────────┐
                           false                            true
                             │                               │
                   丢弃；重规划或制动退避             是否选择新全局 route？
                                                       ┌─────┴─────┐
                                                      是           否
                                                      │             │
                                               安装 route，      应用 c* 的
                                               丢弃旧 vector，   maneuver 与速度
                                               立即重新规划      下一周期再规划
```

### 0.3 从采样到离散轨迹

规划模式决定原始移动样本数：

\[
N_\mu=
\begin{cases}
55,&\mu=normal,\\
11,&\mu=recovery.
\end{cases}
\]

采样算子使用共享上下文 \(\Gamma\) 和伪随机数流产生

\[
u_i=Sample(i,\Gamma,U)
=(\delta_i,l_i,L_i,\ell_{s,i},v_i^*),
\qquad i=0,\ldots,N_\mu-1.
\]

这里

\[
\Gamma=
\{\mu,v_{ceil},v_{max},requiresStop,lead,queue,
merging,mergeTraffic,stopLine,\mathcal R,\mathcal O_{near}\}.
\]

这些值在进入采样循环前计算一次；不同 \(i\) 使用同一 \(\Gamma\)，但每次出现 \(U\) 都从规划器随机数发生器取得一个新值。

对每个候选都从同一个初态复制 ghost：

\[
q_i^0=q_0.
\]

固定

\[
T=3\text{ s},
\qquad K=60,
\qquad h=T/K=0.05\text{ s},
\]

并递推

\[
q_i^{k+1}=F(q_i^k,u_i,\Gamma,kh),
\qquad k=0,\ldots,59.
\]

其中 \(F\) 是第 10 节定义的速度、车轮转角、航向和位置更新函数。得到

\[
\Pi_i=(q_i^0,q_i^1,\ldots,q_i^{60}).
\]

正常模式先计算 55 条移动轨迹；恢复模式先计算 11 条。候选缩减后还会额外对 \(\gamma_{stop}\) 执行一次 rollout，因此实际 rollout 次数分别为

\[
56\quad(\mu=normal),
\qquad
12\quad(\mu=recovery).
\]

### 0.3.1 \(\Gamma\) 环境上下文的构造

\(\Gamma\) 不是主线程预先传入的独立对象，而是 worker 在收到快照并恢复 `Simulation` 后，于 `decisionState()` 和 `createDrivingPlan()` 中按依赖顺序计算的本轮共享上下文。其构造顺序为

\[
S_{msg}
\xrightarrow{restore}
\Sigma_w
\xrightarrow{scanScene,navigation,speedEnvelope}
(n_0,section,rule,v_{ceil})
\xrightarrow{createDrivingPlan}
\Gamma
\xrightarrow{sample\ loop}
\{u_i\}.
\]

因此，\(\Gamma\) 的所有值都在第一个 maneuver 进入采样循环前确定；同一轮中的不同 \(u_i\) 不会重新计算这些环境值。每个候选自己的道路覆盖、碰撞、车道和 route 误差，则是在候选 rollout 后单独评估的 \(a_i\)，不属于预先固定的 \(\Gamma\)。

具体各项如下。

#### (a) 当前 route \(\mathcal R\) 与投影状态 \(n_0\)

worker 从恢复后的 `player.route` 取得当前全局路线 \(\mathcal R\)，并计算

\[
n_0=N(q_0,\mathcal R)
=(x_R,z_R,\theta_R,s_R,d_R,i_R).
\]

其中 \(s_R\) 是车辆在 route 弧长轴上的当前位置，\(d_R\) 是车辆中心到 route 折线的最短二维距离，\(i_R\) 是最近 route 线段索引。ego 对象中虽然保存 `player.s`，但局部规划入口会再次通过 `nearestOnPath` 得到当前投影 `near.s`，并以此作为本轮 route 几何计算的主要进度。

#### (b) 规划模式 \(\mu\)

先计算车辆足迹是否在道路上，以及车辆相对 route 的位置和航向：

\[
recovering=
\neg onRoad(q_0)
\lor d_R>6
\lor|\operatorname{ang}(\theta_R-\theta_0)|>1.2.
\]

然后：

\[
\mu=
\begin{cases}
recovery,&recovering,\\
normal,&\neg recovering.
\end{cases}
\]

#### c 入口速度上限 \(v_{ceil}\)

`decisionState()` 先调用 `speedEnvelope(player)`。其中：

\[
v_{traffic}=
\min(v_{theme},v_{route},v_{destination},v_{follow}),
\]

然后结合 U 形弯和城市转弯上限：

\[
v_{ceil}=operatorname{round}_1
\left(\min(v_{traffic},v_U,v_{turn})\right).
\]

这里的 `planningMax` 用于 \(v_{traffic}\)；当前已选 maneuver 的实时冲突限制 `env.max` 不会直接把所有新候选的速度一起压成零。每条新候选仍会在自己的 rollout 中独立预测冲突。

#### (d) 当前 crossing、`stopLine` 与 `requiresStop`

从 route crossing 序列中选取第一个满足

\[
c.stopS-s_R>-19
\]

的 crossing。若存在，则构造停车线位姿：

\[
stopLine=
\left(R(c.stopS).x,R(c.stopS).z,c.approach\right).
\]

令 \(\Delta s=c.stopS-s_R\)。只有在非恢复模式、crossing 是 stop/signal 控制、\(-d<\Delta s<100\)、当前规则存在且 `rule.distance\ge-0.7` 时，才进一步检查停车规则：

\[
requiresStop\iff
(control=stop\land\neg stopCompleted)
\lor
(control=signal\land color\in\{red,amber\}).
\]

这里 `rule` 是 `Simulation.rule(player)` 根据当前仿真时间、信号灯、停车记忆、路口锁、礼让和 crossing 状态生成的规则记录；`rule.distance` 是

\[
rule.distance=c.stopS-player.s,
\]

即 route 弧长差，不是中心点欧式距离。若没有相关 crossing，`rule.distance=+\infty`，`stopLine=null`，停车线约束不成立。

#### (e) 局部采样上限 \(v_{max}\)

在得到 \(requiresStop\) 后，局部规划器计算：

\[
v_{max}=
\begin{cases}
2,&\mu=recovery,\\
\operatorname{round}_2(\min(v_{ceil},v_{approach})),&requiresStop,\\
\operatorname{round}_2(v_{ceil}),&\text{其他}.
\end{cases}
\]

其中 `round2` 就是 `round(value, 2)`，实现为 `Number(value.toFixed(2))`。停车接近速度以 ego 前保险杠到停车线的有符号距离为基础，并预留 0.5 m 停车间隔：

\[
r=\max(0,D_{stop}(q_0,stopLine)-0.5),
\]

\[
v_{approach}=
\max\left(0.6,
\sqrt{(4.5\cdot0.45)^2+2\cdot4.5r}-4.5\cdot0.45\right).
\]

#### (f) `lead`：同向前车

恢复模式中直接令 `lead=null`。正常模式在完整障碍物列表中的汽车和摩托车里寻找最近的同向前车。对对象 \(o\)，先转换到 ego 局部坐标：

\[
F=(o-p_0)\cdot\mathbf f(\theta_0),
\qquad
R=(o-p_0)\cdot\mathbf r(\theta_0).
\]

只有满足

\[
F>0,
\qquad
\cos(o.\theta-\theta_0)\ge0.5,
\]

并且位于 ego 前方横向车道带内的对象才可能成为 `lead`。对象沿 ego 前向的投影半长为

\[
l_o=
\frac{|\cos\Delta\theta|d_o+|\sin\Delta\theta|w_o}{2},
\qquad
\Delta\theta=o.\theta-\theta_0.
\]

ego 与对象的保险杠净间距为

\[
gap=F-\frac{d_e}{2}-l_o.
\]

横向筛选使用对象沿 ego 横向的投影半宽

\[
b_o=
\frac{|\cos\Delta\theta|w_o+|\sin\Delta\theta|d_o}{2},
\]

并要求

\[
|R|<\frac{w_e}{2}+b_o+\beta,
\]

其中 \(\beta=0.55\)（摩托车），汽车则为 \(\min(0.45,0.18+0.025|v_e|)\)。满足条件的对象中取 `gap` 最小者。这个 `gap` 不是两个中心点的距离，而是扣除双方车身投影后的纵向间距。

#### (g) `queue`：受控 crossing 前的排队状态

`queue` 只有在存在 `lead` 时才可能成立：

\[
\begin{aligned}
queue\neq null\iff{}&
lead\neq null\land crossing\neq null\\
&\land control\in\{stop,signal\}\land-3<\Delta s<90\\
&\land lead.gap<45\land lead.gap<\Delta s+8.
\end{aligned}
\]

成立时记录前车 ID、间距、前车速度和跟车目标间距。它主要用于禁止 ego 通过较大的横向偏移绕过路口队列，不直接替代候选 rollout 中的跟车速度计算。

#### (h) `merging` 与 `mergeTraffic`

当前 route section 类型为 `onramp` 或 `merge` 时：

\[
merging=true.
\]

然后从附近对象中判断是否有合流交通。首先构造 route 局部片段：

\[
\mathcal R_{local}
=\mathcal P_R[\max(0,i_R-40):i_R+180].
\]

若存在汽车或摩托车 (o) 满足

\[
\cos(\operatorname{ang}(o.\theta-\theta_R))>0
\land
d(o,\mathcal R_{local})<8,
\]

则：

\[
mergeTraffic=true.
\]

`merging` 只说明道路类型是合流段；`mergeTraffic` 还要求附近实际存在大致同向的车辆或摩托车。

#### (i) `scanScene` 的输出与 \(\mathcal O_{near}\) 的区别

`scanScene()` 的直接输出是仿真的感知镜像 `this.perception`，记为
\(\mathcal P_{sens}\)，而不是规划函数中的 \(\mathcal O_{near}\)。对每个通过筛选的对象，
它输出 `id`、`type`、相对前向距离 `ahead_m`、相对右向距离 `right_m`、速度，以及车辆相对运动状态、
信号灯颜色或行人穿越状态。筛选顺序为：

\[
\mathcal O
\xrightarrow{\text{距离范围}}
\xrightarrow{\text{静态对象 }\pm65^\circ\text{ 视角}}
\xrightarrow{\text{建筑遮挡}}
\mathcal P_{sens}.
\]

动态对象（汽车、摩托车、行人）不执行静态对象的 \(\pm65^\circ\) 视角排除；但仍受距离和建筑遮挡检查影响。
规划函数 `createDrivingPlan()` 接收完整的 `obstacles` 几何对象集合，并在内部重新计算下面定义的
\(\mathcal O_{near}\)。因此，\(\mathcal P_{sens}\) 与 \(\mathcal O_{near}\) 不是同一个集合：前者服务于感知/状态输出，后者服务于候选轨迹的前车、合流和碰撞预测。

#### (ii) \(\mathcal O_{near}\)：规划内部的附近障碍物集合

原始障碍物集合为建筑、汽车、摩托车和行人的并集。对象 (o) 被纳入 \(\mathcal O_{near}\) 的条件是：

\[
\operatorname{dist}(q_0,o)
<
\left(\max(|v_0|,v_{max})+|v_o|\right)3
+\frac{\sqrt{w_o^2+d_o^2}}2+6.
\]

其中：

\[
\operatorname{dist}(q_0,o)
=\sqrt{(x_o-x_0)^2+(z_o-z_0)^2}
=\operatorname{Math.hypot}(x_o-x_0,z_o-z_0).
\]

所以这里使用的是两个对象中心点在 (x,z) 平面上的二维欧式距离，不考虑高度、朝向或 OBB 边界之间的精确最短距离。对象车身尺寸通过包围盒半对角线项保守补偿，真正的车身碰撞仍由 rollout 后的扫掠 OBB 检测完成。

至此，\(\Gamma\) 的计算完成；随后才进入 `for (i=0; i<N_\mu; i++)` 的 maneuver/速度采样循环。\(\Gamma\) 中的 `lead`、`queue`、`mergeTraffic` 等值在该循环内被所有候选共享，但每个候选的碰撞、道路覆盖和车道状态仍由自己的 \(\Pi_i\) 独立计算。

### 0.4 轨迹评估对象与谓词

完整几何评估只取索引集合

\[
J=\{0,2,4,\ldots,60\},
\qquad |J|=31,
\]

即每隔 0.1 s 检查一次，并在相邻检查位姿间执行扫掠 OBB 碰撞检测。评估算子产生

\[
a_i=Eval(\Pi_i,\Sigma_0),
\]

其中至少包括以下谓词和数值：

\[
a_i=\{
staysOnRoad_i,
staysInLane_i,
returningToLane_i,
followsRoute_i,
queueCompatible_i,
\]

\[
collisionPredicted_i,
collisionImminent_i,
collisionTime_i,
routeError_i,
laneError_i,
routeProgress_i,
\ldots
\}.
\]

这些符号分别表示：

- \(staysOnRoad_i\)：31 个评估位姿的最大车身道路外比例小于 \(10^{-5}\)；
- \(staysInLane_i\)：31 个位姿的最大车道越界量小于 0.12 m；
- \(returningToLane_i\)：最坏越界量没有比起点增加超过 0.15 m，且终点更接近 route 中心；
- \(followsRoute_i\)：候选非倒车、最大 route 航向差小于 \(\pi/2\)，且终点进度未后退超过 0.1 m；
- \(queueCompatible_i\)：\(l_i\neq null\land |l_i|\le0.2\)；
- \(collisionPredicted_i\)：3 秒预测时域内存在扫掠 OBB 首次接触；
- \(collisionImminent_i\)：首次接触时间不超过 \(\max(0.75,|v_0|/8+0.3)\)。

严格地说，\(a_i\) 是混合评估记录，而不是纯谓词集合。除上述布尔谓词外，还包含：`steering`、`velocity_mps`、`lane_offset_m`、`lookahead_m`、`following_vehicle_id`、`end_speed_mps`、`stop_line_after_m`、`crosses_stop_line`、`stopping_distance_after_m`、`lane_error_m`、`lane_error_after_m`、`route_error_m`、`route_progress_m`、`heading_error_deg`、`offroad_fraction`、`max_offroad_fraction`、`first_offroad`、`end_position`、`on_road_after`、`road_distance_after_m`、`recovery_distance_m`、`collision_in_s` 和 `collision_object_id`。其中：

- `stays_on_road`：`max_offroad_fraction < 10^{-5}`；
- `stays_in_lane`：最大车道越界量 `<0.12 m`；
- `returning_to_lane`：最大越界量不超过起点越界量 `0.15 m` 以上，且终点更接近车道中心；
- `follows_route_direction`：速度非负、最大航向误差 `<\pi/2`，且 route 进度未后退超过 `0.1 m`；
- `collision_predicted`：预测时域内至少一次 OBB 冲突；
- `collision_imminent`：首次冲突时间不超过 `max(0.75, |v_0|/BRAKING+0.3)`。

OBB 检测将 ego 和预测对象都表示为二维有向包围盒。对双方各取车身前向轴和横向轴，共四条候选分离轴；在每条轴上投影两个 OBB，计算有符号分离量

\[
s_k=|\Delta\cdot e_k|-r_a(e_k)-r_b(e_k),
\qquad
clearance=\max_k s_k.
\]

若存在某条轴使 `s_k>0`，则两个 OBB 可分离；若四条轴上均 `s_k<=0`，则发生相交。实际预测把对象类型和相对速度形成的安全缓冲加入判定：摩托车 `0.3 m`、行人 `0.35 m`、其他对象 `0.12 m`，再加随速度增长且上限为 `0.8 m` 的动态缓冲；因此实际冲突条件是 `clearance <= buffer`。检测在每隔 `0.1 s` 的评估位姿之间执行扫掠检查，而不是只比较离散采样点。

候选记录统一为

\[
\gamma_i=(u_i,\Pi_i,a_i,\sigma_i),
\qquad
\sigma_i=Score(a_i,\Pi_i,\Sigma_0).
\]

于是原始移动候选池为

\[
\mathcal P=\langle\gamma_0,\gamma_1,\ldots,\gamma_{N_\mu-1}\rangle.
\]

\(\sigma_i\) 只用于本地候选排序与代表性缩减，不直接作为一个字段发送给 Jev。Jev 接收的是 \(a_i\) 的压缩摘要。

### 0.5 12 个展示候选如何组成

正常模式下，从 55 个 \(\gamma_i\) 中依次构成：

\[
\mathcal V_{best}\quad(\text{补足至 3 个低分或关键速度候选}),
\]

\[
\mathcal V_{lat}\quad(4\text{ 个按 }l_i\text{ 排序的分位候选}),
\]

\[
\mathcal V_{steer}\quad(4\text{ 个按 }\delta_i\text{ 排序的固定转向分位候选}).
\]

通常

\[
\mathcal V_{move}
=\mathcal V_{best}\mathbin{\Vert}
\mathcal V_{lat}\mathbin{\Vert}
\mathcal V_{steer},
\qquad |\mathcal V_{move}|=3+4+4=11,
\]

其中 \(\Vert\) 表示保持顺序的序列连接。恢复模式不缩减：

\[
\mathcal V_{move}=\mathcal P,
\qquad |\mathcal V_{move}|=11.
\]

随后单独定义零速控制

\[
u_{stop}=(m_{stop},0),
\qquad
m_{stop}=\left(
\delta_{cmd,0}\mathbin{\mathrm{or}}0,
\begin{cases}null,&\mu=recovery,\\0,&\mu=normal,\end{cases}
4.5,
null
\right),
\]

其中 \(\delta_{cmd,0}\mathbin{\mathrm{or}}0\) 精确表示实现的 `car.steering || 0`。对它执行 rollout 和评估，得到 \(\gamma_{stop}\)，再追加：

\[
\mathcal V^+
=\mathcal V_{move}\mathbin{\Vert}\langle\gamma_{stop}\rangle,
\qquad |\mathcal V^+|=12.
\]

`vectors[id]` 保存 \(a_i\) 和控制参数摘要，`projections[id]` 保存 \(\Pi_i\)。二者以相同候选 ID 关联。

### 0.6 停止可用谓词与实际展示集合

定义四个停止理由：

\[
B_{near}\iff
blockingObject\neq null
\land blockingObject.gap\le2.5,
\]

\[
L_{near}\iff
intersection\neq null
\land\neg alreadyEntered
\land -0.5\le stopLineAhead\le2.5
\land C_{required},
\]

\[
C_{required}\iff
(control=stop\land\neg stopCompleted)
\lor(control=signal\land signal\in\{red,amber\}),
\]

\[
D_{arrive}\iff destinationDistance\le2.5,
\]

\[
M_{empty}\iff \mathcal E_{move}^{pre}=\varnothing,
\]

其中 \(\mathcal E_{move}^{pre}\) 是尚未加入停止候选时由第 0.7 节合法集合算子得到的移动集合。停止可用谓词为

\[
\boxed{
\chi_{stop}=B_{near}\lor L_{near}\lor D_{arrive}\lor M_{empty}
}.
\]

实际展示集合为

\[
\mathcal V=
\begin{cases}
\mathcal V^+,&\chi_{stop}=true,\\
\mathcal V_{move},&\chi_{stop}=false.
\end{cases}
\]

当 \(\chi_{stop}=false\) 时，停止候选不仅不交给 Jev，也从 `state.vectors`、`plan.vectors` 和 `plan.projections` 删除。

### 0.7 最终 Jev 合法集合

首先定义非零且不具有紧迫碰撞的集合：

\[
\mathcal C_0=
\{\gamma\in\mathcal V
\mid \gamma.v^*\neq0
\land\neg(\gamma.collisionImminent\mathbin{??}\gamma.collisionPredicted)\}.
\]

这里的 `??` 是 JavaScript 的空值合并运算符，不是逻辑或 `||`：仅当左侧为 `null` 或 `undefined` 时才取右侧值。当前摘要总有布尔型 `collision_imminent`，所以实际等价于 \(\neg collisionImminent_\gamma\)。

若 `recovery.blocked` 或 \(v_{max}=0\)，直接令 \(\mathcal C_0=\varnothing\)。这里合法集合实现读取的 `state.speed_ceiling_mps` 正是本轮局部规划使用的 \(v_{max}\)。令

\[
forwardOnly\iff
\mu=normal
\land phase\in
\{onramp,merge,interstate,exit,offramp\}.
\]

道路过滤为

\[
\mathcal C_{road}=
\begin{cases}
\mathcal C_0,&\mu=recovery,\\
\{\gamma\in\mathcal C_0\mid
staysOnRoad_\gamma
\land(\neg forwardOnly\lor followsRoute_\gamma)\},&\mu=normal.
\end{cases}
\]

队列过滤为

\[
\mathcal C_{queue}=
\begin{cases}
\{\gamma\in\mathcal C_{road}\mid queueCompatible_\gamma\},
&queue\neq null\land\mu=normal,\\
\mathcal C_{road},&\text{其他}.
\end{cases}
\]

再定义

\[
\mathcal C_{in}=\{\gamma\in\mathcal C_{queue}\mid staysInLane_\gamma\},
\]

\[
\mathcal C_{return}=\{\gamma\in\mathcal C_{queue}\mid returningToLane_\gamma\}.
\]

最终移动合法集合是一个有优先级的集合算子：

\[
\mathcal E_{move}=
\begin{cases}
\mathcal C_{queue},&\mu=recovery,\\
\mathcal C_{in},&\mu=normal\land|\mathcal C_{in}|>0,\\
\mathcal C_{return},&\mu=normal\land|\mathcal C_{in}|=0
\land|\mathcal C_{return}|>0,\\
\varnothing,&\mu=normal\land|\mathcal C_{in}|=|\mathcal C_{return}|=0
\land forwardOnly,\\
\mathcal C_{queue},&\text{其他}.
\end{cases}
\]

最终集合为

\[
\boxed{
\mathcal E=
\mathcal E_{move}
\mathbin{\Vert}
\begin{cases}
\langle\gamma_{stop}\rangle,&\chi_{stop}=true,\\
\langle\rangle,&\chi_{stop}=false.
\end{cases}
}
\]

因此 `state.vectors` 与 `plan.vectors` 都编码 \(\mathcal V\)，`plan.projections` 以同一批 ID 保存对应轨迹，而 `plan.eligible` 编码 \(\mathcal E\subseteq\mathcal V\)。原始池 \(\mathcal P\)、展示集合 \(\mathcal V\) 与 Jev 合法集合 \(\mathcal E\) 不能混为一体。

### 0.8 Jev 输入对象

worker 返回给主线程并提交给本地决策服务的对象是

\[
\mathcal D_0=\{
batchId,routeVersion,global,drivingStyle,speed,braking,
limit,speedCeiling,speedConstraints,road,lane,traffic,recovery,
bend,destination,turn,trip?,scene,vectors,stopAvailability
\}.
\]

以上驼峰名是文档记号，对应实现中的蛇形 JSON 字段。`scene.intersection` 保存路口上下文，`vectors` 保存候选摘要。完整轨迹与便于渲染的规划信息另存于

\[
\mathcal L_0=\{batchId,vectors,projections,eligible,\ldots\},
\qquad projections[id_j]=\Pi_j.
\]

主线程只把 \(\mathcal D_0\) 提交给 `/api/decide`；\(\mathcal L_0\) 留在客户端用于显示和核对。本地服务会从 \(\mathcal D_0\) 再次执行 `candidateChoices`，而不是接收或无条件信任 \(\mathcal L_0.eligible\)。

从 \(\mathcal E\) 构造

\[
\mathcal E_{move}=\{\gamma\in\mathcal E\mid\gamma.v^*\neq0\}
\]

和停止候选 ID \(id_{stop}\)。运动层选项为

\[
\mathcal M=
\bigl(\{drive\}\text{ if }|\mathcal E_{move}|>0\bigr)
\cup
\bigl(\{stop\}\text{ if }id_{stop}\neq null\bigr).
\]

移动候选 ID 通过双射别名

\[
\phi:v_j\mapsto id_j
\]

压缩，其中 \(v_j\) 是发送给 Jev 的短别名，\(id_j\) 是 `batch_v*` 形式的原候选 ID。服务端收到答案后直接以 \(\phi(choice)\) 恢复原 ID。Jev 输入为

\[
(X_J,Q_J,Q_F,\phi)=Prepare(\mathcal D_0).
\]

\(X_J\) 包含场景摘要以及每个移动候选的速度、终点、route/车道误差、道路状态、停车线状态、恢复状态和预测冲突。它不包含完整轨迹 \(\Pi_i\)。

先构造本轮适用的概念问题集合

\[
\mathcal Q\subseteq\{Q_{motion},Q_{vector},Q_{route}\}.
\]

- \(Q_{motion}\)：在 \(\mathcal M\) 中选择 `drive` 或 `stop`；
- \(Q_{vector}\)：假设选择 `drive`，在 \(\mathcal E_{move}\) 中选择局部 maneuver；
- \(Q_{route}\)：仅在偏航后存在多个全局路线选项时，从 `keep` 和最多 3 条替代 route 中选择。

对每个 \(Q\in\mathcal Q\)，令 \(Criteria_Q\) 为该问题允许的 ID 集合。问题被划分为

\[
Q_F=\{Q\in\mathcal Q\mid |Criteria_Q|=1\},
\qquad
Q_J=\{Q\in\mathcal Q\mid |Criteria_Q|>1\}.
\]

对 \(Q\in Q_F\)，系统本地生成

\[
choice_Q=only(Criteria_Q),
\qquad P_Q(choice_Q)=1,
\]

并不把该问题发给 Jev。仅当 \(|Q_J|>0\) 时才发起 Jev API 调用；否则整次决策均在本地确定。

### 0.9 Jev 输出与两阶段局部选择

对一个发送给 Jev 的 choice 问题 \(Q\in Q_J\)，Jev 原始输出记为

\[
A_{raw,Q}=(choice_Q,P_Q),
\]

其中 \(choice_Q\) 是离散选项 ID，\(P_Q\) 是针对该问题所有合法 ID 的概率映射。将这些答案与 \(Q_F\) 的本地固定答案合并，并用 \(\phi\) 恢复 vector 原 ID，得到完整答案 \(A_J\)。Jev 模型内部如何从提示和摘要产生 \((choice_Q,P_Q)\) 不在仓库中；项目可形式化验证的是输入输出契约，而不是模型内部推理算法。

合并本地固定答案与 Jev 答案后，最终局部选择为

\[
\boxed{
c^*=\begin{cases}
choice_{vector},&choice_{motion}=drive,\\
id_{stop},&choice_{motion}=stop.
\end{cases}
}
\]

系统不对返回概率再次执行 `argmax`；最终 ID 直接使用 Jev 返回的 `choice`。为显示和记录生成组合概率：

\[
P_{final}(\gamma)=
P_{motion}(drive)P_{vector}(\gamma\mid drive),
\qquad \gamma\in\mathcal E_{move},
\]

\[
P_{final}(\gamma_{stop})=P_{motion}(stop).
\]

响应中的 `confidence` 不是 \(P_{final}(c^*)\)，而固定定义为运动层所选项的概率：

\[
confidence=P_{motion}(choice_{motion}).
\]

### 0.10 服务端检查谓词

令 \(Resp_J\) 为 Jev 原始响应。服务端只在以下合取谓词成立时接受：

\[
\Psi_S(Resp_J,\mathcal D_0,c^*)=
StateValid(\mathcal D_0)
\land AnswerValid(A_J)
\land UsageValid(Resp_J)
\land ChoiceSafe(c^*).
\]

该公式描述接受条件，不表示求值顺序：实现先计算 `StateValid`；失败时不会构造 Jev 请求，也不会调用 API。只有前置状态检查通过后才执行 `Prepare` 和可能的 Jev 调用；`AnswerValid`、`UsageValid`、route choice 检查及 `ChoiceSafe` 均在得到并展开响应后执行。

各部分语义为：

令 \(v_{obs}=\mathcal D_0.speed\)、\(v_{ceil}=\mathcal D_0.speedCeiling\)、\(B=\mathcal D_0.batchId\)；以下 `road`、`recovery`、`turn` 和 `vectors` 也都表示 \(\mathcal D_0\) 的同名字段。服务端状态检查可写为

\[
\begin{aligned}
StateValid(\mathcal D_0)\iff{}&
v_{obs},v_{ceil}\in\mathbb R_{finite}
\land 0\le v_{ceil}\le28\\
&\land B\text{ 匹配正则 }\mathtt{b[0-9]+}
\land road.onRoad\in\mathbb B\\
&\land recovery.active\in\mathbb B
\land turn\neq null\\
&\land 1\le|vectors|\le12
\land\forall(id,\gamma)\in vectors:\ VectorValid(id,\gamma)\\
&\land\bigl(\chi_{stop}\Rightarrow
\exists\gamma\in vectors:\gamma.v^*=0\bigr).
\end{aligned}
\]

其中 \(\mathbb B=\{true,false\}\)，且

\[
\begin{aligned}
VectorValid(id,\gamma)\iff{}&
id\text{ 以 }B\_\text{ 开头且只含字母、数字、下划线}\\
&\land\left(\gamma.l=null\ \lor\
  (\gamma.l\in\mathbb R_{finite}\land|\gamma.l|\le1.4
   \land2\le\gamma.L\le10)\right)\\
&\land\gamma.\delta\in\mathbb R_{finite}
\land|\gamma.\delta|\le0.85\\
&\land\gamma.v^*\in\mathbb R_{finite}
\land\gamma.v^*\ge
\begin{cases}
-2,&recovery.active,\\
0,&\neg recovery.active
\end{cases}\\
&\land|\gamma.v^*|\le v_{ceil}+0.001\\
&\land StopFieldValid(\gamma.\ell_s,\gamma.l)\\
&\land collisionPredicted_\gamma\in\mathbb B
\land staysOnRoad_\gamma\in\mathbb B\\
&\land routeError_\gamma,offroadFraction_\gamma\in\mathbb R_{finite}.
\end{aligned}
\]

这里 \(StopFieldValid(null,l)=true\)；非空时要求 \(l\) 有限，且停车线的 \(x,z,heading,clearance,deceleration\) 均有限，并满足 \(clearance=0.5\)、\(4\le deceleration\le4.8\)。

令局部加权问题集合为

\[
\mathcal Q_W=\mathcal Q\cap\{Q_{motion},Q_{vector}\}.
\]

对这些问题定义

\[
WeightValid(Q)\iff
choice_Q\in Criteria_Q
\land
\forall id\in Criteria_Q:\ P_Q(id)\in[0,1]\cap\mathbb R_{finite}.
\]

则实现的答案检查是

\[
\begin{aligned}
AnswerValid(A_J)\iff{}&
\bigwedge_{Q\in\mathcal Q_W}WeightValid(Q)\\
&\land\left(
Q_{route}\in Q_J
\Rightarrow choice_{route}\in Criteria_{route}
\right).
\end{aligned}
\]

因此 motion/vector 的每个预期概率必须有限且处于 \([0,1]\)，但实现没有验证概率之和等于 1、没有验证 `choice` 必须是最大概率项，也不拒绝概率映射中的额外键。route 问题只验证 `choice`，不验证其概率映射。

\[
UsageValid\iff
inputTokens,outputTokens\in\mathbb R_{finite}.
\]

\[
ChoiceSafe(c^*)\iff
c^*\in\mathcal E
\land
\left(
\neg\operatorname{Nullish}(collisionImminent_{c^*},collisionPredicted_{c^*})
\lor v_{c^*}^*=0
\right).
\]

服务端随后自己生成而不是信任模型提供的控制：

\[
controls=
(steering_{c^*},velocity_{c^*}).
\]

### 0.11 客户端检查谓词

令 \(t_{wall}^{start}\) 是请求开始的墙钟时间，\(t_{wall}^{now}\) 是响应处理时间；令 \(g_{req}\)、\(r_{req}\) 分别是发请求时的 generation token 和 `routeVersion`。客户端接受谓词为

\[
\begin{aligned}
\Psi_C(Resp_S,\Sigma_{now})={}&
(g_{req}=generation_{now})
\land(r_{req}=routeVersion_{now})\\
&\land autopilot
\land\neg paused
\land\neg crash\\
&\land(t_{wall}^{now}-t_{wall}^{start}\le1800\text{ ms})\\
&\land BatchConsistent
\land ContextUnchanged.
\end{aligned}
\]

`BatchConsistent` 要求：

- 响应 `batch_id` 等于规划状态的 `batch_id`；
- \(c^*\) 仍存在于该批 `state.vectors`；
- 客户端重新计算的 choice、组合概率和 confidence 与响应完全一致；
- `controls.steering` 和 `controls.velocity` 与 \(c^*\) 的摘要完全一致。

`ContextUnchanged` 重新取得当前 crossing 和交通规则，并要求：

- crossing `node_id` 没有变化；
- 若先前信号灯可见，灯色没有变化；
- `stop_completed` 没有变化；
- `already_entered` 没有变化；
- 当前 approach 的累计停车次数没有变化。

任一条件失败，旧响应不会作用于车辆，而是触发重新规划或错误退避。

### 0.12 全局 route 分支与局部 maneuver 执行

如果 \(A_J\) 同时包含

\[
choice_{route}\neq keep,
\]

系统先验证并尝试安装该全局 route。成功条件还包括重规划仍有必要、候选存在、车辆距候选生成原点不超过 12 m，并且车辆没有以超过 2 m/s 的速度处于当前停车线前后 18 m 内。安装成功后：

\[
\mathcal R\leftarrow\mathcal R_{choice},
\]

本响应中基于旧 \(\mathcal R\) 的 \(c^*\) 被丢弃，并立即重新规划。

如果不换全局 route，则应用局部候选：

\[
player.maneuver\leftarrow m_{c^*},
\]

\[
player.steering\leftarrow steering_{c^*},
\qquad
player.target\leftarrow velocity_{c^*}.
\]

车辆不会逐点播放 \(\Pi_{c^*}\)。执行期间仍在每个仿真子步根据真实状态重新计算闭环转向、停车线速度和实时安全限制；正常场景约 650 ms、弯道/路口/合流/恢复等场景约 250 ms 后再次决策。因此

\[
\Pi_{c^*}=\text{选择时的预测},
\qquad
m_{c^*}=\text{实际持续执行到下一决策的控制策略}.
\]

---

## 1. 符号与坐标约定

车辆状态记为

\[
q=(x,z,\theta,v,\delta_w,w,d),
\]

其中：

- \((x,z)\)：世界平面位置；
- \(\theta\)：航向角；
- \(v\)：有符号纵向速度；
- \(\delta_w\)：实际车轮转角 `wheelSteering`；
- \(w,d\)：车宽和车长；
- \(+x\) 与 \(+z\) 是世界坐标轴，车辆前向单位向量为

\[
\mathbf f(\theta)=(\sin\theta,-\cos\theta),
\]

右向单位向量为

\[
\mathbf r(\theta)=(\cos\theta,\sin\theta).
\]

\(\theta\) 与 \(\delta_w\) 不是同一角度：

- \(\theta\) 描述整个车身当前朝向；
- \(\delta_w\) 描述前轮相对车身纵轴的偏转；
- \(\delta_w\) 先决定路径曲率，再决定 \(\theta\) 的变化率。

二者的因果关系为

\[
\delta_w
\longrightarrow
\kappa=\frac{\tan(\delta_wQ(v))}{2.7}
\longrightarrow
\dot\theta=v\kappa.
\]

因此刚打方向时可以有 \(\delta_w\neq0\)，而车身航向 \(\theta\) 尚未明显改变。

角度归一化函数记为

\[
\operatorname{ang}(\alpha)\in[-\pi,\pi).
\]

限幅与舍入分别记为

\[
\operatorname{clip}(y,a,b)=\min(\max(y,a),b),
\qquad
\operatorname{round}_n(y)=\text{保留 }n\text{ 位小数}.
\]

限幅是分段函数

\[
\operatorname{clip}(y,a,b)=
\begin{cases}
a,&y<a,\\
y,&a\le y\le b,\\
b,&y>b.
\end{cases}
\]

它约束数值允许范围；舍入只降低数值精度，并不施加安全上下界。实现中的舍入结果仍是 JavaScript 数值，例如 `round(5, 2)` 的存储值是 `5`，不是字符串 `5.00`。

当前 route 是按弧长参数化的折线 \(R(s)\)。对任意位姿点 \(p\)，定义最近投影

\[
N(p,R)=(x_R,z_R,\theta_R,s_R,d_R,i_R),
\]

其中 \(s_R\) 是 route 弧长进度，\(d_R\) 是点到折线的最短二维距离，\(i_R\) 是最近线段索引。

每次出现 \(U\) 时，均表示对规划器伪随机数发生器的一次新调用：

\[
U\sim\mathcal U[0,1).
\]

不同公式中的 \(U\) 不是同一个随机值。

---

## 2. 主线程快照与 worker 状态重建

### 2.1 传输状态

主线程并不发送整个 `Simulation`。一次规划请求只发送

\[
S_{msg}=\{
t,player,traffic,pedestrians,perception,sensorRange,
locks,courtesy,freeExplore,complete,autopilot,routeVersion,
destinationApproach,destinationPoint,offRouteSince,lastReroute,routeHoldUntil
\}.
\]

其中 `locks` 和 `courtesy` 由映射转成键值对数组。`postMessage` 对该对象执行结构化克隆，所以 worker 获得的是独立副本，而不是主线程对象引用。

静态世界不属于 \(S_{msg}\)。worker 按

\[
K_W=epoch:type:seed
\]

缓存一个独立 `Simulation`。当 \(K_W\) 改变时重新生成世界；否则复用已有静态世界。

### 2.2 状态恢复

收到快照后，worker 执行以下状态变换：

\[
\Sigma_w\leftarrow S_{msg},
\qquad
locks\leftarrow\operatorname{Map}(locks),
\qquad
courtesy\leftarrow\operatorname{Map}(courtesy),
\]

并令

\[
world.route\leftarrow player.route.
\]

若 worker 原有 `routeVersion` 与快照版本不同，则先清空 worker 局部的路线分支记忆：

\[
routeChoices\leftarrow\varnothing,
\quad nextRouteChoices\leftarrow0,
\quad routeChoicesOrigin\leftarrow null.
\]

随后调用决策状态生成过程。该过程首先重新执行场景扫描，所以快照携带的旧 `perception` 会被基于当前快照位置重新计算的感知结果覆盖。

### 2.3 route 的形式化表示

一条 route 表示为

\[
\mathcal R=(\mathcal I_R,\mathcal P_R,\mathcal C_R,\mathcal S_R,L_R),
\]

其中普通城市 route 通常没有 \(S\)：

- \(\mathcal I_R=(n_0,\ldots,n_m)\) 对应 `ids`，是依次经过的道路图节点；
- \(\mathcal P_R=(p_0,\ldots,p_N)\) 对应 `points`，是车辆实际跟踪的二维折线；
- \(\mathcal C_R\) 对应 `crossings`，记录受控路口在 route 弧长轴上的位置；
- \(\mathcal S_R\) 对应 `sections`，记录高速 route 的道路类型、限速和车道宽；
- \(L_R\) 对应 `length`，是折线总弧长。

每个折线点为

\[
p_i=(x_i,z_i,s_i),
\qquad s_0=0,
\]

\[
s_i=s_{i-1}+\sqrt{(x_i-x_{i-1})^2+(z_i-z_{i-1})^2},
\qquad L_R=s_N.
\]

`ids` 只给出拓扑路径；真正用于最近点投影、前视目标和车辆跟踪的是 `points`。城市 route 的 `points` 默认位于行驶方向右侧车道中心，节点先沿右法向平移 3 m，并非道路几何中心线。

城市路口处先构造进入点 \(a\) 和离开点 \(b\)。左右转使用二次 Bézier 曲线

\[
B(t)=(1-t)^2a+2(1-t)tc+t^2b,
\qquad t_k=k/16, k=1,\ldots,16;
\]

U 形转弯使用半径 3 m 的半圆

\[
\phi_k=k\pi/24,
\qquad k=1,\ldots,24.
\]

两者都只把曲线离散为临时折线点，不保存连续曲线对象。随后对所有临时折线段统一重采样：若某段长度为 \(d_i\)，默认令

\[
n_i=\max(1,\lceil d_i/1\rceil),
\]

并在线段上均匀插入 \(n_i\) 个子段。因此最终 `route.points` 仍是每段不超过约 1 m 的折线；第二次采样只细分第一次曲线近似，不重新计算 Bézier 曲线或圆弧。

对弧长 \(s\in[s_i,s_{i+1})\)，route 点由线性插值得到：

\[
t=\frac{s-s_i}{s_{i+1}-s_i},
\quad
R(s)=p_i+t(p_{i+1}-p_i).
\]

每个 crossing 表示为

\[
c=(nodeId,x,z,approach,exit,stopS),
\]

其中 `stopS` 是停车线在同一弧长轴上的位置。高速 route 的每个 section 表示为

\[
(kind,name,startS,endS,speedLimit,laneHalfWidth),
\]

覆盖区间 \([startS,endS)\)。因此 route 将道路拓扑、可跟踪几何、路口位置和分段道路属性统一到同一条 \(s\) 轴上。

### 2.4 worker 的路线分支记忆

`routeChoices` 不是本章后续的局部轨迹候选，而是 ego 持续偏离当前 route 后，通向原目的地的全局重规划路线选项。仅当

\[
\neg complete\land\neg freeExplore\land\neg crash
\land offRouteSince\neq null
\]

\[
\land(t-offRouteSince\ge6)
\land(t-lastReroute\ge30)
\land(t\ge routeHoldUntil)
\land d(ego,currentRoute)>30
\]

时才需要这些分支。

其结构为

\[
routeChoices:id\mapsto
\{route,progress,distance,relativeHeading,score\}.
\]

候选按前三个 route 节点构造 ID 并去重，最多保留 3 条；Jev 看到的是

\[
\{keep\}\cup routeChoices.
\]

生成后记录

\[
routeChoicesOrigin=(x_{ego},z_{ego}),
\qquad nextRouteChoices=t+4.
\]

若尚未到刷新时间且 ego 距生成原点小于 10 m，就复用这些选项。安装某个选项前还要求 ego 距原点不超过 12 m，且不能正以超过 2 m/s 的速度处于停车线前后 18 m 内。worker 快照不包含这三个缓存字段，所以它们保存在 worker 自己的 `Simulation` 中，并在规划结果返回时同步回主线程；`routeVersion` 改变后旧分支基于旧 route，必须全部清空。

---

## 3. 规划入口的速度上限

令速度包络结果为 `env`，导航结果为 `nav`。规划使用的是

\[
v_{traffic}=env.planningMax,
\]

而不是已经受“当前所选轨迹实时冲突”进一步抑制的 `env.max`。

对 ego，`planningMax` 依次取道路主题限速、当前 route 分段限速、目的地接近速度和跟车允许速度的最小值：

\[
v_{traffic}=\min
(v_{theme},v_{route},v_{destination},v_{follow}),
\]

不适用的项视为 \(+\infty\)。其中非自由驾驶时

\[
v_{destination}
=\sqrt{2\cdot5\cdot\max(0,L_R-s-1.5)}.
\]

当前已选择轨迹上的实时冲突可以进一步降低 `env.max`，但不降低 `planningMax`；新候选会在各自 rollout 中独立预测碰撞，避免一条旧轨迹的冲突把所有新候选速度同时压低。

### 3.1 转弯上限

定义

\[
v_{turn}=
\begin{cases}
+\infty, & nav.phase\neq null,\\
7, & nav.next\_turn=right\land C_1,\\
8, & nav.next\_turn\neq right\land C_1,\\
12, & C_2,\\
v_{theme}, & \text{其他},
\end{cases}
\]

其中

\[
C_1=|nav.heading\_error\_deg|>15
\ \lor\
(nav.next\_turn\in\{left,right\}\land nav.turn\_distance\_m<24),
\]

\[
C_2=nav.next\_turn\in\{left,right\}
\land nav.turn\_distance\_m<48.
\]

注意条件按上述顺序求值；满足 \(C_1\) 时不会再进入 \(C_2\)。

### 3.2 U 形转弯上限

若 route crossing 中第一个满足

\[
\cos(\theta_{exit}-\theta_{approach})<-0.99
\quad\land\quad
s_{stop}-0.5+3\pi>s
\]

的 U 形转弯，设弯道起点距离为

\[
d_U=\max(0,s_{stop}-0.5-s),
\]

则

\[
v_U=\sqrt{3^2+2\cdot5\cdot\max(0,d_U-1.5)}.
\]

无此转弯时 \(v_U=+\infty\)。

### 3.3 route 分段限速

令当前位置所属 route section 的标称限速为 \(v_{sec}\)。初值

\[
v_{route}\leftarrow v_{sec}.
\]

如果当前 section 类型为 `onramp`，令

\[
\eta=\operatorname{clip}
\left(\frac{s-s_{start}}{s_{end}-s_{start}},0,1\right),
\qquad
v_{entry}=\min(18,v_{route}),
\]

并更新

\[
v_{route}\leftarrow v_{entry}+(v_{route}-v_{entry})\eta.
\]

随后按 route sections 的存储顺序检查每个未来 section \(j\)。仅当

\[
s_{start,j}>s
\land v_j<v_{route}
\]

时更新

\[
v_{route}\leftarrow\min\left(
v_{route},
\sqrt{v_j^2+8\max(0,s_{start,j}-s-3)}
\right).
\]

这相当于在进入更低限速区之前，以 \(4\,m/s^2\) 的运动学关系提前形成速度上限。

### 3.4 传入局部规划器的 ceiling

\[
v_{ceil}=\operatorname{round}_1
\left(\min(v_{traffic},v_U,v_{turn})\right).
\]

v_ceil（入口速度上限）是主规划器交给局部规划器的本轮速度上限，不是车辆当前速度，也不是最终选中的目标速度。局部规划器会在这个上限内生成多个候选速度。它取以下各项中的最小值：v_traffic 综合道路主题限速、当前 route 分段限速、目的地接近速度上限和跟车速度上限；v_U 是前方 U 形弯的接近速度上限，无此类弯道时不构成限制；v_turn 是根据导航转弯方向、距离和航向误差计算的转弯速度上限。结果四舍五入到 0.1 m/s 后传给局部规划器作为 `ceiling`。

其中 v_traffic 包含跟车限制，但不包含当前已选轨迹的实时冲突限速；新候选会在各自 rollout 中独立预测冲突。正常模式下，局部规划器还可能根据是否需要在路口停车，进一步降低本轮的 v_max（见 §8.6）。

---

## 4. 局部几何、恢复模式与目标点

### 4.1 道路面集合

局部道路查询半径为

\[
r_{road}=\max(100,4|v_0|).
\]

查询结果是该范围内的道路多边形集合 \(\mathcal A\)。车辆矩形足迹为 \(B(q)\)。定义车身在道路外的面积比例

\[
f_{out}(q)=
\frac{\operatorname{area}\left(B(q)\setminus\bigcup_{A\in\mathcal A}A\right)}{wd}.
\]

实现中

\[
onRoad(q)\iff f_{out}(q)<10^{-5}.
\]

#### 4.1.1 道路外面积的实际计算

这里没有调用第三方 geometry 库的 `difference()`，而是用自定义半平面裁剪计算

\[
B(q)\setminus\bigcup_{j=1}^{m}A_j.
\]

车辆足迹由四个旋转角点组成：

\[
p_{\alpha,\beta}
=(x,z)+\alpha\frac d2\mathbf f(\theta)
+\beta\frac w2\mathbf r(\theta),
\qquad \alpha,\beta\in\{-1,1\}.
\]

每个道路面 \(A_j\) 是凸多边形。算法先统一顶点方向，使内部位于每条有向边 \((a,b)\) 的左半平面：

\[
cross(a,b,p)
=(b_x-a_x)(p_z-a_z)-(b_z-a_z)(p_x-a_x)\ge-10^{-9}.
\]

对待裁剪多边形的一条边 \((p,q)\)，若其两端位于裁剪直线两侧，交点参数为

\[
t=\frac{cross(a,b,p)}{cross(a,b,p)-cross(a,b,q)},
\]

交点为 \(p+t(q-p)\)。`subtract(\Omega,A_j)` 依次处理待裁剪多边形 \(\Omega\) 和 \(A_j\) 的每条边：把当前多边形落在该边外侧的片段加入结果，只把内侧片段交给下一条边。处理完全部边后，得到 \(\Omega\setminus A_j\) 的若干多边形片段。

从

\[
R_0=\{B(q)\}
\]

开始，对所有道路面连续执行

\[
R_j=\bigcup_{\Omega\in R_{j-1}}(\Omega\setminus A_j),
\]

最终

\[
R_m=B(q)\setminus\bigcup_jA_j.
\]

每个剩余片段 \(Q=(q_0,\ldots,q_{n-1})\) 的面积使用鞋带公式：

\[
area(Q)=\frac12\left|
\sum_{i=0}^{n-1}(x_i z_{i+1}-x_{i+1}z_i)
\right|,
\qquad q_n=q_0.
\]

鞋带公式只计算裁剪结果的面积，不负责差集。最终

\[
f_{out}=\min\left(1,
\frac{\sum_{Q\in R_m}area(Q)}{wd}
\right).
\]

道路面与原始车身 AABB 不相交时先跳过；这是快速排除，最终面积仍来自旋转多边形差集。

### 4.2 恢复模式判定

令 \(n_0=N(q_0,R)\)。恢复模式谓词为

\[
recovering=
\neg onRoad(q_0)
\lor n_0.d_R>6
\lor |\operatorname{ang}(n_0.\theta_R-\theta_0)|>1.2.
\]

### 4.3 正常模式目标

正常模式的 route 跟踪目标为

\[
g=R\left(n_0.s_R+\max(5,1.2|v_0|)\right).
\]

### 4.4 恢复模式目标

恢复模式依次构造 route 候选站位偏移

\[
\Delta s\in[8,14,20,4,0,-6,28,-12,40].
\]

对每个偏移构造

\[
p_{\Delta s}=R\left(\operatorname{clip}(n_0.s_R+\Delta s,0,L_R)\right),
\]

并删除车辆放置在该位姿后不在道路上的点。

令建筑膨胀余量

\[
\rho_B=w/2+0.25.
\]

若从车辆位置到某个候选点的线段不与任何按 \(\rho_B\) 膨胀后的建筑 OBB 相交，选择候选序列中的第一个可见点作为目标。

若没有直达点，则：

1. 取距离车辆最近的至多 12 栋相关建筑；
2. 将每栋建筑膨胀后四角外再留 \(0.5\text{ m}\)，加入可见图节点；
3. 节点间直线不穿过膨胀建筑时连边，边权为欧氏距离；
4. 从车辆节点执行 Dijkstra；
5. 第一个被取出的 route 候选点作为终点，其最短路上的第一个中间节点作为 waypoint。

若不存在可达 route 候选点，则恢复目标为空，记 `recovery.blocked=true`。

这里包含三个不同层次，不能都称为最终车辆轨迹：

\[
s\text{ 偏移采样}
\longrightarrow
\text{恢复终点候选 }G,
\]

\[
\text{直达测试或可见图+Dijkstra}
\longrightarrow
(waypoint,goal),
\]

\[
11\text{ 个动力学 rollout}
\longrightarrow
\text{真正可执行的恢复轨迹候选}.
\]

其中 \(s\) 偏移只负责回答“回到当前 route 的哪个位置”；Dijkstra 只在所有有效目标都无法直达时，求绕过膨胀建筑的几何引导路径。若最短图路径为

\[
q_0\rightarrow c_1\rightarrow\cdots\rightarrow g,
\]

函数仅返回 `waypoint = c_1` 和 `goal = g`，并不把整条图路径直接作为车辆轨迹。当前实现把规划器局部变量 `goal` 设为 `recovery.waypoint`，因此候选的引导转向和恢复距离评分都针对当前 waypoint，而不是直接针对最终 route 目标 `recovery.goal`：

\[
g_{plan}=recovery.waypoint,
\qquad
S_{distance}=\|q^{60}-g_{plan}\|.
\]

`recovery.goal` 主要保留最终回归点及其 route 航向；下一轮重新规划会再次求 waypoint。真正轨迹仍由车辆动力学积分产生。该可见图只考虑建筑，不考虑动态对象，也不要求图边始终位于道路面上；这些风险由后续 rollout 的碰撞和道路评估处理。

---

## 5. 路口控制、停车线与本轮最高速度

### 5.1 当前 crossing

从 route crossings 中选择第一个满足

\[
s_{stop}-n_0.s_R>-19
\]

的 crossing。停车线位姿为

\[
\ell=(R(s_{stop}).x,R(s_{stop}).z,\theta_{approach}).
\]

令 \(\Delta s=s_{stop}-n_0.s_R\)。`-19` 表示路口在车辆越过停车线后继续作为当前 crossing 保留约 19 m：

\[
-19<\Delta s<0
\quad\Longleftrightarrow\quad
s_{stop}<n_0.s_R<s_{stop}+19.
\]

如果只使用 \(\Delta s>0\)，车辆刚越过停车线就会丢失当前路口上下文，无法在通过路口期间继续维护锁、通行规则和出口方向。源码没有给出 19 的命名常量或正式推导，它是固定启发式。按城市几何可以解释其量级：停车线约在路口中心前 10.5 m，路口半宽约 6.05 m，再加普通车辆半长约 2.1 m，合计约

\[
10.5+6.05+2.1=18.65\approx19\text{ m}.
\]

这只是与实现几何一致的解释性推断，不是源码中声明的推导。`-19` 也不表示越线后仍要求停车；必须停车使用更严格的

\[
\Delta s>-d
\]

等条件。车辆越过停车线较深后，crossing 仍可保留，但 `approachingControl` 已为假。

### 5.2 前保险杠到停车线的有符号距离

前保险杠中心为

\[
p_f=(x,z)+\frac d2\mathbf f(\theta).
\]

停车线距离定义为

\[
D_{stop}(q,\ell)
=(\ell_x-p_{f,x})\sin\ell_\theta
-(\ell_z-p_{f,z})\cos\ell_\theta.
\]

正值表示前保险杠尚未越线，零表示位于线平面，负值表示已经越线。

### 5.3 必须停车谓词

定义

\[
approachingControl=
crossing\neq null
\land controlType\in\{stop,signal\}
\land s_{stop}-n_0.s_R>-d
\land s_{stop}-n_0.s_R<100.
\]

则

\[
requiresStop=
\neg recovering
\land approachingControl
\land rule\neq null
\land rule.distance\ge -0.7
\land C_{stop},
\]

其中

\[
C_{stop}=
(controlType=stop\land\neg rule.stopCompleted)
\lor
(controlType=signal\land rule.color\in\{red,amber\}).
\]

### 5.4 停车接近速度

令

\[
r=\max(0,D_{stop}(q_0,\ell)-0.5),
\quad a=4.5,
\quad \tau=0.45.
\]

停车接近速度为

\[
v_{approach}=\max\left(0.6,
\sqrt{(a\tau)^2+2ar}-a\tau\right).
\]

局部规划使用的最高速度为

\[
v_{max}=
\begin{cases}
2, & recovering,\\
\operatorname{round}_2\left(\min(v_{ceil},v_{approach})\right),
& requiresStop,\\
\operatorname{round}_2(v_{ceil}),&\text{其他}.
\end{cases}
\]

这里的分层不是说停车约束在数学上不能与 \(v_{ceil}\) 合并。正常模式下，可把不需要停车时的 \(v_{approach}\) 定义为 \(+\infty\)，从而写成一次最小值；实现分开是因为必须先在局部规划器中完成 route 投影、选择 crossing、构造停车线并判定 `requiresStop`，才能得到 \(v_{approach}\)。

恢复模式则不是普通限速项。实现直接令

\[
v_{max}=2,
\]

而不是 \(\min(v_{ceil},2)\)。同时它还替换目标点、候选数量、前进/倒车采样、转向范围和评分函数，因此属于规划模式切换，而不是可与其他上限平铺的一项约束。

---

## 6. 前车、队列、合流交通与附近障碍物

这四项不是四种障碍物，而是从规划环境中提取的不同上下文：

| 元素 | 形式 | 主要作用 |
|---|---|---|
| `lead` | 一个前车及间距，或 `null` | 纵向跟车限速，并增加低速候选 |
| `queue` | 队列描述对象，或 `null` | 禁止通过横向偏移绕过路口排队车辆 |
| `mergeTraffic` | 布尔值 | 在合流区生成不同速度的插入方案 |
| `nearby` | 静态和动态障碍物集合 | 轨迹碰撞预测、阻挡检测和合流判断 |

其依赖关系为

\[
\mathcal O_{vehicle}\rightarrow lead,
\qquad
(lead,crossing)\rightarrow queue,
\]

\[
\mathcal O\rightarrow\mathcal O_{near},
\qquad
(\mathcal O_{near},section)\rightarrow mergeTraffic.
\]

它们都来自完整仿真对象，而不是仅来自被感知视野和建筑遮挡保留下来的对象。

### 6.1 前车

只在 `car` 和 `motorcycle` 中寻找前车。对其他车辆 \(o\)，令

\[
\Delta x=o.x-x,
\quad \Delta z=o.z-z,
\]

\[
F=\Delta x\sin\theta-\Delta z\cos\theta,
\qquad
R=\Delta x\cos\theta+\Delta z\sin\theta,
\]

\[
\Delta\theta=o.\theta-\theta.
\]

若

\[
\cos\Delta\theta<0.5,
\]

则该对象不是同向前车。

对象沿 ego 前向投影的半长为

\[
l_o=\frac{|\cos\Delta\theta|d_o+|\sin\Delta\theta|w_o}{2},
\]

横向投影的半宽为

\[
b_o=\frac{|\cos\Delta\theta|w_o+|\sin\Delta\theta|d_o}{2}.
\]

保险杠净间距为

\[
g=F-d/2-l_o.
\]

横向带宽余量

\[
\beta=
\begin{cases}
0.55,&o.type=motorcycle,\\
\min(0.45,0.18+0.025|v|),&\text{其他}.
\end{cases}
\]

前车候选条件为

\[
F>0\land |R|<w/2+b_o+\beta.
\]

所有候选中取 \(g\) 最小者为 `lead`。

### 6.2 跟车速度

期望跟车间距为

\[
g^*(q,o)=
\begin{cases}
\min(3,0.9+0.15|v|),&o.type=motorcycle,\\
\min(2.5,0.5+0.12|v|),&\text{其他}.
\end{cases}
\]

前车沿 ego 航向的非负速度分量为

\[
v_l=\max(0,v_o\cos(o.\theta-\theta)),
\]

可用净空间为

\[
r_g=g-g^*.
\]

定义

\[
B(d)=\max\left(0,
\sqrt{(5\cdot0.35)^2+2\cdot5\max(0,d)}-5\cdot0.35
\right).
\]

跟车允许速度为

\[
v_{follow}=\min\left(
B\left(r_g+\frac{v_l^2}{10}\right),
\max(0,v_l+1.5r_g)
\right).
\]

`lead` 对规划产生三项直接影响：

1. 在未被停车或合流速度分支覆盖时，前 8 个样本使用 \(0.25\sim0.55\) 倍 \(v_{max}\) 的低速范围；
2. 每个 rollout 子步都重新预测前车位置，并令该步目标速度不超过 \(v_{follow}\)；
3. 候选摘要写入 `following_vehicle_id`。

因此 `lead` 负责纵向速度关系，而不负责判断是否允许绕行。

### 6.3 队列谓词

令 \(\Delta s=s_{stop}-n_0.s_R\)。存在前车时，队列成立当且仅当

\[
queue=
lead\neq null
\land crossing\neq null
\land controlType\in\{stop,signal\}
\land -3<\Delta s<90
\land g<45
\land g<\Delta s+8.
\]

等价地：

\[
queue\iff
lead\neq null
\land crossing\neq null
\land controlType\in\{stop,signal\}
\land -3<\Delta s<90
\land g<\min(45,\Delta s+8).
\]

它不是直接检查“前车自己的 crossing”。实现没有验证

\[
crossingFor(lead)=crossingFor(ego),
\]

也不要求前车静止、完成停车或持有路口锁。它使用的是几何启发式：ego 有同向前车、ego 前方有受控路口，并且前车位置与该路口足够接近。条件 \(g<\Delta s+8\) 允许前车位于停车线前，或者粗略位于停车线后不超过约 8 m；由于 \(g\) 是保险杠投影间距而 \(\Delta s\) 是 route 弧长差，这只是近似关系。

队列成立后，候选必须满足

\[
queueCompatible(\gamma)\iff
\gamma.l\neq null\land|\gamma.l|\le0.2
\]

才能进入队列下的安全优选集合和最终 Jev 移动候选集合。因此 `lead` 负责减速跟车，`queue` 负责禁止横向绕过；队列谓词本身不直接施加新的制动公式。

### 6.4 附近障碍物集合

原始障碍物集合包含全部建筑、NPC 车辆、摩托车和行人，不使用遮挡后的感知子集。仅保留满足

\[
\operatorname{dist}(q_0,o)<
(\max(|v_0|,v_{max})+|v_o|)\cdot3
+\frac{\sqrt{w_o^2+d_o^2}}2+6
\]

的对象，形成 \(\mathcal O_{near}\)。缺失速度按 0，缺失宽/长按 1 处理。

\(\mathcal O_{near}\) 可包含 `building`、`car`、`motorcycle` 和 `pedestrian`。它用于：

- 为每条 3 秒候选预测障碍物运动并执行扫掠 OBB 碰撞检测；
- 生成 `collision_predicted`、`collision_imminent`、碰撞时间和对象 ID；
- 搜索前方至多 2.5 m 的 `blockingObject`，决定紧急停止候选是否可用；
- 在 `onramp` 或 `merge` 中判断是否存在合流交通。

`lead` 在构造 `nearby` 之前从全部车辆和摩托车中计算；因此前车跟随对象不要求先通过 `nearby` 距离筛选。

### 6.5 合流交通

当前 route section 类型属于 `onramp` 或 `merge` 时，记 `merging=true`。

取 route 局部切片

\[
R_{local}=R[i_0-40:i_0+180].
\]

若 \(\mathcal O_{near}\) 中至少存在一个车辆或摩托车满足

\[
\cos(\operatorname{ang}(o.\theta-n_0.\theta_R))>0
\land d(o,R_{local})<8,
\]

则 `mergeTraffic=true`。

`mergeTraffic` 本身不命令车辆减速，而是扩充速度方案：索引 \(0\ldots4\) 优先采样 \(0.30\sim0.55\) 倍 \(v_{max}\)，索引 \(5\ldots9\) 通常采样 \(0.25\sim0.55\) 倍 \(v_{max}\)。随后定义

\[
mergeConflict=mergeTraffic
\land\exists\gamma\in\mathcal P:
queueCompatible(\gamma)
\land \gamma.v^*>0.7v_{max}
\land \gamma.collisionPredicted.
\]

如果只是附近有合流车辆，但高速居中候选并不冲突，则删除所有

\[
\gamma.v^*<0.7v_{max}
\]

的慢速候选，避免在清晰匝道上无故爬行；只有实际 rollout 显示高速合流冲突时，才保留低、中、高速插入方案。

---

## 7. 控制参数化

每个候选的控制参数记为

\[
m=(\delta,l,L,\ell_s),
\]

其中：

- \(\delta\)：基础转向命令；
- \(l\)：相对 route 中心线的目标横向偏移；
- \(L\)：前视距离；
- \(\ell_s\)：可选的停车线控制参数。

候选还具有目标速度 \\(v^*\)。

### 7.1 maneuver 与预测轨迹的区别

\(m\) 是代码中 `maneuver` 对象的形式化表示：

\[
m=
(steering,lane\_offset\_m,lookahead\_m,stop\_at\_line).
\]

它描述的是参数化控制策略，不是轨迹本身。包含速度后的完整候选控制可记为

\[
u=(m,v^*)=(\delta,l,L,\ell_s,v^*).
\]

给定初始状态和环境后，动力学 rollout 才产生轨迹：

\[
\Pi=Rollout(q_0,m,v^*,\Gamma)
=(q^0,q^1,\ldots,q^{60}).
\]

所以同一个 \(m\) 在不同初始速度、位置、route 或障碍物环境中可以产生不同轨迹。它也不是预先存好的转向序列 \((\delta^0,\ldots,\delta^{59})\)：当 \(l\) 有限时，每一步都根据新状态重新计算转向；只有 \(l=null\) 时才退化为固定转向策略。

### 7.2 横向偏移与前视距离

横向偏移 \(l\) 可以理解为对当前 route 行驶中心线的连续微调。原始 route 参考线是 \(R(s)\)，对应的偏移参考线近似为

\[
R_l(s)=R(s)+l\mathbf r(\theta_R(s)).
\]

因此

\[
l<0\Rightarrow\text{向 route 左侧微调},
\quad
l=0\Rightarrow\text{跟踪原 route},
\quad
l>0\Rightarrow\text{向 route 右侧微调}.
\]

它不是换车道指令，也不是把车辆位置瞬间平移 \(l\) 米，而是定义车辆逐渐接近并持续跟踪的横向目标。它的用途包括：

- 精确保持 route 参考线；
- 在同一车道或道路范围内生成左右不同的避障轨迹；
- 从当前横向偏离位置返回 route 中心；
- 为候选池提供可分别做碰撞、道路和车道评估的横向多样性。

候选是否真的可用仍由 `stays_in_lane`、`returning_to_lane`、`stays_on_road` 和碰撞谓词决定。在队列中还要求

\[
|l|\le0.2,
\]

防止通过横向偏移绕过前车。

\(L\) 决定控制器沿 route 向前观察多少米。对当前状态 \(q\)，先投影得到 \(n=N(q,R)\)，再取

\[
c=R(n.s_R+L').
\]

其中一般有 \(L'=L\)，U 形转弯时按下一节缩短。然后利用 \(l\) 构造二维追踪目标

\[
g_l=c+l\mathbf r(c.\theta)
=
(c_x+l\cos c.\theta,\;c_z+l\sin c.\theta).
\]

因此可以把 \((L,l)\) 理解为 route 局部坐标中的“纵向前视距离、横向中心线偏移”：

\[
q\xrightarrow{\text{投影}}n.s_R
\xrightarrow{\text{前进 }L'}c
\xrightarrow{\text{横移 }l}g_l.
\]

较小的 \(L\) 使目标更近、转向响应更强；较大的 \(L\) 使目标更远、转向更平缓。\(L\) 的单位是米，不是 3 秒 rollout 的时间时域。

特别注意：

\[
l=0
\Rightarrow
\text{启用 route 闭环跟踪},
\]

而

\[
l=null
\Rightarrow
\text{关闭 route 闭环，直接使用固定 }\delta.
\]

### 7.3 route 跟踪转向

若 \(l\) 不是有限数，则

\[
\delta_m(q)=\delta.
\]

否则，令 \(n=N(q,R)\)。如存在 U 形转弯：

\[
L'=\min(L,\max(2.6,d_U));
\]

否则 \(L'=L\)。取中心目标

\[
c=R(n.s_R+L'),
\]

并沿 route 右法向移动 \(l\)：

\[
g_l=c+l\mathbf r(c.\theta).
\]

纯追踪曲率为

\[
\kappa=\frac{2\sin(\operatorname{ang}(heading(q,g_l)-\theta))}
{\max(2,\operatorname{dist}(q,g_l))}.
\]

速度相关的转向缩放为

\[
Q(v)=0.58+0.37\left(1-
\operatorname{clip}\left(\frac{|v|-3}{5},0,1\right)\right).
\]

曲率到转向命令的逆变换为

\[
\delta_m(q)=
\operatorname{clip}\left(
\frac{\arctan(2.7\kappa)}{Q(v)},-0.85,0.85
\right).
\]

在 rollout 的第 \(k\) 个 0.05 s 子步中，依次重新计算

\[
n^k\rightarrow c^k\rightarrow g_l^k
\rightarrow\kappa^k\rightarrow\delta_m(q^k).
\]

真实车辆执行被选 maneuver 时也在每个仿真时间步使用同一控制器。因此 \(l,L\) 共同定义的是离散时间闭环转向，而不是一次性计算后保持不变的转向角。实际车轮转角还以最大 \(1.8\,rad/s\) 追踪这个动态命令。

### 7.4 初始参考转向与采样范围

用于引导采样的目标 \(g\) 来自第 4 节。参考曲率为

\[
\kappa_g=
\frac{2\sin(\operatorname{ang}(heading(q_0,g)-\theta_0))}
{\max(4,\operatorname{dist}(q_0,g))},
\]

参考转向为

\[
\delta_g=\operatorname{steeringForCurvature}(\kappa_g,v_0).
\]

采样转向范围为

\[
\delta_{lim}=
\begin{cases}
0.85,&recovering,\\
0.85\min\left(1,\dfrac9{\max(5,|v_0|)}\right),&\text{正常模式}.
\end{cases}
\]

---

## 8. 原始候选池的精确采样

### 8.1 样本数

\[
N_\mu=
\begin{cases}
11,&\mu=recovery,\\
55,&\mu=normal.
\end{cases}
\]

因此 `N_mu` 不是一个跨模式固定值：正常模式确实固定为 55，恢复模式为 11。它表示 `pool` 在候选采样阶段的长度；后续过滤可能缩短候选池，而缩减展示和合法性检查又是不同阶段。索引从 `i=0` 到 `i=N_mu-1`；额外的零速候选 `gamma_stop` 在候选池处理后单独生成，不计入 `N_mu`。

### 8.2 转向样本

恢复模式：

\[
\delta_i=-\delta_{lim}
+2\delta_{lim}\frac{i+U}{11}.
\]

正常模式：

\[
\delta_i=
\begin{cases}
(2U-1)\delta_{lim},&i\bmod3=0,\\
\operatorname{clip}\left(
\delta_g+(2U-1)\max(0.015,0.25\delta_{lim}),
-\delta_{lim},\delta_{lim}\right),&\text{其他}.
\end{cases}
\]

这里 i 是候选索引（从 0 开始），U 是每次采样得到的 [0,1) 随机数；δ_lim 是转向采样范围的半宽，整体范围为 [-δ_lim, δ_lim]。δ_g 是由 route 前方跟踪目标计算出的参考转向。

恢复模式将该范围分成 11 个区间，索引 i 选定区间，U 决定样本在区间内的位置。因此 11 个样本分布在左右两个方向，覆盖整个转向范围，并带有少量随机变化。

正常模式混合宽范围探索和参考值附近的细采样：当 i mod 3 = 0 时，样本在整个转向范围内随机选取；其他索引围绕 δ_g 在两侧各取最多 max(0.015, 0.25δ_lim) 的随机偏差，若超出整体范围则截回边界。这样大约每三个候选有一个宽范围转向样本，其余候选集中在路线跟踪参考转向附近。这里的 δ_i 是候选轨迹模拟所用的转向命令；候选还要经过模拟、评分和筛选，最终不一定会被采用。

### 8.3 横向偏移

\[
l_i=
\begin{cases}
null,&recovering\lor i\ge44,\\
\operatorname{round}_3((2U-1)\cdot0.1),&i<14,\\
\operatorname{round}_3((2U-1)\cdot0.65),&14\le i<30,\\
\operatorname{round}_3((2U-1)\cdot1.35),&30\le i<44.
\end{cases}
\]

前 44 个正常候选由 route 跟踪控制器闭环转向；后 11 个候选保持采样得到的固定基础转向。

这里 l_i 是相对 route 中心线的横向偏移，单位为米；正负号表示中心线两侧。正常模式前 44 个候选按索引分三档，在中心线附近随机取偏移：i<14 时范围为 [-0.1, 0.1] m，14≤i<30 时为 [-0.65, 0.65] m，30≤i<44 时为 [-1.35, 1.35] m。每次在对应范围内均匀随机采样，并四舍五入到 0.001 m。偏移通过闭环 route 跟踪控制器实现，因此这三档分别提供窄、中、宽的横向选择。

恢复模式以及正常模式中 i≥44 的候选令 l_i=null，表示不指定横向偏移目标；它们不走上述 route 跟踪转向分支，而使用采样得到的固定基础转向。这里的 `null` 不是偏移量为 0 的另一种写法。

### 8.4 前视距离

恢复模式令 \(L_i=null\)。正常模式令

\[
L_i=\operatorname{round}_2\left(
\operatorname{clip}(3.5+0.36\max(v_0,v_{max})+0.8U,4,10)
\right).
\]

这里实现直接使用 \(\max(v_0,v_{max})\)，没有对 \(v_0\) 取绝对值。

正常模式下，L_i 是 route 闭环转向控制器的前视距离，单位为米：控制器从车辆当前投影位置沿 route 向前取 L_i 米处作为参考点，再结合候选横向偏移计算转向。U 在 [0,1) 内随机变化，使每个候选的前视距离略有不同；速度项越大，前视距离通常越长，转向会参考更远处的路线。公式最后将结果限制在 4–10 m，并四舍五入到 0.01 m。

若前方存在 U 形弯，实际用于取参考点的距离还会缩短为 min(L_i, max(2.6, d_U))，其中 d_U 是到弯道的距离，避免前视点越过近处的急弯。恢复模式令 L_i=null；此时不使用 route 闭环前视转向，而使用候选的固定基础转向。

### 8.5 停车线控制

仅当

\[
requiresStop\land i<8
\]

时，候选携带

\[
\ell_{s,i}=\{
x=\ell_x,z=\ell_z,\theta=\ell_\theta,node\_id,
c=0.5,a_i=\operatorname{round}_2(4+0.8U)
\}.
\]

其余候选 \(\ell_{s,i}=null\)。这里 \(c\) 是停车线前保留距离，\(a_i\) 是候选制动参数。

也就是说，只有规划器判定当前必须在 stop/signal crossing 停车时，才给前 8 个候选附加停车线控制。记录中的 x、z、θ 和 `node_id` 标识停车线的位置、朝向和对应路口；c=0.5 m 要求车辆在停车线前保留 0.5 m。每个候选独立采样 a_i∈[4,4.8) m/s²，作为该候选的减速参数。

rollout 的每个积分步都会根据车辆到停车线的剩余距离计算停车速度上限：先扣除 0.5 m 保留距离，再结合减速参数限制速度；越接近停车线，上限越低，接近目标位置时降为 0。这个字段是候选轨迹中的速度约束，目的是让候选在停车线前减速停车；它不是在生成候选时立即把车辆速度设为零。没有附加该字段的其他候选不受这项停车线速度约束。

### 8.6 目标速度样本

这里的 v_max 是局部规划器本轮的目标速度采样上限，对应代码中的 `maxSpeed`，不是车辆当前速度。正常模式下，它取入口速度上限 v_ceil 与停车接近速度上限（仅在当前必须停车时适用）中的较小值，并四舍五入到 0.01 m/s：

\[
v_{max}=\operatorname{round}_2\left(\min\left(v_{ceil},v_{approach}\right)\right),
\]

若当前不需要停车，则不施加停车接近上限，所以 v_max = v_ceil。v_ceil 已综合道路/route 限速、目的地与跟车限制、转弯和 U 形弯限制（见 §3）；若必须在 stop 或 signal crossing 停车，则停车接近速度上限根据到停车线的剩余距离计算，用来限制接近停车线时的候选速度。恢复模式不使用上述正常模式计算，直接令 v_max = 2 m/s。

首先，若

\[
v_{max}<0.15,
\]

则对所有 \(i\) 均令

\[
v_i^*=0.
\]

否则，恢复模式使用

\[
v_i^*=(-1)^i v_{max}(0.6+0.4U).
\]

因此偶数索引为前进，奇数索引为倒车。

正常模式按以下有序分支选择第一条成立的规则：

\[
v_i^*=v_{max}\times
\begin{cases}
0.9+0.1U,&requiresStop\land i<8,\\
0.3+0.25U,&mergeTraffic\land i<5,\\
0.25+0.3U,&((requiresStop\lor mergeTraffic)\land i<10)\\
&\quad\lor(lead\neq null\land i<8),\\
0.95+0.05U,&merging,\\
0.94+0.06U,&i\bmod5\neq0,\\
0.78+0.12U,&i\bmod5=0.
\end{cases}
\]

这里按公式从上到下判断；一旦条件成立，就使用该行的速度比例，不再检查后续分支。每个比例再乘以 v_max，所以结果是候选目标速度 v_i*，而不是车辆当前速度。U 属于 [0,1)，让速度在该行给出的比例区间内随机变化。

前三个分支为特定情况留出速度样本：若需要在停车线停车，索引 i<8 的候选仍以 0.9 到 1.0 倍 v_max 行驶，并由单独的停车线约束引导其接近停车；否则，若合流交通冲突成立，i<5 的候选取 0.30 到 0.55 倍上限；再否则，停车/合流相关候选（i<10），或存在前车时的早期候选（i<8），取 0.25 到 0.55 倍上限，提供较慢的跟随或让行选项。

若未命中上述分支，合流路段的候选取 0.95 到 1.00 倍上限。其余情况按索引每 5 个一组交错采样：i mod 5 不等于 0 时取 0.94 到 1.00 倍，i mod 5 等于 0 时取 0.78 到 0.90 倍。这样普通行驶也会同时有接近上限的候选和较慢的候选可供后续比较。


---

## 9. 候选目标速度的逐步约束

本节说明速度上限从规划入口到 rollout 每个积分步的传递。各个速度符号并不相同：\(v_{ceil}\) 是传给局部规划器的入口上限，\(v_{max}\) 是本轮候选速度采样上限，\(v_i^*\) 是候选 \(i\) 的名义目标速度，\(v_{target}(q,t)\) 是 rollout 在预测状态 \(q\)、时刻 \(t\) 实际采用的目标速度。车辆当时的速度 \(v(q)\) 是动力学状态，受加减速度限制，不会瞬间变成目标速度（§10）。

### 9.1 入口上限与候选速度

主规划器先计算：

\[
v_{traffic}=\min(v_{theme},v_{route},v_{destination},v_{follow}),
\qquad
v_{ceil}=\operatorname{round}_1\left(\min(v_{traffic},v_U,v_{turn})\right).
\]

各上限及其原始输入如下，完整定义见 §3：

| 上限 | 来源及原始输入 |
|---|---|
| \(v_{theme}\) | 道路主题标称限速，来自 \(\texttt{world.theme.limit}\)。 |
| \(v_{route}\) | 当前 route section 限速及前方较低限速 section 的提前减速上限。原始量是 route 进度 \(s\)、section 起止进度和标称速度；onramp 还按 section 内进度从入口速度向 section 限速插值。 |
| \(v_{destination}\) | 非自由驾驶时，\(v_{destination}=\sqrt{2\cdot5\cdot\max(0,L_R-s-1.5)}\)。原始量是路线总长 \(L_R\)、当前进度 \(s\)、1.5 m 余量和 \(5\,\mathrm{m/s^2}\) 减速度；自由驾驶时该项为 \(+\infty\)。 |
| \(v_{follow}\) | 有同向前车时的跟车上限，无前车时为 \(+\infty\)。依赖前车速度和朝向、保险杠净间距、ego 速度、前车类型对应的期望跟车间距、制动减速度和反应时间；具体见 §6 和 9.2。 |
| \(v_U\) | 前方有符合条件的 U 形弯时的接近上限，无此类弯时为 \(+\infty\)。依赖 crossing 进出朝向、停车线进度 \(s_{stop}\) 和当前 route 进度 \(s\)；见 §3.2。 |
| \(v_{turn}\) | 导航转弯上限。依赖导航 phase、航向误差、下一转向类型、转弯距离和道路主题限速；按条件取 7、8 或 12 m/s，否则取道路主题限速；导航已有 phase 时不施加该启发式上限，见 §3.1。 |

\(v_{ceil}\) 四舍五入到 0.1 m/s 后传给局部规划器。\(v_{traffic}\) 使用 \(\texttt{planningMax}\)，包含道路/route、目的地和跟车限制，但不包含当前已选轨迹的实时冲突减速；各新候选分别预测自身碰撞，不共用旧轨迹的冲突速度上限。

局部规划器按模式和停车规则计算 \(v_{max}\)：

\[
v_{max}=
\begin{cases}
2\ \mathrm{m/s},&recovering,\\
\operatorname{round}_2(\min(v_{ceil},v_{approach})),&\neg recovering\land requiresStop,\\
\operatorname{round}_2(v_{ceil}),&\neg recovering\land\neg requiresStop.
\end{cases}
\]

正常模式下，停车接近上限来自本轮起点到停车线的距离：

\[
r_0=\max(0,D_{stop}(q_0,stopLine)-0.5),
\qquad
v_{approach}=
\max\left(0.6,\sqrt{(4.5\cdot0.45)^2+2\cdot4.5r_0}-4.5\cdot0.45\right).
\]

其原始量是 ego 前保险杠到停车线的有符号距离 \(D_{stop}\)、0.5 m 停车余量、\(4.5\,\mathrm{m/s^2}\) 减速度和 0.45 s 反应余量。\(requiresStop\) 由当前 crossing 类型、交通规则、停车记忆或信号灯状态等决定，完整判定见 §5。

若 \(v_{max}<0.15\,\mathrm{m/s}\)，所有候选的名义目标速度 \(v_i^*\) 都设为 0。否则，\(v_i^*\) 按 §8.6 的有序分支和候选索引 \(i\)，从 \(v_{max}\) 的不同比例采样；恢复模式还用索引决定速度正负。因此 \(v_i^*\) 是带方向的候选目标速度，不是当前车辆速度。采样分支中的 \(requiresStop\)、\(mergeTraffic\)、\(merging\)、\(\texttt{lead}\)、索引和随机数 \(U\) 决定候选落在哪个速度档；它们不是 rollout 中逐步重算的限速。

### 9.2 rollout 每步的速度约束

每个 rollout 子步都根据预测状态 \(q\) 重新计算约束。首先判断：

\[
v_i^*\le0\quad\lor\quad l_i\text{ 不是有限数}.
\]

满足任一条件时，route/U 形弯/停车线控制器直接返回 \(v_i^*\)。这涵盖倒车或零速候选，以及 \(l_i=null\) 的恢复候选和固定基础转向候选。否则，候选属于正向且指定横向偏移的 route 闭环候选，先计算：

\[
\bar v(q)=\min\left(v_i^*,v_{route}(q),v_U(q)\right).
\]

\(v_{route}(q)\) 与入口处的 route 限速使用同一组原始量，但按预测车辆当前 route 进度 \(s(q)\) 重算：当前 section 限速、onramp 进度，以及未来较低限速 section 的起点和标称速度。\(v_U(q)\) 也按 \(s(q)\) 重算。若存在符合 §3.2 条件的 U 形弯：

\[
d_U(q)=\max(0,s_{stop}-0.5-s(q)),
\qquad
v_U(q)=\sqrt{3^2+2\cdot5\cdot\max(0,d_U(q)-1.5)}.
\]

这里使用停车线进度、预测 route 进度、0.5 m 起点余量、3 m/s 弯道速度、\(5\,\mathrm{m/s^2}\) 减速度和 1.5 m 余量；没有符合条件的 U 形弯时 \(v_U(q)=+\infty\)。车辆越接近较低限速区或弯道，这些上限就可能越低。

若候选带有 \(\texttt{stop\_at\_line}\)，再施加停车线约束。该字段只附加给 \(requiresStop\land i<8\) 的候选。原始量是预测前保险杠到停车线位姿 \(\ell\) 的有符号距离 \(D_{stop}(q,\ell)\)、停车余量 \(c=0.5\,\mathrm m\)，以及候选独立采样的减速度 \(a_i\in[4,4.8)\,\mathrm{m/s^2}\)：

\[
r_s=\max(0,D_{stop}(q,\ell)-c),
\qquad
v_s=\min\left(\sqrt{2a_i r_s},2.5r_s\right).
\]

若 \(v_s<0.05\,\mathrm{m/s}\)，实现将其置为 0，再令

\[
\bar v(q)=\min(\bar v(q),v_s).
\]

\(r_s\) 是扣除停车余量后的可用距离；平方根项按剩余距离和减速度限制停车速度，\(2.5r_s\) 则进一步限制接近停车线时的速度。没有 \(\texttt{stop\_at\_line}\) 的候选不受此项约束。

最后，若本轮起点找到了 \(\texttt{lead}\)，rollout 外层每一步都会基于预测前车位置 \(o_l(t)\) 重算跟车上限，并与已有上限取最小值：

\[
v_{target}(q,t)=\min\left(\bar v(q),v_{follow}(q,t)\right).
\]

若没有初始 \(\texttt{lead}\)，\(v_{follow}=+\infty\)，不限制候选。动态跟车上限使用以下原始量：

- \(gap\)：预测 ego 与预测前车的保险杠净间距。
- \(v_l=\max(0,v_{lead}\cos(\theta_{lead}-\theta_{ego}))\)：前车速度在 ego 朝向上的非负投影。
- \(g_{follow}\)：期望跟车间距，按 ego 当前速度绝对值及前车类型计算。摩托车为 \(\min(3,0.9+0.15|v_{ego}|)\,\mathrm m\)；其他车辆为 \(\min(2.5,0.5+0.12|v_{ego}|)\,\mathrm m\)。
- \(room=gap-g_{follow}\)：超过期望跟车间距的可用空间。

跟车上限取制动距离上限和温和跟随上限中的较小者：

\[
v_{follow}(q,t)=
\min\left(
brakingSpeed\left(room+\frac{v_l^2}{10}\right),
\max(0,v_l+1.5\,room)
\right),
\]

其中

\[
brakingSpeed(d)=
\max\left(0,\sqrt{(5\cdot0.35)^2+2\cdot5\cdot\max(0,d)}-5\cdot0.35\right).
\]

因此这里还使用 \(5\,\mathrm{m/s^2}\) 制动减速度、0.35 s 反应时间、前车速度停止距离项 \(v_l^2/10\)，以及 1.5 的间距反馈系数。恢复模式不寻找 \(\texttt{lead}\)；正常模式即使某候选因倒车或 \(l_i=null\) 跳过 route/U 形弯/停车线约束，只要规划起点存在 \(\texttt{lead}\)，外层跟车限制仍会生效。

速度约束链概括如下：

\[
v_i^*
\xrightarrow[\ v_i^*>0,\ l_i\text{ 有限}\ ]{\text{route 与 U 形弯}}
\bar v(q)
\xrightarrow[\ \text{若有 }\texttt{stop\_at\_line}\ ]{\text{停车线}}
\xrightarrow[\ \text{若有初始 }\texttt{lead}\ ]{\text{动态跟车}}
v_{target}(q,t).
\]

\(v_{target}\) 是传给本步车辆动力学的目标速度；车辆实际速度通过有限加速度/制动逐步追踪它（§10）。碰撞预测会影响候选评估和筛选，但这里没有把它作为对所有候选共同生效的速度上限。

---

## 10. 轨迹 rollout 的离散动力学

### 10.1 时间离散

预测时域与步数固定为

\[
T=3\text{ s},\qquad K=60,\qquad h=T/K=0.05\text{ s}.
\]

初始状态为真实快照车辆状态的浅拷贝：

\[
q_i^0=q_0.
\]

轨迹包含初始点和 60 个积分后点：

\[
\Pi_i=(q_i^0,q_i^1,\ldots,q_i^{60}),
\]

共 61 个状态。

### 10.2 每一步的更新顺序

对 \(k=0,\ldots,59\)：

1. 由当前预测状态计算转向命令

\[
\delta_c^k=\delta_m(q_i^k).
\]

2. 计算本步目标速度 \(v_t^k=v_{target}(q_i^k,kh)\)。

3. 以最大制动 \(8\,m/s^2\) 和最大加速 \(5\,m/s^2\) 更新速度

\[
v^{k+1}=v^k+
\operatorname{clip}(v_t^k-v^k,-8h,5h).
\]

4. 车轮转角以最大 \(1.8\,rad/s\) 追踪命令

\[
\delta_w^{k+1}=\delta_w^k+
\operatorname{clip}(\delta_c^k-\delta_w^k,-1.8h,1.8h).
\]

若初态缺少 `wheelSteering`，则依次回退到 `steering` 和 0。

5. 若 \(|v^{k+1}|<0.01\)，先将其置为 0。

6. 计算曲率

\[
\kappa^{k+1}=\frac{\tan(\delta_w^{k+1}Q(v^{k+1}))}{2.7}.
\]

7. 先更新航向

\[
\theta^{k+1}=\operatorname{ang}
(\theta^k+v^{k+1}\kappa^{k+1}h).
\]

8. 使用更新后的航向更新位置

\[
x^{k+1}=x^k+\sin\theta^{k+1}\,v^{k+1}h,
\]

\[
z^{k+1}=z^k-\cos\theta^{k+1}\,v^{k+1}h.
\]

第 24 次积分完成后的状态，即 \(q_i^{24}\)，另存为短期评估状态：

\[
q_i^{eval}=q_i^{24},
\]

对应预测时间 \(1.2\,s\)。

---

## 11. 障碍物运动预测与连续碰撞检测

### 11.1 一般障碍物预测

- 建筑物：位姿保持不变；
- 有 route、弧长 \(s_o\) 的车辆：按

\[
s_o(t)=s_o+\max(0,v_o)t
\]

沿 route 前进；超过 route 终点后沿末端航向继续直线运动；
- 普通动态对象：按恒速直线模型

\[
x_o(t)=x_o+\sin\theta_o\,v_ot,
\quad
z_o(t)=z_o-\cos\theta_o\,v_ot;
\]
- crossing 行人：沿当前 crossing 航向运动，但进度最多到 16 m；
- 有 `walkPath` 的行人：按路径长度的 \(2L\) 周期执行往返运动。

### 11.2 ego 后方跟随车辆

被判定为 ego 后方跟随者的车辆不使用从 \(t=0\) 开始的恒速外推，而按相邻评估时刻递推。设相邻评估间隔为 \(\Delta t\)，其速度目标为

\[
v_o^*=\min\left(
\max(v_o^{orig},v_{ego}),
v_{follow}(o,lead(o))
\right),
\]

速度更新为

\[
v_o'=max\left(0,
v_o+\operatorname{clip}(v_o^*-v_o,-7\Delta t,2.8\Delta t)
\right).
\]

这段特殊处理用于 ego 候选轨迹的障碍物/碰撞预测，不是 NPC 实际运行时的更新代码。只有汽车和摩托车可能被标记为后方跟随者：它要位于 ego 后方，航向差小于 \(45^\circ\)，并且横向距离落在两车半宽之和再加 0.5 m 的范围内。

在每个预测采样时刻，\(v_o^{orig}\) 是该 NPC 在本轮预测开始时的原始速度，\(v_{ego}\) 是当前 ego 候选轨迹在上一采样时刻的速度；\(v_o\) 则是该后车在前一预测采样时刻的预测速度。代码把 \(\max(v_o^{orig},v_{ego})\) 作为期望速度下界，再用 \(v_{follow}(o,lead(o))\) 限制它：后车会尝试跟上 ego，但如果相对前车距离不足，就按跟车限速减速。这里的 \(lead(o)\) 会在预测交通状态中重新寻找该后车前方的车辆；预测交通状态包括上一时刻的 ego 和其他车辆。

随后按采样间隔 \(\Delta t\) 更新后车速度，最多减速 \(7\,\mathrm{m/s^2}\)、加速 \(2.8\,\mathrm{m/s^2}\)，并将速度限制为非负；再用更新后的速度通过该车辆自己的 route（若有）推进位置。每个采样时刻都把新预测状态保存下来，作为下一步的起点。因此后车的位置会随 ego 候选轨迹的减速或停车而变化，避免把它错误地当成始终保持原速度的障碍物。

### 11.3 评估采样

只对轨迹索引

\[
j\in\{0,2,4,\ldots,60\}
\]

做完整道路、车道与碰撞评估，共 31 个位姿，相邻时间间隔为 \(0.1\,s\)。

### 11.4 扫掠 OBB 碰撞

对相邻评估位姿 \(q^{j-2}\to q^j\) 与障碍物上一位姿 \(o^-\to o^+\)，先做包围范围排除。未被排除时，令

\[
D=\|q^j-q^{j-2}\|+\|o^+-o^-\|
+|\Delta\theta_q|r_q+|\Delta\theta_o|r_o,
\]

\[
r_p=\frac{\sqrt{w_p^2+d_p^2}}2,
\qquad
n=\max(1,\lceil D/0.12\rceil).
\]

在 \([0,1]\) 上以 \(n\) 段扫描双方线性插值的 OBB。发现首个重叠区间后，再执行最多 12 次二分，得到该区间内首次接触比例 \(\lambda\)。

首次碰撞时间为

\[
t_{col}=\max(0,(j-2)\cdot0.05)+\lambda
\begin{cases}
0,&j=0,\\
0.1,&j>0.
\end{cases}
\]

碰撞紧迫谓词为

\[
collisionImminent\iff
t_{col}\le\max\left(0.75,\frac{|v_0|}{8}+0.3\right).
\]

`collision_predicted` 表示 3 秒轨迹内是否存在碰撞；`collision_imminent` 只表示碰撞时间是否落入上述紧迫窗口。

---

## 12. rollout 轨迹的道路、车道与 route 指标

### 12.1 车道横向量

对评估位姿 \(q\)，令 \(n=N(q,R_{local})\)。横向偏移为

\[
e_y=(x-n.x)\cos n.\theta_R+(z-n.z)\sin n.\theta_R.
\]

航向差为

\[
e_\theta=|\operatorname{ang}(\theta-n.\theta_R)|.
\]

车辆矩形在 route 横向轴上的投影半径为

\[
\rho=\frac{|\cos e_\theta|w+|\sin e_\theta|d}{2}.
\]

令 \(s_{local}\) 为 `nearestOnPath(q,R_local)` 返回的切片内弧长。实现将这个切片内弧长直接传给整条 route 的 section 查询；由该查询得到的半车道宽记为 \(b\)。若 section 没有半车道宽，则回退到初始 section 的半宽；初始 section 也没有时，高速公路取 2.25 m，其他世界取 3 m。越出车道的量为

\[
e_{lane}=\max(0,|e_y|+\rho-b).
\]

直观地说，先将车辆中心投影到 route 上，得到最近投影点 \(n\)；\(e_y\) 表示车辆中心相对该投影点沿 route 横向轴偏到哪一侧、偏了多少。横向轴方向为 \((\cos n.\theta_R,\sin n.\theta_R)\)，正值表示路线右侧，负值表示左侧；绝对值越大，横向偏移越大。

\(e_\theta\) 是车头与路线方向的夹角，越大表示车身相对路线越斜。\(\rho\) 是车辆矩形在 route 横向轴上的投影半径，即从车身中心到横向投影最外缘的距离；它不是固定世界坐标 \(x\) 轴上的投影。车身与 route 平行时，\(\rho=w/2\)；车身斜着时，车长也会占用一部分横向空间，所以公式同时使用车宽 \(w\)、车长 \(d\) 和夹角 \(e_\theta\)。

因此，\(|e_y|+\rho\) 表示车身横向投影最外缘离 route 中心线的距离，\(b\) 是车道半宽，也就是从 route 中心线到车道边界的距离。\(e_{lane}=\max(0,|e_y|+\rho-b)\) 就是车身横向最外侧超出车道边界的距离：车身未越界时取 0，越界时取超出的米数。这里衡量的是横向投影，不是车身角点到车道线的真实最短距离。

### 12.2 31 点聚合量

令评估位姿集合为 \(J=\{0,2,\ldots,60\}\)，则

\[
E_y=\sum_{j\in J}|e_y^j|,
\quad
E_{lane}^{max}=\max_{j\in J}e_{lane}^j,
\]

\[
F_{out}=\sum_{j\in J}f_{out}(q^j),
\quad
F_{out}^{max}=\max_{j\in J}f_{out}(q^j),
\]

每个评估位姿的 \(f_{out}(q^j)\) 表示车辆车身落在道路面之外的面积比例。令 \(B(q^j)\) 为该位姿下车辆的旋转矩形，\(\mathcal A\) 为局部可行驶道路多边形集合，则

\[
f_{out}(q^j)=
\frac{\operatorname{area}\left(B(q^j)\setminus\bigcup_{A\in\mathcal A}A\right)}{w d},
\]

其中 \(w d\) 是车身矩形的总面积。因此 \(f_{out}=0\) 表示车身完全落在道路面内；\(f_{out}=0.1\) 表示约 10\% 的车身面积在道路面外。\(F_{out}\) 将 31 个评估位姿的比例相加，\(F_{out}^{max}\) 取其中最大比例。

\[
E_\theta^{max}=\max_{j\in J}e_\theta^j.
\]

候选满足

\[
staysInLane\iff E_{lane}^{max}<0.12,
\]

\[
staysOnRoad\iff F_{out}^{max}<10^{-5}.
\]

令起点车道越界量为 \(e_{lane}^0\)，终点横向偏移为 \(e_y^{60}\)，则

\[
returningToLane\iff
E_{lane}^{max}\le e_{lane}^0+0.15
\land |e_y^{60}|<|e_y^0|.
\]

这些量把整条 3 秒候选轨迹压缩成几个容易比较的数字。评估点每隔 0.1 秒取一个，共 31 点：

- \(E_y\) 把各点的横向偏移绝对值加起来；越小，轨迹整体越贴近 route。实际记录中的 \(\texttt{lane\_error\_m}\) 是 \(E_y/31\)，也就是这些点的平均横向偏移。
- \(E_{lane}^{max}\) 是所有评估点中最大的车道越界距离。\(\texttt{staysInLane}\) 要求它小于 \(0.12\,\mathrm m\)；阈值留有很小的数值容差。
- \(F_{out}\) 累加各评估点车身落在道路外的面积比例；\(F_{out}^{max}\) 取其中最大值。\(\texttt{staysOnRoad}\) 检查最大比例是否低于 \(10^{-5}\)，也就是每个被检查的车身都几乎完全落在道路面内。
- \(E_\theta^{max}\) 是整段预测中最大的车头偏角；它用于判断轨迹是否严重横着偏离路线。
- \(\texttt{returningToLane}\) 不表示车辆已经完全回到车道内：它只要求最大越界量没有比起点恶化超过 \(0.15\,\mathrm m\)，并且终点横向偏移绝对值小于起点，即 \(|e_y^{60}|<|e_y^0|\)，表示总体上在朝车道方向恢复。

### 12.3 route 跟踪量

令

\[
n_{end}=N(q^{60},R_{local}),
\quad
n_{soon}=N(q^{24},R_{local}).
\]

短期跟踪误差为

\[
T_{track}=n_{soon}.d_R
+3|\operatorname{ang}(n_{soon}.\theta_R-\theta^{24})|.
\]

终点航向误差，以角度计，为

\[
H_{end}=\frac{180}{\pi}
|\operatorname{ang}(n_{end}.\theta_R-\theta^{60})|.
\]

route 方向一致谓词为

\[
followsRoute\iff
v_i^*\ge0
\land E_\theta^{max}<\pi/2
\land n_{end}.s_R\ge n_0.s_R-0.1.
\]

\(T_{track}\) 在 1.2 秒处衡量候选对 route 的跟踪情况。第一项 \(n_{soon}.d_R\) 是车辆位置到 route 的最短距离；第二项是车头朝向与 route 朝向之间的角度差的绝对值，再乘以权重 \(3\)。角度以弧度计，因此朝向差每增加 \(1\,\mathrm{rad}\)，\(T_{track}\) 就增加 \(3\)。两项相加后，\(T_{track}\) 越小通常表示车辆越靠近 route、车头方向也越贴合 route；它是带朝向惩罚的加权指标，不是纯粹的距离。

\(H_{end}\) 只看 3 秒终点的航向误差，并用度表示。\(\texttt{followsRoute}\) 是单独的方向一致检查：候选必须不是倒车，沿途航向偏差不能达到 \(90^\circ\)，终点 route 进度也不能比起点倒退超过 \(0.1\,\mathrm m\)。它不要求横向误差为零。

### 12.4 停车线穿越

只有起点满足

\[
D_{stop}(q_0,\ell)>0
\]

时才检测穿越。若任一评估位姿满足

\[
D_{stop}(q^j,\ell)\le0,
\]

则 `crosses_stop_line=true`。

这项指标只回答“候选是否从停车线前开到了线的另一侧”，不判断是否遵守停车规则，也不判断是否在正确位置停住。先要求起点前保险杠还在线前；再检查 31 个评估位姿，只要其中一个位姿的前保险杠距离 D_stop≤0，就记为穿越。是否必须停车、停车余量和停车速度控制由 §5、§8.5 和 §9 处理。

---

## 13. 候选评分函数

记布尔量在算术中取 0 或 1。所有模式共享碰撞代价

\[
S_{col}=10000\,[collisionImminent]
+20\,[collisionPredicted\land\neg collisionImminent].
\]

### 13.1 正常模式

\[
\begin{aligned}
S_{normal}={}&S_{col}
+1000F_{out}^{max}
+30E_{lane}^{max}
+8\frac{E_y}{31}\\
&+T_{track}
+2n_{end}.d_R
+0.3|\delta_i-\delta_{w,0}|.
\end{aligned}
\]

其中若初态缺少 `wheelSteering`，\(\delta_{w,0}\) 回退为 `steering`。

### 13.2 恢复模式

若恢复目标存在：

\[
S_{recovery}=S_{col}
+\operatorname{dist}(q^{60},g_{plan})
+0.035H_{end}
+0.5d_{road}(q^{60}).
\]

这里 \(g_{plan}=recovery.waypoint\)；若 Dijkstra 路径包含多个拐点，它不是最终的 `recovery.goal`。

若恢复目标不存在，将第一项目标距离替换为固定惩罚 100：

\[
S_{recovery}=S_{col}+100+0.035H_{end}+0.5d_{road}(q^{60}).
\]

分数越小，候选越优。

---

## 14. 原始池内的安全集合与合流预过滤

### 14.1 高速合流冲突

定义

\[
mergeConflict=mergeTraffic
\land\exists\gamma\in\mathcal P:
\gamma.queueCompatible
\land \gamma.v^*>0.7v_{max}
\land \gamma.collisionPredicted.
\]

这是一个针对候选池的整体布尔量，对应 `src/driving-plan.js` 中的 `mergeConflict`。它分两步判断：

1. `mergeTraffic` 表示当前处于合流路段，而且 `nearby` 中至少有一辆汽车或摩托车朝向大致与本车路线一致，并且它到本地路线 `localRoute` 的最近距离小于 8 m。`nearby` 先按相对速度和车辆尺寸筛出可能相关的障碍物，因此这个条件只说明路线附近有同向交通，尚未说明会发生冲突。
2. 再检查 `pool` 中是否存在一个同时满足三个条件的候选：`queueCompatible` 表示候选的横向偏移 `lane_offset_m` 非空且绝对值不超过 0.2 m，即基本沿当前车道行驶；`v^* > 0.7v_{max}` 表示候选给出的目标速度 `velocity_mps` 高于当前速度上限的 70%；`collisionPredicted` 表示该候选的前向 rollout 在 3 秒预测时域内检测到扫掠车身 OBB 接触。这里检查的是 `collision_predicted`，不是更严格的 `collision_imminent`。

这里的 \\(v^*\) 不是根据碰撞结果反算出来的速度，而是生成候选时采样的目标速度。代码先计算

\[
v_{max}=\operatorname{round}\left(\min\left(ceiling,
\begin{cases}
2,& recovering,\\
stopApproachSpeed(car,stopLine),& requiresStop,\\
\infty,& otherwise
\end{cases}\right),2\right),
\]

随后按候选序号 \(i\) 和场景分支采样 \(v^*\)，并四舍五入到 0.01 m/s。普通（非恢复）合流且检测到 `mergeTraffic` 时，前 5 个候选取 \([0.30,0.55]v_{max}\)，接下来的候选（至第 10 个）取 \([0.25,0.55]v_{max}\)；从第 10 个开始走合流巡航分支，取 \([0.95,1.05]v_{max}\)。若处于合流但 `mergeTraffic` 为假，则所有普通候选取 \([0.95,1.05]v_{max}\)。这些是带随机扰动的候选范围，不是固定档位。故在普通且有车的合流情形下，前面的慢速候选本身不会满足 \(v^*>0.7v_{max}\)；谓词主要检查后续较高速候选是否会与邻近车辆发生预测碰撞。

候选里的 \\(v^*\) 保存在 `data.velocity_mps`，交给 `projectVector` 生成预测轨迹；它表示目标速度控制量，不是 rollout 结束时的实际速度，后者另记为 `end_speed_mps`。

因此，附近有车本身不会触发 `mergeConflict`：必须有一个基本保持车道、仍以较高速度行驶的候选，其预测轨迹确实与交通车辆相撞。谓词中的存在量词作用于原始候选池 \(\mathcal P\)，而不是只作用于安全集合；被判定为冲突的候选也不会因此自动从池中删除。

若

\[
merging\land\neg recovering\land\neg requiresStop
\land\neg mergeConflict\land v_{max}\ge0.15,
\]

则从 \(\mathcal P\) 删除所有满足

\[
v^*<0.7v_{max}
\]

的候选。

这一步只在合流、非恢复、无需停车、没有上述实际冲突且 `maxSpeed >= 0.15` 时执行，作用是移除低于速度上限 70% 的慢速候选，避免只因附近有车就减速爬行。若 `mergeConflict` 为真，则跳过该过滤；随后在正常模式候选选择阶段：先从原始池前 5 条中找一条属于 `eligible` 且速度为正的低分候选；再按分数排序，尝试加入一条队列兼容、速度高于 `0.7 × maxSpeed` 的候选，以及一条队列兼容、速度高于 `0.25 × maxSpeed` 且不高于 `0.7 × maxSpeed` 的候选。最多先选 3 条，之后再由整体排序补足。这里的 `eligible` 要求候选通过道路、紧迫碰撞、路线方向（高速路）和车道安全条件；但 `collision_predicted` 若尚未达到 `collision_imminent`，并不会单独排除候选。因此 `mergeConflict` 是让减速档继续参与比较，不是直接选中某条轨迹，也不保证最终选择无任何预测碰撞。最终会从池中再抽取横向和转向代表候选展示，并单独追加零速候选。`v_{max}` 在公式中对应代码局部量 `maxSpeed`，`v^*` 对应每个候选的 `data.velocity_mps`。

### 14.2 正常模式安全集合

在预过滤后的 \(\mathcal P\) 上定义

\[
\mathcal P_{safe}=\{\gamma\in\mathcal P\mid
\gamma.staysOnRoad
\land\neg \gamma.collisionImminent
\land(world.type\neq highway\lor \gamma.followsRoute)
\land(\gamma.staysInLane\lor \gamma.returningToLane)
\}.
\]

队列兼容定义为

\[
queueCompatible(\gamma)\iff \gamma.l\neq null\land|\gamma.l|\le0.2.
\]

若 `queue` 成立：

\[
\mathcal P_{eligible}=\{\gamma\in\mathcal P_{safe}\mid queueCompatible(\gamma)\};
\]

否则

\[
\mathcal P_{eligible}=\mathcal P_{safe}.
\]

构造排序序列

\[
\mathcal L_{rank}=sortScore(\mathcal P_{eligible})
\Vert sortScore(\mathcal P\setminus\mathcal P_{eligible}),
\]

即先放全部安全合法候选，再放其他候选；两个子序列内部均按分数升序。

注意：第二段中的不安全候选仍可进入展示候选集，但稍后不会因此自动成为 Jev 合法候选。

---

## 15. 从原始池缩减为展示候选集

### 15.1 恢复模式

恢复模式保留全部 11 个原始候选：

\[
\mathcal V_{move}=\mathcal P.
\]

### 15.2 正常模式前三个优选候选

初始化

\[
\mathcal V_{best}=().
\]

若 `requiresStop` 或 `mergeConflict` 成立，则：

1. 在原始池前 8 个候选（停车）或前 5 个候选（合流冲突）中，选择属于 \(\mathcal P_{eligible}\)、速度大于 0 且分数最小者加入；
2. 依次检查速度带

\[
(0.7v_{max},+\infty],
\qquad
(0.25v_{max},0.7v_{max}],
\]

在序列 \(\mathcal L_{rank}\) 中找第一个同时满足以下条件的候选加入：

\[
\gamma\in\mathcal P_{eligible},
\quad queueCompatible(\gamma),
\quad \gamma.stopAtLine=null,
\]

并且速度属于当前速度带、此前未被选择。

3. 一旦 \(|\mathcal V_{best}|=3\) 即停止此过程。

之后按 \(\mathcal L_{rank}\) 的顺序继续加入尚未选择的候选，直到

\[
|\mathcal V_{best}|=3.
\]

### 15.3 四个横向代表

从剩余候选中取

\[
\mathcal P_l=sort_{l}\{\gamma\in\mathcal P\setminus\mathcal V_{best}\mid \gamma.l\neq null\}.
\]

令 \(N_l=|\mathcal P_l|\)。对 \(k=0,1,2,3\)，选择索引

\[
j_k=\left\lfloor\frac{k(N_l-1)}3\right\rfloor.
\]

将 \(\mathcal P_l[j_k]\) 加入展示集。因此取到的是从最左偏移到最右偏移的四个分位位置，而不是分数最优的四个候选。

### 15.4 四个固定转向代表

从剩余且 \(l=null\) 的候选中取

\[
\mathcal P_\delta=sort_{\delta}\{\gamma\in\mathcal P\mid \gamma.l=null\}.
\]

同样按

\[
j_k=\left\lfloor\frac{k(|\mathcal P_\delta|-1)}3\right\rfloor,
\quad k=0,1,2,3
\]

取四个从左转到右转的代表。

正常情况下，移动展示候选数量为

\[
3+4+4=11.
\]

### 15.5 强制追加零速候选

最后总是额外评估一个零速候选：

\[
\delta=car.steering\ \text{或 }0,
\quad v^*=0,
\]

\[
l=
\begin{cases}
null,&recovering,\\
0,&\text{正常模式},
\end{cases}
\qquad L=4.5,
\qquad \ell_s=null.
\]

因此正常与恢复模式通常都形成 12 个展示候选。ID 依次为

\[
batch\_v0,batch\_v1,\ldots,batch\_v10,batch\_stop.
\]

每个 ID 在两个映射中各有一份表示：

\[
vectors[id]=\text{评估摘要},
\qquad
projections[id]=\text{61 点完整预测轨迹}.
\]

---

## 16. 停止候选是否进入决策状态

先对当前 `state.vectors` 计算移动合法候选，见第 17 节。停止候选只在至少一个下列条件成立时可用。

### 16.1 近距离阻挡物

\[
blockingObject\neq null
\land blockingObject.gap\le2.5.
\]

### 16.2 必须停车线在近距离内

\[
intersection\neq null
\land\neg intersection.alreadyEntered
\land -0.5\le d_{line}\le2.5
\land C_{required},
\]

其中

\[
C_{required}=(control=stop\land\neg stopCompleted)
\lor(control=signal\land signal\in\{red,amber\}).
\]

### 16.3 到达目的地

\[
destinationDistance\le2.5.
\]

### 16.4 没有合法移动候选

\[
|\mathcal E_{move}|=0.
\]

若四类条件均不成立，零速候选会从 `state.vectors`、`plan.vectors` 和 `plan.projections` 中删除，而不只是从 Jev 合法集合中隐藏。

---

## 17. 最终 Jev 合法候选集合

本节过滤作用于缩减后的展示集合 \(\mathcal V\)，不是 55 个原始样本。

### 17.1 初始移动集合

若

\[
recovery.blocked\lor speedCeiling=0,
\]

则

\[
\mathcal E_0=\varnothing.
\]

否则

\[
\mathcal E_0=\{\gamma\in\mathcal V\mid \gamma.v^*\neq0
\land\neg(\gamma.collisionImminent\mathbin{??}\gamma.collisionPredicted)\}.
\]

这里 `??` 是空值合并，不是逻辑或。当前候选总是具有布尔字段 `collision_imminent`，因此实际效果是：

\[
collisionImminent=false
\]

的候选即便 `collision_predicted=true`，也不会在这一层仅因远期碰撞被删除。

### 17.2 道路与 route 方向

定义仅允许正向 route 的阶段集合

\[
\Phi=\{onramp,merge,interstate,exit,offramp\}.
\]

若不处于恢复模式且 `trip.phase` 属于 \(\Phi\)，则 `forwardOnly=true`。

恢复模式：

\[
\mathcal E_{road}=\mathcal E_0.
\]

正常模式：

\[
\mathcal E_{road}=\{\gamma\in\mathcal E_0\mid
\gamma.staysOnRoad
\land(\neg forwardOnly\lor \gamma.followsRoute)
\}.
\]

### 17.3 队列约束

若正常模式且存在 `traffic.queue`：

\[
\mathcal E_{queue}=\{\gamma\in\mathcal E_{road}\mid \gamma.queueCompatible\};
\]

否则

\[
\mathcal E_{queue}=\mathcal E_{road}.
\]

### 17.4 车道优先级

恢复模式直接令

\[
\mathcal E_{move}=\mathcal E_{queue}.
\]

正常模式依次应用非对称优先级：

\[
\mathcal E_{in}=\{\gamma\in\mathcal E_{queue}\mid \gamma.staysInLane\},
\]

\[
\mathcal E_{return}=\{\gamma\in\mathcal E_{queue}\mid \gamma.returningToLane\}.
\]

最终

\[
\mathcal E_{move}=
\begin{cases}
\mathcal E_{in},&|\mathcal E_{in}|>0,\\
\mathcal E_{return},&|\mathcal E_{in}|=0\land|\mathcal E_{return}|>0,\\
\varnothing,&|\mathcal E_{in}|=|\mathcal E_{return}|=0\land forwardOnly,\\
\mathcal E_{queue},&\text{其他}.
\end{cases}
\]

因此，只要存在至少一个 `stays_in_lane` 候选，所有仅仅“正在返回车道”的候选都会被排除；两者不是并集关系。

### 17.5 追加停止候选

若第 16 节的停止可用谓词成立，从 \(\mathcal V\) 中取最后一个 \(v^*=0\) 的候选：

\[
\mathcal E_{stop}=\langle last(\gamma\in\mathcal V:\gamma.v^*=0)\rangle.
\]

否则

\[
\mathcal E_{stop}=\langle\rangle.
\]

最终交给 Jev 的合法选择集合为

\[
\mathcal E=\mathcal E_{move}\mathbin{\Vert}\mathcal E_{stop}.
\]

并写入

\[
plan.eligible=\mathcal E.
\]

---

## 18. 候选的四种表示

同一个候选在系统中不是单一对象，而有四种不同粒度：

1. 控制参数

\[
m_i=(\delta_i,l_i,L_i,\ell_{s,i}),\quad v_i^*;
\]

2. 完整预测轨迹

\[
projections[id]=\Pi_i=(q_i^0,\ldots,q_i^{60});
\]

3. 评估摘要

\[
vectors[id]=\{
steering,velocity,lane,road,route,stopLine,collision,endState,\ldots
\};
\]

4. Jev 合法子集

\[
eligible=\mathcal E\subseteq vectors.
\]

`state.vectors` 是评估摘要集合，不包含完整的 61 点轨迹；完整轨迹保存在 worker 返回的 `plan.projections` 中。Jev 可以选择的候选也不是 `state.vectors` 全集，而是 `plan.eligible`。

---

## 19. 一次规划的完整状态变换

整个过程可以压缩为以下复合映射：

\[
\Sigma_{main}
\xrightarrow{snapshot}
S_{msg}
\xrightarrow{clone+restore}
\Sigma_w
\xrightarrow{scanScene}
(\Sigma_0,\mathcal P_{sens})
\xrightarrow{constraints}
(v_{ceil},v_{max},queue,mergeTraffic,recovering)
\]

\[
\xrightarrow{sample}
\mathcal P_{55}\ \text{或}\ \mathcal P_{11}
\xrightarrow{60\times0.05s\ rollout}
\{\Pi_i\}
\xrightarrow{31\text{ 点几何/碰撞评估}}
\{\gamma_i=(u_i,\Pi_i,a_i,\sigma_i)\}
\]

\[
\xrightarrow{rank+quantile}
\mathcal V_{11}
\xrightarrow{append\ stop}
\mathcal V_{12}
\xrightarrow{stop\ availability}
\mathcal V
\xrightarrow{legal\ filtering}
\mathcal E.
\]

正常模式的典型数量链为

\[
55\text{ 个随机控制样本}
\longrightarrow
55\text{ 条、每条 61 点的预测轨迹}
\longrightarrow
11\text{ 个代表移动候选}+1\text{ 个停止候选}
\longrightarrow
\mathcal E.
\]

最后一个箭头没有固定候选数量；它取决于紧迫碰撞、道路覆盖、route 方向、队列、车道状态以及停止候选当前是否可用。

---

## 20. 实现依据

- 快照构造与 worker 通信：`src/background-planner.js`
- worker 状态恢复：`src/planner.worker.js`
- 规划入口、速度 ceiling 与最终状态装配：`src/simulation.js`
- 原始候选采样、评估、评分和缩减：`src/driving-plan.js`
- 离散动力学、rollout 与最终合法候选过滤：`src/planning.js`
- Jev 压缩输入、单选项本地固定和候选 ID 别名：`src/jev-request.js`
- Jev 响应展开、服务端状态/答案/碰撞检查：`server/jev.js`
- 客户端批次、时效、上下文与 route 分支复核：`src/main.js`
- 障碍物预测、前车和跟车速度：`src/traffic-safety.js`
- 扫掠 OBB 碰撞检测：`src/collisions.js`
- 道路覆盖面积计算：`src/road-geometry.js`
