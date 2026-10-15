# airseai_grasp：用 Diffusion Policy 替换 GraspNet 的实施步骤

本文给出把 `airseai_grasp` 的抓取推理从 Scale-Balanced-Grasp / GraspNet 换成 Diffusion Policy 的可行做法。范围停在技能内核：Planner、Nav2、语义地图和 `AirseaiGrasp` 服务契约保持不变。本文不包含可运行的训练框架代码。

## 1. 结论

可行，但是技能内核替换，不是 GraspNet 权重热插拔。

现有抓取是“单帧点云预测一个 6-DoF 夹爪位姿，再开环走过去”。Diffusion Policy 是“用一段观测历史作条件，去噪未来一段关节动作，再滚动执行”。两边的输出空间不同，不能把策略网络塞进 `UtilityGraspNet.grasp_pose_estimation()`。

推荐做法：

- 对外继续提供 `/airseai_grasp/grasp_server`。
- 技能开始时调用一次分割，把物体名变成视觉目标。
- 服务内部用 10 Hz 的 receding-horizon 闭环替代 GraspNet + 解析 IK。
- 用 launch 参数 `grasp_backend:=graspnet|diffusion` 切换，GraspNet 先留作对照和脚本专家。
- 第一阶段只替换仿真里的 `pick`。`place` 仍走现有张开夹爪。真机 Cartesian 控制放到仿真关节策略跑通之后。

第一阶段使用 2D Diffusion Policy（Chi et al.：图像编码器 + 1D 条件 U-Net）。`airseai_embodidata` 的手机手轨迹、`airseai_embodicore` 对 3D-Diffusion-Policy / DP3-Mamba 的硬件语义审计，都不作为训练数据或运行时依赖。点云版 DP3 只在 2D 策略几何泛化不足时再考虑。

## 2. 现有抓取链，以及为什么不能 1:1 替换

### 2.1 对外契约

[`airseai_interface/srv/AirseaiGrasp.srv`](airseai_interface/srv/AirseaiGrasp.srv) 定义：

| 方向 | 字段 |
|---|---|
| 请求 | `task`（`pick` / `place`）、`obj` |
| 响应 | `ret`、`x`、`y`、`z` |
| 返回码 | `SUCCESS = 0`、`FAILED_TO_FIND_OBJECT = 1`、`FAILED_TO_REACH_OBJECT = 2` |

[`airseai_planner/airseai_planner/llm_planner_node.py`](airseai_planner/airseai_planner/llm_planner_node.py) 同步调用 `/airseai_grasp/grasp_server`。`pick_up` 映射为 `task=pick`，`place` 映射为 `task=place`。Planner 只看返回码，不看内部是 GraspNet 还是策略网络。

### 2.2 当前 pick 路径

[`airseai_grasp/airseai_grasp/grasp_server_node.py`](airseai_grasp/airseai_grasp/grasp_server_node.py) 的 `_handle_pick_task()`：

1. `set_arm_init_states()` 把臂收到固定关节角。
2. `request_segmentaion_service()` 调用 `/airseai_perception/SegmentationGrasping`，得到掩码、RGB、深度。
3. 掩码为空则返回 `FAILED_TO_FIND_OBJECT`。
4. 仿真分支用 `UtilityIk` 读末端 TF；真机分支用 `pymycobot` 读当前位姿。
5. `UtilityGraspNet.get_3d_center_mask()` 反投影掩码中心。
6. `UtilityGraspNet.grasp_pose_estimation()` 生成 GraspNet 候选。
7. `select_best_grasp()` 做相机到基座的齐次变换，并按距离和偏航角筛选。
8. 仿真 `excute_grasp()` 只取 XYZ，调用解析 IK，发布一次 `joint_command`，再闭合夹爪。真机 `execute_grasp()` 用 `send_coords` 一次到位。

仿真执行器接口已经存在：

- 订阅：`/camera/camera/color/image_raw`、`/camera/camera/aligned_depth_to_color/image_raw`、`joint_states`
- 发布：`joint_command`（六轴）和 `gripper_controller`（张开 `0.2`，闭合 `-0.4`）

六轴名称：

```text
joint_link1_to_base
joint_link2_to_link1
joint_link3_to_link2
joint_link4_to_link3
joint_link5_to_link4
joint_link6_to_link5
```

### 2.3 不能直接替换的三点

1. GraspNet 输出的是相机系夹爪位姿集合。Diffusion Policy 输出的是未来若干控制周期的关节与夹爪增量或绝对位置。没有“把 DP 输出再送进 `select_best_grasp()`”这一步。
2. [`airseai_grasp/lib/utility_ik.py`](airseai_grasp/lib/utility_ik.py) 的 `inverse()` 只根据目标 XYZ 计算关节角，腕部姿态固定。它不是通用运动规划，也不适合作为策略动作的执行器。
3. 现有 `place` 在仿真里只是把夹爪开到 `0.2`。它不是视觉引导放置。第一阶段不要让 DP 同时承担 pick 和 place，否则数据、条件和验收会缠在一起。

### 2.4 若仍用 GraspNet 生成专家数据，必须先修的缺陷

这些缺陷来自当前代码，不修则脚本专家轨迹不可用：

| 问题 | 位置 | 后果 |
|---|---|---|
| `get_and_process_data(rgb, depth, mask, use_isaac_sim)` 需要四个参数，`grasp_pose_estimation()` 只传三个 | [`airseai_grasp/lib/utility_graspnet.py`](airseai_grasp/lib/utility_graspnet.py) | 分割成功后抛 `TypeError` |
| 掩码中心把行和 `cx`、列和 `cy` 配对 | `get_3d_center_mask()` | 三维中心偏移，最佳抓取选错 |
| `np.bool` | `grasp_server_node.py` 的 `req_seg_sync()` | 新版 NumPy 直接报错 |
| `grasp_sim.launch.py` 使用 `use_isaac_sim` 但没有 `DeclareLaunchArgument` | [`airseai_grasp/launch/grasp_sim.launch.py`](airseai_grasp/launch/grasp_sim.launch.py) | 仿真开关不能稳定传入 |

遥操作数据不依赖这些修复。GraspNet 专家数据依赖。

## 3. 保留与替换边界

```text
Planner
  -> AirseaiGrasp(task=pick, obj)
       -> SegmentationGrasping 一次
       -> 观测：腕部 RGB + 掩码 + joint_states
       -> Diffusion Policy 预测动作序列
       -> 关节限位 / 工作空间 / 速度变化率护栏
       -> /joint_command
       -> 成功或超时
            返回 SUCCESS 或 FAILED_TO_*
```

| 层 | 当前实现 | 决策 |
|---|---|---|
| ROS 技能契约 | `AirseaiGrasp` | 保留。Planner 不改 |
| 目标指定 | GroundingDINO + SAM | 保留，且只在技能开始时调用一次 |
| 抓取推理 | GraspNet 单帧 6-DoF | 替换为观测条件的动作序列去噪 |
| 候选筛选 | 点云碰撞检测、NMS、角度约束 | 退出主路径。工作空间盒降级为安全护栏 |
| 仿真到位 | 解析 IK 一次发布关节角 | 替换为闭环关节命令 |
| 执行器 | `joint_command` / `pymycobot` | 仿真保留 `joint_command`。真机第一阶段不做 |
| 放置 | 张开夹爪 | 第一阶段保留。第二阶段再学 |
| 数据 | 无机器人示教 | 必须新建。不能用 EmbodiData |

底盘仍由 Nav2 负责。Diffusion Policy 只在基座到位且锁定后控制手臂和夹爪。

## 4. 观测、动作、频率与成功判定

第一阶段动作空间用关节绝对位置，和仿真 `joint_command` 一致，避免再经过解析 IK。

| 通道 | 来源 | 约定 | 用途 |
|---|---|---|---|
| 腕部 RGB | `/camera/camera/color/image_raw` | 裁到 224 或 256；叠上 SAM 掩码，或做物体裁剪 | 视觉条件 |
| 本体感觉 | `joint_states` 六轴 + `gripper_controller` | 7 维，训练与推理使用同一归一化 | 状态条件 |
| 目标 | `request.obj` 经一次分割 | 掩码在技能开始时冻结，循环内不重跑 SAM | 多物体消歧 |
| 动作 | `/joint_command` | 6 关节 + 1 夹爪，10 Hz | 闭环执行 |
| 时域 | 策略内部 | 观测历史 `n_obs_steps = 2`；预测 horizon 8–16；每次执行前 6–8 步再重规划 | receding horizon |
| 推理 | DDIM | 10–20 步 | 控制延迟预算 |

夹爪训练标签与仿真命令对齐：张开 `0.2`，闭合 `-0.4`。

成功判定不要再用“关节命令已发出”。仿真可读物体位姿时：

- 成功：夹爪闭合，且物体相对桌面抬升超过 5 cm，并保持到抬升结束。
- 未找到：分割掩码为空，返回 `FAILED_TO_FIND_OBJECT`。`x,y,z` 可不填。
- 失败：超时（建议 15 s）、越出工作空间、关节命令非法、物体未被抬起。返回 `FAILED_TO_REACH_OBJECT`。若能估计物体或末端位置，填入 `x,y,z`，供 Planner 现有“不可达则改导航点”分支使用。

安全护栏沿用 [`utility_ik.py`](airseai_grasp/lib/utility_ik.py) 的工作空间盒作为末端位置检查，不作为 IK 求解：

```text
x in [0.24, 0.6] m
y in [-0.4, 0.4] m
z in [-0.55, 0.3] m
```

另外限制相邻控制周期的关节变化量，以及 URDF 关节上下限。策略输出越界时裁剪；连续越界或超时则失败返回，不继续执行。

## 5. 分阶段实施

### 阶段 0：冻结契约和后端开关

目标：DP 与 GraspNet 可以并存，Planner 无感。

改动：

1. 保持 [`AirseaiGrasp.srv`](airseai_interface/srv/AirseaiGrasp.srv) 字段和三个返回码不变。不在第一阶段新增错误码。
2. 在 [`grasp_sim.launch.py`](airseai_grasp/launch/grasp_sim.launch.py) 中声明 `use_isaac_sim`（默认 `true`）和 `grasp_backend`（默认 `graspnet`，可选 `diffusion`）。
3. 在 [`airseai_grasp_sim_config.yaml`](airseai_grasp/config/airseai_grasp_sim_config.yaml) 增加 DP 段：checkpoint 路径、控制频率、horizon、执行步数、超时、图像尺寸、归一化统计文件路径。GraspNet 的 `model_checkpoint_path` 保留。
4. `GraspServer` 按 `grasp_backend` 选择 `UtilityGraspNet` 或后续的 `UtilityDiffusionPolicy`。

完成标准：

- `ros2 launch airseai_grasp grasp_sim.launch.py use_isaac_sim:=true grasp_backend:=graspnet` 能创建服务。
- `grasp_backend:=diffusion` 在没有 checkpoint 时明确报“权重缺失”，而不是落到 GraspNet 路径。

仿真验证：Isaac Sim 已 Play 时，`ros2 service list` 能看到 `/airseai_grasp/grasp_server`。此阶段不要求抓取成功。

### 阶段 1：可重置桌面任务与录制器

目标：得到可以反复采样的训练环境。现有办公室 USD 适合演示，不适合模仿学习。

新增内容建议放在仿真侧，而不是塞进 GraspNet 库：

1. 单独的桌面任务：一张桌子、少量物体（先与现有 demo 对齐，例如苹果或糖盒）、腕部 D435、机械臂。办公室场景 `airseai_sim/airseai/airsbot2_manipulator_ros2_office.usd` 留到阶段 5 做回归。
2. 任务脚本提供 `reset()`、物体 XY 与偏航随机化、读取物体世界位姿。
3. 抓取期间不发布底盘速度，避免 `/cmd_vel` 漂移进入示教。
4. episode 录制器同步保存：RGB、深度、掩码、`joint_states`、实际发出的 `joint_command`、`obj` 文本、成功或失败。时间对齐误差目标小于 20 ms。格式用 zarr 或 hdf5；若先录 ROS bag，必须有离线转换脚本和字段说明。

完成标准：

- 连续 reset 100 次，场景不崩溃，物体回到随机初始位姿。
- 回放 10 条录制关节命令，机械臂轨迹与录制时一致。
- 图像、关节、夹爪无 NaN。

仿真验证：只开 Isaac Sim 和录制器，不启动 Planner。`ros2 topic hz` 确认彩色图、深度和 `joint_states` 在出。

### 阶段 2：采集单物体 pick 示教

目标：100–200 条成功的单物体 pick episode。这是经验规模，不是本仓库实测数字。

两条来源，都要进阶段 1 的同一数据格式：

1. 遥操作（质量优先）。用键盘或 SpaceMouse 映射到关节或末端，在桌面任务里录 50–100 条成功抓取。每条包含接近、闭合、抬升。
2. 脚本专家（数量优先）。修完第 2.4 节的 GraspNet 缺陷后，把“选一个抓取姿态”展开成预抓取、接近、闭合、抬升的关节轨迹，再录 200–500 条。解析 IK 忽略姿态，所以这类数据有质量上限，不能单独当作最终策略的唯一来源。

不要使用 `airseai_embodidata` 的人手轨迹作为 action。那是相机系手部轨迹，不是 MyCobot 630 的关节命令。

完成标准：

- 成功 episode 数量达到 100 条以上，且物体抬升判据与第 4 节一致。
- 训练集和验证集按 episode 划分，不按时间步打散到两边。
- 每条数据带 `obj` 标签，即使第一阶段只有一个物体。

仿真验证：随机抽 10 条成功 episode 回放，回放后物体仍被抬起。回放失败的轨迹不要进入训练集。

### 阶段 3：训练 2D Diffusion Policy

目标：得到一个只依赖腕部图像、掩码和本体感觉的 pick checkpoint。训练代码放在仓库外或独立目录，不把第三方训练框架整份拷进 `airseai_grasp`。

训练设定：

- 视觉编码器：ResNet，输入为 RGB 与掩码的组合。
- 动作模型：1D temporal U-Net。
- 条件：`n_obs_steps = 2` 的图像与 7 维本体感觉。
- 采样：DDIM 10–20 步。
- 动作：7 维绝对关节与夹爪位置，horizon 8–16。
- 归一化统计从训练集计算，并和 checkpoint 一起保存。推理必须使用同一份统计。

离线检查只说明训练是否收敛，不能代替仿真成功率：

- 验证集动作误差下降。
- 解码轨迹落在关节限位内。
- 夹爪通道在接近段接近 `0.2`，在闭合段接近 `-0.4`。

完成标准：保存 checkpoint、归一化文件、训练配置和数据版本说明。配置写入阶段 0 的 yaml 路径。

### 阶段 4：接入 grasp_server

目标：`grasp_backend:=diffusion` 时，pick 走策略闭环。

新增 [`airseai_grasp/lib/utility_diffusion_policy.py`](airseai_grasp/lib/utility_diffusion_policy.py)，职责限于：

- 加载 checkpoint 与归一化统计。
- 接收一帧观测，维护长度为 `n_obs_steps` 的历史。
- `predict_action()` 返回未来若干步的 7 维动作。

[`grasp_server_node.py`](airseai_grasp/airseai_grasp/grasp_server_node.py) 的 `_handle_pick_task()` 在 DP 分支改为：

1. 分割一次。掩码为空则 `FAILED_TO_FIND_OBJECT` 并返回。
2. 仿真调用现有 `prepare_for_grasp()` 张开夹爪。不要调用 `set_arm_init_states()` 里对 `self.ik` 的无条件依赖，除非该初始化被明确限制在仿真 GraspNet 分支。当前 `set_arm_init_states()` 在真机模式也会访问只在仿真创建的 `self.ik`，DP 仿真分支应使用明确的初始关节角，并只在 `use_isaac_sim` 时执行。
3. 以 10 Hz 循环：读最新 RGB 与 `joint_states`，叠上冻结掩码，预测动作，做限位与变化率裁剪，发布 `joint_command` 与 `gripper_controller`。
4. 每轮只执行预测序列的前 6–8 步，然后重新观测。
5. 抬升成功返回 `SUCCESS`。超时或护栏失败返回 `FAILED_TO_REACH_OBJECT`。

`grasp_backend:=graspnet` 保持现有 `_handle_pick_task()` 行为，便于对照。第一阶段 `_handle_place_task()` 不改。

完成标准：

- 无权重时启动失败信息明确。
- 有权重时，服务在一次 `pick` 调用结束前保持同步，Planner 仍按一次请求等待最终返回码。
- 循环内不调用 GroundingDINO 或 SAM。

仿真验证：桌面任务中调用

```bash
ros2 service call /airseai_grasp/grasp_server airseai_interface/srv/AirseaiGrasp "{task: 'pick', obj: 'apple'}"
```

物体在视野内应返回 `0`。移出视野或把分割结果置空应返回 `1`。把物体放到工作空间外应返回 `2`。

### 阶段 5：仿真分层验收

按层验收。上一层没过，不进入下一层。

| 层级 | 设置 | 通过标准 |
|---|---|---|
| L0 接口 | Isaac Sim Play，DP 节点启动 | RGB、深度、`joint_states` 进；`joint_command` 出；无 NaN；对齐误差 < 20 ms |
| L1 孤立抓取 | 底盘锁定，单物体，位姿随机，50 次 | 成功率约 80%（抬升 > 5 cm）；明显碰撞 < 10%；平均完成时间 < 8 s；失败能停并回到初始姿态 |
| L2 轻微分布外 | 改变初始距离、左右偏置、少量光照 | 记录成功率均值和方差。若低于 50%，先加数据和随机化，不接 Planner |
| L3 技能服务 | 同一 `AirseaiGrasp` 调用 | 可见可抓为 `0`；无掩码为 `1`；太远或超时为 `2`；`place` 仍张开夹爪并返回 `0` |
| L4 导航后抓取 | 先 `airseai_localization_gt_sim` 与 `airseai_navigation_sim`，再 pick | 报告导航残差下的成功率。若 L1 通过而 L4 明显下降，在策略开始前等待底盘静止，或在示教中加入小幅基座位姿扰动 |
| L5 语言任务 | Planner 调度 `go_to -> pick -> go_to -> place` | 单独报告任务完成率，并拆开导航失败、分割失败和策略失败 |

与 GraspNet 基线在同一 L1 场景对比三项：现有 GraspNet + 解析 IK、脚本专家回放、Diffusion Policy。指标为成功率、碰撞率、完成时间。

L1 是替换是否成立的主指标。L5 是系统回归，不是训练分布。

## 6. 建议的文件落点

| 文件 | 动作 |
|---|---|
| [`airseai_interface/srv/AirseaiGrasp.srv`](airseai_interface/srv/AirseaiGrasp.srv) | 不改 |
| [`airseai_planner/airseai_planner/llm_planner_node.py`](airseai_planner/airseai_planner/llm_planner_node.py) | 不改 |
| [`airseai_grasp/launch/grasp_sim.launch.py`](airseai_grasp/launch/grasp_sim.launch.py) | 声明 `use_isaac_sim`、`grasp_backend` |
| [`airseai_grasp/config/airseai_grasp_sim_config.yaml`](airseai_grasp/config/airseai_grasp_sim_config.yaml) | 增加 DP 配置，保留 GraspNet 配置 |
| [`airseai_grasp/airseai_grasp/grasp_server_node.py`](airseai_grasp/airseai_grasp/grasp_server_node.py) | pick 增加 DP 闭环分支 |
| `airseai_grasp/lib/utility_diffusion_policy.py` | 新增推理封装 |
| [`airseai_grasp/lib/utility_graspnet.py`](airseai_grasp/lib/utility_graspnet.py) | 仅在要做脚本专家时修复参数、坐标和 `np.bool` |
| Isaac 桌面任务与录制器 | 新增，不放进 GraspNet 目录 |

## 7. 明确不做的事

- 不让 Diffusion Policy 输出底盘速度或替代 Nav2。
- 不用 EmbodiData 手轨迹直接监督机器人关节。
- 不把 DP3-Mamba 或 EmbodiCore 语义控制器接进抓取节点。
- 不在每个控制周期运行 GroundingDINO 和 SAM。
- 不在第一阶段改真机 `send_coords`。仿真关节接口和 `pymycobot` Cartesian 接口不是同一控制空间。真机要等仿真 L1 通过后，再对齐 `send_angles` 并做少量真机数据。
- 不在第一阶段用 DP 学习 `place`。现有放置仍是到达区域后释放。
- 不把完整办公室 USD 当作训练集。它只用于 L4/L5 回归。

## 8. 最小验收顺序

1. 桌面任务 reset 100 次不崩溃。
2. 10 条遥操作可以回放，且回放后物体被抬起。
3. `grasp_backend:=diffusion` 的 L1 成功率达到约 80%（50 次）。
4. 空视野返回 `FAILED_TO_FIND_OBJECT`。
5. `grasp_backend` 可在 `graspnet` 与 `diffusion` 之间切换。
6. 再在办公室场景做一次服务调用，作为演示，不作为训练分布。

未完成第 1–3 步时，不接入 Planner。
