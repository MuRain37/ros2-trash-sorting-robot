# `task_server.py` 中文注释版说明

这个文件不参与编译，只用于理解 `src/robot_behavior_tree/robot_behavior_tree/task_server.py` 的行为树调度逻辑。

## 总体定位

`RobotTaskServer` 是“用户交互到机器人执行”的中间管理层。

前端、语音、其他上层模块不再直接调用导航、视觉、机械臂、夹爪、倾倒等底层能力，而是统一发送 `RobotTask` Action 给行为树。行为树收到任务后，再按任务类型决定调用哪些 ROS2 service/action。

核心思想：

1. 上层只发“我要做什么”，例如：清扫垃圾、倒垃圾、导航到某点。
2. 行为树负责拆成多个步骤，例如：导航、视觉识别、底盘微调、MoveIt 解算、夹爪开合、倾倒。
3. 每个底层功能尽量独立成服务，避免像之前 `robot_vision_grasp` 那样把识别、抓取、控制都混在一起。

## 当前支持的任务类型

这些任务类型定义在 `src/robot_interfaces/action/RobotTask.action`：

```text
TASK_CLEAN_TRASH      = 1  清扫/抓取垃圾
TASK_DUMP_TRASH       = 2  倒垃圾
TASK_CLEAN_REGION     = 3  清扫指定地图区域
TASK_NAVIGATE_TO      = 4  导航到单个目标点
TASK_STOP             = 5  停止当前任务/停车
TASK_RESUME           = 6  预留恢复任务
TASK_NAVIGATE_PATH    = 7  按路径点导航
TASK_SET_DUMP_POSE    = 8  设置卸载桶位置
```

## 任务接收和互斥

行为树内部有一个 `_task_lock`。

含义：

1. 同一时间只允许一个主要任务执行。
2. 如果已有任务正在执行，新来的普通任务会被拒绝。
3. `TASK_STOP` 是特殊任务，即使已有任务运行，也允许进来。
4. 停止任务会设置 `_stop_requested = True`，同时发布空 `cmd_vel` 让底盘停止。

入口流程：

```text
收到 RobotTask Goal
    -> _handle_goal 判断任务类型是否合法
    -> _execute_task 抢任务锁
    -> _dispatch_task 根据 task_type 分发到具体行为
    -> 执行完成后发布停车指令
    -> 返回 success / message
```

## TASK_CLEAN_TRASH：视觉抓取垃圾

对应函数：

```text
_run_clean_trash_tree()
```

行为顺序：

```text
1. 机械臂回中位
   -> 调用 ArmPreset Action
   -> action_type = ARM_CENTER

2. 调用视觉识别服务
   -> 服务：vision_grasp/detect
   -> 类型：DetectGraspTarget
   -> 只负责识别并返回目标，不直接控制机械臂

3. 判断目标是否太远
   -> 使用 target_y_cm 判断前后距离
   -> 如果 target_y_cm <= approach_y_threshold_cm，认为可抓
   -> 默认阈值是 30 cm

4. 如果目标 y 超过 30 cm
   -> 底盘向前移动一小段
   -> 每次最大 approach_max_step_m，默认 0.08 m
   -> 然后重新识别
   -> 最多 approach_max_attempts 次

5. 目标可抓后，执行抓取
   -> 打开夹爪
   -> 调用 ArmTarget Action 做 MoveIt2 逆解/轨迹规划
   -> 闭合夹爪

6. 根据视觉返回的 bucket_id 投入车上桶
   -> bucket_id 为 1 或 2 时使用对应桶
   -> 否则默认桶 1
   -> 调用 ArmPreset Action 到桶位置
   -> 打开夹爪释放

7. 机械臂回中位
   -> 调用 ArmPreset Action
   -> 夹爪闭合到空闲状态
```

关键点：

- `robot_vision_grasp` 现在只做识别，不再自己抓取。
- 行为树根据识别结果决定是否前进、是否重新识别、何时抓取。
- 这就是后续行为树能做“目标太远就先挪底盘再抓”的原因。

## TASK_DUMP_TRASH：倒垃圾

对应函数：

```text
_run_dump_trash_tree()
_dump_single_bucket()
```

`bucket_id` 含义：

```text
bucket_id = 1  只倒卸载桶 1
bucket_id = 2  只倒卸载桶 2
bucket_id = 0  依次倒卸载桶 1，再倒卸载桶 2
```

语音“倒垃圾”现在发送的是：

```text
TASK_DUMP_TRASH + bucket_id = 0
```

所以行为树会按顺序执行：

```text
倒桶 1 -> 倒桶 2
```

单个桶的完整行为顺序：

```text
1. 读取该桶在地图上的卸载位置
   -> 来自 TASK_SET_DUMP_POSE 保存的桶位
   -> 如果桶位没设置，任务失败

2. 计算“目标位置前方”的导航点
   -> 桶位是用户在地图上标定的位置
   -> 行为树不会直接导航到桶中心
   -> 会退后 dump_approach_distance_m
   -> 默认退后 0.45 m

   计算方式：
   front_x = bucket_x - cos(bucket_yaw) * 0.45
   front_y = bucket_y - sin(bucket_yaw) * 0.45
   front_yaw = bucket_yaw

3. 调用 Nav2 导航到桶前方
   -> Action：navigate_to_pose
   -> 类型：NavigateToPose

4. 到达桶前方后，调用 dump_task.py 的识别服务精确调整
   -> 服务：detect_dump_bucket
   -> 类型：std_srvs/Trigger
   -> 这个服务使用深度相机彩色图和深度图识别卸载桶

5. 如果识别服务返回 True
   -> 说明桶颜色被检测到，并且平均深度 <= 20 cm
   -> 认为距离足够近

6. 如果识别服务返回 False
   -> 行为树让底盘向前挪一点
   -> 每次 dump_align_step_m，默认 0.03 m
   -> 然后再次调用 detect_dump_bucket
   -> 最多 dump_align_max_attempts 次，默认 10 次

7. 对准成功后，底盘原地旋转 180 度
   -> 发布 /cmd_vel angular.z
   -> 默认角速度 dump_rotate_angular_speed = 0.5 rad/s
   -> 当前实现是按时间积分旋转 pi 弧度
   -> 没有读取 IMU 或 odom 做闭环校正

8. 调用 dump_bridge.py 的倾倒服务
   -> 服务：dump_trash
   -> 类型：DumpTrash
   -> 参数：bucket_id

9. 一个桶完成后，如果是 bucket_id=0
   -> 继续执行下一个桶
```

当前倾倒行为的限制：

- `dump_task.py` 现在只返回 True/False，没有返回左右偏移量。
- 所以行为树只能做“向前小步靠近”，不能做左右横向精确对准。
- 180 度旋转目前是开环时间控制，不是 IMU/odom 闭环。

后续如果要更智能，可以把 `dump_task.py` 服务扩展成：

```text
bool detected
float64 depth_m
float64 offset_x_px
float64 offset_y_px
```

这样行为树就能根据视觉偏差做左右和角度调整。

## TASK_SET_DUMP_POSE：设置卸载桶位置

对应函数：

```text
_run_set_dump_pose()
```

来源：

```text
前端导航页面
    -> 点击“设定卸载桶1位置”或“设定卸载桶2位置”
    -> 在地图上按下并拖拽
    -> 起点是桶位置
    -> 拖拽方向是桶朝向 yaw
    -> Flask 调用 RobotTask
    -> 行为树保存桶位
```

保存的数据：

```text
bucket_id
x
y
yaw
frame_id
```

保存位置：

```text
/home/xjw/ros2_robot/src/robot_behavior_tree/config/dump_bucket_poses.json
```

行为树启动时会读取这个 JSON。

所以：

1. 前端设过桶位后，主页按钮可以直接倒对应桶。
2. 语音“倒垃圾”也可以直接倒桶 1 和桶 2。
3. 行为树重启后，只要 JSON 还在，桶位仍然存在。

## TASK_CLEAN_REGION：清扫指定地图区域

对应函数：

```text
_run_clean_region_tree()
```

行为顺序：

```text
1. 接收前端地图框选区域
   -> min_x
   -> min_y
   -> max_x
   -> max_y
   -> frame_id

2. 调用覆盖清扫服务
   -> 服务：add_coverage_rectangle
   -> 类型：AddCoverageRectangle

3. 具体路径生成/覆盖清扫由下游服务实现
   -> 行为树这里只负责把上层区域任务转交给覆盖清扫模块
```

当前行为树没有在这个任务里直接控制 Nav2 逐点走覆盖路径，覆盖路径逻辑在 `add_coverage_rectangle` 服务后面。

## TASK_NAVIGATE_TO：导航到单点

对应函数：

```text
_run_navigate_tree()
```

行为顺序：

```text
1. 从 RobotTask 读取：
   -> x
   -> y
   -> yaw
   -> frame_id

2. 组装 NavigateToPose Goal

3. 调用 Nav2：
   -> Action：navigate_to_pose
   -> 类型：NavigateToPose

4. 等待导航结果

5. 成功返回 navigate to pose completed
```

这个任务主要给前端地图“单点导航”使用。

## TASK_NAVIGATE_PATH：按路径点导航

对应函数：

```text
_run_navigate_path_tree()
```

行为顺序：

```text
1. 从 RobotTask.path 读取多个 PoseStamped

2. 如果路径点为空，直接失败

3. 调用 Nav2：
   -> Action：navigate_through_poses
   -> 类型：NavigateThroughPoses

4. Nav2 按顺序经过多个路径点

5. 最后一个点是最终目标
```

这个任务主要给前端地图“按下拖拽设置路径点”使用。

## TASK_STOP：停止

对应函数：

```text
_dispatch_task()
_execute_task()
_handle_cancel()
```

行为：

```text
1. 设置 _stop_requested = True
2. 发布空 Twist 到 /cmd_vel
3. 当前正在运行的任务会在等待、移动、服务调用循环中检查 _is_canceling()
4. 检查到停止后，尽快退出
```

注意：

- 如果底层 action/service 自己没有立即响应 cancel，行为树只能尽快停止自己的后续步骤。
- 底盘速度会被置零。

## TASK_RESUME：恢复

当前只是预留：

```text
return True, 'resume accepted'
```

还没有真正恢复上一个任务的状态。

以后如果要做暂停/恢复，需要额外保存：

```text
当前任务类型
当前步骤
当前目标点
当前桶编号
当前抓取目标
```

## 公共调用函数

### `_call_service()`

统一调用 service。

行为：

```text
1. 判断是否正在停止/取消
2. 等待 service 可用
3. 异步调用 service
4. 等待结果
5. 超时或异常返回 None
```

被这些能力使用：

```text
vision_grasp/detect
detect_dump_bucket
dump_trash
add_coverage_rectangle
arm_gripper
```

### `_send_action_goal_wrapped()`

统一调用 action。

行为：

```text
1. 判断是否正在停止/取消
2. 等待 action server 可用
3. 发送 goal
4. 等待 goal 被接受
5. 等待 action result
6. 超时则尝试 cancel
```

被这些能力使用：

```text
ArmTarget
ArmPreset
NavigateToPose
NavigateThroughPoses
```

### `_publish_stop()`

发布空速度：

```text
Twist()
```

作用：

```text
让底盘停止
```

### `_is_canceling()`

判断是否需要中断任务：

```text
_stop_requested == True
或
goal_handle.is_cancel_requested == True
```

底盘前进、倾倒对准、等待 future 时都会检查它。

## 当前任务调用关系总览

```text
前端主页 “卸载桶1”
    -> /api/robot/dump_bucket bucket_id=1
    -> RobotTask TASK_DUMP_TRASH bucket_id=1
    -> 行为树：导航到桶1前方
    -> detect_dump_bucket 对准
    -> 原地旋转 180 度
    -> dump_trash 服务倾倒桶1

前端主页 “卸载桶2”
    -> /api/robot/dump_bucket bucket_id=2
    -> RobotTask TASK_DUMP_TRASH bucket_id=2
    -> 行为树执行桶2倾倒流程

语音 “倒垃圾”
    -> RobotTask TASK_DUMP_TRASH bucket_id=0
    -> 行为树先倒桶1，再倒桶2

前端导航页 “设定卸载桶1位置”
    -> 地图拖拽设定 x/y/yaw
    -> RobotTask TASK_SET_DUMP_POSE bucket_id=1
    -> 行为树保存桶1位置

前端导航页 “设定卸载桶2位置”
    -> RobotTask TASK_SET_DUMP_POSE bucket_id=2
    -> 行为树保存桶2位置

前端地图路径导航
    -> RobotTask TASK_NAVIGATE_PATH
    -> 行为树调用 NavigateThroughPoses

前端地图区域清扫
    -> RobotTask TASK_CLEAN_REGION
    -> 行为树调用 add_coverage_rectangle

视觉抓取启动
    -> RobotTask TASK_CLEAN_TRASH
    -> 行为树识别、必要时前进、MoveIt2 抓取、投桶、归中
```

## 需要重点注意的设计边界

1. `robot_vision_grasp` 应只做识别，不应该直接控制机械臂和夹爪。
2. `dump_task.py` 应只做卸载桶检测，不应该直接控制底盘和倾倒机构。
3. `dump_bridge.py` 应只做倾倒机构动作，不应该自己导航。
4. 前端和语音只发高层任务，不直接调用底层服务。
5. 行为树统一决定下一步调度谁。

这样后续加复杂行为时，比如：

```text
识别不到 -> 换角度
目标太远 -> 底盘前进
目标偏左 -> 底盘微转
导航失败 -> 重新规划
桶位没设 -> 前端提示用户先设桶位
```

都应该放在行为树这一层，而不是塞进视觉、导航或倾倒服务内部。
