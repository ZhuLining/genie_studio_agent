# gsa_taskflow_executor

自研 GSA Taskflow Executor。第一阶段目标是作为 mock / dry-run 执行器接入自研 GSA 客户端：

```text
自研客户端 -> publish taskflow/taskflow_yaml
-> gsa_taskflow_executor 接收 YAML
-> 解析 / 校验 / mock 执行
-> publish taskflow/{aid}/status
-> 客户端更新节点状态和日志
```

第一阶段不调用真实 GDK，不控制机器人运动。

## 部署目标

```text
OS: Ubuntu 22.04 LTS
Python: 3.10+
Process manager: systemd
Runtime mode: mock / dry-run
```

## 本地开发

```bash
python3.10 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e ".[dev]"
```

检查配置：

```bash
gsa-taskflow-executor --print-config
```

使用 `.env` 文件检查配置：

```bash
gsa-taskflow-executor --env-file .env.example --print-config
```

查看当前 Skill Registry：

```bash
gsa-taskflow-executor --env-file .env.example --print-skills
```

写入一条示例 JSONL 运行日志：

```bash
gsa-taskflow-executor --env-file .env.example --write-sample-log
```

在 G2 真机环境运行一次只读 GDK 探针：

```bash
gsa-taskflow-executor --env-file .env.example --gdk-readonly-probe
```

该命令只会懒加载 `agibot_gdk` 并调用已实测的只读接口：

```text
agibot_gdk.Robot().get_joint_states()
```

它会打印 JSON 摘要并写入一条 `gdk_readonly_probe` JSONL 事件。输出字段包括：

```text
available
backend
joint_count
joint_names
left_arm_joint_names
right_arm_joint_names
nonzero_error_joints
raw
```

本地没有 `agibot_gdk` 时命令会正常退出，并输出 `available: false` 与错误原因。
该探针不接入 MQTT taskflow 主链路，不调用运动控制，不改变 `motion_plan_skill` mock 行为。

在 G2 真机环境运行一次手动 GDK 控制探针：

```bash
ENABLE_GDK_CONTROL=1 \
CONFIRM_GDK_CONTROL=HOLD_CURRENT_DUAL_ARM \
gsa-taskflow-executor --env-file .env.example --gdk-control-probe hold_current
```

当前只开放两个白名单动作：

```text
hold_current          双臂 14 维当前位置保持
nudge_right_j7_0p005  右臂 J7 +0.005 rad 后回原位
```

`nudge_right_j7_0p005` 需要使用对应确认 token：

```bash
ENABLE_GDK_CONTROL=1 \
CONFIRM_GDK_CONTROL=NUDGE_RIGHT_J7_0P005 \
gsa-taskflow-executor --env-file .env.example --gdk-control-probe nudge_right_j7_0p005
```

控制探针会显式调用 `agibot_gdk.gdk_init()` / `gdk_release()`，并在动作前后做只读预检、
关节限位校验和位置差值复核。实测有效控制调用形态：

```text
robot.move_arm_joint(positions14, velocities14, 2)
positions14 = 左臂 7 关节 + 右臂 7 关节
velocities14 = 14 维速度
control_group = 2
```

该控制探针仍不接入 MQTT taskflow 主链路，不开放 `adapter: gdk`。未设置双环境变量确认时，
命令会在导入 `agibot_gdk` 前拒绝执行，并写入 `gdk_control_probe` JSONL 事件。

监听 `taskflow/taskflow_yaml`：

```bash
gsa-taskflow-executor --env-file .env.example --listen
```

监听模式会订阅 YAML、解析 Taskflow、按 DAG 顺序进入 mock Skill Runtime。第一阶段不会调用 GDK，也不会控制机器人运动。

当前已接入 Taskflow YAML Parser。收到 YAML 后会额外输出解析摘要：

```text
parsed taskflow app_execution_id=... nodes=... workers=... transitions=...
```

当前已接入 DAG Scheduler。解析成功后会额外输出调度路径：

```text
skill runtime schedule outcome=success terminal=结束 steps=3 path=开始 -> 位姿调整-位控 -> 结束
```

该调度器负责节点顺序、transition 选择和变量写入。真实技能调用由 Skill Runtime 封装；第一阶段只有 mock skill。

当前已接入 VariableStore。dry-run 调度过程中会维护运行时变量空间：

```json
{
  "variables": {
    "位姿调整-位控": {
      "detail": {
        "status": "success",
        "mode": "dry-run"
      }
    }
  }
}
```

支持解析变量引用：

```text
$.variables.二维码定位.detail.action_data.抓取点A
```

当前已接入 Skill Runtime：

```text
assign              解析 assignments，写入节点 detail.outputs.assignments
motion_plan_skill   mock 解析 ABS_JOINT 参数，输出 final_joint / motion_targets / resolved_params_template
```

当前已接入配置化 Skill Registry。默认内置配置等价于：

```yaml
skills:
  motion_plan_skill:
    adapter: mock
    mock_type: motion_plan
```

也可以通过环境变量指定配置文件：

```text
SKILL_REGISTRY_FILE=skills.example.yaml
```

第一阶段 registry 只允许 `adapter: mock`。`gdk`、`python_script`、`http`
这类真实 adapter 暂不启用，避免误触机器人或外部算法进程。

当前已接入状态回传。每个节点会按生命周期发布：

```text
RUNNING -> OVER
RUNNING -> ERROR
```

状态统一发布到：

```text
taskflow/{aid}/status
```

单个节点回传 payload 示例：

```json
{
  "aid": "gsa-dev",
  "app_execution_id": "977ddeb3-3a42-4027-9e6f-5a11bbb6ced9",
  "task_state": "OVER",
  "status": "OVER",
  "executor_mode": "mock",
  "sub_task": {
    "node_id": "位姿调整-位控",
    "node_name": "位姿调整-位控",
    "state": "OVER",
    "status": "OVER",
    "outputs": {
      "final_joint": [0.282, -1.039, -0.304, -1.751, -0.621, -0.169, 1.122]
    }
  }
}
```

`sub_task.outputs`、`sub_task.detail`、`sub_task.variables` 会进入自研客户端执行历史，便于查看二维码定位结果、位姿调整参数和变量快照。

运行测试：

```bash
pytest
```

也可以执行：

```bash
bash scripts/check.sh
```

## 端到端联调

联调范围：

```text
本机 MQTT broker -> executor 订阅 taskflow/taskflow_yaml
-> mock 执行 -> executor 发布 taskflow/{aid}/status
-> probe 收到 RUNNING / OVER / ERROR 状态
```

该联调不调用 GDK，不控制机器人。

准备本机 broker 后，终端 1 启动 executor：

```bash
gsa-taskflow-executor --env-file .env.local --listen
```

终端 2 发布样例 YAML 并等待状态回传：

```bash
python -m gsa_taskflow_executor.e2e_probe \
  --broker-url mqtt://127.0.0.1:1883 \
  --status-topic taskflow/gsa-dev/status
```

也可以使用脚本入口：

```bash
python scripts/e2e_local_mqtt.py \
  --broker-url mqtt://127.0.0.1:1883 \
  --status-topic taskflow/gsa-dev/status
```

预期会看到 8 条左右状态：

```text
[01] taskflow/gsa-dev/status task_state=RUNNING node=- state=-
[02] taskflow/gsa-dev/status task_state=RUNNING node=开始 state=RUNNING
[03] taskflow/gsa-dev/status task_state=OVER node=开始 state=OVER
...
received 8 status payloads
```

样例 YAML 位于：

```text
examples/right_arm_abs_joint.yaml
```

如果你的 `.env.local` 改了 `EXECUTOR_AID`，`--status-topic` 也要同步改成 `taskflow/{EXECUTOR_AID}/status`。

## Ubuntu 22.04 部署

部署文件位于：

```text
deploy/gsa-taskflow-executor.service
deploy/gsa-taskflow-executor.env.example
deploy/README.md
```

部署默认路径：

```text
/opt/gsa_taskflow_executor
/etc/gsa-taskflow-executor/gsa-taskflow-executor.env
/var/log/gsa-taskflow-executor
```

完整部署步骤见 `deploy/README.md`。

## 环境变量

复制 `.env.example` 后按现场环境调整：

```text
MQTT_BROKER_URL=mqtt://127.0.0.1:1883
MQTT_CLIENT_ID=gsa-taskflow-executor-dev
TASKFLOW_INPUT_TOPIC=taskflow/taskflow_yaml
TASKFLOW_STATUS_TOPIC_TEMPLATE=taskflow/{aid}/status
EXECUTOR_AID=gsa-dev
EXECUTOR_MODE=mock
EXECUTOR_LOG_DIR=logs
SKILL_REGISTRY_FILE=
```

配置说明：

```text
MQTT_BROKER_URL                 MQTT Broker 地址，第一阶段支持 mqtt/mock 等 scheme
MQTT_CLIENT_ID                  执行器连接 broker 时使用的客户端 ID
TASKFLOW_INPUT_TOPIC            执行器订阅的 YAML 下发 topic
TASKFLOW_STATUS_TOPIC_TEMPLATE  状态回传 topic 模板，必须包含 {aid}
EXECUTOR_AID                    当前执行器发布状态时使用的 aid
EXECUTOR_MODE                   第一阶段只支持 mock / dry-run
EXECUTOR_LOG_DIR                运行日志根目录
SKILL_REGISTRY_FILE             可选 Skill Registry YAML；为空时使用内置默认 registry
```

运行日志：

```text
stdout                          人可读运行日志
logs/executions/YYYYMMDD.jsonl  可复盘的结构化运行事件
```

## 当前阶段

已完成：

```text
1. Python 项目骨架
2. CLI 入口
3. 环境变量配置模型
4. 最小测试
5. .env 文件加载
6. 配置校验
7. stdout 日志
8. JSONL 运行事件写入
9. MQTT Gateway
10. 订阅 taskflow/taskflow_yaml
11. 发布 taskflow/{aid}/status 的基础方法
12. Taskflow YAML Parser
13. motion_plan_skill + ABS_JOINT 参数校验
14. DAG Scheduler dry-run
15. assign / worker 节点顺序调度
16. success transition 执行路径计算
17. VariableStore
18. 变量引用递归解析
19. 节点输出写入 variables.{output_var}.detail
20. Skill Runtime
21. assign mock skill
22. motion_plan_skill mock skill
23. worker 参数进入 runtime 后二次校验
24. taskflow/{aid}/status 状态回传
25. 节点 RUNNING / OVER / ERROR 生命周期事件
26. 节点 outputs / detail / variables 快照回传
27. 端到端 MQTT smoke probe
28. 样例 Taskflow YAML
29. Ubuntu 22.04 systemd 部署文件
30. 配置化 Skill Registry
31. skills.example.yaml
32. mock adapter 白名单
33. GDK 只读 CLI 探针
34. get_joint_states 摘要与 JSONL 事件记录
```

下一步：

```text
在 G2 上运行只读探针并补充 agibot_gdk_methods.md 的真实返回数据
```
