# Ubuntu 22.04 部署说明

本部署文件用于正式 MVP executor：客户端下发 Taskflow YAML，executor 调用已验证的
GDK `ABS_JOINT` 接口。上线前应先运行只读探针和手动控制探针，确认现场环境安全。

## 1. 系统准备

```bash
sudo apt update
sudo apt install -y python3 python3-venv python3-pip git
```

`gsa` 不是 Ubuntu 默认用户，是建议专门创建的 executor 运行用户。
先检查是否已存在：

```bash
id gsa
```

如果提示 `no such user`，创建该用户：

```bash
sudo useradd --system --home /opt/gsa_taskflow_executor --shell /usr/sbin/nologin gsa
```

如果机器上已经有你们自己的服务用户，也可以不用 `gsa`，但需要同步替换本文档和
`deploy/gsa-taskflow-executor.service` 里的：

```text
User=gsa
Group=gsa
```

如果服务器本机也需要跑 MQTT broker：

```bash
sudo apt install -y mosquitto mosquitto-clients
sudo systemctl enable --now mosquitto
```

## 2. 部署代码

```bash
sudo mkdir -p /opt/gsa_taskflow_executor
sudo chown -R gsa:gsa /opt/gsa_taskflow_executor
sudo -u gsa git clone <your-git-url> /opt/gsa_taskflow_executor
cd /opt/gsa_taskflow_executor
sudo -u gsa python3 -m venv .venv
```

> 说明：下面所有命令都显式使用 `.venv/bin/python` 或
> `.venv/bin/gsa-taskflow-executor`，因此部署脚本里不需要
> `source .venv/bin/activate`。`source` 只适合人工进入交互式调试 shell。

### 2.1 安装 Python 依赖

如果服务器可以直接访问 PyPI：

```bash
sudo -u gsa .venv/bin/python -m pip install --upgrade pip setuptools wheel
sudo -u gsa .venv/bin/python -m pip install -e .
```

如果因为国内网络访问 PyPI 失败，使用清华 PyPI 镜像：

```bash
sudo -u gsa .venv/bin/python -m pip install \
  -i https://pypi.tuna.tsinghua.edu.cn/simple \
  --trusted-host pypi.tuna.tsinghua.edu.cn \
  --upgrade pip setuptools wheel

sudo -u gsa .venv/bin/python -m pip install \
  -i https://pypi.tuna.tsinghua.edu.cn/simple \
  --trusted-host pypi.tuna.tsinghua.edu.cn \
  -e .
```

如果 `pip install -e .` 仍然卡在 build dependency 下载，可以在已经安装好
`setuptools` 和 `wheel` 后禁用 build isolation：

```bash
sudo -u gsa .venv/bin/python -m pip install \
  -i https://pypi.tuna.tsinghua.edu.cn/simple \
  --trusted-host pypi.tuna.tsinghua.edu.cn \
  --no-build-isolation \
  -e .
```

安装完成后先做本地验证：

```bash
sudo -u gsa .venv/bin/python -m gsa_taskflow_executor.cli \
  --env-file /etc/gsa-taskflow-executor/gsa-taskflow-executor.env \
  --print-config
sudo -u gsa .venv/bin/gsa-taskflow-executor \
  --env-file /etc/gsa-taskflow-executor/gsa-taskflow-executor.env \
  --print-config
```

`--print-config` 会显示 `taskflow_gdk_safety_gate`。正式 Taskflow 控制前必须确认：

```json
"taskflow_gdk_safety_gate": {
  "enabled": true,
  "confirmed": true,
  "expected_confirmation": "TASKFLOW_ABS_JOINT"
}
```

交互调试时可以手动激活 venv，但这不是 systemd 或部署命令的必要步骤：

```bash
source .venv/bin/activate
gsa-taskflow-executor --print-config
deactivate
```

## 3. 配置环境变量

```bash
sudo install -d -m 755 /etc/gsa-taskflow-executor
sudo install -m 640 -o root -g gsa \
  deploy/gsa-taskflow-executor.env.example \
  /etc/gsa-taskflow-executor/gsa-taskflow-executor.env
sudo install -m 640 -o root -g gsa \
  skills.example.yaml \
  /etc/gsa-taskflow-executor/skills.yaml
sudo vim /etc/gsa-taskflow-executor/gsa-taskflow-executor.env
```

至少确认：

```text
MQTT_BROKER_URL=mqtt://<broker-host>:1883
EXECUTOR_AID=<client-subscribed-aid>
EXECUTOR_MODE=gdk
SKILL_REGISTRY_FILE=/etc/gsa-taskflow-executor/skills.yaml
ENABLE_GDK_CONTROL=1
CONFIRM_GDK_CONTROL=TASKFLOW_ABS_JOINT
PAYLOAD_INCLUDE_FULL_VARIABLES=false
```

`EXECUTOR_AID` 必须和客户端订阅的 `gsa/self/{aid}/status` 一致。
`PAYLOAD_INCLUDE_FULL_VARIABLES` 现场建议保持 `false`，避免 status 和 JSONL 携带完整变量、
GDK raw 或相机大图；需要排障时优先查看摘要字段和节点输出 key。

Skill Registry 只允许 MVP 的 `motion_plan_skill` + `adapter: gdk`，以及用于验证调度扩展的
受控 `script_skill` + `adapter: gdk`。如果配置成其他 adapter 或非 MVP skill，服务会拒绝启动。
未设置 `ENABLE_GDK_CONTROL=1` 与 `CONFIRM_GDK_CONTROL=TASKFLOW_ABS_JOINT` 时，
taskflow 会在导入 GDK 前拒绝执行控制。

正式执行前建议先做只读与手动控制验证。完整现场闭环见
`docs/mvp_field_validation.md`：

```bash
sudo -u gsa .venv/bin/gsa-taskflow-executor \
  --env-file /etc/gsa-taskflow-executor/gsa-taskflow-executor.env \
  --health-check

sudo -u gsa .venv/bin/gsa-taskflow-executor \
  --env-file /etc/gsa-taskflow-executor/gsa-taskflow-executor.env \
  --gdk-readonly-probe

sudo -u gsa ENABLE_GDK_CONTROL=1 CONFIRM_GDK_CONTROL=HOLD_CURRENT_DUAL_ARM \
  .venv/bin/gsa-taskflow-executor \
  --env-file /etc/gsa-taskflow-executor/gsa-taskflow-executor.env \
  --gdk-control-probe hold_current

sudo -u gsa ENABLE_GDK_CONTROL=1 CONFIRM_GDK_CONTROL=NUDGE_LEFT_J7_0P005 \
  .venv/bin/gsa-taskflow-executor \
  --env-file /etc/gsa-taskflow-executor/gsa-taskflow-executor.env \
  --gdk-control-probe nudge_left_j7_0p005
```

## 4. 安装 systemd 服务

```bash
sudo install -m 644 deploy/gsa-taskflow-executor.service \
  /etc/systemd/system/gsa-taskflow-executor.service
sudo systemctl daemon-reload
sudo systemctl enable --now gsa-taskflow-executor
```

查看状态：

```bash
systemctl status gsa-taskflow-executor
journalctl -u gsa-taskflow-executor -f
```

## 5. 端到端 smoke

在部署机器上发布样例 YAML，并等待 `gsa/self/{aid}/status` 回传：

```bash
cd /opt/gsa_taskflow_executor
sudo -u gsa .venv/bin/python -m gsa_taskflow_executor.mqtt.e2e_probe \
  --broker-url mqtt://127.0.0.1:1883 \
  --status-topic gsa/self/gsa-dev/status
```

看到类似输出即代表 broker、executor、status topic 三段链路已通：

```text
[01] gsa/self/gsa-dev/status task_state=RUNNING node=- state=-
[02] gsa/self/gsa-dev/status task_state=RUNNING node=开始 state=RUNNING
[03] gsa/self/gsa-dev/status task_state=OVER node=开始 state=OVER
...
received 8 status payloads
```
