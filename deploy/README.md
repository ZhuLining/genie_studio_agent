# Ubuntu 22.04 部署说明

本部署文件用于当前 G2 真机 executor：客户端通过 MQTT 下发 Taskflow YAML 或二维码建图工具请求，
executor 统一调用 GDK、二维码建图 SDK 与二维码定位 / 点位录制 SDK，并把正式产物写入 Ubuntu 侧
`GSA_DATA_ROOT`。

当前已验证闭环：

```text
二维码建图工具 -> 二维码建图 -> 点位录制
应用编排 -> 开始 -> 二维码定位 -> 位姿调整-位控 -> 结束
```

上线前应先完成预检、只读探针和必要的手动控制探针。完整交付基线见
`docs/qr_pose_delivery_baseline.md`。

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

创建正式数据根：

```bash
sudo install -d -m 775 -o gsa -g gsa /data/gsa
```

如果服务器本机也需要跑 MQTT broker：

```bash
sudo apt install -y mosquitto mosquitto-clients
sudo systemctl enable --now mosquitto
```

G2 现场还需要先确认 GDK / DDS 环境脚本可用，并把最终环境固化为 systemd 可读取的
静态 `KEY=VALUE` 文件。`source` 只用于人工确认和生成候选值，正式服务不会继承交互
shell 环境：

```bash
source /home/u/.cache/agibot/app/env.sh
python3 -c 'import agibot_gdk; print("agibot_gdk import ok")'
```

## 2. 部署代码

```bash
sudo mkdir -p /opt/gsa_taskflow_executor
sudo chown -R gsa:gsa /opt/gsa_taskflow_executor
sudo -u gsa git clone <your-git-url> /opt/gsa_taskflow_executor
cd /opt/gsa_taskflow_executor
sudo -u gsa python3 -m venv .venv
```

如果现场的 `agibot_gdk` 只安装在系统 Python site-packages 中，优先把对应包路径写入
`/etc/gsa-taskflow-executor/gdk.env` 的 `PYTHONPATH`，或把 GDK wheel 明确安装进 `.venv`。
只有在现场无法取得明确包路径或 wheel 时，才考虑用 `python3 -m venv --system-site-packages .venv`
重建 venv，并在验收记录里注明该机器依赖系统 Python 包。

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

### 2.2 安装二维码 SDK

二维码建图和点位录制 / 二维码定位 SDK 运行在 executor 同一个 Ubuntu Python 环境中。
如果 SDK 已放在仓库 `sdk/` 目录下，可按如下方式安装：

```bash
sudo -u gsa /opt/gsa_taskflow_executor/.venv/bin/python -m pip install -e \
  /opt/gsa_taskflow_executor/sdk/qr_mapping_sdk

sudo -u gsa /opt/gsa_taskflow_executor/.venv/bin/python -m pip install -e \
  /opt/gsa_taskflow_executor/sdk/qr_localize_sdk
```

现场人工验证使用 `/home/u/project/gsa_taskflow_executor` 时，把上面的 `/opt/gsa_taskflow_executor`
替换为实际路径即可。

## 3. 配置环境变量

```bash
sudo install -d -m 755 /etc/gsa-taskflow-executor
sudo install -m 640 -o root -g gsa \
  deploy/gsa-taskflow-executor.env.example \
  /etc/gsa-taskflow-executor/gsa-taskflow-executor.env
sudo install -m 640 -o root -g gsa \
  deploy/gsa-taskflow-executor.gdk.env.example \
  /etc/gsa-taskflow-executor/gdk.env
sudo install -m 640 -o root -g gsa \
  skills.example.yaml \
  /etc/gsa-taskflow-executor/skills.yaml
sudo vim /etc/gsa-taskflow-executor/gsa-taskflow-executor.env
sudo vim /etc/gsa-taskflow-executor/gdk.env
```

至少确认：

```text
MQTT_BROKER_URL=mqtt://<broker-host>:1883
ALLOW_LOCAL_MQTT_BROKER=<broker-host 为 127.0.0.1/localhost 时填 1，否则留空>
EXECUTOR_AID=<client-subscribed-aid>
EXECUTOR_MODE=gdk
SKILL_REGISTRY_FILE=/etc/gsa-taskflow-executor/skills.yaml
GSA_DATA_ROOT=/data/gsa
QR_MAPPING_SDK_PATH=/opt/gsa_taskflow_executor/sdk/qr_mapping_sdk
QR_MAPPING_SDK_PYTHON=/opt/gsa_taskflow_executor/.venv/bin/python
QR_LOCALIZE_SDK_PATH=/opt/gsa_taskflow_executor/sdk/qr_localize_sdk
QR_LOCALIZE_SDK_PYTHON=/opt/gsa_taskflow_executor/.venv/bin/python
ENABLE_GDK_CONTROL=1
CONFIRM_GDK_CONTROL=TASKFLOW_ABS_JOINT
PAYLOAD_INCLUDE_FULL_VARIABLES=false
TASKFLOW_ABS_POSE_MAX_TRANSLATION_M=0.30
TASKFLOW_ABS_POSE_MAX_ROTATION_DEG=90
TASKFLOW_ABS_POSE_LIFE_TIME_SECONDS=0.02
```

`deploy/gsa-taskflow-executor.env.example` 故意把 `MQTT_BROKER_URL` 和 `EXECUTOR_AID`
留空，复制模板后必须填写。`EXECUTOR_AID=gsa-dev` 只允许本地开发，不允许进正式服务；
否则状态会上报到 `gsa/self/gsa-dev/status`，和客户端目标 AID 错配。若 MQTT broker
确实部署在 executor 同一台 Ubuntu 主机，才把 `MQTT_BROKER_URL` 配成
`mqtt://127.0.0.1:1883` 或 `mqtt://localhost:1883`，并显式设置
`ALLOW_LOCAL_MQTT_BROKER=1`。

`/etc/gsa-taskflow-executor/gdk.env` 至少要覆盖目标 G2 机器 `env.sh` 中影响
`agibot_gdk` 导入和 DDS 通信的最终值，例如：

```text
PYTHONPATH=<agibot_gdk Python 包路径；若已安装进 .venv 可留空或不设置>
LD_LIBRARY_PATH=<agibot_gdk 依赖的原生库路径>
CYCLONEDDS_URI=<如果 env.sh 设置了该项，填写最终值>
FASTRTPS_DEFAULT_PROFILES_FILE=<如果 env.sh 设置了该项，填写最终值>
RMW_IMPLEMENTATION=<如果 env.sh 设置了该项，填写最终值>
ROS_DOMAIN_ID=<如果 env.sh 设置了该项，填写最终值>
AMENT_PREFIX_PATH=<如果 env.sh 设置了该项，填写最终值>
```

`/etc/gsa-taskflow-executor/gdk.env` 只允许静态 `KEY=VALUE`，不要写 `source ...`、
`$LD_LIBRARY_PATH`、命令替换或凭据。`deploy/gsa-taskflow-executor.service` 默认仍启用
`ProtectHome=true`，但会把 G2 默认 `/home/u/.cache/agibot/app` 以只读 bind 方式暴露给
executor；如果现场 GDK 路径不同，请同步修改 service 的 `BindReadOnlyPaths`，或把 GDK
运行时安装/复制到 `/opt` 这类正式服务路径。

`EXECUTOR_AID` 必须和客户端订阅的 `gsa/self/{aid}/status` 一致。
`PAYLOAD_INCLUDE_FULL_VARIABLES` 现场建议保持 `false`，避免 status 和 JSONL 携带完整变量、
GDK raw 或相机大图；需要排障时优先查看摘要字段和节点输出 key。

Skill Registry 只允许当前已接入的 GDK 能力和受控白名单 skill，包括 `motion_plan_skill`、
`qr_pose_skill`、`control_end_effector_skill`、`force_control_skill` 和受控 `script_skill`。
如果配置成其他 adapter 或未注册 skill，服务会拒绝启动。
未设置 `ENABLE_GDK_CONTROL=1` 与 `CONFIRM_GDK_CONTROL=TASKFLOW_ABS_JOINT` 时，
taskflow 会在导入 GDK 前拒绝执行控制。

二维码建图、点位录制和应用二维码定位都依赖同一个 `GSA_DATA_ROOT`。客户端正式链路不选择本机
目录或本机地图文件，只展示 executor 返回的远端路径和项目快照。

配置完成后建议先跑交付预检。该脚本只检查配置、SDK 路径、Python 可执行文件、数据根可写性和
指定项目产物形态，不调用 GDK 运动接口，也不等价于 systemd 启动前的 `LD_LIBRARY_PATH`
注入检查：

```bash
sudo -u gsa /opt/gsa_taskflow_executor/.venv/bin/python \
  /opt/gsa_taskflow_executor/scripts/qr_pose_delivery_smoke.py \
  --env-file /etc/gsa-taskflow-executor/gsa-taskflow-executor.env \
  --robot-serial G2A0004BC01053 \
  --project-name test10 \
  --strict-safety-gate
```

正式执行前建议先做只读与手动控制验证。完整现场闭环见
`docs/mvp_field_validation.md`：

```bash
sudo systemd-run --wait --collect --pipe \
  -p User=gsa \
  -p WorkingDirectory=/opt/gsa_taskflow_executor \
  -p EnvironmentFile=/etc/gsa-taskflow-executor/gsa-taskflow-executor.env \
  /opt/gsa_taskflow_executor/.venv/bin/gsa-taskflow-executor \
  --deployment-config-check

sudo systemd-run --wait --collect --pipe \
  -p User=gsa \
  -p WorkingDirectory=/opt/gsa_taskflow_executor \
  -p EnvironmentFile=/etc/gsa-taskflow-executor/gsa-taskflow-executor.env \
  -p EnvironmentFile=/etc/gsa-taskflow-executor/gdk.env \
  /opt/gsa_taskflow_executor/.venv/bin/gsa-taskflow-executor \
  --gdk-env-check

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

如果现场使用 `GSA_DATA_ROOT=/home/u/gsa_data` 并通过 systemd 启动，需要同步修改
`deploy/gsa-taskflow-executor.service`。默认 service 的沙箱只显式放开 `/data/gsa` 和日志目录，
并且只读暴露 G2 默认 GDK app 目录，避免 executor 意外读写任意本机路径。

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
set -a
. /etc/gsa-taskflow-executor/gsa-taskflow-executor.env
set +a
sudo -u gsa .venv/bin/python -m gsa_taskflow_executor.mqtt.e2e_probe \
  --broker-url "$MQTT_BROKER_URL" \
  --status-topic "gsa/self/${EXECUTOR_AID}/status"
```

看到类似输出即代表 broker、executor、status topic 三段链路已通：

```text
[01] gsa/self/<client-subscribed-aid>/status task_state=RUNNING node=- state=-
[02] gsa/self/<client-subscribed-aid>/status task_state=RUNNING node=开始 state=RUNNING
[03] gsa/self/<client-subscribed-aid>/status task_state=OVER node=开始 state=OVER
...
received 8 status payloads
```
