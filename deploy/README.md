# Ubuntu 22.04 部署说明

本部署文件用于第一阶段 mock executor，不调用 GDK，不控制机器人。

## 1. 系统准备

```bash
sudo apt update
sudo apt install -y python3 python3-venv python3-pip git
sudo useradd --system --home /opt/gsa_taskflow_executor --shell /usr/sbin/nologin gsa
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
sudo -u gsa .venv/bin/python -m pip install --upgrade pip
sudo -u gsa .venv/bin/python -m pip install -e .
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
EXECUTOR_MODE=mock
SKILL_REGISTRY_FILE=/etc/gsa-taskflow-executor/skills.yaml
```

`EXECUTOR_AID` 必须和客户端订阅的 `taskflow/{aid}/status` 一致。

第一阶段 Skill Registry 只允许 `adapter: mock`。如果配置成 `gdk` 或
`python_script`，服务会拒绝启动，避免误触真实机器人能力。

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

在部署机器上发布样例 YAML，并等待 `taskflow/{aid}/status` 回传：

```bash
cd /opt/gsa_taskflow_executor
sudo -u gsa .venv/bin/python -m gsa_taskflow_executor.e2e_probe \
  --broker-url mqtt://127.0.0.1:1883 \
  --status-topic taskflow/gsa-dev/status
```

看到类似输出即代表 broker、executor、status topic 三段链路已通：

```text
[01] taskflow/gsa-dev/status task_state=RUNNING node=- state=-
[02] taskflow/gsa-dev/status task_state=RUNNING node=开始 state=RUNNING
[03] taskflow/gsa-dev/status task_state=OVER node=开始 state=OVER
...
received 8 status payloads
```
