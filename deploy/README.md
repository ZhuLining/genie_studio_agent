# Ubuntu 22.04 部署说明

本部署文件用于第一阶段 mock executor，不调用 GDK，不控制机器人。

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
sudo -u gsa .venv/bin/python -m gsa_taskflow_executor.cli --print-config
sudo -u gsa .venv/bin/gsa-taskflow-executor --print-config
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
