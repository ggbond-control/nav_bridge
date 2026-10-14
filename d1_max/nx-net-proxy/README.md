# NX 通过 D1M-B 访问外网

本目录备份 D1M-B 的代理程序依赖、Shell 脚本和 systemd 系统服务。
设备部署目录为 `/home/cat/Scripts/nx-net-proxy/`，服务名为
`nx-net-proxy.service`。服务以 `cat` 用户运行，开机自启，异常退出后
每 5 秒重试；开机时网口地址尚未就绪也会重试。

代理地址：`http://192.168.168.111:18080`。
仅允许 NX `192.168.168.100` 和 D1M-B `192.168.168.111` 使用。
目标域名由 D1M-B 解析，HTTP/HTTPS 经 D1M-B 当前外网出口访问。
不修改两台设备的 IP、路由、网关、DNS 或网络配置文件。

## 文件说明

- `nx-net-proxy.service`：安装到 `/etc/systemd/system/` 的系统服务。
- `run.sh`：供 systemd 调用的前台入口，不使用 nohup。
- `start.sh`、`stop.sh`：通过 systemctl 控制同一个服务，避免重复代理进程。
- `install.sh`：安装或更新服务，并启用开机自启。
- `proxy.py-2.4.10-py3-none-any.whl`：proxy.py 2.4.10 离线依赖，部署时解压到 `lib/`。
- `legacy/`：部署前原有的 nohup 启停脚本备份，仅供回溯和迁移时停止旧进程。

## D1M-B 日常操作

以下命令在 D1M-B 上以 `cat` 用户执行。启停需要 sudo 权限，可能提示
输入 `cat` 用户的系统密码；脚本、服务文件中不保存密码。

```bash
# Shell 方式：与 systemctl 控制的是同一个服务
sh ~/Scripts/nx-net-proxy/start.sh
sh ~/Scripts/nx-net-proxy/stop.sh

# systemd 方式
sudo systemctl start nx-net-proxy.service
sudo systemctl stop nx-net-proxy.service
sudo systemctl restart nx-net-proxy.service

# 查看状态、日志和开机自启状态
systemctl status nx-net-proxy.service --no-pager
journalctl -u nx-net-proxy.service -n 50 --no-pager
systemctl is-enabled nx-net-proxy.service
```

手动 stop 会停止当前服务，不会被自动重启，但下次开机仍会启动。
若需停止并取消开机自启：

```bash
sudo systemctl disable --now nx-net-proxy.service
```

恢复开机自启并立即启动：

```bash
sudo systemctl enable --now nx-net-proxy.service
```

## 从本目录重新部署

在本机本目录执行下列命令，将本目录的离线包及脚本复制到 D1M-B；
SSH 密码通过交互输入。

```bash
ssh -p 20004 cat@47.99.202.196 'mkdir -p ~/Scripts/nx-net-proxy/legacy'
scp -P 20004 run.sh start.sh stop.sh install.sh nx-net-proxy.service README.md \
  proxy.py-2.4.10-py3-none-any.whl cat@47.99.202.196:/home/cat/Scripts/nx-net-proxy/
scp -P 20004 legacy/*.sh cat@47.99.202.196:/home/cat/Scripts/nx-net-proxy/legacy/
ssh -t -p 20004 cat@47.99.202.196 'sh ~/Scripts/nx-net-proxy/install.sh'
```

安装脚本在缺少依赖时解压本地 wheel，不需要联网安装系统软件。
修改代理参数后 `sudo systemctl restart nx-net-proxy.service` 生效；
修改服务文件后重新运行安装脚本。

## NX 使用方式

先登录 D1M-B，再 `ssh robot@192.168.168.100`。
单次请求：

```bash
curl -x http://192.168.168.111:18080 https://www.baidu.com
```

让支持 HTTP 代理的程序在当前终端使用代理：

```bash
export http_proxy=http://192.168.168.111:18080
export https_proxy=http://192.168.168.111:18080
```

不要写入 `.bashrc`、`/etc/environment` 等持久配置。
关闭终端即失效；如原变量为空，也可 `unset http_proxy https_proxy`。
sudo 可能过滤环境变量，APT 建议按命令指定：

```bash
sudo apt-get \
  -o Acquire::http::Proxy=http://192.168.168.111:18080 \
  -o Acquire::https::Proxy=http://192.168.168.111:18080 \
  update
```

此方案支持 HTTP/HTTPS，不提供 ping/ICMP、UDP 或不支持代理程序的透明联网。
HTTPS 使用 CONNECT 隧道，不解密 TLS，也不需要安装证书。

## 恢复到无服务状态

```bash
sudo systemctl disable --now nx-net-proxy.service
sudo rm /etc/systemd/system/nx-net-proxy.service
sudo systemctl daemon-reload
```

保留部署目录供重新安装。移除服务不改变网络配置。
