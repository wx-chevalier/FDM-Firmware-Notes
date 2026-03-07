# 在 macOS 上运行 Klipper 指南

> **核心提示**：Klipper 是为 Linux 内核设计的，严重依赖 Linux 的底层特性（如 precise timing, serial ioctl 等）。因此，在 macOS 上运行 Klipper 最稳定、最推荐的方式是使用 **轻量级 Linux 虚拟机 (VM)**。

虽然理论上可以在 macOS 原生 Python 环境中运行 Klipper 主机程序，但会面临 USB 串口通信不稳定、缺少 `epoll` 支持以及无法使用 `linux-mcu` 等问题。本指南将介绍目前最成熟的方案：**UTM + Debian + KIAUH**。

---

## 📋 方案概览

我们将在 macOS 上运行一个极轻量的 Debian Linux 虚拟机，并通过 **USB Passthrough (USB 直通)** 技术将 3D 打印机主板直接挂载给虚拟机。

- **宿主机**: macOS (Apple Silicon 或 Intel 均可)
- **虚拟化软件**: [UTM](https://mac.getutm.app/) (免费、开源、基于 QEMU，性能优异)
- **Guest OS**: Debian 12 (Bookworm) Server (无桌面版，资源占用极低)
- **安装工具**: KIAUH (Klipper Installation And Update Helper)

---

## 🛠️ 准备工作

1.  **下载 UTM**:
    - 访问 [UTM 官网](https://mac.getutm.app/) 下载最新版 (免费)。
    - _注：Mac App Store 版本为付费版，功能相同，建议官网下载。_

2.  **下载 Debian ISO**:
    - 访问 [Debian 下载页](https://www.debian.org/download)。
    - **Apple Silicon (M1/M2/M3)**: 下载 `arm64` 架构的 Netinst ISO。
    - **Intel Mac**: 下载 `amd64` 架构的 Netinst ISO。

---

## 🚀 详细步骤

### 第一步：创建虚拟机

1.  打开 UTM，点击 **"创建新虚拟机"**。
2.  选择 **"虚拟化 (Virtualize)"** (性能最佳)。
3.  选择 **"Linux"**。
4.  **引导镜像**: 点击 "浏览"，选择下载好的 Debian ISO 文件。
5.  **硬件配置**:
    - **内存**: 建议 `1024MB` (1GB) 或 `2048MB` (2GB)。Klipper 很轻量，1GB 足够。
    - **CPU**: `2` 核即可。
6.  **存储**: `20GB` - `32GB` 足够。
7.  **共享目录 (可选)**: 可以设置一个本地目录以便传文件。
8.  **命名**: 例如 "Klipper-Server"。
9.  点击保存。

### 第二步：安装 Debian Linux

1.  选中虚拟机，点击巨大的 **Play (▶)** 按钮启动。
2.  进入 Debian 安装界面 (Install)，按照提示操作：
    - **语言**: English (推荐) 或 Chinese。
    - **网络**: 默认即可。
    - **主机名**: `klipper` (随意)。
    - **Root 密码** & **用户账号**: 务必记住。
    - **分区**: "Guided - use entire disk"。
3.  **软件选择 (Software selection)**:
    - **关键**: 取消勾选 "Debian desktop environment" 和 "GNOME" (我们需要纯命令行服务器，节省资源)。
    - 勾选 **"SSH server"** 和 **"Standard system utilities"**。
4.  安装完成后，虚拟机将重启。
    - _注：重启前可能需要手动在 UTM 窗口右上角的光驱图标中 "清除" ISO 文件。_

### 第三步：配置网络与 SSH

1.  登录虚拟机 (使用刚才创建的用户)。
2.  查看 IP 地址：
    ```bash
    ip addr
    ```
    找到类似 `192.168.64.x` 的地址。
3.  打开 macOS 的 **终端 (Terminal)**，通过 SSH 连接虚拟机 (体验更好，方便复制粘贴)：
    ```bash
    ssh username@192.168.64.x
    # 输入密码登录
    ```

### 第四步：安装 Klipper (使用 KIAUH)

在 SSH 终端中执行以下操作：

1.  **安装 Git**:

    ```bash
    sudo apt-get update && sudo apt-get install git -y
    ```

2.  **克隆 KIAUH**:

    ```bash
    git clone https://github.com/dw-0/kiauh.git
    ```

3.  **运行 KIAUH**:

    ```bash
    ./kiauh/kiauh.sh
    ```

4.  **通过菜单安装**:
    - 选择 `1) [Install]`。
    - 依次安装推荐组件：
      1.  **Klipper** (Python 3)
      2.  **Moonraker** (API Server)
      3.  **Mainsail** 或 **Fluidd** (Web 界面，二选一)
    - 安装过程中会提示输入 sudo 密码，一路回车确认即可。

### 第五步：USB 设备直通 (关键)

为了让虚拟机里的 Klipper 能控制打印机，必须把 USB 端口“直通”给它。

1.  **连接打印机**: 将 3D 打印机通过 USB 线连接到 Mac。
2.  **配置 UTM**:
    - 在 UTM 虚拟机窗口顶部的工具栏，点击 **USB 图标**。
    - 在下拉列表中找到你的打印机 (通常显示为 Serial 芯片名，如 `QinHeng Electronics HL-340` 或 `STM32...`)。
    - 点击它，勾选状态即表示已挂载到虚拟机。
    - _提示：每次重启虚拟机或插拔 USB 后，可能需要重新勾选。_
3.  **验证连接**:
    在 SSH 中输入：
    ```bash
    ls /dev/serial/by-id/*
    ```
    如果能看到类似 `/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0` 的路径，说明连接成功！

### 第六步：配置与运行

1.  **访问 Web 界面**:
    - 在 macOS 浏览器 (Safari/Chrome) 中输入虚拟机的 IP 地址 (如 `http://192.168.64.5`)。
    - 你应该能看到 Mainsail/Fluidd 的界面。

2.  **配置 `printer.cfg`**:
    - 在 Web 界面中找到 "Machine" 或 "Configuration" 标签。
    - 编辑 `printer.cfg`。
    - 将 `[mcu]` 段落下的 `serial` 修改为第五步中获取的 ID：
      ```ini
      [mcu]
      serial: /dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0
      ```
    - 根据你的打印机型号，从 [Klipper Configs](https://github.com/Klipper3d/klipper/tree/master/config) 复制相应的配置内容。

3.  **重启 Klipper**:
    - 点击 Web 界面右上角的 "Firmware Restart"。
    - 如果一切正常，状态将变为 "Ready"。

---

## ⚠️ 常见问题 (FAQ)

### 1. 为什么不用 Docker for Mac?

Docker on macOS 本质上也是运行在一个轻量级 Linux VM 中，但 Docker Desktop 对 USB 设备的直通支持（USB Passthrough）非常麻烦，通常无法直接将宿主机的 `/dev/tty.*` 映射到容器内部供 Klipper 使用。相比之下，UTM 的 USB 直通功能非常稳定且易于点击操作。

### 2. 虚拟机休眠问题

macOS 休眠时，虚拟机也会挂起，导致打印中断。

- **解决方案**: 打印期间使用 [Amphetamine](https://apps.apple.com/us/app/amphetamine/id937984704) 等软件防止 Mac 休眠。
- 或者在 "系统设置" -> "显示器" -> "高级" 中开启 "防止自动休眠"。

### 3. 如何访问虚拟机中的摄像头?

如果你有 USB 摄像头用于监控打印：

1.  同样使用 UTM 的 USB 直通功能将摄像头挂载给虚拟机。
2.  在 KIAUH 中安装 `Crowsnest` (Webcam 串流服务)。
3.  配置 `crowsnest.conf`。

### 4. 性能如何?

Apple Silicon (M1/M2/M3) 运行 ARM64 Debian 效率极高，几乎不消耗宿主机资源。即使是老款 Intel Mac，运行无界面的 Linux Server 也是轻而易举。

---

## 🔗 参考资源

- [Klipper 官方文档](https://www.klipper3d.org/)
- [KIAUH GitHub](https://github.com/dw-0/kiauh)
- [UTM 虚拟机下载](https://mac.getutm.app/)
