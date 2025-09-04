# 远程控制中心说明
远程控制中心是基于ESP8266 NODEMCU实现的网页版手柄控制，ESP8266连接主控的uart2串口，将操作指令下发到主控，进而控制小车。
使用手机或其他移动设备连接ESP8266的WIFI，使用网页进行操作。

# 控制按键说明
左侧8方向按钮，提供8个方向指令。同时在上方提供左转、右转、快速调头按钮。按钮按下触发，松开停止。
右侧4方向按钮，上下方向为加减速，点击生效，实现速度加减一档。 左右按钮为调方向按钮，通过PWM进行方向微调，按钮按下触发，松开恢复相同PWM。
中间部分提供速度展示框。

# 开发环境搭建说明

```
步骤 1：安装必要的工具
安装VS Code
安装Python 3.7+（确保勾选 "Add Python to PATH"）
安装 Git（Windows / Mac）
步骤 2：安装 ESP-IDF 插件
打开 VS Code
进入扩展面板（左侧菜单或 Ctrl+Shift+X）
搜索 "ESP-IDF" 并安装由 Espressif Systems 提供的插件
步骤 3：配置 ESP-IDF 环境
安装完成后，按Ctrl+Shift+P打开命令面板
输入并选择 "ESP-IDF: Configure ESP-IDF Extension"
选择 "Express" 配置方式（适合初学者）
选择 ESP-IDF 版本（建议选择 v4.4 或更高版本，支持 ESP8266）
选择安装路径，等待工具链和框架下载安装（可能需要 30 分钟以上）
步骤 4：创建 ESP8266 项目
打开命令面板（Ctrl+Shift+P）
输入并选择 "ESP-IDF: New Project"
选择项目路径和名称
在 "Select board" 中搜索并选择你的 ESP8266 开发板（如 "NodeMCU 1.0 (ESP-12E Module)"）
选择一个模板（如 "blink" 示例）
步骤 5：配置项目目标为 ESP8266
打开命令面板
输入并选择 "ESP-IDF: Set Target"
选择 "esp8266" 作为目标芯片
步骤 6：编译和烧录项目
连接 ESP8266 开发板到电脑
点击 VS Code 左下角的 "选择端口"，选择你的设备端口
点击底部状态栏的 "Build" 按钮（hammer 图标）编译项目
点击 "Flash" 按钮（箭头图标）将程序烧录到设备
点击 "Monitor" 按钮（眼睛图标）查看串口输出
步骤 7：配置调试环境（可选）
如需调试，需要额外的调试器硬件（如 ESP-Prog）
连接调试器到开发板和电脑
打开命令面板，选择 "ESP-IDF: Debug Configuration"
选择合适的调试配置，点击 F5 开始调试
```