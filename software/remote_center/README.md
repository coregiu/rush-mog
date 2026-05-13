# 远程控制中心说明
远程控制中心是基于ESP32-S3实现的网页版手柄控制，ESP32-S3连接主控的uart2串口，将操作指令下发到主控，进而控制小车。
使用手机或其他移动设备连接ESP32-S3的WIFI，使用网页进行操作。

## 功能特性
- **实时视频流**: 通过摄像头实时传输视频画面
- **YOLO物体识别**: 支持实时物体检测，自动框选并标注物体名称和置信度
- **手柄控制**: 完整的PS2风格虚拟手柄控制
- **语音录制**: 支持录制语音并上传

# 控制按键说明
左侧8方向按钮，提供8个方向指令。同时在上方提供左转、右转、快速调头按钮。按钮按下触发，松开停止。
右侧4方向按钮，上下方向为加减速，点击生效，实现速度加减一档。 左右按钮为调方向按钮，通过PWM进行方向微调，按钮按下触发，松开恢复相同PWM。
中间部分提供速度展示框。

![ps2](ps2.png)

# YOLO物体识别使用说明
## 功能介绍
系统集成了基于TensorFlow.js的COCO-SSD物体检测模型，可在浏览器端实时识别视频流中的物体。支持识别80种常见物体类别，包括：
- 人物(person)、车辆(car, truck, bus)、动物(cat, dog, bird)等
- 日常物品(cell phone, cup, chair, table)等

## 使用方法
1. **开启摄像头**: 点击顶部"RUSH-MOG"Logo启动视频流
2. **启用YOLO**: 点击底部右侧的🎯按钮（绿色）开启物体识别
3. **查看检测结果**: 
   - 识别到的物体会被彩色方框标记
   - 方框上方显示物体类别和置信度（如"person 85%"）
   - 左下角状态栏显示当前检测到的物体信息

## ⚠️ 离线使用重要说明
由于ESP32的LittleFS存储空间有限（16MB），无法存储完整的AI模型文件（约5-10MB）。COCO-SSD模型需要从外部服务器下载权重文件。

### 解决方案：

#### 方案一：预先缓存模型（推荐）
1. 首次使用时，将设备连接到互联网
2. 打开网页，点击YOLO按钮加载模型
3. 模型会被浏览器缓存，之后可以在离线环境使用
4. 浏览器缓存位置：
   - Chrome: `chrome://settings/siteData`
   - 缓存通常在IndexedDB中

#### 方案二：部署模型到本地服务器
1. 下载COCO-SSD模型文件到本地服务器：
   ```
   模型地址: https://storage.googleapis.com/tfjs-models/savedmodel/ssd_lite_mobilenet_v2/
   需要下载的文件:
   - model.json
   - group1-shard1of1.bin (权重文件)
   ```
2. 修改`data/index.html`中的模型加载代码：
   ```javascript
   // 将模型URL改为您的本地服务器地址
   yoloModel = await cocoSsd.load({ 
       modelUrl: 'https://your-local-server.com/models/ssd_lite_mobilenet_v2' 
   });
   ```

#### 方案三：使用手机热点
1. 用手机开启热点并连接ESP32
2. 手机同时保持移动数据/WiFi连接互联网
3. 首次加载模型后，浏览器会缓存模型文件

### 注意事项
- 首次开启YOLO时需要下载模型文件（约5MB），请耐心等待"Loading model..."完成
- 物体识别在浏览器端运行，会消耗一定的设备性能
- 如果模型加载失败，会弹出详细的错误信息和解决方案
- 关闭视频流时，YOLO会自动停止检测
- 建议使用Chrome浏览器，其对TensorFlow.js支持最好
# 开发环境搭建说明

```
步骤 1：安装必要的工具
安装VS Code
安装Python 3.7+（确保勾选 "Add Python to PATH"）
安装 Git（Windows / Mac）

步骤 2：安装 PlatformIO 插件
打开 VS Code
进入扩展面板（左侧菜单或 Ctrl+Shift+X）
搜索 "PlatformIO" 并安装

步骤 3：开发编译：
点击vscode最下侧 √ 按钮进行编译。（或用 ctrl + alt + B 快捷键）

步骤 4：上传index.html文件
打开左侧在插件安装下方的PlatformIO头像图标，在di_mini/platform中选择 "Upload Filesystem Image"进行执行，上传文件。

步骤 5: 烧录程序
将ESP8266板子使用USB线连接到电脑。
点击vscode最下侧 -> 按钮进行编译。

步骤 6：监控串口
点击vscode最下侧 插头图标的 Serial Monitor 按钮进行编译。

步骤 7：手机连接wifi进行页面访问
在手机WIFI中找到RUSH-MOG，连接后，打开浏览器，输入192.168.4.1，即可访问到页面。

```