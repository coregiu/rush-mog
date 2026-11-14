# 单个连接设备的健康状态（由 health_monitor_node 周期填充）。
#
# status 取值：
#   ok       正常
#   degraded 数据可用但质量下降（频率低、可选设备掉线、电量低等）
#   error    必需设备无数据/数据超时/数据非法，系统不应继续执行任务
#   unknown  可选设备没有反馈通道，或尚未收到第一帧
#   disabled 配置中显式关闭，不参与健康判定
std_msgs/Header header

string id                       # camera / lidar / imu / microphone / speaker / arm / base / battery / odom
string type                     # camera / lidar / imu / microphone / speaker / arm / base / battery / odometry / other
string status                   # ok / degraded / error / unknown / disabled
bool healthy                    # status == ok
bool ready                      # status == ok 或 degraded，可继续降级运行

bool enabled
bool required
bool connected                  # ROS graph 中存在发布者，或已经收到过数据
bool data_fresh                 # 最后一帧在 timeout_sec 内

string topic
string message_type
uint32 publisher_count

# 最后一条消息距现在的时间；从未收到过为 -1
float64 age_sec
# 最近 rate_window_sec 内的实测频率；样本不足为 -1
float64 rate_hz
float64 expected_rate_hz

string detail                   # 人类可读原因（机器判断请用 status）
string metrics_json             # 设备指标 JSON，如分辨率/电压/关节；可为空
