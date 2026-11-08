# ROS 系统 + 全部连接设备的健康快照。
# 发布：/center/health（TRANSIENT_LOCAL，约 1 Hz）
# 服务：/center/healthcheck（custom_msgs/srv/HealthCheck）
# HTTP：GET /ros/health（见 software/edge_server/HEALTHCHECK.md）
std_msgs/Header header

# 整体状态：
#   ok        所有已启用设备正常
#   starting  启动宽限期内，必需设备/节点尚未就绪
#   degraded  可降级运行（可选设备掉线、频率偏低、电量低、急停等）
#   error     必需设备/节点失效，不应继续执行任务
#   disabled  健康监控被配置关闭
string status
bool healthy                    # status == ok
bool ready                      # status == ok 或 degraded，且未急停

float64 uptime_sec
string service
string version
string reason                   # 非 ok 状态的摘要；ok 时为 "ok"

bool emergency_stop
string safety_reason
bool safety_available

uint32 device_count
uint32 ok_count
uint32 degraded_count
uint32 error_count
uint32 unknown_count
uint32 disabled_count

DeviceHealth[] devices
string ros_json                 # 节点发现/缺失节点等 ROS 运行信息 JSON
