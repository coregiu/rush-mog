# 定位健康度。robot_localization/AMCL 负责融合，本消息用于门控导航与告警。
std_msgs/Header header
bool healthy
# 0..1，低于阈值时导航节点拒绝 map 坐标系目标
float64 quality
float64 yaw_rad
float64 yaw_variance
float64 imu_rate_hz
float64 odom_rate_hz
bool imu_online
bool mag_ok
bool tf_ok
# amcl / tf / odom / none
string source
string detail
