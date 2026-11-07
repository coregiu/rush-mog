std_msgs/Header header
bool emergency_stop      # 安全模块当前是否处于急停
bool data_ok             # 传感器数据是否在超时窗口内
float64 min_obstacle_distance
string reason

# --- localization / power supervision (v2) ---
bool localization_ok
bool battery_ok
float64 battery_percent
