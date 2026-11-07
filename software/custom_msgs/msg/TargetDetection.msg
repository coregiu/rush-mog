# 单个目标检测结果（相机 + 2D 雷达/尺寸估距 + 位姿投影）。
std_msgs/Header header
string task_id
string label
string track_id
float64 confidence
# 归一化 bbox，范围 [0, 1]
float64 x_min
float64 y_min
float64 x_max
float64 y_max
# 相机坐标系下的方位角（右正）
float64 bearing_rad
# 估计距离，<=0 表示不可用
float64 distance_m
# 目标在 position_frame 下的位置
geometry_msgs/Point position
geometry_msgs/Point position_variance
string position_frame
# lidar / bbox_height / depth / none
string distance_source
