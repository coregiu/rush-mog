# 栅格地图摘要，由 map_manager_node 周期发布。
std_msgs/Header header
string map_id
# idle / mapping / localized / saved / error
string status
float64 resolution
uint32 width
uint32 height
geometry_msgs/Pose origin
# known_ratio = 已知栅格 / 全部栅格，作为建图覆盖率
float64 known_ratio
float64 free_ratio
uint32 frontier_count
string map_file
