string task_id
string target
string intent
float64 value
float64 confidence
bool reverse_request

# --- SLAM / navigation / object-search extension (v2) ---
# task_type: vla / map / search / navigate / return_home / explore
string task_type
# 目标物体标签（search 任务），例如 cup / bottle / person
string object_label
float64 goal_x
float64 goal_y
float64 goal_yaw
string waypoint
