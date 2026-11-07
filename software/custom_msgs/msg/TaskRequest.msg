string task_id
string language_instruction
string voice_instruction
int32 priority
bool enable_reverse

# --- mission extension (v2) ---
# task_type: vla / map / explore / search / navigate / return_home
string task_type
string target_label
float64 target_x
float64 target_y
float64 target_yaw
float64 max_duration
bool save_map
float64 target_coverage
