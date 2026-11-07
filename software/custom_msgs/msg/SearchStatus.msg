std_msgs/Header header
string task_id
# idle / waiting_map / exploring / confirming / approaching / found / failed / canceled
string state
string target_label
string best_track_id
float64 best_confidence
geometry_msgs/PoseStamped best_pose
float64 explored_ratio
float64 elapsed_sec
string detail
