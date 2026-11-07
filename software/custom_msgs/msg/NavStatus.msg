std_msgs/Header header
# idle / navigating / succeeded / failed / canceled / blocked
string state
string goal_id
# nav2 / direct / passthrough
string backend
geometry_msgs/PoseStamped goal
geometry_msgs/PoseStamped current_pose
float64 distance_remaining
float64 eta_sec
string detail
