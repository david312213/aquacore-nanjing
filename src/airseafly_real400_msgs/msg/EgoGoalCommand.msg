# On goal_position_cmd: the request that generated the sampled trajectory.
# On goal_with_start: the new goal and the controller reference to continue.
geometry_msgs/PoseStamped goal
quadrotor_msgs/PositionCommand command

# Optional per-leg map profile, applied atomically with a new goal.
bool apply_map_inflation
float64 map_inflation_m
