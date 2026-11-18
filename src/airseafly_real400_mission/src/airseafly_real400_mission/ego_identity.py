"""Stable waypoint identity across ROS publishers; never use Header.seq."""


def goal_key(goal):
    if goal is None:
        return None
    stamp = goal.header.stamp
    # ROS Time supplies integer nanoseconds; the fallback supports offline doubles.
    ns = stamp.to_nsec() if hasattr(stamp, 'to_nsec') else int(round(stamp.to_sec() * 1e9))
    p = goal.pose.position
    return ns, goal.header.frame_id, p.x, p.y, p.z
