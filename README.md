# gps_goal_manager_igvc
GPS Goal Manager – What it does & how to use it

It reads a list of GPS waypoints (lat, lon) from a CSV.

These waypoints are converted GPS → UTM → map frame using robot_localization + TF.

Internally, it keeps track of one active waypoint at a time (mission-level state).

It publishes the current active global goal as:
Topic: /global_gps_goal
Type: geometry_msgs/PoseStamped
Frame: map

When the robot comes within ~1.0 m of the current goal (based on map → base_link TF),
it automatically advances to the next waypoint and publishes the new goal.

How you should use it (Nav2 side)
Subscribe to /global_gps_goal.
Treat it as the current high-level navigation target.

You can:
Forward it to Nav2’s goal interface, or
Use it as a reference for your own Nav2 logic (lane following, obstacle avoidance, etc.).
Orientation in the message is identity (yaw not enforced).

The node guarantees:
Only one active goal at a time
Goals are in the map frame
Next goal is published only after reaching the previous one
