# Workspace Environment

- This workspace is mounted from the robot through SSHFS.
- The robot-side canonical path is `/userdata/dev_ws`; the local mount path is `/home/saioog/remote_dev_ws`.
- Never run `colcon build`, CMake builds, package builds, or other compilation commands in this mounted workspace.
- Do not delete or rewrite `build/`, `install/`, or `log/` artifacts in order to repair a build.
- The development machine and robot share the same ROS 2 network. Use read-only ROS 2 commands such as `ros2 node list`, `ros2 topic list`, `ros2 topic info`, `ros2 topic echo`, and `ros2 param get` to inspect the robot's live state.
- When compilation is required, provide the exact command for the user to run directly on the robot; do not execute it from this workspace.
