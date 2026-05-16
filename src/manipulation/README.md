# Manipulation W6 Bringup

This package is the Week 6 starting point for the Isaac Sim mobile
manipulator.

The current ROS2 environment does not expose MoveIt2 packages, so the first
validation path publishes `trajectory_msgs/JointTrajectory` directly to the
Isaac Sim arm bridge. After MoveIt2 is installed and the arm model is fixed,
the same controller topics can be reused from MoveIt2.

## Expected Isaac Sim Setup

- AMR base is already stable with the W5 LiDAR-only navigation scene.
- A robot arm USD is mounted on the AMR.
- The arm root frame is represented as `arm_base_link` or passed via launch
  argument.
- Isaac Sim publishes `/joint_states`.
- Isaac Sim subscribes to the configured arm and gripper trajectory topics.

Default topics:

- `/arm_controller/joint_trajectory`
- `/gripper_controller/joint_trajectory`
- `/joint_states`
- `/cmd_vel`

## Build

```bash
cd ~/isaac_amr_ws
colcon build --packages-select manipulation
source install/setup.bash
```

## Run Franka Joint Command Sequence

If Isaac Sim is configured with:

- `/franka/joint_states`
- `/franka/joint_command` using `sensor_msgs/msg/JointState`

run:

```bash
ros2 launch manipulation franka_basic_motion.launch.py auto_start:=true
```

This publishes:

1. home
2. pre_pick
3. pick
4. gripper close
5. lift
6. pre_place
7. place
8. gripper open
9. home

To start manually:

```bash
ros2 launch manipulation franka_basic_motion.launch.py auto_start:=false
ros2 service call /franka_basic_motion_node/start std_srvs/srv/Trigger {}
```

Tune the hard-coded joint poses in `config/franka_basic_motion.yaml`.

## Run MoveIt2 With Isaac Sim

Use this as the main W6 path after `/franka/joint_states` and
`/franka/joint_command` are available from Isaac Sim:

```bash
ros2 launch manipulation moveit_isaac_franka.launch.py use_sim_time:=true
```

In RViz MotionPlanning:

1. Select `panda_arm`.
2. Set start state to current.
3. Set a valid goal such as `ready` or `extended`.
4. Click Plan, then Execute.

The launch starts a `FollowJointTrajectory` action bridge that converts MoveIt2
execution commands to `/franka/joint_command`.

## Configuration

Edit `config/franka_basic_motion.yaml` for direct JointState command tests, and
`config/follow_joint_trajectory_bridge.yaml` for the MoveIt2 action bridge.
