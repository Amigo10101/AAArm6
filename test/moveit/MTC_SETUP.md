# MoveIt Task Constructor Setup for aar6

## Overview
This setup adds MoveIt Task Constructor (MTC) support to your aar6 robot configuration.

## What is MTC?
MoveIt Task Constructor allows you to:
- Create complex motion planning pipelines
- Chain multiple motion stages together
- Build pick-and-place tasks
- Create reusable motion templates
- Visualize task hierarchies in RViz

## Installation

### 1. Install MTC packages (in Docker container)
```bash
docker exec -it ros2_jazzy bash
sudo apt update
sudo apt install ros-jazzy-moveit-task-constructor-core \
                 ros-jazzy-moveit-task-constructor-msgs \
                 ros-jazzy-moveit-task-constructor-visualization
```

### 2. Build your workspace
```bash
cd ~/ros2_ws
colcon build --packages-select moveit
source install/setup.bash
```

## Usage

### Running the Simple Example
The simple example moves through 3 waypoints:

```bash
# Start your MoveIt setup first
ros2 launch moveit demo.launch.py

# In another terminal (in Docker):
docker exec -it ros2_jazzy bash
cd ~/ros2_ws
source install/setup.bash
ros2 run moveit simple_mtc.py
```

### Running the Full Pick-and-Place Example
```bash
ros2 run moveit mtc_demo.py
```

## Available Examples

### 1. `simple_mtc.py`
- Basic sequential motion planning
- Moves through multiple joint-space waypoints
- Good starting point for learning MTC

### 2. `mtc_demo.py`
- Full pick-and-place pipeline
- Demonstrates:
  - Approach motions
  - Grasp generation
  - Object attachment
  - Lift and place
  - Gripper control

## Visualization in RViz

To see MTC task visualization:

1. In RViz, click "Add" → "By topic"
2. Find `/mtc/solutions` or `/task_constructor/solutions`
3. Add the MarkerArray display
4. You'll see the planned trajectory stages

## Modifying the Examples

### Change Waypoints (simple_mtc.py)
Edit the joint values in `build_simple_task()`:
```python
joint_values = {
    'L1': 0.5,   # Change these values
    'L2': 0.3,
    'L3': -0.5,
    # ...
}
```

### Add More Stages
```python
# Add a new waypoint
new_waypoint = stages.MoveTo("my waypoint", core.JointInterpolationPlanner())
new_waypoint.setGroup(self.group_name)
new_waypoint.setGoal(my_joint_values)
self.task.add(new_waypoint)
```

## Troubleshooting

### "moveit_task_constructor_core not found"
Install the MTC packages (see Installation step 1)

### "Task planning failed"
- Check that all joint limits are valid
- Ensure waypoints are reachable
- Verify no collisions at waypoints

### Scripts not executable
```bash
chmod +x ~/ros2_ws/src/moveit/scripts/*.py
```

## Next Steps

1. **Learn MTC stages**: Study the different stage types
   - `MoveTo`: Move to a goal pose/joints
   - `MoveRelative`: Move relative to current pose  
   - `GenerateGraspPose`: Generate grasp candidates
   - `ModifyPlanningScene`: Add/remove objects

2. **Create custom tasks**: Build your own task sequences

3. **Integrate with perception**: Add object detection

4. **Add error handling**: Handle planning/execution failures

## Resources

- MTC Documentation: https://moveit.github.io/moveit_task_constructor/
- MTC Tutorials: https://moveit.picknik.ai/main/doc/tutorials/task_constructor/task_constructor_tutorial.html
- Examples: https://github.com/moveit/moveit_task_constructor

## Current Velocity Limits

Your robot is configured with:
- Max velocity: 30.0 rad/s
- Max acceleration: 5.0 rad/s²
- Velocity scaling: 1.0 (100%)
- Acceleration scaling: 1.0 (100%)

These limits apply to MTC-planned trajectories as well!
