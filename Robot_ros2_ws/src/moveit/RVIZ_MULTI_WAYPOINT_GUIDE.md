# RViz Motion Planning - Multi-Waypoint Guide

![Screenshot](/home/idgaf/.gemini/antigravity/brain/60d3fc47-bda8-41a7-97a0-725c3e69fa8e/uploaded_image_1764055409097.png)

## 🎯 How to Plan Multiple Waypoints in RViz

### Method 1: Use Predefined Waypoints (Easiest!)

I've added waypoint poses to your configuration. After rebuilding:

1. **Restart your setup**:
   ```bash
   docker compose down
   ./aar6.sh
   ```

2. **In RViz Motion Planning panel**:
   - Look for **"Goal State"** dropdown (in your screenshot it shows `<current>`)
   - Click the dropdown
   - You'll now see:
     - `Home` - Zero position
     - `arms_up` - Upright pose
     - `waypoint_1` - First demo position
     - `waypoint_2` - Second demo position
     - `waypoint_3` - Third demo position

3. **Plan a sequence**:
   ```
   Select "waypoint_1" → Click "Plan & Execute"
   Select "waypoint_2" → Click "Plan & Execute"  
   Select "waypoint_3" → Click "Plan & Execute"
   Select "Home" → Click "Plan & Execute"
   ```

---

### Method 2: Manual Interactive Planning

1. **Drag the colored sphere** (interactive marker) on the robot
2. Position it where you want
3. Click **"Plan & Execute"**
4. After motion completes, repeat from step 1

---

### Method 3: Use Joints Tab for Precise Control

1. Switch to **"Joints"** tab (next to "Planning" in your panel)
2. **Adjust sliders** for each joint (L1-L6)
3. Click **"Plan & Execute"**

This gives you **precise joint-space control**!

---

## 🚀 Better Option: Python Script for Sequences

For **true multi-waypoint** planning, use the script we created:

```bash
# In Docker terminal
ros2 run moveit simple_mtc.py
```

This automatically:
- Plans through waypoint_1 → waypoint_2 → waypoint_3 → Home
- Executes smoothly
- No manual clicking required!

---

## 🌐 Alternative: MoveIt Web Interface

For a **better UI** with multi-waypoint support:

### Install:
```bash
docker exec -it ros2_jazzy bash
sudo apt update
sudo apt install -y ros-jazzy-moveit2-tutorials
```

### Run:
```bash
# Terminal 1: Start MoveIt
ros2 launch moveit demo.launch.py

# Terminal 2: Start web interface  
ros2 launch moveit2_tutorials move_group_interface_tutorial.launch.py
```

Then open browser: `http://localhost:8080`

---

## 📝 Current Limitations

The standard RViz Motion Planning panel:
- ❌ **No built-in waypoint sequence planner**
- ❌ **No waypoint list/queue**
- ✅ **Can save/select named poses**
- ✅ **Good for single goal planning**

For multi-step tasks, you need either:
1. **Scripts** (like `simple_mtc.py`) ⭐ Recommended
2. **Web interface**
3. **Custom RViz plugin**

---

## 🎨 Your Current Setup

From your screenshot, you have:
- Planning Group: `kin_group` ✓
- Planning Time: 5.0s ✓
- Planning Attempts: 10 ✓
- Velocity Scaling: **0.40** (40% speed)
- Accel Scaling: **0.40** (40%)

To make it faster, increase those scaling factors in the UI!

---

## ✅ Quick Start After Rebuild

```bash
# Rebuild to load new waypoints
colcon build --packages-select moveit
source install/setup.bash

# Restart RViz (or restart demo.launch.py)
```

Then in RViz:
1. **Goal State dropdown** → Select `waypoint_1`
2. Click **"Plan & Execute"**
3. **Goal State dropdown** → Select `waypoint_2`
4. Click **"Plan & Execute"**
5. Continue...

This is the **easiest way** to plan multiple movements in the current RViz UI! 🚀
