# SLAM

## 📦 Prerequisites

- Ubuntu 22.04  
- ROS 2 Humble installed and sourced  
- `turtlebot3_description`, `turtlebot3_gazebo`, and other core TurtleBot3 packages  

---

## 🔧 Installing Dependencies

1. **Update apt cache**  
   ```bash
   sudo apt update
   ```

2. **Install extra system deps**  
   Make sure you have `apt_requirements.txt` in the workspace root containing:
   ```
   xterm
   ```
   Then run:
   ```bash
   xargs sudo apt install -y < apt_requirements.txt
   ```

3. **Install all ROS 2/Gazebo/TurtleBot3 deps**  
   ```bash
   sudo rosdep init || true
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```
