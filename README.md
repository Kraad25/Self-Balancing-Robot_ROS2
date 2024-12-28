# Self-Balancing-Robot_ROS2
 This project simulates a Self-Balancing Robot using Linear Quadratic Algorithm(LQR). Developed with ROS2 Humble and Gazebo, The Robot uses IMU sensor to gather data and the LQR-based corrections to stabilize itself.

## Key Features
 * ROS2 Humble for robotic control.
 * Gazebo for physics based simulation.
 * LQR algorithm for dynamic balancing.
 * Real-time corrections using IMU sensor feedback.

## User Instruction
### Prerequisites:
 * Install ROS2 Humble [Installation Guide](https://docs.ros.org/en/humble/Installation.html) (if not already installed)
 * Install Gazebo by running,
    ```bash
    sudo apt update
    sudo apt install -y curl gnupg lsb-release
    curl -sSL http://repo.ros2.org/repos.key | sudo apt-key add -
    sudo sh -c 'echo "deb [arch=amd64] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
    sudo apt update
    sudo apt install gazebo -y

 * Install tf-transformations for ROS2
    ```bash
    sudo apt install ros-humble-tf-transformations

### Installation and Setup:
 * Navigate to Your ROS2 Workspace.
 * Clone the Project (as a ROS2 Package)
    ```bash
    git clone https://github.com/Kraad25/Self-Balancing-Robot_ROS2.git [package_name]
 * Build the Workspace and source it.
 * Launch the Simulation.
