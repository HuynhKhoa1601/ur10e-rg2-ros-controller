#!/bin/bash

# Array of repository names and their URLs
declare -A REPOS=(
  ["ROS-TCP-Endpoint"]="https://github.com/PhuNguyenPT/ROS-TCP-Endpoint.git"
  ["moveit_msgs"]="https://github.com/PhuNguyenPT/moveit_msgs.git"
  ["onrobot"]="https://github.com/PhuNguyenPT/onrobot.git"
  ["roboticsgroup_upatras_gazebo_plugins"]="https://github.com/PhuNguyenPT/roboticsgroup_upatras_gazebo_plugins.git"
  ["universal_robot"]="https://github.com/PhuNguyenPT/universal_robot.git"
  ["ur_msgs"]="https://github.com/PhuNguyenPT/ur_msgs.git"
  ["ros_industrial_cmake_boilerplate"]="https://github.com/PhuNguyenPT/ros_industrial_cmake_boilerplate.git"
  ["stomp"]="https://github.com/PhuNguyenPT/stomp.git"
  ["stomp_ros"]="https://github.com/PhuNguyenPT/stomp_ros.git"
)

# Loop through each repository to add as a submodule
for repo in "${!REPOS[@]}"; do
  echo "Processing $repo..."

  # Check if the repo already exists as a submodule
  if [ -d "$repo" ]; then
    echo "Removing existing repository at '$repo' from the index."
    git rm -rf --cached "$repo"  # Forcefully remove from the index
  fi
  
  # Add the repository as a submodule
  echo "Adding $repo as a submodule..."
  git submodule add "${REPOS[$repo]}" "$repo"
done

echo "All submodules have been added."
