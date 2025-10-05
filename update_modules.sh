#!/bin/bash

# Initialize submodules
echo "Initializing submodules..."
git submodule init

# Grouped updates by branch

# Update submodules on the main branch
for repo in "ROS-TCP-Endpoint" "onrobot" "stomp"; do
  echo "Updating $repo on branch main..."
  git -C $repo fetch origin main
  git -C $repo checkout main
  git -C $repo pull origin main
done

# Update submodules on the master branch
for repo in "moveit_msgs" "roboticsgroup_upatras_gazebo_plugins" "ros_industrial_cmake_boilerplate"; do
  echo "Updating $repo on branch master..."
  git -C $repo fetch origin master
  git -C $repo checkout master
  git -C $repo pull origin master
done

# Update submodules on the melodic-devel branch
for repo in "stomp_ros" "ur_msgs"; do
  echo "Updating $repo on branch melodic-devel..."
  git -C $repo fetch origin melodic-devel
  git -C $repo checkout melodic-devel
  git -C $repo pull origin melodic-devel
done

# Update submodule on the noetic-devel branch
for repo in "universal_robot"; do
  echo "Updating $repo on branch melodic-devel..."
  git -C $repo fetch origin noetic-devel
  git -C $repo checkout noetic-devel
  git -C $repo pull origin noetic-devel
done
echo "All specified submodules have been updated to their latest commits on their respective branches."