# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS2 (Robot Operating System 2) workspace containing the `robot_description` package - a basic ament_cmake package skeleton.

## Build Commands

This is a ROS2 workspace using `colcon` as the build tool:

```bash
# Build all packages in the workspace
colcon build

# Build a specific package
colcon build --packages-select robot_description

# Source the workspace to use built packages
source install/setup.bash

# Run tests
colcon test
```

## Architecture

- `src/robot_description/` - The main package directory
  - `package.xml` - ROS2 package metadata (package format 3)
  - `CMakeLists.txt` - Build configuration using ament_cmake

This workspace uses the standard ROS2 build system (ament_cmake). The `robot_description` package follows the conventional ROS2 package structure.