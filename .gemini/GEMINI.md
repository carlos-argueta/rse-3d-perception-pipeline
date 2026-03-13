# Gemini CLI Project Instructions

## ROS 2 Conventions
- Always create new packages using the provided utility `ros2 pkg create` instead of manually
- Always use `colcon build --symlink-install` when building ROS 2 packages to allow for faster iteration.

## Documentation and Tracking
- **Q&A Maintenance**: Maintain a `QA.md` (or similar) file where every question posed by the user gets an entry with a summary of the answer provided.
- **Package Documentation**: Maintain a `README.md` at the root of the workspace.
- **Subpackage Documentation**: Ensure every subpackage (under `ros2_ws/src/`, `sim/`, etc.) has its own `README.md` describing its purpose, dependencies, and usage.

## Workflow
1. **Startup**: Always maintain and read `.gemini/CHECKPOINT.md` to retrieve the last session's context and next steps.
2. **Scope Definition**: Before implementation, discuss the project to define the scope further.
3. **Implementation Tracking**: Document all implemented features and steps taken in a formal scope/status file.
