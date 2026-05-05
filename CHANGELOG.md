# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased] - branch: jazzy

### Added

- `ros2_LinkAttacher/include/ros2_linkattacher/gz_link_attacher.hpp`: new
  gz-sim 8 system plugin header (`ISystemConfigure` + `ISystemPreUpdate`).
- `ros2_LinkAttacher/src/gz_link_attacher.cpp`: gz-sim 8 plugin implementation.
  Exposes `/ATTACHLINK` and `/DETACHLINK` ROS 2 services (same interface as the
  Gazebo Classic plugin). Attachment is kinematic: records the relative transform
  `T_rel = T_gripperLink⁻¹ · T_boxModel` at attach time and sets
  `WorldPoseCmd(box) = T_gripperLink · T_rel` every `PreUpdate` step so the box
  rigidly follows the gripper. Detach removes the `WorldPoseCmd` component.
- `ros2_LinkAttacher/CMakeLists.txt`: replaced Gazebo Classic build with
  gz-sim 8 using `gz_sim_vendor`, `gz_plugin_vendor`, and `gz-sim8::gz-sim8`;
  includes manual path for `gz_plugin_vendor` headers required by `GZ_ADD_PLUGIN`.
- `ros2_LinkAttacher/package.xml`: replaced `gazebo_ros`/`gazebo_dev` dependencies
  with `gz_sim_vendor`, `gz_plugin_vendor`, and `gz-sim8`.

### Fixed

- `gz_link_attacher.cpp`: replaced `ecm.Component<components::WorldPose>()` reads
  for the gripper link with `gz::sim::worldPose(entity, ecm)` from `gz/sim/Util.hh`.
  `components::WorldPose` is not populated by the physics system for articulated
  robot links driven by ros2_control; `gz::sim::worldPose()` traverses the `Pose`
  component parent chain which is updated every step for all entities, making the
  attached box correctly track the moving arm.
- `gz_link_attacher.cpp`: on first `PreUpdate` the plugin now scans for every
  top-level model whose name starts with `box_` and pins each at its spawn world
  pose via `WorldPoseCmd` every step. This prevents physics drift (contact impulses,
  residual velocity) entirely independently of SDF gravity or kinematic settings.
  On detach the locked pose is updated to the final placed position so the box
  stays wherever the arm put it. Removes reliance on `<kinematic>` SDF tag which
  was misinterpreted by gz-sim 8 and caused boxes to spawn at the wrong height.
