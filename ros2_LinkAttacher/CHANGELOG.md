# Changelog

All notable changes to this package will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Migrated plugin from Gazebo Classic (`gazebo_ros`/`gazebo_dev`) to Gazebo
  Harmonic (gz-sim 8). New plugin class `GzLinkAttacher` replaces the Classic
  implementation.
- Kinematic attachment via `WorldPoseCmd` component: on attach, records
  `T_rel = T_gripperLink⁻¹ · T_boxModel`; every `PreUpdate` step sets
  `WorldPoseCmd(boxModel) = T_gripperLink · T_rel`. On detach, removes
  `WorldPoseCmd` and clears attachment state.
- `CMakeLists.txt`: uses `gz_sim_vendor` and `gz_plugin_vendor` (ROS 2 Jazzy
  vendored packages) instead of system-level gz-sim. Adds manual
  `target_include_directories` for `gz_plugin_vendor` headers since its cmake
  target does not expose `INTERFACE_INCLUDE_DIRECTORIES`.
- Exposes same ROS 2 service interface (`/ATTACHLINK`, `/DETACHLINK`) as the
  original Classic plugin, keeping `mairon_gripper_sim.py` unchanged.
