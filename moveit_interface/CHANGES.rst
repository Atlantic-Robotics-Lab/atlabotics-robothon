Changelog
=========

Phase 1 — Bug Fixes and Dead Code Removal
------------------------------------------

**Files changed**: ``src/moveit_interface.cpp``, ``include/moveit_interface.h``

Bug Fixes
~~~~~~~~~

- **1.2** ``executeMaze``: Added missing ``return false;`` after the catch block.
  Without it, control fell off the end of a ``bool``-returning function (UB).

- **1.3** ``addStagesFromYaml`` (``move_to`` handler): Fixed crash when a
  ``move_to`` stage targeting a TF frame omits the ``offset`` field. The field
  is now optional; missing values default to ``[0.0, 0.0, 0.0]``.

- **1.4** ``doTask``: ``task_.execute()`` was called unconditionally, ignoring
  the ``execute: false`` config flag. Execution is now guarded by
  ``m_config["execute"].as<bool>(true)``.

- **1.6** Renamed ``m_screenRIght`` → ``m_screenRight`` in both the header and
  source file.

Dead Code Removed
~~~~~~~~~~~~~~~~~

Removed the following functions entirely (declarations + definitions):

- ``printRobotTrajectory`` — fully commented-out body, never called.
- ``writeTrajectoryToPickle`` — wrote YAML to a ``.pkl`` file (not valid pickle
  format), never called.
- ``createWaypointTrajectory(std::vector<Pose>&)`` overload — the old
  pre-MTC Cartesian path, never called in the MTC pipeline.
- ``planTrajectory`` — the old ``MoveGroupInterface`` execution path, replaced
  by the MTC ``doTask`` flow.

Removed from header:

- Declarations of the four functions above.
- ``#include <boost/archive/binary_oarchive.hpp>`` (only needed by
  ``writeTrajectoryToPickle``).
- ``#include <moveit/trajectory_processing/iterative_time_parameterization.h>``
  (only needed by ``planTrajectory``).
- ``moveit_msgs::msg::RobotTrajectory m_robotTrajectory`` member (only used in
  removed functions).

All remaining commented-out code blocks stripped throughout the file.

Logging
~~~~~~~

- **1.10** Replaced all ``std::cout`` / ``std::cerr`` calls with the appropriate
  ROS logging macros (``RCLCPP_INFO``, ``RCLCPP_WARN``, ``RCLCPP_ERROR``,
  ``RCLCPP_DEBUG``). Output now respects ROS log levels, timestamps, and log
  aggregation.
  
