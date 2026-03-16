Changelog
=========

0.4.0 (2026-03-16)
------------------

Phase 4.1 — Template Variable System

- Add ``resolveTemplateVar`` to ``TaskConfigInterpreter``: replaces ``{{key}}`` placeholders
  in stage ``target`` and ``frame`` fields with values from a ``params`` map.
- Add ``params`` argument (default empty) to ``doTask``, ``createTask``, ``addStagesFromYaml``.
- ``TaskOrchestrator::doTask`` forwards ``m_stepParams`` to the interpreter so template
  variables flow from ``task_sequence[i].params`` all the way into stage construction.
- ``TaskSequenceEntry`` struct (``types.h``) holds ``type`` and ``params`` for each
  ``task_sequence`` entry.

To use: add a ``params:`` block to a ``task_sequence`` entry and write ``{{variable}}``
in the corresponding task's stage ``target`` or ``frame`` fields.

0.3.0 (2026-03-16)
------------------

Phase 3 — YAML Schema Update

**YAML** (``config/move_group_params.yaml``):

- **3.7** Added ``robot:`` section with ``arm_group``, ``end_effectors``, ``planners``,
  and ``gripper`` profiles. ``GripperController`` reads from ``robot.gripper``
  (falls back to top-level ``gripper:``).
- **3.1** Added ``target_type: named_pose | tf_frame`` to every ``move_to`` stage,
  replacing the hardcoded string-matching list in ``addStagesFromYaml``.
- **3.3** Replaced ``hand_frame:`` with ``end_effector:`` referencing
  ``robot.end_effectors.<name>.frame_id``. Legacy ``hand_frame:`` still accepted.
- **3.5** Added explicit ``frame: world`` to all ``move_relative`` stages.
- **3.6** Added ``task_sequence:`` listing the eight Robothon steps in order.

**C++**:

- ``TaskConfigInterpreter::createTask``: reads ``arm_group`` and planner defaults from
  ``robot:``; resolves ``end_effector`` name to frame ID.
- ``TaskConfigInterpreter::addStagesFromYaml``: uses ``target_type`` for clean
  named-pose vs. tf-frame dispatch; logs and skips stages with missing ``tf_frame`` targets.
- ``TaskOrchestrator``: replaced hardcoded switch-case with ``loadTaskSequence()`` +
  ``executeStep()``; added ``m_sequenceIndex``, ``m_sequenceLoaded``, ``m_stepParams``.
- ``MoveitInterface::triggerTaskCallback``: calls ``m_orchestrator->reset()`` on
  re-trigger so the sequence restarts from step 1.

0.2.0 (2026-03-16)
------------------

Phase 2 — Structural Refactoring (split monolith into classes)

**New files**:

- ``include/types.h`` — shared enums (``InterfaceState``, ``TaskType``, ``ScreenTextTask``) and structs (``ServiceInfo``, ``ParsedTask``)
- ``include/gripper_controller.h`` / ``src/gripper_controller.cpp`` — ``GripperController``: owns gripper service client and ``gripperService()``
- ``include/perception_bridge.h`` / ``src/perception_bridge.cpp`` — ``PerceptionBridge``: owns TF buffer, topic subscribers, service clients, all pose state, ``generateStaticTFPose()``, ``callTriggerService()``, ``parseTaskCommand()``
- ``include/task_config_interpreter.h`` / ``src/task_config_interpreter.cpp`` — ``TaskConfigInterpreter``: owns ``doTask()``, ``createTask()``, ``addStagesFromYaml()``
- ``include/task_orchestrator.h`` / ``src/task_orchestrator.cpp`` — ``TaskOrchestrator``: owns ``executeTasks()`` and all ``execute*()`` methods

**Modified files**:

- ``include/moveit_interface.h`` — slimmed to ~50 lines; owns the four sub-components via ``unique_ptr``
- ``src/moveit_interface.cpp`` — slimmed to ~120 lines; constructs sub-components, runs state machine
- ``CMakeLists.txt`` — added four new source files; fixed ``include_directories`` path

No logic or implementation changes. All function bodies moved verbatim.

0.2.0 (2026-03-16)
------------------

Phase 2 — Structural Refactoring (split monolith into classes)

**New files**:

- ``include/types.h`` — shared enums (``InterfaceState``, ``TaskType``, ``ScreenTextTask``) and structs (``ServiceInfo``, ``ParsedTask``)
- ``include/gripper_controller.h`` / ``src/gripper_controller.cpp`` — ``GripperController``: owns gripper service client and ``gripperService()``
- ``include/perception_bridge.h`` / ``src/perception_bridge.cpp`` — ``PerceptionBridge``: owns TF buffer, topic subscribers, service clients, all pose state, ``generateStaticTFPose()``, ``callTriggerService()``, ``parseTaskCommand()``
- ``include/task_config_interpreter.h`` / ``src/task_config_interpreter.cpp`` — ``TaskConfigInterpreter``: owns ``doTask()``, ``createTask()``, ``addStagesFromYaml()``
- ``include/task_orchestrator.h`` / ``src/task_orchestrator.cpp`` — ``TaskOrchestrator``: owns ``executeTasks()`` and all ``execute*()`` methods

**Modified files**:

- ``include/moveit_interface.h`` — slimmed to ~50 lines; owns the four sub-components via ``unique_ptr``
- ``src/moveit_interface.cpp`` — slimmed to ~120 lines; constructs sub-components, runs state machine
- ``CMakeLists.txt`` — added four new source files; fixed ``include_directories`` path

No logic or implementation changes. All function bodies moved verbatim.

0.1.0 (2026-03-13)
------------------------------------------
Phase 1 — Bug Fixes and Dead Code Removal

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