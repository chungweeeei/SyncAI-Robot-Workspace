# The gait controller's OWN low-level state machine, as it reports it back over
# the telemetry link. Carried on RobotState.low_level_mode.
#
# THIS IS NOT RobotState.mode. Three unrelated vocabularies in this workspace are
# called "mode", and two of them meet inside one RobotState:
#
#   RobotState.mode      RobotMode: MAINTENANCE / MANUAL / AUTO — which byobu
#                        session is up. Ours; also used by SwitchMode and
#                        GetMode. Currently hardcoded to AUTO.
#   policy_state below   the controller's RL policy index. Same vocabulary as
#                        SetPolicyMode.mode.
#   motion_state below   the controller's motion state. Same vocabulary as
#                        SetMotionKey.key — which is a STRING there.
#
# The ROS topic the last two arrive on is ALSO called `mode`: <robot_id>/mode,
# std_msgs/Int32MultiArray, published by syncai_driver_manager out of the
# MODE_STATE telemetry section (data[0] = policy state, data[1] = motion state).
# syncai_robot_state subscribes it and copies the two integers here verbatim.
#
# COMMANDED vs MEASURED is the whole point of this message. SetPolicyMode and
# SetMotionKey are one-way UDP that nothing acknowledges — the driver's udpSend()
# discards sendto()'s return — so every other view of "which mode is the robot
# in" shows what was ASKED FOR. This is the only thing in the stack that reports
# what the controller says is live.
#
# THERE IS NO FRESHNESS INFORMATION HERE, and that has a consequence you have to
# design around. syncai_driver_manager publishes `mode` ONLY when a telemetry
# datagram happens to contain the MODE_STATE section — event-driven, no periodic
# republish, no TRANSIENT_LOCAL latch — and syncai_robot_state republishes
# whatever it last heard on every tick, forever. So:
#
#   * `0 / 0` before the first sample is INDISTINGUISHABLE from a genuine
#     "PPO / Stand" reading. A receipt timestamp used to make that difference
#     visible and was removed; nothing replaced it.
#   * `motion_state == 8` (UNKNOWN) is the only hint left, it comes from the
#     controller rather than from us, and it exists on that index only —
#     `policy_state` has no sentinel at all.
#   * A frozen value is equally consistent with a dead driver, a controller that
#     stopped sending the section, and nothing having changed.
#
# A consumer that needs to know whether this is live has to get it from somewhere
# else — RobotState.motor_status.timestamp advancing is the nearest available
# proxy for "syncai_driver_manager is alive", though it says nothing about the
# MODE_STATE section specifically.
#
# THERE ARE DELIBERATELY NO CONSTANTS HERE, unlike RobotMode and RobotStatus.
# Those vocabularies are ours to define; this one is the gait controller's, we
# have it from a Readme rather than a spec, and we already know it is incomplete
# (see MPC below) — a constant block would read as a closed enum and invite
# `if (x == CONST_A) ... else error`. syncai_common/README.md records the same
# decision for SetPolicyMode.mode. The fields are int32, not the uint8 those
# constant blocks use, because they come straight out of
# std_msgs/Int32MultiArray's int32[] data.

# The controller's reinforcement-learning policy index:
#
#   0  PPO        1  HIMLOCO        2  CHAMP        3  ISSAC
#
# NO SENTINEL EXISTS ON THIS INDEX. 0 is a legitimate value (PPO), and it is also
# what this field holds before anything has ever been heard from the controller —
# see the freshness note above.
int32 policy_state

# The controller's motion state:
#
#   0  Stand      1  Locomotion     2  LieDown      3  Damping
#   4  ESTOP      8  UNKNOWN  <- the controller's own startup sentinel
#
# 8 is the only sentinel in either vocabulary and it exists only on this index. It
# is the controller saying "I have not entered a state yet", NOT this stack saying
# "I have not heard from the controller" — 0 (Stand) is what that looks like.
#
# MPC IS ABSENT FROM THIS TABLE AND ITS CODE IS UNKNOWN. Both tables are
# transcribed from the reference implementation's Readme (GaitMPC
# udp_ros_bridge), which has no MPC entry at all. This workspace added one
# anyway: syncai_driver_manager maps SetMotionKey key "5" to the controller
# command `MODE M`. Whatever the controller then reports on this index is
# unknown, so a consumer WILL eventually see an integer that is not in the table
# above. Pass it through — do not clamp it, do not treat it as an error, and do
# not assume the absence of MPC. Resolving this is a hardware-observation task:
# put the robot in MPC and read the value.
int32 motion_state
