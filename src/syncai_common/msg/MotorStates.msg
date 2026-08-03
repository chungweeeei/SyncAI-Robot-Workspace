# One per-joint sample: the array plus the instant it was taken.
#
# Published by syncai_driver_manager on `motor_states`, and NESTED VERBATIM as
# RobotState.motor_status — which is why RobotState no longer carries a separate
# `motor_timestamp` beside a bare MotorState[]. Keep the two fields together: a
# consumer that can hold the array without its timestamp cannot tell a live
# reading from one frozen by a dead driver_manager.
#
# NANOSECONDS on this topic, stamped by syncai_driver_manager.
#
# BUT NOT IN RobotState: syncai_robot_state converts to **seconds** when it nests
# this message as RobotState.motor_status, so that copy matches every other
# timestamp that message exposes. The topic keeps nanoseconds because the
# backend's telemetry WebSocket reads it as the high-rate joint channel and whole
# seconds cannot order samples at 20 Hz. The unit therefore depends on where you
# found the message — scale before comparing the two.
uint64 timestamp
MotorState[] states
