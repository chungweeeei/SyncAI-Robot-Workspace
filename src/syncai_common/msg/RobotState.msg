# The robot's aggregated status, published at 10 Hz by syncai_robot_state on the
# relative topic `robot_state` (so it lands on <robot_id>/robot_state).
#
# One message, one publisher, one consumer: syncai_backend, which re-serialises
# a SUBSET of it for GET /api/v1/robot/state. That REST payload is a frozen
# third-party contract, so the fields below are not all equal in reach — see the
# note above motor_status.
#
# SECONDS since epoch, stamped by the publishing node. Note the unit: the other
# timestamped interfaces here (ArtifactState, ExecuteTask) use milliseconds.
# Kept in seconds because it is passed through verbatim to
# GET /api/v1/robot/state, whose consumers already treat it as seconds.
#
# At 10 Hz that means ten consecutive messages share one timestamp value. It is
# a wall clock for display, not a sequence number: it cannot order samples or
# measure the publish rate. motor_timestamp below has nanosecond resolution if
# you need that.
uint64 timestamp
string robot_id
string map
# mode values are defined as constants in RobotMode (MAINTENANCE/MANUAL/AUTO)
uint8 mode
# state values are defined as constants in RobotStatus
# (UNINITIALIZED/IDLE/RUNNING/WARNING/ERROR/CHARGING). Derived by
# syncai_robot_state, most-severe-first; today only UNINITIALIZED (no pose),
# WARNING (low battery) and IDLE are ever emitted.
uint8 state
RobotLocalizationStatus localization_status
RobotNetworkStatus network_status
RobotBatteryStatus battery_status

# False when the map -> base_link TF was unavailable at build time. The node
# publishes anyway (it used to drop the whole message, which meant battery, wifi
# and joint temperatures all vanished during exactly the window you most want to
# inspect them: before the localizer has been relocalized). When this is false
# `localization_status` is ZEROED, not held at the last known pose — a stale pose
# with no age attached gets mistaken for a live one, whereas the map origin is an
# obviously suspicious reading.
#
# This overlaps with `state == UNINITIALIZED` on purpose: the flag is the precise
# answer for a consumer that only cares whether the pose can be trusted, the
# state is the coarse rollup. They are derived from the same lookup and cannot
# disagree.
#
# syncai_backend gates its RobotRepo write on this flag, which is what keeps
# GET /api/v1/robot/state returning 404 rather than a pose at (0, 0, 0).
bool localization_valid

# Latest per-joint sample, copied verbatim from the motor_states topic.
# MotorState is reused rather than trimmed: it already carries exactly the fields
# this exists for (temperature, tau_est, error), and q/dq come along for free.
#
# NOT PART OF THE REST PAYLOAD. GET /api/v1/robot/state names its fields
# explicitly and must keep doing so — per-joint temperatures and motor error
# codes are for operators, not for third-party integrators, and there is no
# longer a separate message type stopping them from leaking.
#
# Do NOT build motion from q/dq here — this is a 10 Hz diagnostic snapshot and
# `timestamp` has whole-second resolution, so the samples cannot even be ordered.
# The high-rate joint channel is the backend telemetry WebSocket, which
# subscribes motor_states directly for that reason.
#
# Empty until the first motor_states message arrives (i.e. whenever
# syncai_driver_manager is not up).
MotorState[] motor_status

# Source timestamp of the motor_status sample above, so a consumer can tell a
# fresh reading from one frozen by a dead driver_manager. 0 if none has ever
# arrived.
#
# NOTE THE UNIT: this is NANOSECONDS, copied unconverted from
# MotorStates.timestamp, while `timestamp` at the top of this message is in
# seconds. The mismatch is inherited from the source message and is left
# unconverted so the value can be compared directly against motor_states.
uint64 motor_timestamp
