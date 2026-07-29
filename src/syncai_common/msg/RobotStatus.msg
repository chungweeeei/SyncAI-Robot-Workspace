# UNINITIALIZED deliberately takes 0 so that it is also the value a
# default-constructed message carries. Before this constant existed IDLE was 0,
# which meant "nobody has derived a state yet" was indistinguishable from
# "everything is fine, the robot is just idle" — the worst possible default for
# a field consumers use to decide whether to trust the rest of the message.
#
# syncai_robot_state uses it for the pre-relocalization window: the map ->
# base_link TF is unavailable, so RobotState.localization_status is zeroed and
# must not be read as a pose at the map origin. WARNING would be the wrong
# word for that — nothing is malfunctioning, the robot has simply not been
# localized yet.
#
# It overlaps RobotState.localization_valid, which carries the same fact as a
# boolean. That is intended: the flag is the precise answer, this is the coarse
# rollup, and both come from one TF lookup so they cannot disagree.
#
# WARNING means degraded but still operable, and today it has exactly one cause:
# battery below 20%, latched with hysteresis so it does not flap at the publish
# rate. It is advisory — nothing acts on it.
#
# RUNNING and ERROR are declared but never emitted. CHARGING cannot currently be
# derived at all: syncai_driver_manager hardcodes
# BatteryState.power_supply_status to UNKNOWN and the sign convention of
# `current` is undocumented.
#
# Renumbering was safe when it was done: no consumer in the workspace read the
# numeric value (GET /api/v1/robot/state does not even expose `state`). It does
# mean the `state` numbers inside bags recorded before the change decode to the
# wrong constant.
uint8 UNINITIALIZED = 0
uint8 IDLE          = 1
uint8 RUNNING       = 2
uint8 WARNING       = 3
uint8 ERROR         = 4
uint8 CHARGING      = 5
