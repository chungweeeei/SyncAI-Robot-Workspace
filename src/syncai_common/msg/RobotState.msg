# SECONDS since epoch, stamped by the publishing node. Note the unit: the
# other timestamped interfaces here (ArtifactState, ExecuteTask) use
# milliseconds. Kept in seconds because it is passed through verbatim to
# GET /api/v1/robot/state, whose consumers already treat it as seconds.
uint64 timestamp
string robot_id
string map
# mode values are defined as constants in RobotMode (MAINTENANCE/MANUAL/AUTO)
uint8 mode
# state values are defined as constants in RobotStatus (IDLE/RUNNING/WARNING/ERROR/CHARGING)
uint8 state
RobotLocalizationStatus localization_status
RobotNetworkStatus network_status
RobotBatteryStatus battery_status
