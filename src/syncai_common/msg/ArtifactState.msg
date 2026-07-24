uint64 timestamp    # ms since epoch, stamped by the publishing node
string artifact_id  # unique instance id, e.g. "conveyor_0", "door_1"
string type         # live_info discriminator: "conveyor", "door", ...

# -- health: uniform across artifact types, so monitors/alarms never parse JSON
bool connected      # transport session (e.g. modbus tcp) is up
bool stale          # connected but the device stopped updating (liveness lost)
uint16 error_code   # device-reported status code, 0 = ok

# -- type-specific state: JSON object of DECODED values, never raw registers.
# conveyor: {"phase": "handoff", "box_id": 1, "speed_mps": 0.0,
#            "carried_by": -1, "dropped_at": -1, "gated": true, "seq": 63641}
# door:     {"set_door_1": false, "read_door_1": false, ...}
string live_info
