# The task queue is per-robot: its name is the robot_id (see temporal/worker.py
# and WorkflowGateway).
WORKFLOW_TYPE_NAME = "RobotWorkflow"


# --- "What is running right now" (GET /api/v1/active_tasks) -----------------
#
# That endpoint is answered by one Temporal visibility query, cached in a single
# slot on the gateway. The numbers below are the whole cost model, so they are
# derived here rather than left as literals at the call site.
#
# One refresh is ONE RPC: the visibility list already carries status, start
# time, task queue and the search attributes, so neither describe() nor the
# get_step_states query is needed. With the TTL below the refresh rate is
# therefore capped at 1/1.5 s = 0.67 RPC/s for the whole robot -- independent of
# how many browser tabs, MCP clients or curl loops are asking, because they all
# replay the same snapshot.
#
# The frontend polls at 2000 ms, deliberately LONGER than this TTL: a single tab
# then misses on every poll (0.5 RPC/s) and every additional client is absorbed
# for free. A TTL >= the poll interval would only make the served snapshot older
# than the poll without saving anything further. Worst-case staleness an
# operator sees is TTL + poll ~ 3.5 s, on a state that changes a few times an
# hour.
ACTIVE_TASK_CACHE_TTL_S = 1.5

# A robot does one thing at a time in practice, so this is a ceiling rather than
# a page size worth tuning: it stops a pathological state (someone dispatching
# twenty tasks) from paging the iterator forever inside a request a browser
# polls twice a second.
ACTIVE_TASK_LIST_LIMIT = 10

# Explicit, because the single-flight lock serialises callers: a wedged Temporal
# frontend must not be able to hold that lock for the gRPC default timeout.
ACTIVE_TASK_RPC_TIMEOUT_S = 3.0
