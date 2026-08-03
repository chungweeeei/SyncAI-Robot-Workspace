import os
import subprocess
import threading
from typing import Any, Optional

import yaml
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from syncai_common.msg import RobotMode
from syncai_common.srv import GetMode, SwitchMode

from syncai_sys_manager.managers.conf_manager import ConfManager

# One session spec per operating mode. The mode vocabulary is RobotMode's
# (MAINTENANCE / MANUAL / AUTO), reused rather than redeclared so this manager,
# RobotState.mode, the backend's REST layer and the frontend all speak the same
# constants.
#
# MAINTENANCE is deliberately absent: there is no session for "nothing running".
# It is what get_mode *reports* when neither session exists, not a mode you can
# switch into — see _detect_mode.
SESSION_SPECS = {
    RobotMode.AUTO: os.path.expanduser("~/robot_ws/config/sessions/start_nav.yaml"),
    RobotMode.MANUAL: os.path.expanduser(
        "~/robot_ws/config/sessions/start_mapping.yaml"
    ),
}

# What comes up when nothing is running yet. AUTO because navigation is the
# robot's job; mapping is the occasional, deliberate detour.
DEFAULT_MODE = RobotMode.AUTO

# Reverse lookup for log messages only. Mirrors the backend's MODE_NAMES in
# syncai_backend/interfaces/rest/routers/robot.py — kept local rather than
# imported because the backend's copy is a REST serialisation detail.
MODE_NAMES = {
    RobotMode.MAINTENANCE: "MAINTENANCE",
    RobotMode.MANUAL: "MANUAL",
    RobotMode.AUTO: "AUTO",
}

# Same log tree the deleted scripts/byobu_session*.sh wrote, so the multilog
# dirs stay where the docs expect them. (Its companion reader, scripts/tailog.sh,
# is gone as well — read a subsystem back with `tail -f <dir>/current` and
# `zcat <dir>/@*.s`.)
LOG_ROOT_TEMPLATE = "log/stack/{robot_id}"

# 16 MiB per file x 10 rotated files, gzipped — lifted verbatim from the deleted
# byobu_session.sh so a stack launched from here rotates identically to one
# launched by hand.
MULTILOG_ARGS = "t s16777215 n10 '!gzip'"

# Per-invocation timeout. byobu commands are near-instant; anything that hangs
# this long means the server is wedged and we would rather log it than block a
# ROS callback thread forever.
BYOBU_TIMEOUT = 10.0


class NodeManager:
    """Brings the robot stack up and down as a byobu session, one per mode.

    An operating mode *is* a byobu session here: AUTO is the navigation session
    from start_nav.yaml, MANUAL is the mapping session from start_mapping.yaml.
    Switching mode therefore means killing every known session and building the
    target one — there is no shared subset kept alive across the switch, because
    the two specs disagree about which nodes may run at all (map_server and the
    localizer die without a finished map, so they cannot be up while one is
    being built).

    The current mode is never stored. It is derived on demand from which session
    byobu actually has, which keeps it correct across a sys_manager restart and
    stops it drifting when someone builds or kills a session by hand.

    The session layout is data, not code: it is read from the spec for the mode,
    whose schema is

        session: <session name>
        select:  <window name to select after building>
        windows:
          - name: <window name>
            cwd:  <path relative to the workspace root>   # optional
            panes:
              - cmd:   <shell command>
                sleep: <seconds to delay before cmd>       # optional
                log:   <multilog dir name>                 # optional
                enter: <bool, default true>                # optional

    `sleep` is how the startup ordering is encoded — there is no lifecycle
    manager in this stack, so map_server / LIO must be up before lio_bridge,
    which must be up before planner / controller, and so on.
    """

    def __init__(self, node: Node, conf_manager: ConfManager):
        self._node = node
        self._logger = node.get_logger()
        self._conf_manager = conf_manager

        # switch_mode is a long, destructive sequence of ~40 byobu commands. Two
        # of them interleaving would build one session out of two specs, so they
        # are serialised even though each callback already has its own
        # MutuallyExclusiveCallbackGroup (those only serialise a callback with
        # itself, not with the other service).
        self._mode_lock = threading.Lock()

        self.init_services()

    def init_services(self):

        self._node.create_service(
            srv_type=SwitchMode,
            srv_name="switch_mode",
            callback=self._switch_mode,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self._node.create_service(
            srv_type=GetMode,
            srv_name="get_mode",
            callback=self._get_mode,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

    def _switch_mode(
        self,
        request: SwitchMode.Request,
        response: SwitchMode.Response,
    ) -> SwitchMode.Response:
        mode = request.mode

        if mode not in SESSION_SPECS:
            # Covers MAINTENANCE (a real RobotMode with no session) and any
            # out-of-range byte.
            known = ", ".join(f"{MODE_NAMES[m]}={m}" for m in sorted(SESSION_SPECS))
            response.success = False
            response.message = (
                f"Mode {mode} ({MODE_NAMES.get(mode, 'unknown')}) has no session "
                f"spec; switchable modes are {known}"
            )
            return response

        with self._mode_lock:
            live = self._live_sessions()
            current = live[0][0] if live else RobotMode.MAINTENANCE

            # Refuse to rebuild the mode that is already live. In MANUAL that
            # would drop an unsaved map on the floor — pgo_node keeps its
            # keyframes in RAM, so tearing the session down loses the drive.
            #
            # `len(live) == 1` matters: if BOTH sessions are somehow up, the
            # state is ambiguous and the requested mode being among them is not
            # good enough. Fall through to the kill-and-rebuild, which is exactly
            # the cleanup that situation needs.
            if len(live) == 1 and current == mode:
                response.success = True
                response.message = f"Already in {MODE_NAMES[mode]}; nothing to do"
                return response

            self._logger.info(
                f"[NodeManager][switch_mode] {MODE_NAMES.get(current, current)} -> "
                f"{MODE_NAMES[mode]}"
            )

            # Kill every known session, not just the current one: if both were
            # somehow up, leaving one behind would make get_mode ambiguous
            # immediately after a successful switch.
            for known_mode in SESSION_SPECS:
                self.kill_session(known_mode)

            self.launch_session(mode)

            # Report what actually happened rather than what was asked for.
            landed, session = self._detect_mode()
            if landed != mode:
                response.success = False
                response.message = (
                    f"Tried to switch to {MODE_NAMES[mode]} but ended up in "
                    f"{MODE_NAMES.get(landed, landed)} — check the sys_manager log"
                )
                return response

            response.success = True
            response.message = f"Switched to {MODE_NAMES[mode]} (session {session})"
            return response

    def _get_mode(
        self,
        _: GetMode.Request,
        response: GetMode.Response,
    ) -> GetMode.Response:
        live = self._live_sessions()
        mode, session = live[0] if live else (RobotMode.MAINTENANCE, "")
        response.mode = mode
        response.session = session

        if len(live) > 1:
            names = " and ".join(MODE_NAMES[m] for m, _ in live)
            response.success = False
            response.message = (
                f"Ambiguous: sessions for {names} are both running; reporting "
                f"the first ({MODE_NAMES.get(mode, mode)})"
            )
            return response

        response.success = True
        response.message = MODE_NAMES.get(mode, str(mode))
        return response

    def _live_sessions(self) -> list[tuple[int, str]]:
        """Every known mode whose byobu session currently exists.

        Ordered by mode value, so callers that want a single answer can take the
        first. Normally holds 0 or 1 entries; 2 means someone built both by hand,
        which is the ambiguity _get_mode reports and _switch_mode cleans up.
        """
        live = []
        for mode in sorted(SESSION_SPECS):
            session = self._session_name_of(SESSION_SPECS[mode])
            if session and self._session_exists(session):
                live.append((mode, session))

        return live

    def _detect_mode(self) -> tuple[int, str]:
        """The live mode, or MAINTENANCE with an empty session name if none."""
        live = self._live_sessions()
        return live[0] if live else (RobotMode.MAINTENANCE, "")

    def _session_name_of(self, spec_path: str) -> str:
        spec = self._read_spec(spec_path)
        return spec.get("session") or ""

    def setup_session(self) -> None:
        """Bring the stack up on startup, adopting whatever is already running.

        If a session for a known mode exists, that mode is the live one and is
        left strictly alone — this is what makes a sys_manager restart harmless
        in the middle of a mapping run. Only when nothing is up does this build
        DEFAULT_MODE.

        The check is also what keeps the startup path from destroying itself if
        sys_manager is ever put back into a session spec as a window: it would
        build the session, spawn a second sys_manager there, and that one would
        find the session already present and do nothing — instead of calling
        launch_session() again, whose first act is `kill-session`.

        A session left over from a crashed run therefore also blocks the
        rebuild. That is deliberate: silently killing a session someone may be
        attached to is worse. Use switch_mode (or launch_session) to force one.
        """
        mode, session = self._detect_mode()
        if mode != RobotMode.MAINTENANCE:
            self._logger.info(
                f"[NodeManager][setup_session] Adopting running {MODE_NAMES[mode]} "
                f"session {session}, leaving it alone"
            )
            return

        self._logger.info(
            f"[NodeManager][setup_session] Nothing running, bringing up "
            f"{MODE_NAMES[DEFAULT_MODE]}"
        )
        self.launch_session(DEFAULT_MODE)

    def _session_exists(self, session: str) -> bool:
        # `has-session` exits 0 when the session is there, non-zero otherwise.
        result = self._byobu("has-session", "-t", session)
        return result is not None and result.returncode == 0

    def launch_session(self, mode: int = DEFAULT_MODE) -> None:
        spec_path = SESSION_SPECS.get(mode)
        if spec_path is None:
            self._logger.error(
                f"[NodeManager][launch_session] No session spec for mode {mode} "
                f"({MODE_NAMES.get(mode, 'unknown')})"
            )
            return

        spec = self._read_spec(spec_path)
        if not spec:
            return

        session = spec.get("session")
        if not session:
            self._logger.error(
                f"[NodeManager][launch_session] {spec_path} has no 'session' name"
            )
            return

        windows = spec.get("windows") or []
        if not windows:
            self._logger.error(
                f"[NodeManager][launch_session] {spec_path} declares no windows"
            )
            return

        # Unconditional, stderr swallowed: a missing session is the normal case,
        # not an error. Same as the deleted shell script did.
        self._byobu("kill-session", "-t", session)

        built = 0
        for index, window in enumerate(windows):
            if self._build_window(session=session, window=window, first=index == 0):
                built += 1
                continue

            # Only the first window's failure is fatal: without a session every
            # later command targets something that does not exist. A later
            # window failing costs one subsystem, so keep going and let the
            # count below report the shortfall rather than claiming a full stack.
            if index == 0:
                self._logger.error(
                    "[NodeManager][launch_session] Could not create session "
                    f"{session}; aborting"
                )
                return

        select = spec.get("select")
        if select:
            self._byobu("select-window", "-t", f"{session}:{select}")

        # Deliberately no `attach-session` — the deleted shell script ended with
        # one because a human ran it from a terminal. Here the caller is a ROS
        # node with no TTY, so the session is built detached and left for
        # whoever attaches next.
        if built < len(windows):
            self._logger.warning(
                f"[NodeManager][launch_session] Built detached byobu session "
                f"{session} with only {built} of {len(windows)} windows"
            )
            return

        self._logger.info(
            f"[NodeManager][launch_session] Built detached byobu session {session} "
            f"with {built} windows"
        )

    def kill_session(self, mode: int = DEFAULT_MODE) -> None:
        spec_path = SESSION_SPECS.get(mode)
        if spec_path is None:
            self._logger.error(
                f"[NodeManager][kill_session] No session spec for mode {mode} "
                f"({MODE_NAMES.get(mode, 'unknown')})"
            )
            return

        session = self._session_name_of(spec_path)
        if not session:
            self._logger.error(
                f"[NodeManager][kill_session] {spec_path} has no 'session' name"
            )
            return

        # Killing a session that was never there is the normal case (switch_mode
        # kills every known mode before building one), so this is not an error
        # and _byobu keeps its non-zero exit at debug level.
        #
        # Note this becomes SELF-TERMINATING the moment sys_manager is added as a
        # window to the spec being killed — it would kill the pane this process
        # runs in. Neither spec does that today (sys_manager runs outside both,
        # which is what makes switch_mode possible at all), so the log line below
        # does reliably get to say what it did. If a sys_manager window ever
        # comes back, move this log above the call: nothing after `kill-session`
        # would be guaranteed to run.
        self._logger.info(
            f"[NodeManager][kill_session] Killing byobu session {session} "
            f"({MODE_NAMES.get(mode, mode)})"
        )
        self._byobu("kill-session", "-t", session)

    def _read_spec(self, spec_path: str) -> dict[str, Any]:
        try:
            with open(spec_path, "r") as spec_file:
                spec = yaml.safe_load(spec_file)
        except FileNotFoundError:
            self._logger.error(
                f"[NodeManager][_read_spec] {spec_path} not found — is the cwd "
                "the workspace root?"
            )
            return {}
        except yaml.YAMLError as err:
            self._logger.error(
                f"[NodeManager][_read_spec] Failed to parse {spec_path}: {str(err)}"
            )
            return {}

        if not isinstance(spec, dict):
            self._logger.error(
                f"[NodeManager][_read_spec] {spec_path} is not a YAML mapping"
            )
            return {}

        return spec

    def _build_window(self, session: str, window: dict[str, Any], first: bool) -> bool:
        name = window.get("name")
        if not name:
            self._logger.warning(
                "[NodeManager][_build_window] Skipping a window with no 'name'"
            )
            return False

        # Panes inherit the window's cwd. Most windows want the workspace root
        # (relative config paths); the frontend window overrides it, because npm
        # needs the directory its package.json lives in.
        cwd = os.path.abspath(window.get("cwd", "."))

        if first:
            result = self._byobu(
                "new-session", "-d", "-s", session, "-n", name, "-c", cwd
            )
        else:
            result = self._byobu("new-window", "-t", session, "-n", name, "-c", cwd)

        if result is None or result.returncode != 0:
            self._logger.error(
                f"[NodeManager][_build_window] Failed to create window {name}"
            )
            return False

        panes: list[dict[str, Any]] = window.get("panes") or []
        for index, pane in enumerate(panes):
            # The first pane comes free with the window; the rest are splits.
            if index > 0:
                self._byobu("split-window", "-v", "-t", f"{session}:{name}", "-c", cwd)

            # A single-pane window is addressed without an index, matching how
            # the deleted shell script targeted its panes.
            target = f"{session}:{name}"
            if len(panes) > 1:
                target = f"{target}.{index}"

            self._send_pane_command(target=target, pane=pane)
            self._pipe_pane_log(target=target, pane=pane)

        return True

    def _send_pane_command(self, target: str, pane: dict[str, Any]) -> None:
        command = pane.get("cmd")
        if not command:
            # A pane with no cmd is a deliberate spare shell — nothing to type.
            return

        sleep = pane.get("sleep")
        if sleep:
            command = f"sleep {sleep} && {command}"

        keys = ["send-keys", "-t", target, command]

        # `enter: false` types the command but leaves it unexecuted, for the
        # panes a human is meant to review and fire by hand (the localizer
        # relocalize call, rviz2).
        if pane.get("enter", True):
            keys.append("Enter")

        self._byobu(*keys)

    def _pipe_pane_log(self, target: str, pane: dict[str, Any]) -> None:
        log_name = pane.get("log")
        if not log_name:
            # No `log:` means the pane is interactive — tapping a spare shell
            # would only record the operator's own keystrokes.
            return

        log_dir = os.path.abspath(
            os.path.join(
                LOG_ROOT_TEMPLATE.format(robot_id=self._conf_manager.get_robot_id()),
                log_name,
            )
        )

        # `-o` *copies* the pane's output into the pipe, so the live pane view is
        # untouched. The shell fragment is byobu's own argument, hence a string
        # rather than a list — multilog needs its directory to exist first.
        self._byobu(
            "pipe-pane",
            "-o",
            "-t",
            target,
            f"mkdir -p '{log_dir}' && exec multilog {MULTILOG_ARGS} '{log_dir}'",
        )

    def _byobu(self, *args: str) -> Optional[subprocess.CompletedProcess]:
        try:
            result = subprocess.run(
                ["byobu", *args],
                capture_output=True,
                text=True,
                timeout=BYOBU_TIMEOUT,
            )
        except FileNotFoundError:
            self._logger.error(
                "[NodeManager][_byobu] byobu not found; is it installed in this image?"
            )
            return None
        except subprocess.TimeoutExpired:
            # Not necessarily a failure: kill-session tears down our own pane, so
            # the call can be interrupted before it reports back.
            self._logger.warning(
                f"[NodeManager][_byobu] `byobu {args[0]}` did not return within "
                f"{BYOBU_TIMEOUT} seconds"
            )
            return None

        # byobu exits non-zero for benign cases too (notably kill-session on a
        # session that was never there), so this is a debug-level note rather
        # than an error — the caller decides what matters.
        if result.returncode != 0:
            self._logger.debug(
                f"[NodeManager][_byobu] `byobu {' '.join(args)}` exited "
                f"{result.returncode}: {result.stderr.strip()}"
            )

        return result


def init_node_manager(node: Node, conf_manager: ConfManager) -> NodeManager:
    node.get_logger().info(
        "[NodeManager][init_node_manager] Initializing Node Management module"
    )
    node_manager = NodeManager(node=node, conf_manager=conf_manager)

    # setup_session(), not launch_session(): it adopts a session that is already
    # running instead of rebuilding it, so restarting sys_manager in the middle
    # of a mapping run does not throw the run away. See NodeManager.setup_session.
    try:
        node_manager.setup_session()
    except Exception as err:
        node.get_logger().error(
            f"[NodeManager][init_node_manager] Failed to bring up the stack session: {str(err)}"
        )

    return node_manager
