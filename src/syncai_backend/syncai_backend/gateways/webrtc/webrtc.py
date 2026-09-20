import ctypes
import json
import os
import threading
import time
from typing import Dict, FrozenSet, NamedTuple, Optional, Tuple

import structlog


# Where the Go worker lands. Built out of tree in the SyncAI-WebRTC-Worker repo
# with
#
#   go build -buildmode=c-shared -o dist/libsyncai_worker.so ./cmd/lib
#
# and copied into the workspace by hand, the same arrangement as the kokoro
# weights under models/: an artifact the build does not produce and git does
# not carry (*.so is gitignored), reachable in the container through the
# workspace bind mount.
_DEFAULT_LIB = os.path.expanduser("~/robot_ws/lib/libsyncai_worker.so")

# Applied with setdefault immediately before InitWorker, never after: the
# worker reads its configuration once under a sync.Once and freezes it for the
# life of the process.
#
# Set here rather than left to the worker's own godotenv.Load(): that call
# reads .env relative to the HOST PROCESS cwd, and this package already learned
# that "every entrypoint runs from the workspace root" is false for tests and
# ad-hoc shells -- which is why SYNCAI_SYSTEM_INI resolves absolutely. A
# backend started from elsewhere would otherwise silently get the worker's
# compiled-in defaults.
#
# setdefault, not assignment: the workspace .env is already in os.environ via
# main.py's dotenv.load_dotenv(), and it stays the override point. These are
# only the floor.
#
# TURN_SERVERS / TURN_USERNAME / TURN_PASSWORD are deliberately absent -- an
# empty-string default is worse than no value at all, and the only sane source
# for them is the workspace .env.
_WORKER_ENV_DEFAULTS = {
    "VIDEO_RTP_PORT": "5006",
    "VIDEO_AUDIO_RTP_PORT": "5007",
    "AUDIO_RTP_PORT": "5004",
    "STUN_SERVERS": "stun:stun.l.google.com:19302",
}

# The one export nothing works without. The per-kind exports are looked up
# individually and are allowed to be missing -- see _bind_exports. FreeString is
# in neither list: it returns void and frees rather than allocating.
_INIT_EXPORT = "InitWorker"


class _Slot(NamedTuple):
    """The one session of its kind this backend believes it has.

    ``session_id`` is None while a create is in flight -- the slot is claimed
    before the (multi-second) call so two concurrent POSTs cannot both think
    they own the device, and there is no id to preempt until the worker
    answers.
    """

    session_id: Optional[str]
    created_at: float


# The three session kinds, each a single slot. Video is WHEP (the robot sends),
# audio is WHIP (the browser sends), duplex is both over one peer connection;
# apart from direction the signalling is identical, which is why one
# implementation serves all three.
_VIDEO = "video"
_AUDIO = "audio"
_DUPLEX = "duplex"
_EXPORTS = {
    _VIDEO: ("CreateVideoSession", "DeleteVideoSession"),
    _AUDIO: ("CreateAudioSession", "DeleteAudioSession"),
    _DUPLEX: ("CreateDuplexSession", "DeleteDuplexSession"),
}

# What each kind holds while it is live. This is the whole arbitration rule:
# there is one camera and one speaker, so a create preempts every live session
# whose devices its own overlap -- a duplex session displaces both
# single-direction ones, and either of those displaces a duplex session. WHEP
# and WHIP touch different devices and therefore coexist, which is exactly the
# arrangement the duplex session replaces with one handshake.
#
# The worker does not know any of this: it will happily build a second session
# and fail deep inside GStreamer when the device turns out to be busy, with an
# error that names an ALSA or V4L2 code rather than the session holding it.
# Arbitrating here is what turns that into a preemption the operator never sees.
_CAMERA = "camera"
_SPEAKER = "speaker"
_DEVICES = {
    _VIDEO: frozenset({_CAMERA}),
    _AUDIO: frozenset({_SPEAKER}),
    _DUPLEX: frozenset({_CAMERA, _SPEAKER}),
}


class WebRtcGateway:
    """WHEP and WHIP signalling over the Go c-shared worker.

    The backend owns the HTTP surface (SDP exchange, Location, CORS); the media
    path -- capture, H.264 encode, pion -- lives entirely inside
    libsyncai_worker.so, which this process dlopens. That is the first FFI in
    this workspace and it is not free:

      - Loading the library installs the Go runtime's signal handlers (SIGURG
        preemption, SIGSEGV/SIGPROF chaining) into a process that already holds
        rclpy (rcl owns SIGINT), CycloneDDS and ALSA.
      - A Go c-shared library cannot be dlclose'd. Once loaded it is here for
        the life of the process.
      - A fault inside it takes the whole backend down -- task dispatch,
        telemetry and every WebSocket included.

    Hence the lazy load: construction touches nothing, and a boot that never
    streams never pays any of the above. SYNCAI_WEBRTC_LIB pointed at a path
    that does not exist is, for the same reason, the only way to disable this
    at runtime -- there is no unload.
    """

    def __init__(self, logger: structlog.stdlib.BoundLogger):
        self._logger = logger

        self.lib_path = os.environ.get("SYNCAI_WEBRTC_LIB", _DEFAULT_LIB)

        # Loaded on first use, not here. See the class docstring.
        self._lib = None

        # Which session kinds the loaded .so turned out to export. Empty until
        # the first load, and possibly a subset afterwards -- the library is
        # copied in from another repository by hand, so it can be older than
        # this file.
        self._kinds: FrozenSet[str] = frozenset()

        # Two locks, and neither is ever held across a call into the worker.
        # A single lock would be worse than it looks: CreateVideoSession blocks
        # until ICE gathering completes, so one lock would put a DELETE behind
        # the very create it is trying to cancel, and would make GET /status --
        # the one route that must stay answerable -- wait on it too.
        #
        # The Go side already guards its own session map, so Python needs a
        # lock only for its own bookkeeping. Serialising creates is the slot
        # claim in create_video_session(), not a mutex: acquire before start so
        # there is no check-then-start race, the same shape as map.py's
        # _ACTIVE_CONVERSIONS.
        self._load_lock = threading.Lock()
        self._session_lock = threading.Lock()

        self._slots: Dict[str, Optional[_Slot]] = {kind: None for kind in _EXPORTS}

        # Same breadcrumb rationale as TtsGateway: the path is neither a ROS
        # parameter nor an INI key, so if the library is missing this line is
        # the only record of where we looked.
        self._logger.info("[WebRtcGateway] Using worker library", path=self.lib_path)

    # --- Library ------------------------------------------------------------
    def _load_library(self, path: str):
        """dlopen the worker. A seam: tests override this to inject a fake.

        CDLL rather than PyDLL on purpose -- CDLL releases the GIL around every
        foreign call, which is what lets the multi-second CreateVideoSession
        run on a FastAPI worker thread without freezing the event loop.
        """
        return ctypes.CDLL(path)

    @staticmethod
    def _bind_exports(lib) -> FrozenSet[str]:
        """Declare the C signatures. Returns the session kinds this build has.

        restype is c_void_p and NOT c_char_p, and that is load-bearing rather
        than style. Every returning export mallocs its JSON envelope and hands
        ownership to us. With c_char_p ctypes copies the bytes into a Python
        object and throws the pointer away, so FreeString can never be called
        on it and every single request leaks the envelope. c_void_p hands back
        an address that survives long enough to be both read and freed.

        A missing *session* export is tolerated, and that is deliberate. The
        .so is built in another repository and copied in by hand, so the
        backend and the worker drift apart as a matter of course -- and this
        method used to bind every export up front, which meant a library built
        before duplex support existed failed the whole gateway with
        `undefined symbol: CreateDuplexSession`, taking WHEP and WHIP down with
        it even though that build served both perfectly. A kind whose exports
        are absent is simply not offered; the others keep working, and the
        route for the missing one says what to rebuild.

        InitWorker is the exception: without it there is no worker at all, so
        its absence still raises.
        """
        init = getattr(lib, _INIT_EXPORT)
        init.argtypes = []
        init.restype = ctypes.c_void_p

        available = set()
        for kind, names in _EXPORTS.items():
            try:
                exports = [getattr(lib, name) for name in names]
            except AttributeError:
                continue
            for export in exports:
                export.argtypes = [ctypes.c_char_p]
                export.restype = ctypes.c_void_p
            available.add(kind)

        lib.FreeString.argtypes = [ctypes.c_void_p]
        lib.FreeString.restype = None

        return frozenset(available)

    def _ensure_loaded(self) -> Tuple[bool, str]:
        """Load and initialize the worker on first call. Caller must hold _load_lock."""
        if self._lib is not None:
            return True, ""

        if not os.path.isfile(self.lib_path):
            return False, (
                f"webrtc worker library missing: {self.lib_path} — build it in "
                "the SyncAI-WebRTC-Worker repo with `go build -buildmode="
                "c-shared -o dist/libsyncai_worker.so ./cmd/lib` and copy the "
                "result there (see gateways/webrtc/webrtc.py)"
            )

        # Before the load, not after: InitWorker's sync.Once freezes these.
        for key, value in _WORKER_ENV_DEFAULTS.items():
            os.environ.setdefault(key, value)

        try:
            lib = self._load_library(self.lib_path)
            kinds = self._bind_exports(lib)
        except OSError as e:
            # Almost always a missing GStreamer runtime: the worker links
            # libgstreamer-1.0 / libgobject-2.0 through cgo, and the Tegra
            # encoder plugins arrive from the nvidia container runtime.
            return False, (
                f"failed to load webrtc worker library: {e} — check that the "
                "GStreamer 1.0 runtime is installed in this container"
            )
        except AttributeError as e:
            return False, f"webrtc worker library is missing an export: {e}"

        self._lib = lib
        self._kinds = kinds

        success, message, _ = self._call(_INIT_EXPORT)
        if not success:
            # The .so stays mapped -- there is no dlclose -- but dropping the
            # handle lets a later request retry InitWorker rather than calling
            # sessions against an uninitialized worker.
            self._lib = None
            self._kinds = frozenset()
            return False, f"webrtc worker failed to initialize: {message}"

        self._logger.info(
            "[WebRtcGateway] Worker library loaded and initialized",
            path=self.lib_path,
            kinds=sorted(kinds),
        )
        missing = sorted(set(_EXPORTS) - kinds)
        if missing:
            # Worth a line of its own: the routes for these will answer 502 and
            # the operator's first question will be whether the backend or the
            # library is behind.
            self._logger.warning(
                "[WebRtcGateway] Worker library predates some session kinds",
                missing=missing,
                path=self.lib_path,
            )
        return True, ""

    def _call(self, name: str, arg: Optional[str] = None) -> Tuple[bool, str, dict]:
        """Invoke an export and decode its envelope, always freeing the result.

        The single FreeString in this module. The free sits in a ``finally``
        around the read rather than after the json.loads: a malformed envelope
        is a worker bug worth reporting, not a reason to also leak the memory
        that carried it.

        Caller must NOT hold _session_lock -- the session exports block for
        seconds.
        """
        export = getattr(self._lib, name)
        pointer = export(arg.encode("utf-8")) if arg is not None else export()
        if not pointer:
            return False, f"{name} returned NULL — the worker broke its contract", {}

        try:
            raw = ctypes.string_at(pointer)
        finally:
            self._lib.FreeString(pointer)

        try:
            envelope = json.loads(raw.decode("utf-8", errors="replace"))
        except ValueError:
            return False, f"{name} returned non-JSON: {raw[:200]!r}", {}
        if not isinstance(envelope, dict):
            return False, f"{name} returned a non-object envelope", {}
        if not envelope.get("ok"):
            return False, str(envelope.get("error") or f"{name} failed with no reason"), {}

        return True, "", envelope

    # --- API ----------------------------------------------------------------
    def _require_kind(self, kind: str) -> Tuple[bool, str]:
        """Refuse a kind the loaded library does not export.

        Names the repository and the build command, because the fix is never
        in this workspace: the .so is produced elsewhere and copied into lib/,
        so "the backend is newer than the worker" is a routine state rather
        than a corrupted install.
        """
        if kind in self._kinds:
            return True, ""
        return False, (
            f"this worker build has no {kind} session support "
            f"({', '.join(_EXPORTS[kind])} not exported) — rebuild the library "
            "in the SyncAI-WebRTC-Worker repo (`docker compose up build-lib`) "
            "and install it per lib/README.md, then restart the backend; the "
            f"kinds this build does support are: {', '.join(sorted(self._kinds)) or 'none'}"
        )

    def _create_session(self, kind: str, offer_sdp: str) -> Tuple[bool, str, dict]:
        """Exchange an offer for an answer. Returns (success, message, payload).

        Blocks until the worker has finished gathering ICE candidates -- the
        exchange is single-shot, so the answer that comes back is already
        complete and there is no trickle channel to follow it.
        """
        create_export = _EXPORTS[kind][0]

        with self._load_lock:
            success, message = self._ensure_loaded()
        if not success:
            return False, message, {}

        supported, message = self._require_kind(kind)
        if not supported:
            return False, message, {}

        # Every kind whose devices this one needs, itself included. Sorted so
        # the preemption order is stable in the logs rather than dict-order.
        contending = sorted(
            other
            for other, devices in _DEVICES.items()
            if devices & _DEVICES[kind]
        )

        with self._session_lock:
            for other in contending:
                slot = self._slots[other]
                if slot is not None and slot.session_id is None:
                    # The one case that cannot be preempted: a create is
                    # mid-flight and has no id yet, so there is nothing to
                    # delete. Reported under the requested kind's name -- the
                    # caller asked for this kind and cannot act on which other
                    # one is in the way.
                    return False, f"{kind} session creation already in progress", {}

            previous = [
                (other, self._slots[other].session_id)
                for other in contending
                if self._slots[other] is not None
            ]
            for other, _ in previous:
                self._slots[other] = None
            self._slots[kind] = _Slot(session_id=None, created_at=time.monotonic())

        try:
            for other, session_id in previous:
                # Preempt rather than refuse. The common cause of an occupied
                # slot is a browser tab closed without a DELETE: there is no
                # callback from Go, so nothing else would ever free it, and
                # refusing would mean "reload the page, then wait out pion's
                # ICE timeout". Delete is idempotent for video and duplex; for
                # audio an unknown id answers not-found, which is equally
                # harmless here because the result is discarded either way.
                #
                # Across kinds this is not politeness but necessity: the worker
                # would otherwise open a device the previous session still
                # holds and fail somewhere inside GStreamer.
                #
                # The refuse-instead answer would be a 409 whep_viewer_active;
                # the code name is reserved but unused, so the choice stays
                # visible if preemption ever turns out to be wrong.
                self._logger.info(
                    "[WebRtcGateway] Preempting the session",
                    kind=other,
                    for_kind=kind,
                    session_id=session_id,
                )
                self._call(_EXPORTS[other][1], session_id)

            success, message, envelope = self._call(create_export, offer_sdp)
        except Exception as e:  # noqa: BLE001
            # A ctypes call into a foreign runtime can raise OSError,
            # ArgumentError or MemoryError. Releasing the slot on every exit
            # path matters more than the diagnosis: a slot left claimed wedges
            # this kind of session until the backend restarts (the teleop
            # router's finally-cleanup precedent).
            with self._session_lock:
                self._slots[kind] = None
            self._logger.error("Session creation raised", kind=kind, exc_info=True)
            return False, f"{kind} session creation raised: {e}", {}

        with self._session_lock:
            if not success:
                self._slots[kind] = None
                return False, message, {}
            self._slots[kind] = _Slot(
                session_id=envelope.get("session_id"), created_at=time.monotonic()
            )

        return True, "", {
            "session_id": envelope.get("session_id"),
            "answer_sdp": envelope.get("answer_sdp", ""),
        }

    def _delete_session(self, kind: str, session_id: str) -> Tuple[bool, str, None]:
        """Tear a session down."""
        _, delete_export = _EXPORTS[kind]

        with self._load_lock:
            success, message = self._ensure_loaded()
        if not success:
            return False, message, None

        supported, message = self._require_kind(kind)
        if not supported:
            return False, message, None

        # Cleared before the call, not after: a slow or failing delete must not
        # leave the slot pointing at a session the operator already dropped.
        # Only when the id matches -- a late DELETE from a tab we already
        # preempted has no business clearing the live session.
        with self._session_lock:
            slot = self._slots[kind]
            if slot is not None and slot.session_id == session_id:
                self._slots[kind] = None

        success, message, _ = self._call(delete_export, session_id)
        return success, message, None

    def create_video_session(self, offer_sdp: str) -> Tuple[bool, str, dict]:
        """WHEP: the robot's camera and microphone out to the browser."""
        return self._create_session(_VIDEO, offer_sdp)

    def delete_video_session(self, session_id: str) -> Tuple[bool, str, None]:
        """Idempotent, matching the worker's export."""
        return self._delete_session(_VIDEO, session_id)

    def create_audio_session(self, offer_sdp: str) -> Tuple[bool, str, dict]:
        """WHIP: the browser's microphone in to the robot's speaker."""
        return self._create_session(_AUDIO, offer_sdp)

    def delete_audio_session(self, session_id: str) -> Tuple[bool, str, None]:
        """Unlike the video export, this answers not-found for an unknown id."""
        return self._delete_session(_AUDIO, session_id)

    def create_duplex_session(self, offer_sdp: str) -> Tuple[bool, str, dict]:
        """Both directions over one peer connection.

        Claims the camera and the speaker together, so it preempts any live
        WHEP or WHIP session rather than failing inside GStreamer on a device
        somebody else holds.
        """
        return self._create_session(_DUPLEX, offer_sdp)

    def delete_duplex_session(self, session_id: str) -> Tuple[bool, str, None]:
        """Idempotent, matching the worker's export."""
        return self._delete_session(_DUPLEX, session_id)

    def status(self) -> Dict:
        """What this backend believes about its session slots.

        Belief, not truth, and the distinction is the whole reason this is a
        route rather than a health check. There is no callback from Go and no
        per-session stats export, so a session the browser abandoned still
        reads as active here until the next POST preempts it, and a session
        that self-closed on connection failure reads as active forever. The
        honest fix is a SessionStats export in the worker; a timeout-based
        reaper here would have to guess, and would guess wrong against exactly
        the case it exists for -- a long teleop run nobody is clicking through.

        Deliberately does not load the library: a console polling this must not
        be what pulls the Go runtime into an rclpy process, and
        ``library_loaded: false`` is a real and useful answer.
        """
        with self._session_lock:
            slots = dict(self._slots)

        def describe(slot: Optional[_Slot]) -> Optional[dict]:
            if slot is None or slot.session_id is None:
                return None
            return {
                "session_id": slot.session_id,
                "age_seconds": time.monotonic() - slot.created_at,
            }

        return {
            "library_loaded": self._lib is not None,
            "video": describe(slots[_VIDEO]),
            "audio": describe(slots[_AUDIO]),
            "duplex": describe(slots[_DUPLEX]),
        }


def init_webrtc_gateway(logger: structlog.stdlib.BoundLogger) -> WebRtcGateway:
    return WebRtcGateway(logger=logger)
