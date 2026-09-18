from datetime import datetime, timezone
from enum import Enum
from typing import List, Optional

import structlog
from fastapi import APIRouter, Response
from pydantic import BaseModel, Field

from syncai_backend.exceptions import BadRequestError, ConflictError, UpstreamError
from syncai_backend.gateways.recording.recording import (
    DEFAULT_TOPICS,
    RESERVED_NAMES,
    RecordingGateway,
)
from syncai_backend.repositories.recording.catalog import (
    RecordingCatalogRepo,
    StoredRecording,
)


# What a caller may ask for at once. Not a resource limit -- rosbag2 handles far
# more -- but a boundary guard, so a malformed body cannot turn into a several-
# thousand-element argv.
_MAX_TOPICS = 64


class RecordingStatus(str, Enum):
    """How a recording on disk ended.

    ``recording`` is the live one, ``ok`` a bag with a metadata.yaml, and
    ``interrupted`` a directory with neither -- which means the recorder died
    without flushing: the backend pane was killed, a `switch_mode` tore down the
    byobu session, or the process crashed. Such a bag still holds its messages
    and `ros2 bag reindex <dir>` rebuilds the metadata.

    Only two of the three are stored: ``interrupted`` is derived from a
    directory with no metadata and no process behind it, the same way the map
    catalogue derives its own ``interrupted`` from a sidecar with no conversion
    thread behind it.
    """

    RECORDING = "recording"
    OK = "ok"
    INTERRUPTED = "interrupted"


class StartRecordingRequest(BaseModel):
    name: Optional[str] = Field(
        None,
        description="Directory name under record/. Defaults to a UTC timestamp "
        "(rec_YYYYmmdd_HHMMSS). Letters, digits, dot, dash and underscore only — "
        "rosbag2 names its split files after it.",
    )
    topics: Optional[List[str]] = Field(
        None,
        description="Topics to record. A name without a leading slash is resolved "
        "under this robot's namespace (`livox/lidar` → `/<robot_id>/livox/lidar`); "
        "an absolute name is passed through, which is how `/tf` and `/tf_static` "
        "are recorded. Defaults to the LIO inputs "
        f"({', '.join(DEFAULT_TOPICS)}), the pair a lost mapping run is replayed "
        "from.",
    )
    compression: bool = Field(
        False,
        description="Compress each closed 2 GB split with zstd. Off by default: "
        "the compression runs on the recorder's own thread in a burst, and the "
        "run worth recording is usually the one where the Tegra is already busy "
        "with LIO.",
    )


class ActiveRecordingResponse(BaseModel):
    name: str
    path: str = Field(..., description="Absolute path of the bag directory.")
    topics: List[str] = Field(..., description="Fully resolved topic names.")
    started_at: datetime
    elapsed_seconds: float
    compression: bool
    size_bytes: int = Field(
        ..., description="Bytes written so far. Walks the bag directory on each call."
    )


class StopRecordingResponse(BaseModel):
    name: str
    path: str
    topics: List[str]
    started_at: datetime
    elapsed_seconds: float
    size_bytes: int
    stopped_by: str = Field(
        ...,
        description="Which rung of the stop ladder ended it: `sigint` (the normal "
        "one), `sigterm`, `sigkill`, `already_exited` or `kill_failed`.",
    )
    complete: bool = Field(
        ...,
        description="Whether metadata.yaml was written. False means the bag needs "
        "`ros2 bag reindex` before it can be played — the messages are still there.",
    )


class RecordingSummary(BaseModel):
    name: str
    status: RecordingStatus
    size_bytes: int
    modified_at: datetime
    duration_seconds: Optional[float] = Field(
        None, description="From the bag metadata; null until the recording finishes."
    )
    message_count: Optional[int] = Field(
        None,
        description="From the bag metadata. A zero here on a finished bag means the "
        "topics were named but never published — the usual sign of a typo, since "
        "nothing refuses a topic that does not exist yet.",
    )
    topics: List[str] = Field(
        default_factory=list,
        description="The topics actually recorded, from the bag metadata. Empty "
        "while the recording is live: rosbag2 writes the list on shutdown.",
    )
    compression: Optional[str] = Field(
        None, description="Compression format, or null for an uncompressed bag."
    )


class ListRecordingsResponse(BaseModel):
    recordings: List[RecordingSummary]


def _summary(stored: StoredRecording, active_name: Optional[str]) -> RecordingSummary:
    """Project one directory, reconciling disk state with the live slot.

    The reconciliation is the whole point, and it is the same split as the map
    catalogue's: the bag metadata is what survives this process, the gateway's
    slot is what is true only while it is up, and neither alone can tell a
    recording in progress from one whose process is gone.
    """
    if stored.name == active_name:
        status = RecordingStatus.RECORDING
    elif stored.bag is None:
        status = RecordingStatus.INTERRUPTED
    else:
        status = RecordingStatus.OK

    bag = stored.bag
    return RecordingSummary(
        name=stored.name,
        status=status,
        size_bytes=stored.size_bytes,
        modified_at=stored.modified_at,
        duration_seconds=bag.duration_seconds if bag else None,
        message_count=bag.message_count if bag else None,
        topics=bag.topics if bag else [],
        compression=bag.compression if bag else None,
    )


def init_recording_router(
    logger: structlog.stdlib.BoundLogger,
    recording_gw: RecordingGateway,
    recording_catalog_repo: RecordingCatalogRepo,
) -> APIRouter:
    recording_router = APIRouter(prefix="", tags=["Recording"])

    # Plain (non-async) handlers: starting a recorder blocks for the liveness
    # probe, stopping one blocks until the child has flushed its metadata (up to
    # ~22 s), and listing walks every bag directory. All three are threadpool
    # work; an async handler would stall the telemetry, point-cloud and teleop
    # WebSockets for the duration.

    @recording_router.post(
        "/api/v1/recordings", status_code=201, response_model=ActiveRecordingResponse
    )
    def start_recording(request: StartRecordingRequest):
        """Start recording ROS topics into record/<name>/.

        Answers with the recording as it stands one heartbeat in, not with a
        promise: the response is written after the child has survived its
        liveness probe, so a 201 here means a recorder is running.

        Every refusal happens before anything is spawned. Two of them are 409s a
        client must tell apart by ``code``: ``recording_running`` means stop the
        current one first (its name is in the message), ``name_taken`` means
        pick another name — rosbag2 will not write into an existing directory,
        and silently appending to somebody else's bag would be worse if it did.
        """
        name = (request.name or "").strip() or _default_name()
        if name in RESERVED_NAMES:
            raise BadRequestError(
                f"'{name}' is a reserved name — GET /api/v1/recordings/{name} is "
                "this API's own route and a directory of that name could never "
                "be read back. Pick another."
            )

        topics = request.topics if request.topics is not None else list(DEFAULT_TOPICS)
        topics = [topic for topic in topics if topic.strip()]
        if not topics:
            raise BadRequestError(
                "No topics to record. Omit `topics` for the default "
                f"({', '.join(DEFAULT_TOPICS)}) rather than sending an empty list."
            )
        if len(topics) > _MAX_TOPICS:
            raise BadRequestError(f"Too many topics (> {_MAX_TOPICS}).")

        # Asked before the name collision, and only so the operator reads the
        # more actionable of two simultaneously-true refusals: while something
        # is recording, nothing can start whatever it is called, so "stop the
        # current one" beats "pick another name". This is not the check that
        # enforces one-at-a-time -- that one is the slot claimed under the
        # gateway's lock below, and it stays authoritative; losing the race here
        # costs a less helpful sentence, nothing more.
        live = recording_gw.active_name()
        if live is not None:
            raise ConflictError(
                f"This robot is already recording '{live}'. Stop it before "
                "starting another (POST /api/v1/recordings/stop).",
                code="recording_running",
            )

        # resolve_dir is what validates the name against the filesystem-safety
        # pattern; calling it through exists() does both in one step.
        if recording_catalog_repo.exists(name):
            raise ConflictError(
                f"A recording named '{name}' already exists.", code="name_taken"
            )

        success, message, payload = recording_gw.start(
            name=name, topics=topics, compression=request.compression
        )
        if not success:
            logger.error("Failed to start recording", recording=name, message=message)
            _raise_for_start(message, payload)

        logger.info("Recording started", recording=name, topics=payload["topics"])
        return ActiveRecordingResponse(**payload)

    @recording_router.get(
        "/api/v1/recordings/active", response_model=Optional[ActiveRecordingResponse]
    )
    def active_recording():
        """What is being recorded right now, or `null`.

        A route of its own rather than "find the entry with status `recording`
        in the catalogue", which is a mild departure from the map router's rule
        that the catalogue is the only status surface. The reason is cost: a
        console polling this once a second would otherwise walk every bag
        directory on the robot to learn the size of one of them. Nothing is
        *only* knowable here — the listing reports the live recording too.

        Also where a recorder that died on its own is noticed: the slot is
        reaped on every read, so a crashed recording stops reading as active
        without anyone having to call stop.
        """
        status = recording_gw.status()
        return ActiveRecordingResponse(**status) if status is not None else None

    @recording_router.post(
        "/api/v1/recordings/stop", response_model=StopRecordingResponse
    )
    def stop_recording():
        """Stop the live recorder and wait for the bag to be closed.

        Blocks until the child is gone, because the one thing the caller wants
        to know — is this bag playable — is not knowable before that. The answer
        is in ``complete``: false means the recorder had to be killed and the
        bag needs `ros2 bag reindex`, not that the data is lost.
        """
        success, message, payload = recording_gw.stop()
        if not success:
            raise ConflictError(
                "Nothing is being recorded on this robot.", code="not_recording"
            )

        logger.info(
            "Recording stopped",
            recording=payload["name"],
            stopped_by=payload["stopped_by"],
            complete=payload["complete"],
        )
        return StopRecordingResponse(**payload)

    @recording_router.get("/api/v1/recordings", response_model=ListRecordingsResponse)
    def list_recordings():
        """Every bag under record/, newest first.

        The live slot is sampled **once**, before the projection, so the listing
        cannot report two states of the same recording either side of a stop
        that lands mid-scan.
        """
        active_name = recording_gw.active_name()
        return ListRecordingsResponse(
            recordings=[
                _summary(stored, active_name)
                for stored in recording_catalog_repo.list_recordings()
            ]
        )

    @recording_router.delete(
        "/api/v1/recordings/{name}", status_code=204, response_class=Response
    )
    def delete_recording(name: str):
        """Delete record/<name>/ and everything under it. There is no undo.

        Refused for the recording being written — an rmtree under a live sqlite
        writer leaves the recorder happily writing into an unlinked file, so the
        operator stops it first and then decides.
        """
        if recording_gw.active_name() == name:
            raise ConflictError(
                f"'{name}' is being recorded right now. Stop it first "
                "(POST /api/v1/recordings/stop).",
                code="recording_active",
            )

        recording_catalog_repo.delete_dir(name)
        logger.info("Recording deleted", recording=name)
        return Response(status_code=204)

    return recording_router


def _default_name() -> str:
    """A UTC timestamp, in the shape of the bags already on these robots.

    UTC rather than local time, because it is what every other timestamp this
    backend emits uses and a name that sorts is worth more here than one that
    matches the operator's wall clock.
    """
    return datetime.now(timezone.utc).strftime("rec_%Y%m%d_%H%M%S")


def _raise_for_start(message: str, payload: dict) -> None:
    """Map the gateway's sentence onto a status code.

    Keyed on the message, the tts and webrtc routers' pattern: the gateway's
    contract is a (success, message, payload) tuple and the set of failures the
    caller can act on differently is two. Everything else — no `ros2` on PATH,
    a spawn that failed, a recorder that exited on its own arguments — is this
    robot's fault and answers 502 with the gateway's own sentence.
    """
    if message.startswith("already recording"):
        raise ConflictError(
            f"This robot is {message}. Stop it before starting another "
            "(POST /api/v1/recordings/stop).",
            code="recording_running",
        )
    if message.startswith("only "):
        raise ConflictError(message, code="disk_low")
    raise UpstreamError(message)
