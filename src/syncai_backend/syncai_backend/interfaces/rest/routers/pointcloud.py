import struct
import structlog

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from syncai_backend.repositories.pointcloud.pointcloud import PointCloudRepo


def init_pointcloud_router(
    logger: structlog.stdlib.BoundLogger,
    pointcloud_repo: PointCloudRepo,
    map_cloud_repo: PointCloudRepo,
) -> APIRouter:
    pointcloud_router = APIRouter(prefix="", tags=["PointCloud"])

    async def _pump(ws: WebSocket, repo: PointCloudRepo):
        """Drain one single-slot cloud repo onto one WebSocket, forever.

        Wire format per frame: a little-endian uint32 point count followed by
        ``3 * count`` little-endian float32 values (xyz triplets). Both streams
        speak it, which is what lets the frontend reuse one decoder.

        Frame-driven, not timer-driven. This used to poll the slot every 100 ms
        (500 ms for the map cloud) and skip when the seq had not moved; against
        the 10 Hz body_cloud that cost a mean ~50 ms of queueing latency and,
        because two unsynchronised 10 Hz clocks beat, dropped ~5% of frames —
        measured on robot01 as a 9.5 Hz stream with a 205 ms worst-case gap for
        a producer that never missed a beat. See ``PointCloudRepo`` for the
        notification side.

        The single-slot repo still does the dropping, so a client slower than
        the producer is unchanged: ``send_bytes`` applies backpressure, frames
        superseded meanwhile are never sent, and the next ``get_latest`` returns
        the newest one rather than a backlog. That is also why there is no rate
        cap here — the producer's rate is the cap, and nothing queues.
        """
        await ws.accept()
        last_seq = 0
        try:
            # Registered before the first read, so a frame that lands during
            # this iteration sets the Event rather than being missed and waited
            # out. Same reason `clear()` comes before `get_latest` below and not
            # after `wait()`: clear-then-check can only ever cause a spurious
            # wakeup, check-then-clear can lose one and stall until the frame
            # after next.
            with repo.subscribe() as new_frame:
                while True:
                    new_frame.clear()
                    frame = repo.get_latest(after_seq=last_seq)
                    if frame is None:
                        await new_frame.wait()
                        continue
                    last_seq = frame.seq
                    header = struct.pack("<I", frame.num_points)
                    await ws.send_bytes(header + frame.data)
        except WebSocketDisconnect:
            return
        except Exception:
            logger.error("point cloud stream failed", exc_info=True)
            return

    @pointcloud_router.websocket("/api/v1/robot/pointcloud/stream")
    async def stream_pointcloud(ws: WebSocket):
        """Stream the latest map-frame body_cloud as binary frames (~10 Hz)."""
        await _pump(ws, pointcloud_repo)

    @pointcloud_router.websocket("/api/v1/robot/pointcloud/map/stream")
    async def stream_map_cloud(ws: WebSocket):
        """Stream pgo's merged "map so far" cloud as binary frames.

        Frames arrive only while a mapping session is up (pgo is the producer
        and only runs in MANUAL), every few seconds at most, and each one is a
        complete loop-closure-corrected replacement of the last — the client
        swaps its layer wholesale rather than accumulating.
        """
        await _pump(ws, map_cloud_repo)

    return pointcloud_router
