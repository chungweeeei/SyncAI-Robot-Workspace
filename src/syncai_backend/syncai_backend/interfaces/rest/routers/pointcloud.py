import asyncio
import struct
import structlog

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from syncai_backend.repositories.pointcloud.pointcloud import PointCloudRepo


# ~10 Hz push loop. body_cloud is produced at lidar rate; the client only ever
# needs the newest frame, so we poll the single-slot cache at a fixed cadence
# and skip when nothing new has arrived.
_STREAM_INTERVAL_S = 0.1

# The merged map cloud changes at most every few seconds (pgo's
# map_cloud_pub_period), so its poll can be an order of magnitude lazier —
# 0.5 s of extra latency on a multi-MB frame nobody watches frame-by-frame.
_MAP_STREAM_INTERVAL_S = 0.5


def init_pointcloud_router(
    logger: structlog.stdlib.BoundLogger,
    pointcloud_repo: PointCloudRepo,
    map_cloud_repo: PointCloudRepo,
) -> APIRouter:
    pointcloud_router = APIRouter(prefix="", tags=["PointCloud"])

    async def _pump(ws: WebSocket, repo: PointCloudRepo, interval_s: float):
        """Drain one single-slot cloud repo onto one WebSocket, forever.

        Wire format per frame: a little-endian uint32 point count followed by
        ``3 * count`` little-endian float32 values (xyz triplets). Both streams
        speak it, which is what lets the frontend reuse one decoder.
        """
        await ws.accept()
        last_seq = 0
        try:
            while True:
                frame = repo.get_latest(after_seq=last_seq)
                if frame is not None:
                    last_seq = frame.seq
                    header = struct.pack("<I", frame.num_points)
                    await ws.send_bytes(header + frame.data)
                await asyncio.sleep(interval_s)
        except WebSocketDisconnect:
            return
        except Exception:
            logger.error("point cloud stream failed", exc_info=True)
            return

    @pointcloud_router.websocket("/api/v1/robot/pointcloud/stream")
    async def stream_pointcloud(ws: WebSocket):
        """Stream the latest map-frame body_cloud as binary frames (~10 Hz)."""
        await _pump(ws, pointcloud_repo, _STREAM_INTERVAL_S)

    @pointcloud_router.websocket("/api/v1/robot/pointcloud/map/stream")
    async def stream_map_cloud(ws: WebSocket):
        """Stream pgo's merged "map so far" cloud as binary frames.

        Frames arrive only while a mapping session is up (pgo is the producer
        and only runs in MANUAL), every few seconds at most, and each one is a
        complete loop-closure-corrected replacement of the last — the client
        swaps its layer wholesale rather than accumulating.
        """
        await _pump(ws, map_cloud_repo, _MAP_STREAM_INTERVAL_S)

    return pointcloud_router
