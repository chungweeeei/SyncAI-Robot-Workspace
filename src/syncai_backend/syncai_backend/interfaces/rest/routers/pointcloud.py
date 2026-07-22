import asyncio
import struct
import structlog

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from syncai_backend.repositories.pointcloud.pointcloud import PointCloudRepo


# ~10 Hz push loop. body_cloud is produced at lidar rate; the client only ever
# needs the newest frame, so we poll the single-slot cache at a fixed cadence
# and skip when nothing new has arrived.
_STREAM_INTERVAL_S = 0.1


def init_pointcloud_router(
    logger: structlog.stdlib.BoundLogger, pointcloud_repo: PointCloudRepo
) -> APIRouter:
    pointcloud_router = APIRouter(prefix="", tags=["PointCloud"])

    @pointcloud_router.websocket("/api/v1/robot/pointcloud/stream")
    async def stream_pointcloud(ws: WebSocket):
        """Stream the latest map-frame body_cloud as binary frames.

        Wire format per frame: a little-endian uint32 point count followed by
        ``3 * count`` little-endian float32 values (xyz triplets). The frontend
        reads these straight into a three.js BufferGeometry.
        """
        await ws.accept()
        last_seq = 0
        try:
            while True:
                frame = pointcloud_repo.get_latest(after_seq=last_seq)
                if frame is not None:
                    last_seq = frame.seq
                    header = struct.pack("<I", frame.num_points)
                    await ws.send_bytes(header + frame.data)
                await asyncio.sleep(_STREAM_INTERVAL_S)
        except WebSocketDisconnect:
            return
        except Exception:
            logger.error("point cloud stream failed", exc_info=True)
            return

    return pointcloud_router
