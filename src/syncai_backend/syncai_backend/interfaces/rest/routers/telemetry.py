import asyncio
import structlog

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from syncai_backend.repositories.telemetry.telemetry import TelemetryRepo


# 20 Hz push loop — matches the lio_bridge odom rate, so a fresh pose is
# forwarded within one period of being produced. Like the point-cloud stream,
# this polls single-slot caches at a fixed cadence and skips a message type
# when nothing new has arrived.
_STREAM_INTERVAL_S = 0.05


def init_telemetry_router(
    logger: structlog.stdlib.BoundLogger, telemetry_repo: TelemetryRepo
) -> APIRouter:
    telemetry_router = APIRouter(prefix="", tags=["Telemetry"])

    @telemetry_router.websocket("/api/v1/robot/telemetry/stream")
    async def stream_telemetry(ws: WebSocket):
        """Stream high-rate robot telemetry for the frontend 3D viewer.

        This is the internal visualization channel: unlike the frozen
        GET /api/v1/robot/state third-party contract it may change shape
        freely, which is why it deliberately shares no Pydantic models with
        routers/robot.py.

        Wire format: JSON text frames, multiplexed by ``type``:

            {"type": "pose",   "x": .., "y": .., "z": .., "yaw_deg": ..,
             "stamp": ..}
            {"type": "joints", "joints": {"FL_HipX_joint": .., ...},
             "stamp": ..}
            {"type": "path",   "points": [[x, y], ...], "stamp": ..}

        Pose is the map-frame planar pose (yaw in degrees, matching the
        frontend's RobotPose.theta); joints are radians keyed by URDF joint
        name. Each type is sent only when a newer sample exists, so their
        rates are independent (pose ~20 Hz, joints at whatever rate the gait
        controller's telemetry arrives).

        ``path`` is the planner's remaining global route in map-frame metres,
        thinned by the subscriber and stripped of heading. It arrives once per
        replan (~0.333 Hz) — small and rare enough to share this socket rather
        than warrant its own, unlike the point cloud whose frames are two orders
        of magnitude larger. An **empty** ``points`` is a real sample meaning
        "no route": it is the frame that tells a client to erase the one it is
        drawing, which the mere absence of further path frames could not.
        """
        await ws.accept()
        last_pose_seq = 0
        last_joints_seq = 0
        last_path_seq = 0
        try:
            while True:
                pose = telemetry_repo.get_pose(after_seq=last_pose_seq)
                if pose is not None:
                    last_pose_seq = pose.seq
                    await ws.send_json(
                        {
                            "type": "pose",
                            "x": pose.x,
                            "y": pose.y,
                            "z": pose.z,
                            "yaw_deg": pose.yaw_deg,
                            "stamp": pose.stamp,
                        }
                    )

                joints = telemetry_repo.get_joints(after_seq=last_joints_seq)
                if joints is not None:
                    last_joints_seq = joints.seq
                    await ws.send_json(
                        {
                            "type": "joints",
                            "joints": joints.joints,
                            "stamp": joints.stamp,
                        }
                    )

                # Polled at the same 20 Hz as the rest, which costs nothing:
                # get_path returns None until the planner replans, so this sends
                # at the route's own ~0.333 Hz. Polling it here is also what runs
                # the repo's TTL check, so a finished route clears itself even
                # with no further ROS traffic at all.
                path = telemetry_repo.get_path(after_seq=last_path_seq)
                if path is not None:
                    last_path_seq = path.seq
                    await ws.send_json(
                        {
                            "type": "path",
                            "points": path.points,
                            "stamp": path.stamp,
                        }
                    )

                await asyncio.sleep(_STREAM_INTERVAL_S)
        except WebSocketDisconnect:
            return
        except Exception:
            logger.error("telemetry stream failed", exc_info=True)
            return

    return telemetry_router
