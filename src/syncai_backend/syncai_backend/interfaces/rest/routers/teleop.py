import asyncio
import structlog

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from syncai_backend.gateways.robot.robot import RobotGateway


# Stale-input watchdog. The frontend sender runs at 10 Hz, so this is five
# missed ticks of margin; a background-throttled browser tab (intervals
# clamped to >= 1 s) trips it, which is the safe direction. The watchdog
# lives HERE and not in the driver manager because the driver has none: when
# a cmd_vel publisher goes quiet it just stops sending AXES datagrams — it
# never sends a stop — so whoever opens a command channel owns stopping it.
_WATCHDOG_S = 0.5

# The driver->gait-controller hop is fire-and-forget UDP (sendto's return is
# ignored), so a single zero can be silently lost. The ROS hop is reliable;
# repeating the zero N times yields N AXES datagrams on the lossy hop.
_STOP_REPEATS = 3


def init_teleop_router(
    logger: structlog.stdlib.BoundLogger, robot_gw: RobotGateway
) -> APIRouter:
    teleop_router = APIRouter(prefix="", tags=["Teleop"])

    @teleop_router.websocket("/api/v1/robot/teleop")
    async def teleop(ws: WebSocket):
        """Inbound velocity-command channel for the console's manual control.

        The inverse of the telemetry/pointcloud sockets: the client talks,
        the server is silent except for error frames.

        Wire format, client -> server, JSON text frames at ~10 Hz:

            {"vx": .., "vy": .., "wz": ..}

        each axis clamped by the gateway to [-1, 1] and published as-is in
        m/s and rad/s (REP-103 body frame: +vx forward, +vy left, +wz CCW).
        There is deliberately no scale-down below that clamp any more — full
        stick is 1.0 m/s — but the clamp itself still lives gateway-side,
        where a client cannot reach it.

        Server -> client frames exist only to say no:

            {"error": "autonomous move in progress"}   (teleop refused; the
                socket stays open — cancel the task, then drive)
            {"error": "malformed frame..."}            (that frame skipped)

        Publishing happens directly on the event loop, unlike the robot
        router's sync-def handlers: those block on service round-trips, while
        Publisher.publish() is a non-blocking enqueue cheap enough for 10 Hz.
        """
        await ws.accept()
        try:
            while True:
                try:
                    # wait_for cancels the pending receive on timeout; with
                    # uvicorn's asyncio transport that cancellation is safe
                    # (the message queue read is cancellation-atomic), and a
                    # frame that raced the timeout is simply picked up by the
                    # next iteration's receive.
                    msg = await asyncio.wait_for(
                        ws.receive_json(), timeout=_WATCHDOG_S
                    )
                except asyncio.TimeoutError:
                    # Client connected but quiet — keep the robot stopped
                    # until frames resume or the socket dies.
                    robot_gw.teleop_stop()
                    continue
                except ValueError:
                    await ws.send_json({"error": "malformed frame; expected JSON"})
                    continue

                try:
                    vx = float(msg["vx"])
                    vy = float(msg["vy"])
                    wz = float(msg["wz"])
                except (KeyError, TypeError, ValueError):
                    await ws.send_json(
                        {"error": "malformed frame; expected {vx, vy, wz} floats"}
                    )
                    continue

                ok, message = robot_gw.teleop_cmd_vel(vx=vx, vy=vy, wz=wz)
                if not ok:
                    await ws.send_json({"error": message})
        except WebSocketDisconnect:
            return
        except Exception:
            logger.error("teleop stream failed", exc_info=True)
            return
        finally:
            # Runs on every exit path — clean close, kill -9'd browser, bug
            # above. This, not the client's best-effort zero-then-close, is
            # the actual stop-on-disconnect mechanism.
            for _ in range(_STOP_REPEATS):
                robot_gw.teleop_stop()

    return teleop_router
