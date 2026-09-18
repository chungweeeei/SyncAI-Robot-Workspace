import structlog
from typing import Optional
from fastapi import APIRouter, Body, Response
from pydantic import BaseModel, Field

from syncai_backend.exceptions import BadRequestError, ConflictError, UpstreamError
from syncai_backend.gateways.webrtc.webrtc import WebRtcGateway


# An SDP offer from a browser is a few kilobytes. The cap is a boundary guard,
# not a tuning knob: without it a stray upload reaches a cgo call.
_MAX_SDP_BYTES = 64 * 1024


class SessionStatus(BaseModel):
    session_id: str = Field(..., description="The worker's id for the live session.")
    age_seconds: float = Field(
        ..., description="Seconds since this backend created the session."
    )


class WebRtcStatusResponse(BaseModel):
    library_loaded: bool = Field(
        ...,
        description="Whether the Go worker library has been loaded yet. False on a "
        "boot that has never served a WHEP request.",
    )
    video: Optional[SessionStatus] = Field(
        None,
        description="The single WHEP viewer slot, as this backend believes it to be. "
        "There is no callback from the worker, so an abandoned session reads as "
        "active until the next POST preempts it.",
    )
    audio: Optional[SessionStatus] = Field(
        None,
        description="The single WHIP talker slot, with the same caveat as `video`.",
    )
    duplex: Optional[SessionStatus] = Field(
        None,
        description="The bidirectional slot. It holds the camera and the speaker "
        "together, so it is never populated at the same time as `video` or "
        "`audio` — a create of either kind preempts the other.",
    )


def init_webrtc_router(
    logger: structlog.stdlib.BoundLogger, webrtc_gw: WebRtcGateway
) -> APIRouter:
    webrtc_router = APIRouter(prefix="", tags=["WebRTC"])

    # Plain (non-async) handlers: creating a session blocks in the worker until
    # ICE gathering completes (0.1-5 s, in a cgo call that releases the GIL) --
    # threadpool work, not event-loop work. An async handler here would stall
    # the telemetry, point-cloud and teleop WebSockets for the duration.
    #
    # Which is also why the offer arrives as a `bytes` body parameter rather
    # than through Request.body(): that one is a coroutine, and awaiting it
    # would force the handler async. FastAPI reads the body during dependency
    # resolution and only then dispatches the sync handler to the threadpool,
    # which puts the blocking call exactly where it belongs.

    def _decode_offer(raw: bytes) -> str:
        """Turn the request body into an SDP string, or refuse it as a 400.

        Four checks and no more. Python does not parse SDP here: the offer's
        semantics are pion's to judge, and a pre-check that guesses wrong is a
        route that breaks the next time a browser changes its offer shape.
        """
        if not raw:
            raise BadRequestError(
                "the request body is empty; POST the SDP offer as the body"
            )
        if len(raw) > _MAX_SDP_BYTES:
            raise BadRequestError(f"offer SDP is too large (> {_MAX_SDP_BYTES} bytes)")
        try:
            text = raw.decode("utf-8")
        except UnicodeDecodeError:
            raise BadRequestError("offer SDP is not valid UTF-8")
        if not text.lstrip().startswith("v=0"):
            raise BadRequestError("offer SDP must start with 'v=0'")
        if "m=" not in text:
            raise BadRequestError("offer SDP carries no media sections")
        return text

    def _raise_for(message: str) -> None:
        # Keyed on the message, the tts router's `unknown voice` pattern: a
        # typed error crossing the gateway boundary has no precedent here and
        # the set of distinguishable failures is two.
        #
        # Everything else -- a missing library, a failed InitWorker, an offer
        # pion rejected, a GStreamer pipeline that would not start because the
        # camera is held by scripts/publish_camera_crop.sh -- answers 502 with
        # the worker's own sentence. Camera-busy deliberately gets no `code`:
        # it is indistinguishable from any other pipeline failure in the Go
        # error string, and a code that guesses is worse than prose that does
        # not.
        if "creation already in progress" in message:
            raise ConflictError(message, code="whep_session_pending")
        raise UpstreamError(message)

    # No response_model -- the body is the answer SDP itself, so the OpenAPI
    # schema is declared through `responses` instead (the tts /synthesize
    # precedent).
    @webrtc_router.post(
        "/api/v1/webrtc/whep",
        status_code=201,
        responses={
            201: {
                "content": {"application/sdp": {}},
                "description": "The answer SDP. `Location` carries the session URL.",
            }
        },
        response_class=Response,
    )
    # Defaulted to empty rather than required: with Body(...) an empty body is
    # rejected by FastAPI's own validation as a 422 carrying a pydantic blob
    # ("Field required", loc ["body"]), which tells a WHEP client nothing. The
    # default lets _decode_offer answer the 400 with a sentence instead.
    def whep_create(offer: bytes = Body(b"", media_type="application/sdp")):
        offer_sdp = _decode_offer(offer)

        # The worker offers two sendonly tracks (H264 + Opus), so it expects
        # two recvonly m-lines. Logged rather than refused: that is the
        # worker's constraint, and enforcing it here would pin it in two
        # places that then have to be changed together.
        if offer_sdp.count("m=video") != 1 or offer_sdp.count("m=audio") != 1:
            logger.warning(
                "WHEP offer is not the expected two-m-line shape",
                video_lines=offer_sdp.count("m=video"),
                audio_lines=offer_sdp.count("m=audio"),
            )

        success, message, payload = webrtc_gw.create_video_session(offer_sdp=offer_sdp)
        if not success:
            logger.error("WHEP session creation failed", message=message)
            _raise_for(message)

        session_id = payload["session_id"]
        logger.info("WHEP session created", session_id=session_id)

        # Location is relative on purpose. An absolute URL would have to be
        # built from the request, which arrives at 0.0.0.0:3000 behind whatever
        # host the console was pointed at; baking that in yields a teardown URL
        # that works from one network and not another. WHEP permits a relative
        # URI and browsers resolve it against the response URL.
        #
        # status_code is set here as well as on the decorator: the decorator
        # is what OpenAPI reads, this is what goes on the wire.
        return Response(
            status_code=201,
            content=payload["answer_sdp"],
            media_type="application/sdp",
            headers={"Location": f"/api/v1/webrtc/whep/{session_id}"},
        )

    @webrtc_router.delete(
        "/api/v1/webrtc/whep/{session_id}", status_code=204, response_class=Response
    )
    def whep_delete(session_id: str):
        """Tear a WHEP session down.

        Idempotent, matching the worker: a 204 even for an id it has never
        seen. WHEP clients fire this on page unload, and the worker cannot tell
        "already gone" (video sessions self-close on connection failure) from
        "never existed" anyway, so a 404 here would be noise that means nothing.
        """
        success, message, _ = webrtc_gw.delete_video_session(session_id=session_id)
        if not success:
            logger.error(
                "WHEP session teardown failed", session_id=session_id, message=message
            )
            _raise_for(message)

        return Response(status_code=204)

    @webrtc_router.post(
        "/api/v1/webrtc/whip",
        status_code=201,
        responses={
            201: {
                "content": {"application/sdp": {}},
                "description": "The answer SDP. `Location` carries the session URL.",
            }
        },
        response_class=Response,
    )
    def whip_create(offer: bytes = Body(b"", media_type="application/sdp")):
        """Talk to the robot: the browser's microphone out of its speaker.

        The mirror image of the WHEP route and deliberately identical in shape
        -- same body type, same Location contract, same blocking-in-a-worker-
        thread reasoning. Only the direction differs, and only the worker cares
        about that.

        One talker at a time, and that is a property of the hardware rather
        than a policy: the speaker is one ALSA device, so a second session
        would be fighting the first for it. A new POST preempts the old
        session, matching WHEP.
        """
        offer_sdp = _decode_offer(offer)

        # The worker receives exactly one audio track and offers none back, so
        # a browser that sends video here is sending something nothing will
        # consume. Logged rather than refused, for the same reason as the WHEP
        # side: the constraint is the worker's, and enforcing it in two places
        # means changing it in two places.
        if offer_sdp.count("m=audio") != 1:
            logger.warning(
                "WHIP offer is not the expected single-audio shape",
                audio_lines=offer_sdp.count("m=audio"),
                video_lines=offer_sdp.count("m=video"),
            )

        success, message, payload = webrtc_gw.create_audio_session(offer_sdp=offer_sdp)
        if not success:
            logger.error("WHIP session creation failed", message=message)
            _raise_for(message)

        session_id = payload["session_id"]
        logger.info("WHIP session created", session_id=session_id)

        return Response(
            status_code=201,
            content=payload["answer_sdp"],
            media_type="application/sdp",
            headers={"Location": f"/api/v1/webrtc/whip/{session_id}"},
        )

    @webrtc_router.delete(
        "/api/v1/webrtc/whip/{session_id}", status_code=204, response_class=Response
    )
    def whip_delete(session_id: str):
        """Tear a WHIP session down.

        Answers 204 even for an id the worker rejects as unknown. The audio
        export is not idempotent the way the video one is -- it reports
        not-found -- but that difference is the worker's bookkeeping, not
        something a client firing this on page unload can act on, and this
        route's job is to release the speaker. The gateway has already dropped
        the slot by the time the worker is asked.
        """
        success, message, _ = webrtc_gw.delete_audio_session(session_id=session_id)
        if not success:
            # Logged, not raised: see the docstring. A 502 here would make a
            # page unload look like a robot fault.
            logger.info(
                "WHIP session teardown reported no such session",
                session_id=session_id,
                message=message,
            )

        return Response(status_code=204)

    @webrtc_router.post(
        "/api/v1/webrtc/duplex",
        status_code=201,
        responses={
            201: {
                "content": {"application/sdp": {}},
                "description": "The answer SDP. `Location` carries the session URL.",
            }
        },
        response_class=Response,
    )
    def duplex_create(offer: bytes = Body(b"", media_type="application/sdp")):
        """Both directions over one peer connection: camera out, microphone in.

        Deliberately **not** called WHEP or WHIP, and not served under either
        path. Both of those specify one direction, and a client that finds a
        WHEP endpoint is entitled to assume it can send an offer with no
        sendonly m-line and get video. This is a third thing with its own
        contract, so it gets its own name rather than a flag on a route that
        already means something.

        What it buys over holding a WHEP and a WHIP session at once: one
        ICE/DTLS handshake instead of two, one gathering wait instead of two,
        one port pair, and a failure mode where both directions live or die
        together instead of one connecting while the other silently does not.

        What it costs: it holds the camera *and* the speaker, so it preempts
        any live WHEP or WHIP session, and either of those preempts it back.
        The gateway arbitrates that by device rather than by kind.
        """
        offer_sdp = _decode_offer(offer)

        # The shape the worker answers: one video m-line it sends on, one audio
        # m-line it both sends and receives on. Logged rather than refused, for
        # the same reason as the other two routes -- the constraint belongs to
        # the worker, and enforcing it here means changing it in two places.
        # A browser that offers two separate audio m-lines instead gets the
        # second one rejected by pion, which shows up as no microphone rather
        # than as an error.
        if offer_sdp.count("m=video") != 1 or offer_sdp.count("m=audio") != 1:
            logger.warning(
                "Duplex offer is not the expected one-video-one-audio shape",
                video_lines=offer_sdp.count("m=video"),
                audio_lines=offer_sdp.count("m=audio"),
            )

        success, message, payload = webrtc_gw.create_duplex_session(offer_sdp=offer_sdp)
        if not success:
            logger.error("Duplex session creation failed", message=message)
            _raise_for(message)

        session_id = payload["session_id"]
        logger.info("Duplex session created", session_id=session_id)

        return Response(
            status_code=201,
            content=payload["answer_sdp"],
            media_type="application/sdp",
            headers={"Location": f"/api/v1/webrtc/duplex/{session_id}"},
        )

    @webrtc_router.delete(
        "/api/v1/webrtc/duplex/{session_id}", status_code=204, response_class=Response
    )
    def duplex_delete(session_id: str):
        """Tear a duplex session down, releasing both devices.

        Idempotent like the WHEP teardown: the worker's export answers success
        for an id it has never seen, because a duplex session self-closes on
        connection failure and "already gone" is the normal case for a browser
        that dropped off the network.
        """
        success, message, _ = webrtc_gw.delete_duplex_session(session_id=session_id)
        if not success:
            logger.error(
                "Duplex session teardown failed", session_id=session_id, message=message
            )
            _raise_for(message)

        return Response(status_code=204)

    @webrtc_router.get("/api/v1/webrtc/status", response_model=WebRtcStatusResponse)
    def webrtc_status():
        """What this backend believes about the camera and speaker streams.

        Earns its own route because the media path is fire-and-forget: the
        worker sends RTP into a socket and exports no session statistics, so
        without this there is no way to tell "nobody is watching" from
        "somebody is watching, do not steal their stream".

        Not folded into /health: that route is the task server's liveness
        contract, and its docstring argues explicitly against letting a
        subsystem's degradation reach a container healthcheck.
        """
        return WebRtcStatusResponse(**webrtc_gw.status())

    return webrtc_router
