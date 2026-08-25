import structlog
from typing import List
from fastapi import APIRouter, Response
from pydantic import BaseModel, Field

from syncai_backend.exceptions import BadRequestError, UpstreamError
from syncai_backend.gateways.tts.tts import TtsGateway


class SynthesizeRequest(BaseModel):
    text: str = Field(
        ...,
        min_length=1,
        max_length=1000,
        description="The text to speak. English only for now (see TtsGateway).",
    )
    voice: str = Field(
        "af_heart",
        description="Kokoro voice id; the list is at GET /api/v1/tts/voices.",
    )
    speed: float = Field(
        1.0, ge=0.5, le=2.0, description="Playback rate multiplier (0.5–2.0)."
    )


class SpeakResponse(BaseModel):
    message: str = Field(..., description="Human-readable result of the playback.")
    duration: float = Field(..., description="Length of the spoken audio, in seconds.")


class ListVoicesResponse(BaseModel):
    voices: List[str] = Field(..., description="The voice ids the loaded model carries.")


def init_tts_router(
    logger: structlog.stdlib.BoundLogger, tts_gw: TtsGateway
) -> APIRouter:
    tts_router = APIRouter(prefix="", tags=["TTS"])

    # Plain (non-async) handlers: synthesis is CPU-bound for up to a few
    # seconds (plus the one-time ~310 MB model load on the first request), and
    # /speak additionally blocks for the length of the utterance — threadpool
    # work, not event-loop work.

    def _raise_for(message: str) -> None:
        # The gateway's one caller-fixable failure; everything else (missing
        # weights, onnxruntime, aplay) is ours and answers 502.
        if message.startswith("unknown voice"):
            raise BadRequestError(message)
        raise UpstreamError(message)

    @tts_router.get("/api/v1/tts/voices", response_model=ListVoicesResponse)
    def list_voices():
        success, message, voices = tts_gw.list_voices()
        if not success:
            logger.error("Failed to list TTS voices", message=message)
            raise UpstreamError(message)
        return ListVoicesResponse(voices=voices)

    # No response_model — the body is the WAV itself, so the OpenAPI schema is
    # declared through `responses` instead.
    @tts_router.post(
        "/api/v1/tts/synthesize",
        responses={200: {"content": {"audio/wav": {}}, "description": "The rendered WAV."}},
        response_class=Response,
    )
    def synthesize(request: SynthesizeRequest):
        success, message, wav_bytes = tts_gw.synthesize(
            text=request.text, voice=request.voice, speed=request.speed
        )
        if not success:
            logger.error("TTS synthesis failed", message=message)
            _raise_for(message)
        return Response(content=wav_bytes, media_type="audio/wav")

    @tts_router.post("/api/v1/tts/speak", response_model=SpeakResponse)
    def speak(request: SynthesizeRequest):
        success, message, duration = tts_gw.speak(
            text=request.text, voice=request.voice, speed=request.speed
        )
        if not success:
            logger.error("TTS playback failed", message=message)
            _raise_for(message)
        return SpeakResponse(message="Spoken on the robot speaker", duration=duration)

    return tts_router
