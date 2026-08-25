"""Text-to-speech via kokoro-onnx (Kokoro-82M).

A gateway rather than a repository: it holds no state anyone else reads — it is
an outbound surface, same category as the robot/map gateways, except the
"downstream" is an inference session plus the speaker instead of a ROS service.

The engine is kokoro-onnx, not the hexgrad PyTorch package, on purpose: the
model is 82M parameters, well inside CPU-real-time on the Orin, and the ONNX
route avoids dragging torch (and a Jetson-specific torch build) into the
backend image for it. English voices only for now — kokoro's Mandarin needs
misaki[zh] and the PyTorch pipeline, so a Chinese-capable engine is a separate
decision, not a parameter on this one.
"""

import os
import subprocess
import threading
import wave
from io import BytesIO
from typing import List, Optional, Tuple

import numpy as np
import structlog

# Where the weights live. Two files, downloaded once (they are gitignored —
# /models/ at the workspace root):
#   https://github.com/thewh1teagle/kokoro-onnx/releases/download/model-files-v1.1/kokoro-v1.0.onnx
#   https://github.com/thewh1teagle/kokoro-onnx/releases/download/model-files-v1.1/voices-v1.0.bin
# Hardcoded expanded-HOME path, the MapCatalogRepo.maps_dir precedent: logged at
# construction, re-pointed by attribute assignment in tests rather than a
# constructor argument that would exist only for them.
_MODELS_DIR = os.path.expanduser("~/robot_ws/models/kokoro")


class TtsGateway:
    def __init__(self, logger: structlog.stdlib.BoundLogger):
        self._logger = logger
        self.model_path = os.path.join(_MODELS_DIR, "kokoro-v1.0.onnx")
        self.voices_path = os.path.join(_MODELS_DIR, "voices-v1.0.bin")
        # ALSA name of the robot's speaker (the Jieli CD002 USB dongle), by
        # CARD= name rather than card number: the number depends on USB probe
        # order, the name comes from the device and survives a replug. plughw
        # rather than hw so ALSA resamples kokoro's 24 kHz to whatever the
        # dongle actually supports.
        self.playback_device = "plughw:CARD=CD002AUDIO,DEV=0"

        # Loaded on first use, not here: the .onnx is ~310 MB and construction
        # happens in SyncAIBackend.__init__, where three seconds of session
        # building would delay every subscriber and the REST server for a
        # feature most boots never use. The lock covers load-then-infer; it
        # also serialises synthesis itself, which is deliberate — two aplay
        # streams into one speaker is noise, and the upstream caller of the
        # /speak route is a human pressing a button.
        self._kokoro = None
        self._lock = threading.Lock()

        # Same breadcrumb rationale as MapCatalogRepo: the path is neither a
        # parameter nor an env var, so if the weights are missing this line is
        # the only pointer to where we looked.
        self._logger.info("[TtsGateway] Using kokoro model", path=self.model_path)

    # --- Engine ------------------------------------------------------------

    def _ensure_loaded(self) -> Tuple[bool, str]:
        """Build the Kokoro session on first call. Caller must hold the lock."""
        if self._kokoro is not None:
            return True, ""

        for path in (self.model_path, self.voices_path):
            if not os.path.isfile(path):
                return False, (
                    f"kokoro model file missing: {path} — download it from the "
                    "kokoro-onnx 'model-files' GitHub release (URLs at the top "
                    "of gateways/tts/tts.py)"
                )

        try:
            # Imported here, not at module top: kokoro-onnx is a pip-only dep
            # (see package.xml), and a backend booted before `pip install`
            # should come up with TTS degraded, not die on import at wiring
            # time in main.py.
            import onnxruntime as ort
            from kokoro_onnx import Kokoro

            # Our own session via from_session(), not Kokoro(model, voices),
            # because the thread count must be explicit. The default threadpool
            # derives per-thread affinities from the sysfs CPU topology, which
            # on this Orin includes the four cores nvpmodel MODE_30W keeps
            # offline — every session then logs a pthread_setaffinity_np
            # failure per thread (and onnxruntime >= 1.19 corrupts the heap
            # outright probing that topology, which is why requirements.txt
            # pins 1.18.1). Four threads measured *faster* than eight for a
            # 4.5 s utterance (4.4 s vs 5.1 s — the extra threads contend),
            # and it leaves the other online cores to the nav stack.
            opts = ort.SessionOptions()
            opts.intra_op_num_threads = 4
            session = ort.InferenceSession(
                self.model_path, opts, providers=["CPUExecutionProvider"]
            )
            self._kokoro = Kokoro.from_session(session, self.voices_path)
        except Exception as e:  # noqa: B902 — onnxruntime raises its own zoo
            return False, f"failed to load kokoro model: {e}"

        self._logger.info("[TtsGateway] Kokoro model loaded")
        return True, ""

    # --- API ---------------------------------------------------------------

    def list_voices(self) -> Tuple[bool, str, List[str]]:
        with self._lock:
            success, message = self._ensure_loaded()
            if not success:
                return False, message, []
            return True, "", sorted(self._kokoro.get_voices())

    def synthesize(
        self, text: str, voice: str = "af_heart", speed: float = 1.0
    ) -> Tuple[bool, str, bytes]:
        """Render text to a mono 16-bit WAV. Returns (success, message, wav_bytes).

        An unknown voice is reported as "unknown voice ..." — the router keys
        on that prefix to answer 400 instead of 502, because it is the one
        failure here that is the caller's to fix.
        """
        with self._lock:
            success, message = self._ensure_loaded()
            if not success:
                return False, message, b""

            if voice not in self._kokoro.get_voices():
                return False, f"unknown voice: {voice!r}", b""

            try:
                samples, sample_rate = self._kokoro.create(text, voice=voice, speed=speed)
            except Exception as e:
                return False, f"synthesis failed: {e}", b""

        # float32 [-1, 1] -> int16 via stdlib wave: soundfile would work too,
        # but it drags libsndfile into the image for a header we can write in
        # six lines.
        pcm = (np.clip(samples, -1.0, 1.0) * 32767.0).astype(np.int16)
        buffer = BytesIO()
        with wave.open(buffer, "wb") as wav:
            wav.setnchannels(1)
            wav.setsampwidth(2)
            wav.setframerate(sample_rate)
            wav.writeframes(pcm.tobytes())
        return True, "", buffer.getvalue()

    def speak(
        self, text: str, voice: str = "af_heart", speed: float = 1.0
    ) -> Tuple[bool, str, Optional[float]]:
        """Synthesize and play on the robot's speaker.

        Returns (success, message, duration_seconds). Playback needs alsa-utils
        in the image and /dev/snd + the host audio gid on the container (see
        docker-compose.robots.yml) — without them synthesis still works and the
        aplay error lands in `message`.
        """
        success, message, wav_bytes = self.synthesize(text, voice=voice, speed=speed)
        if not success:
            return False, message, None

        # 44-byte canonical header; close enough for a timeout margin.
        duration = (len(wav_bytes) - 44) / 2 / 24000.0
        try:
            # aplay reads the WAV (header included) from stdin. -q so its
            # per-file banner does not land in the backend pane's multilog on
            # every utterance.
            result = subprocess.run(
                ["aplay", "-q", "-D", self.playback_device, "-"],
                input=wav_bytes,
                capture_output=True,
                timeout=duration + 10.0,
            )
        except FileNotFoundError:
            return False, "aplay not found — alsa-utils is not installed", None
        except subprocess.TimeoutExpired:
            return False, "aplay timed out (device wedged?)", None

        if result.returncode != 0:
            stderr = result.stderr.decode(errors="replace").strip()
            return False, f"aplay failed: {stderr}", None

        return True, "", duration


def init_tts_gateway(logger: structlog.stdlib.BoundLogger) -> TtsGateway:
    return TtsGateway(logger=logger)
