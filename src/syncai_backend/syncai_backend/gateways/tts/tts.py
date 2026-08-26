import os
import re
import subprocess
import threading
import wave
from io import BytesIO
from typing import List, Optional, Tuple

import numpy as np
import structlog


class TtsGateway:
    def __init__(self, logger: structlog.stdlib.BoundLogger):
        self._logger = logger

        # Where the weights live. Two files, downloaded once
        #   https://github.com/thewh1teagle/kokoro-onnx/releases/download/model-files-v1.1/kokoro-v1.0.onnx
        #   https://github.com/thewh1teagle/kokoro-onnx/releases/download/model-files-v1.1/voices-v1.0.bin
        self.model_path = os.path.expanduser("~/robot_ws/models/kokoro/kokoro-v1.0.onnx")
        self.voices_path = os.path.expanduser("~/robot_ws/models/kokoro/voices-v1.0.bin")

        # Which ALSA device is the speaker is answered by udev, not here:
        # syncai_sys_manager's 99-syncai-devices.rules keeps this symlink
        # pointed at the speaker's pcm node (pcmC<card>D<dev>p), whatever card
        # number the kernel handed it this boot. ALSA cannot open a /dev path
        # directly, so _resolve_playback_device() turns the link target back
        # into a "plughw:<card>,<dev>" name per utterance — per utterance
        # rather than once here, because a replug mid-run moves the card
        # number and the link, and a name resolved at construction would go
        # stale. (The compose file bind-mounts /dev/syncai for exactly this.)
        self.speaker_pcm_link = "/dev/syncai/speaker_pcm"

        # Fallback for hosts without the udev rules (dev machines, a robot
        # whose rules are not installed yet): the Jieli CD002 USB dongle by
        # CARD= name rather than card number — the number depends on USB probe
        # order, the name comes from the device descriptor and survives a
        # replug. This is the pre-udev behavior, kept so TTS degrades to
        # "works if the usual dongle is present" instead of failing. plughw
        # rather than hw (in both paths) so ALSA resamples kokoro's 24 kHz to
        # whatever the device actually supports.
        self.fallback_playback_device = "plughw:CARD=CD002AUDIO,DEV=0"

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

    # --- Playback device ----------------------------------------------------
    def _resolve_playback_device(self) -> str:
        """ALSA name of the speaker, via the udev symlink when it exists.

        /dev/syncai/speaker_pcm -> ../snd/pcmC2D0p encodes the live card and
        device number in its target's name; realpath follows it and the regex
        lifts the numbers out. Any miss — link absent (rules not installed,
        speaker unplugged) or a target that is not a pcm node — falls back to
        the by-name device rather than raising: aplay itself is the error
        reporter with an actionable message, a broken link should not change
        that into a Python traceback.
        """
        target = os.path.realpath(self.speaker_pcm_link)
        match = re.fullmatch(r"pcmC(\d+)D(\d+)p", os.path.basename(target))
        if match is None:
            return self.fallback_playback_device
        return f"plughw:{match.group(1)},{match.group(2)}"

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
            import onnxruntime as ort
            from kokoro_onnx import Kokoro

            opts = ort.SessionOptions()
            opts.intra_op_num_threads = 4
            session = ort.InferenceSession(
                self.model_path, opts, providers=["CPUExecutionProvider"]
            )
            self._kokoro = Kokoro.from_session(session, self.voices_path)
        except Exception as e:
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

        success, message, wav_bytes = self.synthesize(text=text, voice=voice, speed=speed)
        if not success:
            return False, message, None

        # 44-byte canonical header; close enough for a timeout margin.
        duration = (len(wav_bytes) - 44) / 2 / 24000.0
        try:
            # aplay reads the WAV (header included) from stdin. -q so its
            # per-file banner does not land in the backend pane's multilog on
            # every utterance.
            result = subprocess.run(
                ["aplay", "-q", "-D", self._resolve_playback_device(), "-"],
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
