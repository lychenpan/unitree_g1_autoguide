from __future__ import annotations

import asyncio
import json
import logging
import os
import queue
import threading
import time
import uuid
import wave
from dataclasses import dataclass
from typing import Optional
import sys
sys.path.append("/home/unitree/workspace/unitree_sdk2_python")

@dataclass
class UnitreePlayer:
    """
    Thin wrapper around Unitree G1 audio PlayStream API.

    Input PCM must be: PCM16LE, 16kHz, mono.
    """

    net_interface: str
    timeout_s: float = 10.0
    volume: int = 100

    def __post_init__(self) -> None:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient

        ChannelFactoryInitialize(0, self.net_interface)
        self._client = AudioClient()
        self._client.SetTimeout(self.timeout_s)
        self._client.Init()
        self.set_volume(self.volume)

    def set_volume(self, volume: int) -> None:
        code = self._client.SetVolume(int(volume))
        code_i = int(code[0] if isinstance(code, tuple) else code)
        if code_i != 0:
            raise RuntimeError(f"Unitree SetVolume failed: {code_i}")

    def play_stream(self, app_name: str, stream_id: str, pcm: bytes) -> int:
        code, _ = self._client.PlayStream(app_name, stream_id, pcm)
        return int(code)

    def stop(self, app_name: str) -> int:
        return int(self._client.PlayStop(app_name))


class RemoteTTSPlayer:
    """
    Stream TTS PCM from remote `/ws/tts` and play on G1 speaker.

    - playtext(text): starts streaming+playback in background
    - pause(): immediately stops speaker output, but keeps buffering unplayed audio
    - reuse(): resumes speaker output from buffered audio (if not finished)
    """

    def __init__(
        self,
        *,
        ws_tts_url: str = "ws://112.95.75.67:10010/ws/tts",
        unitree_net_iface: Optional[str] = None,
        app_name: str = "ttsplayer",
        volume: int = 100,
        max_buffer_frames: int = 5000,  # about a few seconds depending on server chunking
        save_wav_dir: Optional[str] = None,
        play_tail_s: float = 0.25,
    ) -> None:
        self._ws_tts_url = ws_tts_url
        if unitree_net_iface is None:
            unitree_net_iface = os.environ.get("UNITREE_NET_IFACE", "eth0")
        self._player = UnitreePlayer(unitree_net_iface, volume=volume)
        self._app_name = app_name
        self._log = logging.getLogger(self.__class__.__name__)
        self._save_wav_dir = save_wav_dir
        self._play_tail_s = float(play_tail_s)

        self._q: "queue.Queue[Optional[bytes]]" = queue.Queue(maxsize=max_buffer_frames)
        self._play_allowed = threading.Event()
        self._play_allowed.set()

        self._stop_all = threading.Event()
        self._session_lock = threading.Lock()
        self._active = False
        self._done = threading.Event()
        self._end_received = threading.Event()
        self._recv_finished = threading.Event()

        self._count_lock = threading.Lock()
        self._frames_in = 0
        self._bytes_in = 0
        self._audio_bps = 16000 * 2 * 1  # PCM16LE, 16kHz, mono => 32000 bytes/sec

        self._recv_thread: Optional[threading.Thread] = None
        self._play_thread: Optional[threading.Thread] = None
        self._current_stream_id = uuid.uuid4().hex
        self._session_id = uuid.uuid4().hex[:8]
        self._wav_path: Optional[str] = None

    def playtext(self, text: str) -> None:
        text = str(text).strip()
        if not text:
            return

        with self._session_lock:
            if self._active:
                self.stop()
            self._stop_all.clear()
            self._done.clear()
            self._active = True
            self._current_stream_id = uuid.uuid4().hex
            self._session_id = uuid.uuid4().hex[:8]
            self._end_received.clear()
            self._recv_finished.clear()
            with self._count_lock:
                self._frames_in = 0
                self._bytes_in = 0
            self._wav_path = self._build_wav_path()

            self._log.info(
                "playtext start session=%s chars=%d url=%s app=%s stream_id=%s",
                self._session_id,
                len(text),
                self._ws_tts_url,
                self._app_name,
                self._current_stream_id[:8],
            )
            self._recv_thread = threading.Thread(
                target=self._recv_loop_thread, args=(text,), daemon=True
            )
            self._play_thread = threading.Thread(target=self._play_loop_thread, daemon=True)
            self._recv_thread.start()
            self._play_thread.start()

    def pause(self) -> None:
        """
        Suspend speaker output immediately.
        Incoming audio is still received and buffered.
        """
        self._log.info("pause session=%s", self._session_id)
        self._play_allowed.clear()
        try:
            self._player.stop(self._app_name)
        except Exception:
            # Best-effort; pause must be robust.
            pass

    def reuse(self) -> None:
        """Resume speaker output if there is buffered/unplayed audio."""
        # New stream id after stop makes resume more reliable on some firmware.
        self._current_stream_id = uuid.uuid4().hex
        self._log.info(
            "reuse session=%s new_stream_id=%s qsize=%s",
            self._session_id,
            self._current_stream_id[:8],
            self._safe_qsize(),
        )
        self._play_allowed.set()

    def is_playing(self) -> bool:
        return self._active and (not self._done.is_set())

    def wait_done(self, timeout_s: Optional[float] = None) -> bool:
        """Block until the current playtext finishes (or timeout)."""
        return bool(self._done.wait(timeout=timeout_s))

    def stop(self) -> None:
        """
        Stop everything and clear buffered audio.
        After stop(), reuse() will do nothing until playtext() is called again.
        """
        with self._session_lock:
            if not self._active:
                return
            self._log.info(
                "stop requested session=%s qsize=%s stream_id=%s",
                self._session_id,
                self._safe_qsize(),
                self._current_stream_id[:8],
            )
            self._stop_all.set()
            self._play_allowed.set()
            self._end_received.clear()
            self._recv_finished.set()
            self._drain_queue()
            try:
                self._player.stop(self._app_name)
            except Exception:
                pass

        # Allow threads to exit quickly.
        try:
            self._q.put_nowait(None)
        except queue.Full:
            pass

        if self._recv_thread and self._recv_thread.is_alive():
            self._recv_thread.join(timeout=1.0)
        if self._play_thread and self._play_thread.is_alive():
            self._play_thread.join(timeout=1.0)

        with self._session_lock:
            self._active = False
            self._done.set()
        self._log.info("stop finished session=%s", self._session_id)

    def _drain_queue(self) -> None:
        try:
            while True:
                self._q.get_nowait()
        except queue.Empty:
            return

    def _safe_qsize(self) -> str:
        try:
            return str(self._q.qsize())
        except Exception:
            return "?"

    def _build_wav_path(self) -> Optional[str]:
        if not self._save_wav_dir:
            return None
        try:
            os.makedirs(self._save_wav_dir, exist_ok=True)
        except Exception as e:
            self._log.warning("wav dir create failed dir=%s err=%r", self._save_wav_dir, e)
            return None
        ts = time.strftime("%Y%m%d-%H%M%S", time.localtime())
        fn = f"tts_{ts}_session-{self._session_id}.wav"
        return os.path.join(self._save_wav_dir, fn)

    def _recv_loop_thread(self, text: str) -> None:
        asyncio.run(self._recv_loop_async(text))

    async def _recv_loop_async(self, text: str) -> None:
        import websockets

        request_id = uuid.uuid4().hex[:8]
        bytes_in = 0
        frames_in = 0
        last_progress_log = time.monotonic()
        start_t = time.monotonic()
        wav_writer: Optional[wave.Wave_write] = None
        try:
            self._log.info(
                "ws connect session=%s request_id=%s url=%s",
                self._session_id,
                request_id,
                self._ws_tts_url,
            )
            async with websockets.connect(self._ws_tts_url, max_size=None) as ws:
                payload = {"text": text, "request_id": request_id}
                await ws.send(json.dumps(payload, ensure_ascii=False))
                self._log.info(
                    "ws sent session=%s request_id=%s chars=%d",
                    self._session_id,
                    request_id,
                    len(text),
                )
                if self._wav_path:
                    try:
                        wav_writer = wave.open(self._wav_path, "wb")
                        wav_writer.setnchannels(1)
                        wav_writer.setsampwidth(2)  # PCM16LE
                        wav_writer.setframerate(16000)
                        self._log.info(
                            "wav saving enabled session=%s path=%s",
                            self._session_id,
                            self._wav_path,
                        )
                    except Exception as e:
                        self._log.warning(
                            "wav open failed session=%s path=%s err=%r",
                            self._session_id,
                            self._wav_path,
                            e,
                        )
                        wav_writer = None

                # Expect a JSON "start" then binary frames then JSON "end"
                while not self._stop_all.is_set():
                    msg = await ws.recv()
                    if isinstance(msg, bytes):
                        # Backpressure is OK: block until playback consumes.
                        bytes_in += len(msg)
                        frames_in += 1
                        with self._count_lock:
                            self._bytes_in = bytes_in
                            self._frames_in = frames_in
                        if wav_writer is not None:
                            try:
                                wav_writer.writeframes(msg)
                            except Exception as e:
                                self._log.warning(
                                    "wav write failed session=%s err=%r (disabling wav write)",
                                    self._session_id,
                                    e,
                                )
                                try:
                                    wav_writer.close()
                                except Exception:
                                    pass
                                wav_writer = None
                        self._q.put(msg)
                        now = time.monotonic()
                        if now - last_progress_log >= 1.0:
                            self._log.info(
                                "ws recv session=%s request_id=%s frames=%d bytes=%d qsize=%s",
                                self._session_id,
                                request_id,
                                frames_in,
                                bytes_in,
                                self._safe_qsize(),
                            )
                            last_progress_log = now
                        continue

                    j = json.loads(msg)
                    t = j.get("type")
                    if t == "error":
                        raise RuntimeError(j.get("message") or "remote tts error")
                    if t == "start":
                        self._log.info(
                            "ws start session=%s request_id=%s meta=%s",
                            self._session_id,
                            request_id,
                            {k: v for k, v in j.items() if k != "type"},
                        )
                    if t == "end":
                        self._end_received.set()
                        self._log.info(
                            "ws end session=%s request_id=%s frames=%d bytes=%d elapsed=%.2fs",
                            self._session_id,
                            request_id,
                            frames_in,
                            bytes_in,
                            time.monotonic() - start_t,
                        )
                        break

        except Exception as e:
            # IMPORTANT: don't swallow exceptions; they are the #1 reason playback "mysteriously stops".
            self._log.exception(
                "ws failed session=%s request_id=%s frames=%d bytes=%d elapsed=%.2fs err=%r",
                self._session_id,
                request_id,
                frames_in,
                bytes_in,
                time.monotonic() - start_t,
                e,
            )
        finally:
            if wav_writer is not None:
                try:
                    wav_writer.close()
                    self._log.info(
                        "wav saved session=%s path=%s frames=%d bytes=%d",
                        self._session_id,
                        self._wav_path,
                        frames_in,
                        bytes_in,
                    )
                except Exception as e:
                    self._log.warning(
                        "wav close failed session=%s path=%s err=%r",
                        self._session_id,
                        self._wav_path,
                        e,
                    )
            self._recv_finished.set()
            # Wake playback thread (it decides when it's truly "done").
            try:
                self._q.put_nowait(None)
            except queue.Full:
                # If full, wait a bit then try again.
                try:
                    self._q.put(None, timeout=0.5)
                except Exception:
                    pass

    def _play_loop_thread(self) -> None:
        frames_out = 0
        bytes_out = 0
        consecutive_rc_fail = 0
        last_progress_log = time.monotonic()
        start_t = time.monotonic()
        first_play_t: Optional[float] = None
        finished_naturally = False
        try:
            while True:
                if self._stop_all.is_set():
                    self._log.info("play loop stop_all session=%s", self._session_id)
                    break

                try:
                    frame = self._q.get(timeout=0.1)
                except queue.Empty:
                    frame = None

                if frame is None:
                    # `None` is a wake-up marker OR a timeout. Only finish when:
                    # - server has sent `end`
                    # - recv thread is finished
                    # - everything received has been played
                    if self._end_received.is_set() and self._recv_finished.is_set():
                        with self._count_lock:
                            frames_in = self._frames_in
                            bytes_in = self._bytes_in
                        if frames_out >= frames_in and self._q.empty():
                            # We have submitted all received PCM to the device.
                            # The device may still be playing buffered audio; estimate remaining time
                            # from received PCM duration vs elapsed since first successful PlayStream.
                            if first_play_t is not None and bytes_in > 0:
                                audio_len_s = bytes_in / float(self._audio_bps)
                                elapsed_s = time.monotonic() - first_play_t
                                remaining_s = audio_len_s - elapsed_s
                                if remaining_s > 0:
                                    self._log.info(
                                        "play tail sleep session=%s audio_len=%.3fs elapsed=%.3fs sleep=%.3fs",
                                        self._session_id,
                                        audio_len_s,
                                        elapsed_s,
                                        remaining_s,
                                    )
                                    time.sleep(remaining_s)
                            self._log.info(
                                "play loop done session=%s frames_in=%d frames_out=%d bytes_in=%d bytes_out=%d elapsed=%.2fs",
                                self._session_id,
                                frames_in,
                                frames_out,
                                bytes_in,
                                bytes_out,
                                time.monotonic() - start_t,
                            )
                            finished_naturally = True
                            break
                    continue

                # Pause gates speaker output without losing buffered frames.
                while (not self._play_allowed.is_set()) and (not self._stop_all.is_set()):
                    time.sleep(0.01)
                if self._stop_all.is_set():
                    self._log.info("play loop stop during pause session=%s", self._session_id)
                    break

                rc = self._player.play_stream(self._app_name, self._current_stream_id, frame)
                if rc != 0:
                    # Non-fatal; keep trying next frames.
                    consecutive_rc_fail += 1
                    if consecutive_rc_fail in (1, 5, 20) or (consecutive_rc_fail % 50 == 0):
                        self._log.warning(
                            "PlayStream rc=%d session=%s stream_id=%s fails=%d qsize=%s frame_bytes=%d",
                            rc,
                            self._session_id,
                            self._current_stream_id[:8],
                            consecutive_rc_fail,
                            self._safe_qsize(),
                            len(frame),
                        )
                    time.sleep(0.01)
                    continue

                consecutive_rc_fail = 0
                if first_play_t is None:
                    first_play_t = time.monotonic()
                    self._log.info(
                        "first play accepted session=%s stream_id=%s",
                        self._session_id,
                        self._current_stream_id[:8],
                    )
                frames_out += 1
                bytes_out += len(frame)
                now = time.monotonic()
                if now - last_progress_log >= 1.0:
                    with self._count_lock:
                        frames_in = self._frames_in
                        bytes_in = self._bytes_in
                    self._log.info(
                        "play ok session=%s frames_out=%d/%d bytes_out=%d/%d qsize=%s stream_id=%s",
                        self._session_id,
                        frames_out,
                        frames_in,
                        bytes_out,
                        bytes_in,
                        self._safe_qsize(),
                        self._current_stream_id[:8],
                    )
                    last_progress_log = now
        finally:
            # If we stop the device immediately on natural completion, it can cut off
            # the last buffered audio on the speaker side. Only force-stop on explicit stop().
            if finished_naturally:
                if self._play_tail_s > 0 and first_play_t is not None:
                    # Extra safety margin (optional) for device buffering jitter.
                    time.sleep(self._play_tail_s)
            else:
                try:
                    self._player.stop(self._app_name)
                except Exception:
                    pass
            with self._session_lock:
                self._active = False
                self._done.set()
            self._log.info(
                "play loop finished session=%s natural=%s frames_out=%d bytes_out=%d elapsed=%.2fs",
                self._session_id,
                finished_naturally,
                frames_out,
                bytes_out,
                time.monotonic() - start_t,
            )


if __name__ == "__main__":
    # Minimal manual test:
    #   python3 tts_player.py "hello world"
    import sys

    logging.basicConfig(
        level=getattr(logging, os.environ.get("TTS_LOG_LEVEL", "INFO").upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    text = open("./1.txt").read().splitlines()
    test = text[76:78]
    test = "\n".join(test)
    print(test)
    tts = RemoteTTSPlayer(save_wav_dir="./wavtest")
    tts.playtext(test)
    tts.wait_done()
    print("wait done")
