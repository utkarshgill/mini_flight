"""Lightweight HTTP server and renderer integration for mini_flight."""

from __future__ import annotations

import os
import copy
import json
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, Iterable, List
from urllib.parse import urlparse


class SharedState:
    """Thread-safe container for simulation snapshots and input state."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._snapshot: Dict[str, Any] = {
            "world_time": 0.0,
            "frame": 0,
            "bodies": [],
            "updated_at": time.time(),
        }
        self._input: Dict[str, bool] = {}

    def set_snapshot(self, snapshot: Dict[str, Any]) -> None:
        with self._lock:
            snapshot = copy.deepcopy(snapshot)
            snapshot.setdefault("updated_at", time.time())
            self._snapshot = snapshot

    def get_snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return copy.deepcopy(self._snapshot)

    def set_input_keys(self, keys: Iterable[str]) -> None:
        normalized = {str(k): True for k in keys}
        with self._lock:
            self._input = normalized

    def get_input_state(self) -> Dict[str, bool]:
        with self._lock:
            return dict(self._input)


def _infer_mime_type(path: Path) -> str:
    if path.suffix == ".js":
        return "application/javascript"
    if path.suffix in {".css", ".wasm"}:
        return "text/css" if path.suffix == ".css" else "application/wasm"
    if path.suffix in {".obj", ".mtl"}:
        return "text/plain"
    if path.suffix in {".json"}:
        return "application/json"
    return "text/html"


def _make_handler(shared_state: SharedState, static_dir: Path):
    class RendererRequestHandler(BaseHTTPRequestHandler):
        server_version = "MiniFlightRenderer/1.0"

        def do_GET(self) -> None:  # noqa: N802 (method name from BaseHTTPRequestHandler)
            parsed = urlparse(self.path)
            route = parsed.path

            if route == "/state":
                snapshot = shared_state.get_snapshot()
                payload = json.dumps(snapshot).encode("utf-8")
                self._send_response(200, "application/json", payload)
                return

            if route in {"/", "/index.html"}:
                target = static_dir / "index.html"
                self._send_static(target)
                return

            if route.startswith("/js/"):
                target = (static_dir / route.lstrip("/")).resolve()
                if not str(target).startswith(str(static_dir.resolve())):
                    self.send_error(403)
                    return
                self._send_static(target)
                return

            if route.startswith("/data/"):
                target = (static_dir / route.lstrip("/")).resolve()
                if not str(target).startswith(str(static_dir.resolve())):
                    self.send_error(403)
                    return
                self._send_static(target)
                return

            self.send_error(404)

        def do_POST(self) -> None:  # noqa: N802
            parsed = urlparse(self.path)
            if parsed.path != "/input":
                self.send_error(404)
                return

            content_length = int(self.headers.get("Content-Length", "0"))
            raw = self.rfile.read(content_length) if content_length else b"{}"
            try:
                data = json.loads(raw.decode("utf-8"))
            except json.JSONDecodeError:
                self.send_error(400, "invalid json")
                return

            keys = data.get("keys", [])
            if not isinstance(keys, list):
                self.send_error(400, "keys must be a list")
                return

            shared_state.set_input_keys(keys)
            self._send_response(204, "application/json", b"")

        def _send_static(self, path: Path) -> None:
            try:
                data = path.read_bytes()
            except FileNotFoundError:
                self.send_error(404)
                return
            mime_type = _infer_mime_type(path)
            self._send_response(200, mime_type, data)

        def _send_response(self, status: int, content_type: str, payload: bytes) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
            self.end_headers()
            if payload:
                self.wfile.write(payload)

        def log_message(self, fmt: str, *args: Any) -> None:  # noqa: A003 (fmt name)
            # Silence default logging to keep simulator output clean.
            return

    return RendererRequestHandler


class RendererServer:
    """Utility wrapper for running the renderer HTTP server in a thread."""

    def __init__(self, shared_state: SharedState, host: str, port: int, static_dir: Path) -> None:
        handler_cls = _make_handler(shared_state, static_dir)
        self._httpd = ThreadingHTTPServer((host, port), handler_cls)
        self._httpd.daemon_threads = True
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def shutdown(self) -> None:
        self._httpd.shutdown()
        self._httpd.server_close()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)


def _open_browser(url: str, delay: float = 0.75) -> None:
    time.sleep(delay)
    try:
        webbrowser.open(url, new=1, autoraise=True)
    except Exception:
        pass


def _quat_wxyz_to_xyzw(quat: List[float]) -> List[float]:
    if len(quat) != 4:
        return [0.0, 0.0, 0.0, 1.0]
    w, x, y, z = [float(c) for c in quat]
    return [x, y, z, w]


def _classify_body(body) -> str:
    urdf = getattr(body, "urdf_filename", None)
    if urdf == "quadrotor.urdf":
        return "quadrotor"
    if urdf == "cube.urdf":
        return "box"
    return getattr(body, "visual_kind", "body")


class BrowserRenderer:
    """Render the simulation by streaming state to a Three.js frontend."""

    def __init__(
        self,
        world,
        config: str | None = "X",
        gui: bool = True,
        *,
        host: str | None = None,
        port: int | None = None,
        open_browser: bool | None = None,
    ) -> None:
        del gui  # retained for backwards compatibility
        self.world = world
        self.config = config or "X"
        self.frame_index = 0

        self._shared_state = SharedState()
        static_dir = Path(__file__).parent

        self.host = host or os.getenv("MINIFLIGHT_RENDER_HOST", "127.0.0.1")
        default_port = int(os.getenv("MINIFLIGHT_RENDER_PORT", "8001"))
        self.port = port or default_port

        self._server = RendererServer(self._shared_state, self.host, self.port, static_dir)
        try:
            self._server.start()
        except OSError as exc:  # pragma: no cover
            raise RuntimeError(
                f"Failed to start renderer server on {self.host}:{self.port}: {exc}"
            ) from exc

        self.url = f"http://{self.host}:{self.port}".replace("0.0.0.0", "127.0.0.1")
        print(f"[renderer] serving browser UI at {self.url}")

        should_open = open_browser
        if should_open is None:
            should_open = os.getenv("MINIFLIGHT_RENDER_OPEN_BROWSER", "1") != "0"
        if should_open:
            threading.Thread(target=_open_browser, args=(self.url,), daemon=True).start()

    def _snapshot_bodies(self) -> List[Dict[str, Any]]:
        bodies: List[Dict[str, Any]] = []
        for idx, body in enumerate(self.world.bodies):
            position = [float(v) for v in getattr(body.position, "v", [0.0, 0.0, 0.0])]
            orientation = _quat_wxyz_to_xyzw(getattr(body.orientation, "q", [1.0, 0.0, 0.0, 0.0]))
            lin_vel = [float(v) for v in getattr(body.velocity, "v", [0.0, 0.0, 0.0])]
            ang_vel = [float(v) for v in getattr(body.angular_velocity, "v", [0.0, 0.0, 0.0])]

            body_payload: Dict[str, Any] = {
                "id": idx,
                "label": getattr(body, "name", type(body).__name__),
                "kind": _classify_body(body),
                "position": position,
                "orientation": orientation,
                "linear_velocity": lin_vel,
                "angular_velocity": ang_vel,
            }

            if hasattr(body, "arm_length"):
                body_payload["arm_length"] = float(getattr(body, "arm_length"))
            if hasattr(body, "half_height"):
                body_payload["half_height"] = float(getattr(body, "half_height"))
            if hasattr(body, "urdf_filename"):
                body_payload["asset"] = getattr(body, "urdf_filename")

            bodies.append(body_payload)
        return bodies

    def draw(self) -> None:
        snapshot = {
            "world_time": float(getattr(self.world, "time", 0.0)),
            "frame": self.frame_index,
            "bodies": self._snapshot_bodies(),
        }
        self.frame_index += 1
        self._shared_state.set_snapshot(snapshot)

    def get_input_state(self) -> Dict[str, bool]:
        return self._shared_state.get_input_state()

    def run(self, frames: int) -> None:
        for _ in range(frames):
            self.world.update()
            self.draw()

    def shutdown(self) -> None:
        self._server.shutdown()


# Default renderer alias used by external modules
Renderer = BrowserRenderer


