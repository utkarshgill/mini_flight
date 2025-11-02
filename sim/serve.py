"""Lightweight HTTP server and renderer integration for mini_flight."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Dict, List

from common.serve import RendererServer, SharedState, maybe_launch_browser

__all__ = ["BrowserRenderer", "Renderer", "SharedState", "RendererServer", "maybe_launch_browser"]


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

        maybe_launch_browser(self.url, open_browser=open_browser)

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


def main() -> None:
    """Run the firmware control loop with the simulation renderer."""
    os.environ.setdefault("TARGET", "sim")
    from miniflight.main import main as firmware_main

    firmware_main()


if __name__ == "__main__":
    main()

