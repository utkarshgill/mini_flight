"""
Rate keeping utilities inspired by openpilot's common.realtime.
Provides a monotonic wall-clock rate keeper suitable for firmware and simulator loops.
"""

from __future__ import annotations

import time


def monotonic_time() -> float:
    """Return monotonic time in seconds."""
    return time.monotonic()


class RateKeeper:
    """
    Maintain a fixed loop rate. Mirrors openpilot's Ratekeeper interface:
    - monitor_time(): update timing statistics, return remaining time (negative if late)
    - keep_time(): call monitor_time() then sleep for remaining time, if positive
    """

    def __init__(self, rate_hz: float, clock=monotonic_time, print_delay_threshold: float | None = 0.01):
        if rate_hz <= 0.0:
            raise ValueError("rate_hz must be positive")
        self.period = 1.0 / rate_hz
        self.clock = clock
        self.print_delay_threshold = print_delay_threshold
        self.frame = 0
        self._last = self.clock()
        self._next = self._last + self.period

    def monitor_time(self) -> float:
        """
        Update timing statistics and compute remaining time before the next frame.
        Returns remaining seconds (negative if the loop is lagging).
        """
        now = self.clock()
        remaining = self._next - now
        if self.print_delay_threshold is not None and remaining < -self.print_delay_threshold:
            print(f"[RateKeeper] Lagging by {-remaining * 1000:.2f} ms (frame {self.frame})")

        self._last = now
        self._next += self.period
        self.frame += 1
        return remaining

    def keep_time(self) -> None:
        """
        Call monitor_time() and sleep for the remaining time, if positive.
        """
        remaining = self.monitor_time()
        if remaining > 0.0:
            time.sleep(remaining)

