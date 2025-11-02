<div align="center">
<picture>
   <img src="https://github.com/user-attachments/assets/e083cd13-de4e-4212-91ec-881500eb5bd8" alt="miniflight logo" width="30%"/>
</picture>

mini_flight is a minimal flight control firmware.

</div>

---

This may not be the best flight stack, but it is a flight stack.

Open source flight controllers are bloated, complex, and nearly impossible to debug or extend. Due to its extreme simplicity, miniflight aims to be the easiest controller to add targets to, with support for both config and simulation.

![Screen Recording 2025-05-19 at 9 16 49 AM (6)](https://github.com/user-attachments/assets/62436609-37f9-44b1-bcea-3ab8a77b1491)

## Getting Started

1. Create a virtual environment and install deps (Python 3.10+):
   ```bash
   python3 -m venv .venv
   source .venv/bin/activate
   pip install -e .
   ```

2. Run the quad simulation (starts the web viewer on `http://127.0.0.1:8001`):
   ```bash
   python target/sim.py
   ```

Keyboard controls in the viewer:

- W/S or left stick vertical: throttle up/down
- Arrow keys or right stick: pitch/roll
- A/D or left stick horizontal: yaw
- X/Cross or Spacebar: pick/drop

## More examples
A quadrotor learning to stabilize itself.

![Screen-Recording-2024-07-26-at-4 11 51 PM](https://github.com/user-attachments/assets/0e245827-e067-4a7f-a535-fe2fb6ce15eb)
