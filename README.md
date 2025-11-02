<div align="center">
<picture>
   <img src="https://github.com/user-attachments/assets/e083cd13-de4e-4212-91ec-881500eb5bd8" alt="miniflight logo" width="30%"/>
</picture>

mini_flight is a minimal flight control firmware.

</div>

---

This may not be the best flight stack, but it is a flight stack.

Open source flight controllers are bloated, complex, and nearly impossible to debug or extend. Due to its extreme simplicity, miniflight aims to be the easiest controller to add targets to, with support for both config and simulation.

<div align="center">
  <video src="https://github.com/user-attachments/assets/165a2c2c-b6de-4b1b-a522-d552a0697766" controls autoplay muted loop playsinline style="width: 100%; max-width: 960px; border-radius: 12px; box-shadow: 0 12px 32px rgba(15, 23, 42, 0.4);"></video>
</div>

## Getting Started

1. Create a virtual environment and install deps (Python 3.10+):
   ```bash
   python3 -m venv .venv
   source .venv/bin/activate
   pip install -e .
   ```
   
2. Run the configurator (IMU viewer at `http://127.0.0.1:8002`):
   ```bash
   python config/serve.py
   ```

2. Run the quad simulation (starts the web viewer on `http://127.0.0.1:8001`):
   ```bash
   python sim/serve.py
   ```

Keyboard/PS5 Dualsense controls in the sim:

- W/S or left stick vertical: throttle up/down
- Arrow keys or right stick: pitch/roll
- A/D or left stick horizontal: yaw
- X/Cross or Spacebar: pick/drop
