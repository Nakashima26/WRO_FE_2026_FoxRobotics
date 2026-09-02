Control software
====

This directory contains the competition software used by the vehicle. See the
[top-level README](../README.md) (§4) for the full architecture.

Two controllers:

- **Raspberry Pi** — vision + Pure Pursuit path planning.
  Current code: [`RASPI/cam/pure_pursuit/`](RASPI/cam/pure_pursuit/),
  entry point [`runtime_nuevo.py`](RASPI/cam/pure_pursuit/runtime_nuevo.py).
  Legacy Open-only runtime: [`RASPI/cam/wro_runtime.py`](RASPI/cam/wro_runtime.py).
- **ESP32** — real-time FSM, cascade PID, actuators.
  Current firmware: [`ESP32/PurePursuit/PurePursuit.ino`](ESP32/PurePursuit/PurePursuit.ino).
  Legacy firmware: [`ESP32/Controller_PI/Controller_PI.ino`](ESP32/Controller_PI/Controller_PI.ino).

## Run commands

### Raspberry Pi

Start the runtime (from `src/RASPI/cam/`):

```bash
cd ~/FoxRobotics
source .venv/bin/activate
cd src/RASPI/cam
python -m pure_pursuit.runtime_nuevo
```

Headless (competition) with track-edge recording:

```bash
python -m pure_pursuit.runtime_nuevo --no-window --serial-port /dev/serial0 \
    --record-orillas --record-output ~/FoxRobotics/videos_orillas
```

Calibration and vision-only testing: see
[`RASPI/cam/pure_pursuit/INSTRUCCIONES.md`](RASPI/cam/pure_pursuit/INSTRUCCIONES.md).

### ESP32

Flash once, then it runs on boot. For the Open Challenge set
`const bool rondaObstaculos = false;` near the top of the sketch first.

```powershell
arduino-cli core update-index
arduino-cli core install esp32:esp32
arduino-cli lib install "MPU6050_tockn"
arduino-cli compile --fqbn esp32:esp32:esp32 src/ESP32/PurePursuit/PurePursuit.ino
arduino-cli upload -p COM5 --fqbn esp32:esp32:esp32 src/ESP32/PurePursuit/PurePursuit.ino
```

Replace `COM5` with your ESP32 port.

### Autostart on Raspberry Pi

```bash
chmod +x /home/user/FoxRobotics/scripts/install_autostart_pi.sh
/home/user/FoxRobotics/scripts/install_autostart_pi.sh
```

Installs `deploy/systemd/wro-runtime.service`, which runs
`pure_pursuit/runtime_nuevo.py` on boot.

All artifacts required to resolve dependencies and build the project must be
included in this directory as well.
