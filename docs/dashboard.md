# Dashboard del carro

A web page served by the Pi itself (`pure_pursuit/dashboard/server.py`, port **8080**). It only uses the Python standard library plus OpenCV, and has no CDNs, so it works without Internet.

## How to connect

| Where | How | URL |
|---|---|---|
| Home / lab | The usual Wi-Fi or Tailscale | `http://<ip-de-la-pi>:8080` |
| At the venue (no known Wi-Fi) | The Pi raises its own **FoxCar** hotspot | `http://10.42.0.1:8080` |
| During the round (radios off, rule 11.10) | **Ethernet** cable from the laptop to the Pi | `http://10.43.0.1:8080` |

- The hotspot has low priority. If the Pi can see a known Wi-Fi network it joins that network instead, so the hotspot doesn't start.
- To force the hotspot: `sudo nmcli con up fox-ap`.
  - Doing this drops any SSH session that came in over Wi-Fi.
- Installation, done once per Pi:

  ```
  bash scripts/instalar_dashboard_pi.sh [clave]
  ```

  It installs `wro-dashboard.service`, the sudoers rules and the `fox-ap` and `fox-eth` profiles.

## From your PC (flash the ESP32 over USB)

Double-click `scripts/dashboard_pc.bat` (or run `python scripts/fox_pc.py`). It opens `http://localhost:8091`.

- It finds the Pi by itself (cable 10.43.0.1, hotspot 10.42.0.1, Tailscale) or takes `--pi IP`.
- The page comes from the PC's repo; all data (`/api/*`) comes from the Pi.
- The **ESP** tab flashes with arduino-cli, without opening the Arduino IDE:
  1. Connect the ESP32 to the PC by USB. The port with a CP210x/CH340 chip is picked automatically.
  2. Press **Flashear Open**, **Obstáculos** or **Obstáculos + sorpresa**. It uses the code in the PC's repo, as it is.
     - If the sketch changed since its last compile: compile (~40 s) + upload (~10 s).
     - If it didn't change (for example, switching between open and obstacles): it only uploads the saved binary (~12 s).
     - The binaries are kept in `%TEMP%\fox_build\<open|obs>`.
     - **compilar de nuevo** forces a compile, for example after updating a library outside the sketch.
  3. When it finishes it restarts `wro-runtime` on the Pi. With **Obstáculos** it also sets `SURPRISE_RULE` in the Pi's `carrera.json`.
- The header shows what the car has: `ESP: Open <build>` or `ESP: Obstáculos <build> · sorpresa ON/OFF`.
  - The build is the compile date and time.
  - The surprise rule shown is the one the runtime loaded.

## Tabs

- **En vivo**
  - Map showing the localizer pose, uncertainty, planned route, trail, confirmed pillars and the parking box.
  - **ToF rays**: yellow is what each sensor measures. The circle is what the map expects. A red circle means they disagree by more than 6 cm.
  - Radar seen from the car (raw and corrected values).
  - Runtime HUD.
  - Sensor readings with history.
  - Pre-run checklist.
- **Calibrar ToF**
  1. Service running, car **not started**.
  2. Place the car parallel to a wall.
  3. Type the real distance and press **Capturar**. It takes the median of about 2 s of raw readings. Capture 3 or more distances.
  4. Press **Enviar L/R al ESP**. The ESP validates the table and stores it in NVS (`CALT`), so it survives a restart.
  5. **Volver a la de fábrica** restores the table built into the firmware.
- **Calibrar color**
  1. Press **Tomar foto**. The photo already has the floor correction applied.
  2. Drag boxes over the floor, the red pillars and the green pillars.
  3. Press **Analizar** to get the proposed bands.
  4. Press **Vista previa** to compare the current mask against the proposed one, region by region.
  5. Press **Guardar**, then restart the service.
- **ESP**
  - Which firmware the car has, when it was compiled, and the surprise rule.
  - **Sorpresa ON/OFF** buttons. They edit `carrera.json` on the Pi and restart the service.
  - Flashing, only when opened from `fox_pc.py`.
- **Corridas** (replay of a recorded run)
  - While the `.avi` records, the runtime writes `orillasN_tele.jsonl` next to it (`run_log.py`): the same state the live view gets (~12/s), plus the important runtime prints and the ESP's log lines.
  - Runs recorded before this file existed are rebuilt from the service journal the first time you open them (ESP ACKs, `[LOCUPD]` pose, `[ROUTE]` box and pillars). That pose is approximate and there is no planned route. The result is cached as `_tele.jsonl` with `"src": "journal"`.
  - **ver** loads the run. The bottom bar plays, pauses, jumps ±5 s and changes speed. Space plays/pauses; ←/→ step one state (Shift: 10).
  - While the replay is on, every tab (En vivo included) draws the recorded state. **salir del replay** goes back to live.
  - The tab shows the map with the trail, the recorded HUD frame for that moment, a summary (duration, corners, distance, ToF minimum and % lost, UART errors, park), a time chart (click to seek; ESP states as background bands, PARK/plan marks), and the event list (click to seek).
- **En vivo** with the open firmware
  - The map draws the island with each side at the width measured in lap 1. 60 cm sides are orange; dashed sides were not measured.
  - Section 1 is drawn at the bottom.
  - The car is drawn at an estimated pose on each straight: inner ToF to the island, front ToF to the end wall. Turns are not estimated.
- **Sistema**
  - Pi state: temperature, low voltage, disk, IPs and radios.
  - **Modo ronda** turns off Wi-Fi and Bluetooth.
  - Run downloads.
  - Filtered log.
  - The saved calibration.

## Files

- `~/wro_calib.json` holds the venue calibration (colors, blue tape, camera gains, ToF records). It keeps a history of 10 versions for **Deshacer**.
  - `calib_file.apply()` loads it when the runtime starts.
  - With no file, the runtime uses the repo values.
- The runtime sends telemetry over UDP to 127.0.0.1:5010 at 12 Hz and receives commands on 5011.
  - Commands are `snap` and `esp` (`CALQ`/`CALT` only, and only before the start button).
  - Turn it off with `DASHBOARD_TELEMETRY = False`.
