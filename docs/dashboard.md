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
