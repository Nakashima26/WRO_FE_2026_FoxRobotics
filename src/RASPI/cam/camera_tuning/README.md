# Camera tuning — Raspberry Pi 5 + Camera v2 with a wide-angle lens

## The problem

Our wide-angle lens adapter has **no infrared-cut filter**. With the stock processing, the image looks reddish, and the edges of the frame show a **magenta ring**, even when the camera looks at a white sheet. On a plain white sheet, the corners measured R/G ≈ 1.31–1.40 and B/G ≈ 1.26–1.35 against ≈ 1.0 at the center.

The ring is dangerous for the robot: magenta is close to the pink parking-lot walls and to the red traffic signs.

White-balance gains alone can't fix it, because they scale the whole image by the same amount. The ring is a **lens shading** problem: the camera's processor corrects the color of each zone of the image with a table (ALSC, Auto Lens Shading Correction), and the factory table is made for the stock lens. For the red channel it even *raises* the corners (×1.33 relative to the center), which is the opposite of what our lens needs.

## The fix (two steps)

1. **A lens shading table for our lens.** We photograph a uniformly lit white sheet, and the official Raspberry Pi Camera Tuning Tool (`rpi-ctt`) computes a new table from those photos. The table goes into a copy of `imx219_noir.json`, the tuning made for cameras without an IR filter. Result: [`imx219_noir_wro_pi5.json`](imx219_noir_wro_pi5.json).
2. **Fixed white-balance gains** measured on the same white sheet: `colour-gains=<1.10,1.51>` (red, blue), with automatic white balance off.

`vision.open_camera()` loads this tuning file through `LIBCAMERA_RPI_TUNING_FILE` and applies the gains, but only on a Pi 5. On a Pi 4 it keeps the v1 configuration.

**Result (2026-10-07, white sheet, ~3700 K light):** R/G and B/G = 1.00 ± 0.01 at the center **and** at all four corners.

## How to redo it (≈10 minutes, for example during practice time at a new venue)

1. Stop anything that uses the camera (`sudo systemctl stop wro-runtime`; the camera can only be opened by one process).
2. Fill the **whole frame** with a white sheet, lit evenly by the venue light: no shadows, no glare, no folds. Out of focus is fine. Check it live with [`pure_pursuit/cam_web.py`](../pure_pursuit/cam_web.py) at `http://<pi>:8080`.
3. Take 5 raw photos under the same light (`python camera_tuning/captura_alsc.py 5` on the Pi; it writes to `~/alsc_cal/`). It measures the color temperature, sets the exposure a bit below automatic so nothing saturates, and saves `alsc_<CT>k_<n>.dng`.
4. Compute the table:
   ```bash
   cp /usr/share/libcamera/ipa/rpi/pisp/imx219_noir.json base_noir.json
   ~/ctt_venv/bin/ctt --alsc-only -i ~/alsc_cal -o out --update base_noir.json
   ```
   The `rpi-ctt` package is installed in `~/ctt_venv` (`pip install rpi-ctt`).
5. Copy `base_noir.json` over `imx219_noir_wro_pi5.json` and run `python simetriza_luminancia.py imx219_noir_wro_pi5.json`. Then open `cam_web.py --tuning <that file>`, put the sheet in the yellow box, press **"Balancear con la hoja blanca"**, and copy the gains into `vision.py`.
6. Re-check the color thresholds on real signs and lines (`calibra_luz.py`). The HSV ranges were tuned on the Pi 4 image.

## Notes

- One light, one table. With automatic white balance off, a single color temperature gives predictable behavior. If the venue light is very different (e.g. fluorescent vs LED), redo the calibration there.
- **Brightness table symmetrized.** The brightness part of the table came out asymmetric (corners ×2.0 at the bottom, ×4.3 at the top): it was also correcting the uneven light on the sheet, which was darker at the top. Lens vignetting is symmetric around the center, so [`simetriza_luminancia.py`](simetriza_luminancia.py) averages the brightness table with its mirror images. That keeps the lens part and cancels the lighting gradient. Result: ×2.93 at every corner, almost the same as the factory table (×2.96). So the lens darkens the edges about as much as the stock lens; what it changes is the **color** of the edges. The color tables are not touched. Run it after step 4 every time.
- A diffuser gives an even flat field without this step: put the sheet flat **on** the lens, pointing at the light.
- Dust on the lens shows up as dark spots in the flat-field photos. Clean the lens before calibrating.
- References: [Raspberry Pi CTT (`rpi-ctt`)](https://pypi.org/project/rpi-ctt/), [Arducam — Lens Shading Calibration](https://docs.arducam.com/Raspberry-Pi-Camera/Native-camera/Lens-Shading/), [OpenFlexure — lens shading correction](https://openflexure.discourse.group/t/lens-shading-correction-for-raspberry-pi-camera/682).
