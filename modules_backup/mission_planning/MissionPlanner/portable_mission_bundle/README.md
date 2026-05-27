# Portable UAV Mission Bundle

This folder is a self-contained inference bundle you can move on its own.

It provides one focused flow:

1. load or upload a terrain GeoTIFF
2. draw an ROI on the terrain preview
3. convert that ROI into a hex occupancy grid
4. pick one start cell and one or more goal cells
5. send the mission to the bundled PPO model
6. get the result immediately as JSON plus an on-screen path

It intentionally excludes training, dashboards, and the larger simulator UI.

## Folder Layout

- `app.py`: Flask entrypoint
- `portable_mission/`: minimal server modules and frontend assets
- `models/latest_model.zip`: copied latest inference model
- `models/model_config.json`: copied config used to derive defaults
- `data/inputs/`: drop GeoTIFF files here
- `data/work/`: runtime ROI crops

## Quick Start

1. Open a terminal in this folder.
2. Install dependencies:

```bash
python -m pip install -r requirements.txt
```

3. Put a terrain file in `data/inputs/` or use the upload button in the UI.
4. Start the app:

```bash
python app.py
```

Or on Windows:

```bat
run_portable.bat
```

5. Open:

```text
http://127.0.0.1:8877
```

## Expected Terrain Input

- GeoTIFF only: `.tif` or `.tiff`
- A georeferenced raster is preferred
- Both geographic and projected CRS are accepted
- The bundle crops the selected ROI into a temporary GeoTIFF under `data/work/`

## Mission Flow

1. Choose a DEM and load the preview.
2. Drag a rectangle on the preview to define the ROI.
3. Set `altitude`, `hex_step`, `max_steps`, and `max_goals`.
4. Click `Build Hex Grid`.
5. In `Pick Start` mode, click one safe hex.
6. In `Pick Goal` mode, click one or more safe hexes.
7. Click `Run Mission`.

The result panel returns:

- mission summary
- returned cell path
- world-coordinate path
- dense world-coordinate path
- per-step log

## Portability Notes

- This folder does not import the parent project at runtime.
- The only model dependency inside the folder is `models/latest_model.zip`.
- You can carry this folder to another machine as long as Python and the packages in `requirements.txt` are installed.

## Runtime Notes

- The PPO model is loaded lazily on the first mission run.
- The default inference device is `cpu`.
- Override host or port with environment variables:

```bash
set MISSION_APP_HOST=0.0.0.0
set MISSION_APP_PORT=9000
python app.py
```

- Override model device if needed:

```bash
set MISSION_MODEL_DEVICE=cpu
python app.py
```

## What Was Kept

- DEM loading and ROI cropping
- hex occupancy generation
- the mission environment used for PPO inference
- start/goal mission execution
- portable web UI

## What Was Removed

- training services
- evaluation dashboards
- 3D validation viewer
- network activation tracing
- unrelated repository tooling

See `MODEL_README.md` for the bundled model note.
