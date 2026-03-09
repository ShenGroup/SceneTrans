# spatial-data-sim

This project generates **paired images** for spatial change detection using **NVIDIA Isaac Sim**. It supports controlled scene changes such as `add`, `remove`, and `move`, and exports RGB images, depth maps, instance segmentation, semantic segmentation, and metadata.

## 1. Project Goal

- Generate A/B image pairs from the same scene.
- Create training/evaluation data with controllable object-level changes.
- Export multi-modal outputs for downstream training and analysis.

## 2. Directory Structure

```text
spatial-data-sim/
├── assets/         # USD asset files used by simulation scenes
├── simulation/     # Isaac Sim scripts organized by room/task
│   ├── bedroom1-bed/
│   ├── bedroom1-shelf/
│   ├── childrenroom-desk/
│   ├── kitchen/
│   ├── livingroom/
│   └── studyroom-desk/
└── annotation/     # Notebooks for post-processing and annotation generation
```

### 2.1 Assets (USD Files)

The `assets/` directory stores scene and object assets in USD format (for example, `.usd` and `.usda` files).  
Simulation scripts load these files as the source environment for rendering and for object-level operations such as add/remove/move.

## 3. NVIDIA Isaac Sim Environment Setup

### 3.1 Placeholder

Detailed environment setup instructions are currently documented in Google Drive.  
This section is intentionally left as a placeholder and will be completed here later.

### 3.2 Typical Command

```bash
./python.sh /workspace/test/livingroom_move.py --pair-count 120 --num-changes 3 --origin
```

> Note: The command above is a reference example for running simulation scripts in Docker.

## 4. Common Arguments

Arguments vary slightly by script, but the most common ones are:

- `--pair-count`: Number of image pairs to generate.
- `--num-changes`: Number of objects changed per pair (e.g., moved/added/removed objects).
- `--origin`: Enable original-layout reference mode (exact behavior depends on script; typically controls which frame in A/B keeps the initial layout).
- `--width` / `--height`: Output resolution.
- `--focal-length`: Optional camera focal length.

## 5. Output Layout (Example)

Outputs are typically written to `/workspace/output/<scene_name>/<setting>/`, with one folder per pair (`pair_xxxx`). A pair folder may include:

- `A_rgb.png` / `B_rgb.png`
- `A_depth.npy` / `B_depth.npy` (some scripts also export visualized `A_depth.png` / `B_depth.png`)
- `A_instance_segmentation.png` / `B_instance_segmentation.png`
- `A_semantic_segmentation.png` / `B_semantic_segmentation.png`
- `metadata.json` (changed objects, coordinates, and related metadata)

## 6. Recommended Workflow

1. Start the Isaac Sim Docker environment.
2. Choose a target scene script (for example, one script under `simulation/livingroom/`).
3. Run it with `./python.sh` and set arguments such as `pair-count` and `num-changes`.
4. Validate generated `pair_xxxx` folders and `metadata.json`.
5. Use notebooks under `annotation/` for post-processing and annotation generation.
