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
├── output/         # Generated data (pair folders); tracked with placeholder file
├── simulation/     # Isaac Sim scripts organized by room/task
│   ├── bedroom1-bed/
│   ├── bedroom1-shelf/
│   ├── childrenroom-desk/
│   ├── kitchen/
│   ├── livingroom/
│   ├── livingroom-shelf/
│   └── studyroom-desk/
└── annotation/     # Notebooks for post-processing and annotation generation
    ├── gen_add.ipynb
    ├── gen_move.ipynb
    └── gen_remove.ipynb
```

### 2.1 Assets (USD Files)

The `assets/` directory stores scene and object assets in USD format (for example, `.usd` and `.usda` files).  
Simulation scripts load these files as the source environment for rendering and for object-level operations such as add/remove/move.

### 2.2 Output and Placeholders

- `assets/.gitkeep` and `output/.gitkeep` are placeholders so these directories are preserved in Git even when empty.
- Generated samples are typically written under `output/<scene_name>/<setting>/pair_xxxx/`.

## 3. NVIDIA Isaac Sim Environment Setup

### 3.1 Install Docker and Required Toolkit

First install Docker and the required NVIDIA container toolkit by following the official Isaac Sim container guide:  
https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_container.html

> Important: In Step 4 of the official guide, keep the cache-folder access configuration and file ID permissions as documented. Do not modify those permission settings arbitrarily.

### 3.2 Launch Isaac Sim Docker with Host Mounts

You can mount host directories so they are accessible inside the container.  
Example command:

```bash
sudo docker run --name isaac-sim --entrypoint bash -it --gpus all -e "ACCEPT_EULA=Y" --rm --network=host \
    -e "PRIVACY_CONSENT=Y" \
    -v ~/docker/isaac-sim/cache/main:/isaac-sim/.cache:rw \
    -v ~/docker/isaac-sim/cache/computecache:/isaac-sim/.nv/ComputeCache:rw \
    -v ~/docker/isaac-sim/logs:/isaac-sim/.nvidia-omniverse/logs:rw \
    -v ~/docker/isaac-sim/config:/isaac-sim/.nvidia-omniverse/config:rw \
    -v ~/docker/isaac-sim/data:/isaac-sim/.local/share/ov/data:rw \
    -v ~/docker/isaac-sim/pkg:/isaac-sim/.local/share/ov/pkg:rw \
    -v YOUR_PROJECT_ROOT:/workspace:rw \
    nvcr.io/nvidia/isaac-sim:5.1.0
```

All `-v` entries above are mount parameters.

During runtime, you may hit file permission issues because the default user ID inside the container is often `1234`, which can conflict with host-side permissions.  
If needed, make the mounted path writable for all users:

```bash
sudo chmod -R 777 <your_path>
```

### 3.3 Start the Simulation App (Headless)

The official recommendation for Docker is running Isaac Sim in headless mode:

```bash
./runheadless.sh -v
```

### 3.4 Connect to Remote GUI via WebRTC

Use the Isaac Sim WebRTC Streaming Client and connect with the server IP:  
https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/manual_livestream_clients.html#isaac-sim-setup-livestream-webrtc

### 3.5 Run Python Scripts (Two Methods)

1. **Run from container CLI** (use the Python tool bundled in the container):

   ```bash
   ./python.sh simulation/livingroom/livingroom_remove.py --pair-count 20
   ```

   If running via command line, your Python script should include:

   ```python
   simulation_app = SimulationApp(launch_config={"headless": True})
   ```

   This is equivalent to starting simulation in headless mode first (as in Section 3.3) before importing related modules.

2. **Run from GUI Script Editor** in Isaac Sim.

### 3.6 Script Entry Pattern

In this project, scripts are typically launched from inside the container with:

```bash
./python.sh simulation/<scene-folder>/<script>.py [args...]
```

## 4. Detailed Example: `simulation/livingroom`

This folder includes:

- `livingroom_add.py`
- `livingroom_remove.py`
- `livingroom_move.py`

All three scripts follow the same workflow (load USD -> randomize changes -> render paired outputs), but they differ in which objects are edited and how the A/B pair is constructed.

### 4.1 How to Run (Inside Container)

```bash
cd /workspace/spatial-data-sim
```

Run `remove`:

```bash
./python.sh simulation/livingroom/livingroom_remove.py \
  --pair-count 20 \
  --warmup-k 3 \
  --num-changes 2 \
  --width 1024 \
  --height 768 \
  --output-dir /workspace/output/livingroom_remove/demo_run
```

Run `add`:

```bash
./python.sh simulation/livingroom/livingroom_add.py \
  --pair-count 20 \
  --warmup-k 3 \
  --num-changes 2 \
  --origin \
  --output-dir /workspace/output/livingroom_add/demo_run
```

Run `move`:

```bash
./python.sh simulation/livingroom/livingroom_move.py \
  --pair-count 20 \
  --warmup-k 3 \
  --num-changes 2 \
  --origin \
  --output-dir /workspace/output/livingroom_move/demo_run
```

### 4.2 Parameters (Livingroom Scripts)

The three `livingroom_*.py` scripts support the following main arguments:

- `--pair-count`  
  Number of A/B pairs to generate.
- `--warmup-k`  
  Warmup multiplier before capture (`warmup_frames = 3 * k`).
- `--num-changes`  
  Number of objects changed per pair.
- `--origin`  
  Reset the scene to the initial layout before constructing each pair.  
  When enabled, Frame A of every pair is the same initial state (A frames are consistent across pairs).  
  When disabled, pairs are generated from the script's normal continuous/randomized flow.
- `--semantic-segmentation`  
  Enable semantic segmentation output when the script supports that writer setting.
- `--width`, `--height`  
  Output image resolution.
- `--focal-length`  
  Optional camera focal length override.
- `--output-dir`  
  Output root directory for generated `pair_xxxx` folders.

### 4.3 Default Output Paths (If `--output-dir` Is Not Provided)

- `livingroom_remove.py` -> `/workspace/output/livingroom_remove/3_items`
- `livingroom_add.py` -> `/workspace/output/livingroom_add/3_items`
- `livingroom_move.py` -> `/workspace/output/livingroom_move_origin/3_items`

## 5. Output Layout (Example)

Each run usually creates one folder per pair (`pair_xxxx`) under the selected output directory.  
A pair folder typically includes:

- `A_rgb.png` / `B_rgb.png`
- `A_depth.npy` / `B_depth.npy` (some scripts also export `A_depth.png` / `B_depth.png`)
- `A_instance_segmentation.png` / `B_instance_segmentation.png`
- `A_semantic_segmentation.png` / `B_semantic_segmentation.png` (if enabled)
- `metadata.json` (changed objects, coordinates, and other metadata)

## 6. Cross-Scene Notes

Different scene folders may have slight argument differences (for example, some scripts add `--run-count`, and some do not use `--origin`), but the overall pattern is very similar.  
For scripts that support `--origin`, the common behavior is: reset to initial state before each pair so all A frames stay aligned.

1. Choose a script under `simulation/<scene>/`.
2. Run it with `./python.sh ...`.
3. Tune `pair-count`, `num-changes`, resolution, and `output-dir`.
4. Validate generated `pair_xxxx` folders and `metadata.json`.
5. Use notebooks under `annotation/` for post-processing and annotation generation.

For exact details, always check each script's CLI arguments in its `argparse` section.

## 7. Building `unchanged` Data from Existing Results

You can construct `unchanged` pairs directly from already generated `add`, `move`, or `remove` data without rerunning simulation:

1. Pick any generated pair folder (for example `pair_0007`).
2. Select one side (`A` or `B`) as the base image.
3. Copy that same side to both outputs, for example:
   - `unchanged_A_rgb.png` <- `A_rgb.png`
   - `unchanged_B_rgb.png` <- `A_rgb.png`
4. Apply the same rule to other modalities if needed (`depth`, instance/semantic segmentation).

This creates a valid no-change sample because both sides in the pair come from the same rendered frame.  
In practice, different scenes may have small differences in file naming or optional outputs, but the construction logic is the same.

## 8. Annotation Workflow (`annotation/`)

The notebooks under `annotation/` are designed to run **after** pair generation is complete.

- `gen_add.ipynb`
- `gen_move.ipynb`
- `gen_remove.ipynb`

### 8.1 Input

Provide a path that contains generated pair folders (for example, a directory with `pair_0000`, `pair_0001`, ...).

### 8.2 What the Notebook Does

For the given input path, the notebook iterates through all pair folders and generates annotations for each pair.

### 8.3 Output

The annotation result for each pair is saved as a `result.json` file under the same pair folder.

### 8.4 Difference Between `add` / `move` / `remove`

The overall processing pipeline is similar across `gen_add.ipynb`, `gen_move.ipynb`, and `gen_remove.ipynb`.  
The main difference is the prompt/template logic used for each task type.
