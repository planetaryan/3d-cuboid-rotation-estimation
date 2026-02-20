# 3D Cuboid Rotation Estimation

> A perception pipeline designed to estimate the kinematic and geometric properties of a rotating cuboid from a ROS 2 depth stream.

---

## 📌 Problem Statement

Given a sequence of `16UC1` depth images containing a tumbling 3D cuboid, this project robustly segments the object from environmental noise, calculates the visible face area and normal angle for each frame, and estimates the global axis of rotation.

### Sample Depth Frame

![Depth Frame Visual Check](visual_check.png)

*Figure: False-colour depth map of a single frame. Purple/dark regions indicate near-zero depth (invalid or background); the bright yellow-orange patch on the left edge is a close reflective surface. The large cuboid face occupies the mid-range (~5–8 m) purple band in the centre-left of the scene.*

---

## 🛠️ Pipeline Architecture

### 1. 3D Projection — Pinhole Model
Converts 2D depth maps into a 3D Euclidean point cloud. Includes a data-driven unit verification step to automatically convert millimetre data to SI metres.

### 2. Spatial ROI Filtering
Applies a strict 3D bounding box to physically exclude background plane interference (floor, walls, and ceiling) and prevent algorithm leakage.

### 3. Planar Segmentation — RANSAC
Robustly fits a plane to the largest visible cuboid face, orienting surface normals toward the camera frame.

### 4. Kinematic Estimation — SVD
Employs Singular Value Decomposition (SVD) on the mean-centred trajectory of surface normals to identify the rotational null-space vector.

---

## 📁 Repository Structure

```text
.
├── depth/
│   ├── depth.db3              # ROS 2 bag file (binary depth data)
│   └── metadata.yaml          # Bag metadata (topics, timestamps, camera info)
├── docs/
│   └── Approach.pdf           # Detailed methodology and mathematical proofs
├── output/
│   ├── results_table.csv      # Per-frame area and normal angle results
│   └── rotation_axis.txt      # Estimated global rotation axis vector
├── src/
│   ├── solution.py            # Main estimation pipeline
│   ├── geometry_utils.py      # Shared 3D geometry helpers
│   ├── check_resolution.py    # Depth image resolution diagnostics
│   ├── inspect_clouds.py      # Interactive 3D point cloud & RANSAC visualiser
│   ├── inspect_2d.py          # 2D depth mask validation tool
│   └── visual_check.png       # Sample depth frame for visual validation
├── .gitignore
├── requirements.txt           # Python dependencies
└── README.md
```

---

## 🚀 Installation & Setup

**1. Clone the repository:**

```bash
git clone https://github.com/YOUR_USERNAME/3d-cuboid-rotation-estimation.git
cd 3d-cuboid-rotation-estimation
```

**2. Install the required dependencies:**

```bash
pip install -r requirements.txt
```

**3. Place your ROS 2 bag files (`depth.db3` and `metadata.yaml`) inside the `depth/` directory.**

---

## 💻 Usage

**Run the main estimation pipeline:**

```bash
python src/solution.py
```

This will generate `results_table.csv` and `rotation_axis.txt` inside the `output/` directory.

**Run the visual debugging tools:**

```bash
# Launches 3D point cloud and RANSAC plane visualiser
python src/inspect_clouds.py

# Launches 2D depth mask validation
python src/inspect_2d.py
```

---

## 📊 Results Summary

| Metric | Value |
|---|---|
| Estimated Rotation Axis | `[0.9998, -0.0041, -0.0149]` |
| Axis Interpretation | Rotation about the camera-frame X-axis |
| Average Visible Face Area | ~1.0 m² |

For a full mathematical breakdown — including manual angle calculations and SVD intuition — please refer to [`docs/Approach.pdf`](docs/Approach.pdf).