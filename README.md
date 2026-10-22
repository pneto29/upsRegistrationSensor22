# UPS: Uniaxial Partitioning Strategy for Point Cloud Registration

C++ implementations of the **Uniaxial Partitioning Strategy (UPS)** for rigid 3D point cloud registration, using the [Point Cloud Library (PCL)](https://pointclouds.org/) and point-to-point Iterative Closest Point (ICP).

This repository provides two implementations associated with the configurations investigated in the paper below. The files are renamed versions of the supplied legacy source code; their registration/search logic has **not** been refactored. The source filenames and CMake targets now explicitly indicate the intended configuration.

## Reference

**Souza Neto, P., Soares, J. M., and Thé, G. A. P.** (2022). “Uniaxial Partitioning Strategy for Efficient Point Cloud Registration.” *Sensors*, **22**(8), 2887. https://doi.org/10.3390/s22082887

If you use this software in academic work, please cite the publication:

```bibtex
@article{souzaneto2022uniaxial,
  title   = {Uniaxial Partitioning Strategy for Efficient Point Cloud Registration},
  author  = {Souza Neto, Polycarpo and Soares, Jos{\'e} Marques and Th{\'e}, George Andr{\'e} Pereira},
  journal = {Sensors},
  volume  = {22},
  number  = {8},
  pages   = {2887},
  year    = {2022},
  doi     = {10.3390/s22082887},
  url     = {https://doi.org/10.3390/s22082887}
}
```

## Method overview

UPS aims to find a rigid transformation aligning a **source** cloud to a **target** cloud. It:

1. Computes coordinate-wise variance to prioritize partition axes.
2. Sorts each cloud along candidate axes and divides it into smaller subclouds.
3. Runs ICP on corresponding candidate subclouds.
4. Applies each candidate rigid transform to the **full source** cloud.
5. Scores the result with nearest-neighbor RMSE against the full target cloud.
6. Stops once the best RMSE reaches a threshold derived from a slightly rotated copy of the target cloud (*micromisalignment*), or uses the best result found.

See Sections 4.2.1–4.2.3 and Figure 3 of the paper for the formulation and pipeline.

### Configuration A vs. Configuration B

| Executable | Source file | Partition-axis pairs considered | Description |
| --- | --- | --- | --- |
| `ups_config_a` | `src/ups_config_a.cpp` | Up to 3 × 3 = 9 pairs per subcloud | Source and target axes may **differ**. Axes are ranked separately.|
| `ups_config_b` | `src/ups_config_b.cpp` | Up to 3 same-axis pairs per subcloud | Source and target use the **same** axis (X/X, Y/Y, or Z/Z). |

The paper defines Configuration B by choosing a common axis based on **target** variance. This legacy Configuration B implementation uses a common axis for each ICP trial, but its axis-priority calculation has an inconsistency noted below. Therefore, regard it as a **legacy, configuration-B-style implementation**, not a claim of exact reproduction of all results in the paper.

## Repository structure

```text
.
├── CMakeLists.txt
├── README.md
├── .gitignore
└── src/
    ├── ups_config_a.cpp
    └── ups_config_b.cpp
```

## Requirements

- C++14-capable compiler (e.g. GCC or Clang)
- CMake 3.10 or newer
- PCL with `common`, `io`, `registration`, `visualization`, `search` and `kdtree` components (plus its Eigen/VTK dependencies)
- A graphical display for the PCL visualizer
- Two existing `.pcd` point clouds containing XYZ coordinates

Example installation on Debian/Ubuntu (package availability depends on distribution):

```bash
sudo apt update
sudo apt install build-essential cmake libpcl-dev
```

## Build

From the repository root:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

The generated executables are usually `build/ups_config_a` and `build/ups_config_b` on Linux. PCL must be installed and discoverable by CMake. If required, provide `-DPCL_DIR=/path/to/PCLConfig-directory` when configuring.

## Run

Both programs use the **same nine positional arguments after the executable name**:

```text
ups_config_a <target.pcd> <source.pcd> <theta_1> <theta_2> <theta_3> <min_points_per_slice> <max_points_per_slice> <icp_iterations> <output.txt>
ups_config_b <target.pcd> <source.pcd> <theta_1> <theta_2> <theta_3> <min_points_per_slice> <max_points_per_slice> <icp_iterations> <output.txt>
```

Example with a 2.5-degree micromisalignment angle on each axis (2.5° ≈ **0.0436332 radians**):

```bash
mkdir -p results
./build/ups_config_a data/target.pcd data/source.pcd 0.0436332 0.0436332 0.0436332 1000 2000 30 results/config_a.txt
./build/ups_config_b data/target.pcd data/source.pcd 0.0436332 0.0436332 0.0436332 1000 2000 30 results/config_b.txt
```

Replace `data/target.pcd` and `data/source.pcd` with your own files. The repository does **not** include point cloud datasets.

**Argument details**

| Argument | Meaning |
| --- | --- |
| `target.pcd` | Reference cloud, in PCD/XYZ format |
| `source.pcd` | Moving cloud to align with the reference |
| `theta_1`, `theta_2`, `theta_3` | Rotation magnitudes, **in radians**, for constructing the micromisalignment reference; legacy code applies these sequentially around **Z**, **Y**, and **X**, respectively |
| `min_points_per_slice` | Nominal lower target number of points per subcloud, used to calculate the largest partition count |
| `max_points_per_slice` | Nominal upper target number of points per subcloud, used to calculate the smallest partition count |
| `icp_iterations` | Maximum ICP iterations per candidate |
| `output.txt` | Text report with runtime, tested partition bounds, chosen axis/axes, RMSE, transformation matrix and angle-axis estimate |

**Important:** `theta_1` is applied around Z, `theta_2` around Y, and `theta_3` around X in the original C++ files. The original variable names `theta_x` and `theta_z` are misleading; the mapping above documents **actual behavior**. The source uses `Eigen::AngleAxisf`, which takes radians. A separate `icp_stop` argument is **not** part of the current command line, despite an outdated legacy comment.

The programs display input and aligned point clouds using `PCLVisualizer`. Close the **ALIGNED** visualization window to exit. These programs are not headless by default.

## Output and interpretation

The `output.txt` file contains, among other fields:

- `time`: approximate CPU time recorded by `clock()` (not a wall-clock benchmark)
- `min partitions`, `max partitions`: bounds for the number of subclouds
- `final source axis` / `final target axis` in A, or `final source` in B: chosen axis index (`0` = X, `1` = Y, `2` = Z)
- `rmse`: full-cloud nearest-neighbor registration error (units follow input coordinates)
- `rmse reference`: micromisalignment-derived stopping threshold
- `rt`: 4×4 rigid transformation applied to the source
- `angle` / `axis`: equivalent angle-axis representation

Lower RMSE is typically better, but a nearest-neighbor error by itself is not an exhaustive metric of alignment quality, particularly for partial overlap or symmetric geometry.



## License

The paper is open access under **CC BY 4.0**, but that license alone does not establish a software license for the C++ code. **No software license is included in this archive** because the copyright holder(s) must choose and authorize one. Add an appropriate `LICENSE` file before representing this repository as open-source licensed.

## Acknowledgment

This repository documents code associated with the UPS methodology reported by Souza Neto, Soares, and Thé (2022). Please cite the original article when using or discussing the method.
