# Cache migration verification — 04.11.2026

Only cache metadata was changed. All sampled points, quaternions, packed reachability masks and completion flags are byte-identical at the .npy payload level. No IK was recalculated.

The package comparator found no reachability-relevant URDF differences. After ignoring named presets and formatting, the only SRDF differences were nine removed disable_collisions pairs. Each pair is proven separated by enclosing spheres for every shoulder rotation, not just sampled angles. Each radius includes a 1 micrometre outward numerical margin. The minimum lower bound on clearance is 64.730 mm.

Verification assumes that the installed base.stl and shoulder.stl match the official Humble files identified below. The exported URDF identifies their paths, but does not include their contents.

The original context and cache ID are preserved in migration provenance. The original generation model fingerprint and parameter record remain unchanged; do not use this migrated complete cache as a resume checkpoint for a new generation.

## Mesh sources

- base.stl: https://raw.githubusercontent.com/UniversalRobots/Universal_Robots_ROS2_Description/humble/meshes/ur5e/collision/base.stl
  SHA-256: `9a145e0d46f2130afdf2a2e8825a00a929870c4c3d6d8e4d1adc5f04db3aac1b`
- shoulder.stl: https://raw.githubusercontent.com/UniversalRobots/Universal_Robots_ROS2_Description/humble/meshes/ur5e/collision/shoulder.stl
  SHA-256: `ceb92532177daa77682f5fbd628e01c2137d168f949a7a706ce1dabe9f002387`

## Pair clearance bounds

| Link 1 | Link 2 | Lower bound (mm) |
|---|---|---:|
| left_base_link_inertia | right_base_link_inertia | 104.572 |
| left_base_link_inertia | right_plate | 84.578 |
| left_base_link_inertia | right_shoulder_link | 255.819 |
| left_plate | right_base_link_inertia | 84.877 |
| left_plate | right_plate | 64.730 |
| left_plate | right_shoulder_link | 238.077 |
| left_shoulder_link | right_base_link_inertia | 252.384 |
| left_shoulder_link | right_plate | 234.390 |
| left_shoulder_link | right_shoulder_link | 374.712 |

## Installation

Stop the selector. Back up the installed config/dexterous_volume.npz, then replace only that cache file with the one in this archive. Restart the selector. No rebuild is needed when volume_file points to the source config path. This migration does not change runtime MoveIt collision rules. Robot runtime testing was not performed here.
