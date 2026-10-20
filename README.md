# ArmY — Grasp-Pose Selection on a Desktop Robot Arm

**English** | [中文](README.zh-CN.md)

The grasp pipeline of **ArmY**, a capstone team's grocery-shelf picking robot built on a Hiwonder JetArm
with a depth camera on its wrist. The open-source [GPD](https://github.com/atenpas/gpd) library proposes
around a hundred candidate gripper poses per scan; this repository is everything around it that turns that
into one pick on this arm — the **grasp-selection node**, the **point-cloud bridge** that puts a
wrist-mounted camera's cloud into the arm's base frame, and a **PyQt5 operator interface** for
calibration, tuning and candidate selection.

<p align="center">
  <img src="docs/media/army_hero.webp" width="560" alt="The green JetArm reaching down and picking an object off a kitchen counter.">
</p>

> **Project context** — Senior design capstone (MAE 151A/B), UC Irvine, five-person team,
> Oct 2025 – Mar 2026, sponsored by Prof. Solmaz Kia. The mechanical design, motion-planning line and
> electrical work were teammates'. **Mine was the grasp pipeline**: the grasp-selection node, the
> point-cloud bridge, the operator interface, and verification of the perception and grasp requirements.
> GPD itself is upstream code, included unchanged.
> Write-up with videos: [krisxia.com/army](https://krisxia.com/army.html)

## Results

Measured against the team's requirements — including the ones that were not met:

| Requirement | Result | Status |
|---|---|---|
| Target detection, baseline + cluttered sets (distractors, partial occlusion) | **15 / 15** | Met |
| Grasp sequence automation — the operator only clicks the item | detection → selection → pick → extraction, no intervention | Met |
| Grasp position within ± 1.0 cm for 95% of samples | **7 / 10** | **Not met** |
| Pick cycle time < 45 s | 10 – 60 s | **Not met** |

15/15 detection is not 15/15 grasping: the system almost always sees the item and still does not always
know where it is well enough to grasp it cleanly. The limit is the depth camera — a grasp pose is only as
accurate as the surface it was fitted to, and this cloud is ragged at its edges. Shelf-aware obstacle
avoidance never reached a working state; the picks below are in open scenes and on a rack.

<table>
  <tr>
    <td width="33%"><img src="docs/media/army_pick_tool.webp" alt="Picking a folding multitool lying flat on the counter."></td>
    <td width="33%"><img src="docs/media/army_pick_bottle.webp" alt="Picking a tapered drink bottle with no flat faces."></td>
    <td width="33%"><img src="docs/media/army_pick_bench.webp" alt="Picking a red block on a steel lab bench under different lighting."></td>
  </tr>
</table>
<p align="center"><sub>Same pipeline, three objects (2.5× speed). Left: rigid, low-profile — a multitool lying flat · Middle: irregular, tapered — no flat faces anywhere · Right: new room, new bench, different lighting</sub></p>

## How it works

```
wrist RGB-D camera
   │   /rgbd_cam/depth/points  (camera frame — changes with every joint move)
   ▼
pointcloud_to_base          live TF → base_link, downsampled
   │   /rgbd_cam/depth/points_base
   ▼
gpd_once_server             gpd_ros + GPD (unchanged); runs once when the
   │                        operator clicks an item → ranked candidates
   ▼
gpd_grasp_node              reachability box → rank → approach / retreat
   │   /grasp action
   ▼
JetArm servo controller
```

### 1. Choosing one grasp out of a hundred

To grasp an object the arm needs a pose in space — where to put the jaws, from which direction, how far
to open them. GPD samples the point cloud and returns a ranked set of candidate gripper poses: a great
many on a cluttered scene, plenty of them unusable — driving the gripper through the surface the item
rests on, or sitting on the ragged fringe of the cloud. `gpd_grasp_node.py` adds:

- **a reachability box** — candidates outside a conservative box in the arm's base frame are dropped
  before anything else is considered; this removes most of them;
- **ranking** — what survives is ordered by the classifier's own score, with distance to a preferred
  point as the tie-break;
- **execution** — a slanted push-pick approach with configurable attack angle, approach distance and
  retreat, sent to the arm's `/grasp` motion action.

<p align="center">
  <img src="docs/media/gpd_candidates_rviz.jpg" width="620" alt="RViz on the robot's laptop: a white point cloud of the scanned scene with blue candidate gripper poses generated on it, and a terminal showing gpd_grasp subscribed to /detect_grasps/clustered_grasps.">
</p>
<p align="center"><sub>The white surface is the scanned point cloud; the blue jaws are candidate grasps generated on it.</sub></p>

### 2. Making three pieces agree on where things are

GPD is written for a camera on a fixed mount, watching the scene from outside while the arm moves through
it. Here the depth camera is bolted to the wrist, directly above the jaws, so there is no fixed
camera-to-base extrinsic, and the frame the cloud arrives in is different after every joint move.

That breaks an assumption GPD never states. It crops with a plain box of metres — which describes the
space above the table **only if the cloud is already in the arm's base frame**. A wrist-relative cloud
gets a different wrong volume cut out of it after every move, and nothing errors: the arm reaches
confidently for a place that is not there. So `pointcloud_to_base.py` pulls every cloud through the
live TF tree into `base_link` first, downsampling on the way.

Transforming the few returned poses afterwards would be far cheaper than transforming a few hundred
thousand points — but the crop takes effect *before* GPD sees the data, so on an untransformed cloud GPD
generates many candidates outside the box, and those can score high enough to crowd out the ones on the
item. That is what happened in testing.

### 3. An interface an operator can drive

<p align="center">
  <img src="docs/media/army_gui.webp" width="720" alt="The PyQt5 operator interface: live camera view, TF calibration sliders, grasp offset and GPD tuning controls, a scan progress window and a candidate-selection window, while the arm completes a pick.">
</p>
<p align="center"><sub>Detection, candidate selection and the pick, end to end (2× speed).</sub></p>

`tf_calibration_control_ui.py` keeps everything that has to be tuned on the real arm adjustable while it
runs, so an experiment costs a slider instead of a rebuild: hand-eye calibration of the
`link5 → rgbd_cam_link` transform, grasp offsets in `base_link`, GPD tuning (attack angle, approach
distance, edge margin and penalty, candidate-cluster radius), a single-shot scan trigger, a candidate picker that clusters near-duplicate grasps, arm speed presets,
and an English / Chinese toggle.

## Repository layout

```
gpd/                  GPD core library (C++, upstream, unchanged)
gpd_ros/              GPD's ROS wrapper (upstream) + gpd_once.launch
jetarm_ui/
  scripts/
    gpd_grasp_node.py              grasp selection and execution
    pointcloud_to_base.py          wrist cloud → base_link bridge
    tf_calibration_control_ui.py   PyQt5 operator interface
    jetarm_ui_node.py              standalone arm-control UI (exposure, WB)
    tf_calibration_tool.py         standalone hand-eye calibration tool
  launch/             tf_calibration_control_ui.launch = full system
  config/             tf_calibration.yaml, ui_config.yaml
user_config/          per-user overrides that survive redeploys
start_tf_ui.sh        one-shot launcher deployed on the arm
docs/media/           images and clips for this README
```

## Build and run

Runs on the JetArm's onboard computer under **ROS Melodic** with the JetArm SDK packages
(`jetarm_bringup`, `jetarm_peripherals`, `jetarm_driver`, `hiwonder_interfaces`, `color_detection`,
`lab_config`), `astra_camera`, PCL ≥ 1.9, Eigen 3, OpenCV ≥ 3.4 and Python 3 + PyQt5.

```bash
# 1. GPD core library
cd gpd && mkdir build && cd build && cmake .. && make -j$(nproc) && sudo make install

# 2. ROS packages, in the JetArm catkin workspace
cp -r gpd_ros jetarm_ui ~/jetarm/src/
cd ~/jetarm && catkin_make && source devel/setup.bash

# 3. Launch the whole system (camera, detection, bridge, GPD, grasp node, UI)
./start_tf_ui.sh            # or: roslaunch jetarm_ui tf_calibration_control_ui.launch
```

`start_tf_ui.sh` hard-codes the arm's `ROS_IP` / `ROS_MASTER_URI` and workspace path — edit those for
another setup.

| Config file | What it holds |
|---|---|
| `gpd/cfg/ros_eigen_params.cfg` | GPD core: workspace box, number of samples, CNN channels, `weights_file` path |
| `jetarm_ui/config/tf_calibration.yaml` | hand-eye transform, grasp offsets, GPD tuning values written by the UI |
| `jetarm_ui/config/ui_config.yaml` | UI defaults (camera topics, arm poses) |
| `user_config/ui_config.user.yaml` | per-user overrides, not overwritten on redeploy |

<details>
<summary><b>Interfaces (topics, services, TF, actions)</b></summary>

**Consumed**

| Name | Type | Provided by |
|---|---|---|
| `/rgbd_cam/depth/points` | `sensor_msgs/PointCloud2` | camera driver |
| `/rgbd_cam/depth/image_raw` | `sensor_msgs/Image` | camera driver (UI display) |
| `/rgbd_cam/color/camera_info` | `sensor_msgs/CameraInfo` | camera driver (projection) |
| `/color_detection/image_result` | `sensor_msgs/Image` | `color_detection` (UI display) |
| `/object/pixel_coords` | `hiwonder_interfaces/ObjectsInfo` | `color_detection` — detected targets |
| `/color_detection/{enter,start,stop,exit}` | `std_srvs/Empty` | `color_detection` |
| `/rgbd_cam/set_uvc_auto_exposure`, `/rgbd_cam/set_uvc_auto_white_balance` | `std_srvs/SetBool` | `astra_camera` |
| `/rgbd_cam/set_uvc_exposure` | `astra_camera/SetInt32` | `astra_camera` |
| `/lab_config_manager/enter` | `std_srvs/Trigger` | `lab_config` |
| `/grasp` (action) | `hiwonder_interfaces/MoveAction` | JetArm motion controller |
| TF `base_link → link1 … link5` | — | URDF / `robot_state_publisher` |

**Provided**

| Name | Type | Node |
|---|---|---|
| `/rgbd_cam/depth/points_base` | `sensor_msgs/PointCloud2` | `pointcloud_to_base` |
| `/gpd_once_server/detect_grasps` (service) | `gpd_ros/detect_grasps` | `gpd_once_server` |
| `/detect_grasps/clustered_grasps` | `gpd_ros/GraspConfigList` | `gpd_once_server` |
| `/gpd_grasp/trigger`, `/gpd_grasp/clear_cache` (services) | `std_srvs/Trigger` | `gpd_grasp_node` |
| `/controllers/multi_id_pos_dur` | `hiwonder_interfaces/MultiRawIdPosDur` | `gpd_grasp_node` / UI — servo ID, position, duration (IDs 1–5 joints, 10 gripper) |
| `/gpd_grasp/debug_markers`, `/gpd_once_server/plot_grasps` | `visualization_msgs/MarkerArray` | RViz debugging |
| TF `link5 → rgbd_cam_link` | — | static publisher / UI calibration (`tf_calibration.yaml`) |

</details>

<details>
<summary><b>Where my work sits in the team's breakdown</b></summary>

<p align="center">
  <img src="docs/media/team_subsystems.jpg" width="760" alt="The project's structural decomposition into six subsystems: simulation, perception, low-level driver and ROS, kinematics and motion planning, hardware modification and calibration, and visual servoing and integration.">
</p>

The project as the team divided it. My work sat in the perception column and the integration column on
the right.

</details>

## License

GPD (`gpd/`, `gpd_ros/`) is distributed under its own BSD 2-Clause license — see the license files in those
directories. The rest of this repository is shared for learning and research.
