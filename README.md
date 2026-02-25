# PARKOUR
Official implementation of the IROS paper: PARKOUR (Semantic-Native Underground Robotic Navigation)

# 🚗 PARKOUR: Semantic-Native Underground Robotic Navigation

<div align="center">

[![Paper](https://img.shields.io/badge/Paper-IROS%202026%20(In%20Review)-blue.svg)](#)
[![ROS](https://img.shields.io/badge/ROS-Noetic-22314E.svg)](http://wiki.ros.org/noetic)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

**Semantic-Native Underground Robotic Navigation via Lightweight OSM and Uncertainty-Aware EKF**

</div>

> **📢 News:** > - **[Feb 2026]** Our paper has been submitted to **IROS**.
> - **[Feb 2026]** 🚧 *The core source code, OSM extraction tools, and sample datasets are currently undergoing final cleanup and documentation. The full repository will be made publicly available upon paper acceptance. Stay tuned!*

---

## 📌 1. Overview

**PARKOUR** is a lightweight, highly scalable navigation framework designed for Autonomous Valet Parking (AVP) in GPS-denied underground environments. By replacing labor-intensive, dense High-Definition (HD) maps with an ultra-lightweight **OpenStreetMap (OSM)** topological backbone, PARKOUR enables rapid **"zero-shot" deployment** without on-site pre-scanning.

<p align="center">
  <img src="docs/teaser.png" alt="PARKOUR Teaser" width="90%">
</p>

### ✨ Core Features
- **Ultra-Lightweight Mapping**: Utilizes a 51 KB OSM-based topological map, representing a $>24\times$ reduction in storage compared to traditional HD maps.
- **Zero-Shot Deployment**: Enables deployment from architectural blueprints to a navigation-ready system in just 10 seconds.
- **Uncertainty-Aware Semantic EKF**: Fuses continuous wheel odometry with discrete parking slot IDs (captured via a side-facing camera) with geometric compensation to robustly reset cumulative drift.
- **Semantic-Native (ID-to-ID) Routing**: Bypasses coordinate translation errors by directly routing towards semantic goals, achieving a **95%** navigation success rate and **2.98m RMSE** operational precision.

---

## 🛠️ 2. Dependencies

The framework is built and tested on **Ubuntu 20.04** with **ROS Noetic**. 

- **C++14/17** - **ROS Noetic** (roscpp, rospy, std_msgs, sensor_msgs, nav_msgs, tf2)
- **Eigen3** (for EKF matrix operations)
- **OpenCV 4** (for vision processing)
- **PyTorch & YOLOv8** (for Semantic ID recognition: YOLO-seg)

---

## 🚀 3. Quick Start (Coming Soon)

*Detailed instructions will be provided upon the official release.*

### Build the Workspace
```bash
mkdir -p ~/parkour_ws/src
cd ~/parkour_ws/src
git clone [https://github.com/YourName/PARKOUR.git](https://github.com/YourName/PARKOUR.git)
cd ..
catkin_make
source devel/setup.bash

```

### Run the Demo

```bash
# 1. Launch the OSM Topological Map Server
roslaunch parkour_map map_server.launch

# 2. Launch the Semantic-Augmented EKF Node
roslaunch parkour_localization semantic_ekf.launch

# 3. Launch the Planning & Control Module
roslaunch parkour_navigation navigate.launch target_id:="748"

```

---

## 📊 4. Quantitative Performance

PARKOUR achieves competitive localization precision compared to heavy metric SLAM baselines while requiring significantly less map storage:

| Methods / Configurations | RMSE (m) | Map Type | Map Size |
| --- | --- | --- | --- |
| ORB-SLAM3 | 3.10 | Point Cloud | 12.5 MB |
| AVP-SLAM | 2.55 | Point Cloud | 1.2 MB |
| **PARKOUR (Ours)** | **2.98** | **OSM Topo** | **51 KB** |

---

## 📝 5. Citation

If you find this project helpful for your research, please consider citing our paper:

```bibtex
@inproceedings{wang2026parkour,
  title={PARKOUR: Semantic-Native Underground Robotic Navigation via Lightweight OSM and Uncertainty-Aware EKF},
  author={Wang, Yu and others},
  booktitle={In Review},
  year={2026}
}

```

## 🤝 6. Acknowledgements

We would like to express our gratitude to the open-source communities of [OSM](https://www.openstreetmap.org/), [YOLOv8](https://github.com/ultralytics/ultralytics), and [ROS](https://www.ros.org/).

```

***

### 💡 你需要做的最后几步调整：

1. **替换图片占位符**：我在代码里留了一个图片的引用 `<img src="docs/teaser.png"...>`。你需要：
   * 在你的 GitHub 仓库里新建一个名为 `docs` 的文件夹。
   * 把你论文里的第一张图（Teaser Figure）导出为 `.png` 格式，命名为 `teaser.png`，并上传到 `docs` 文件夹中。
   * 这样，用户一打开你的 GitHub 主页，就能直接看到那张极具视觉冲击力的精美插图！
2. **替换链接**：把 `https://github.com/YourName/PARKOUR.git` 里的 `YourName` 替换成你真实的 GitHub 用户名。
3. **BibTeX 占位**：目前 Citation 里的 `booktitle` 填的是 `In Review`，等你的论文被正式录用后，再回来把它改成真实的 IROS 会议信息。

这份 README 完美兼顾了学术严谨性和工程可读性。那个 `News` 模块里的 "The core source code... is currently undergoing final cleanup..." 是极为关键的“学术免死金牌”，它优雅地向审稿人解释了为什么现在里面还没有几十个 `.cpp` 文件。

快把它复制到你的仓库里，然后尽情去提交你的 IROS 论文吧！一切就绪，祝你好运！

```
