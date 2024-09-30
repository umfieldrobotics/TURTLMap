# 🐢TURTLMap: Textureless Underwater Real-Time Localization and Mapping
### [Project Page](https://umfieldrobotics.github.io/TURTLMap/) | [Paper](https://arxiv.org/abs/2408.01569) | [LinkedIn Post](https://www.linkedin.com/posts/jingyu-song-93763a132_i-am-happy-to-share-that-our-paper-activity-7227132379373473792-Ci1H?utm_source=share&utm_medium=member_desktop)

We propose the first real-time localization and dense mapping solution that is robust to low-texture underwater scenes. TURTLMap is designed to be low-cost for scalability and is capable of running onboard on an NVIDIA Jetson Orin Nano in real-time. TURTLMap leverages [GTSAM](https://gtsam.org/) to develop a multi-modal robust odometry and [Voxblox](https://github.com/ethz-asl/voxblox) to build a dense 3D map.

<!-- insert a gif media\teaser-ezgif.com-video-to-gif-converter.gif -->
![TURTLMap in action](media/teaser-ezgif.com-video-to-gif-converter.gif)

<!-- `git clone https://github.com/catkin/catkin_simple.git` -->


## Update
<!-- different branch -->
 - (2024/09): We have officially released code and data for TURTLMap!
 - (2024/06): 🔥TURTLMap is getting a lot of attention on [LinkedIn](https://www.linkedin.com/posts/jingyu-song-93763a132_i-am-happy-to-share-that-our-paper-activity-7227132379373473792-Ci1H?utm_source=share&utm_medium=member_desktop)!
 - (2024/06): TURTLMap is accepted to IROS 2024! See you in Abu Dhabi!

## Installation
Please refer to the [installation guide](docs/installation.md) for detailed instructions.

## Data Preparation
We provide logs collected from our field experiments. Please refer to the [data preparation guide](docs/data_preparation.md) for detailed instructions on downloading and using them to run TURTLMap.

## Use TURTLMap
For instructions on how to run, visualize, and evaluate TURTLMap, please refer to the [user guide](docs/getting_started_turtlmap.md).

## TURTLMap on Your Own Robot
For instructions on how to extend TURTLMap to your own robot, please refer to the [extension guide](docs/extend_turtlmap.md).

## Citation
Please cite our paper if you find TURTLMap useful for your research:
```bibtex
@article{song2024turtlmap,
  title={TURTLMap: Real-time Localization and Dense Mapping of Low-texture Underwater Environments with a Low-cost Unmanned Underwater Vehicle},
  author={Song, Jingyu and Bagoren, Onur and Andigani, Razan and Sethuraman, Advaith Venkatramanan and Skinner, Katherine},
  journal={arXiv preprint arXiv:2408.01569},
  year={2024}
}
```
