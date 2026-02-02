<p align="center">
    <img width="70%" src="https://autoapms.github.io/auto-apms-guide/logo/logo.png">
</p>

# ⚙️ AutoAPMS - Streamlining Behaviors in ROS 2

<a href="https://autoapms.github.io/auto-apms-guide/introduction/about">![Docs](https://img.shields.io/website?url=https%3A%2F%2Fautoapms.github.io%2Fauto-apms-guide&label=🎓Documentation)</a>
[![Ask DeepWiki](https://deepwiki.com/badge.svg)](https://deepwiki.com/AutoAPMS/auto-apms)
[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.14790307.svg)](https://doi.org/10.5281/zenodo.14790307)


![jazzy](https://img.shields.io/ros/v/jazzy/autoapms?&logo=ros&label=ROS%202)
![kilted](https://img.shields.io/ros/v/kilted/autoapms?&logo=ros&label=ROS%202)
![rolling](https://img.shields.io/ros/v/rolling/autoapms?&logo=ros&label=ROS%202)

![auto-apms-gif](https://github.com/user-attachments/assets/0039aa09-9448-4102-9eb3-38138a805728)

<img align="right" width="350"  alt="autoapms-hierarchy" src="https://github.com/user-attachments/assets/d28e7c8f-70a2-4fd3-897d-d85b02b768de" />

**Start leveraging the advantages of Behavior Trees 🌳 fully integrated with ROS 2 🤖**

AutoAPMS (Automated Action Planning and Management System) is a **heavily extensible** development framework for **behavior-based ROS 2 applications**. It provides a **highly modular integration of behavior trees**, implements a **powerful execution engine** and offers convenient **CLI tooling** for deploying behaviors with ease.

This project adopts the behavior tree implementation provided by [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) and embeds it into the ROS 2 ecosystem so that developers have a much easier time writing custom behaviors and distributing them among robots.

The intention of this project is to make it significantly more user-friendly and less error prone to develop autonomous robotics with behavior trees. The core packages are written in C++ and a supplementary Python API exposes high-level features for scripting.

> [!NOTE]
> Currently we support **Linux only**!.

## ✨ Highlights

Here are some of the most prominent features offered by this repository:

- Convenient resource management using `ament_cmake` and `ament_index`

- Inherently extensible due to plugin-based design

- Flexible and highly configurable behavior execution engine

- Powerful C++ behavior tree builder API (a supplement to BehaviorTree.CPP)

- High-level node manifests for registering node plugins without writing a single line of code

- Support for custom behavior definitions and tree builder algorithms

- `ros2 behavior` command extending the ROS2 CLI for behavior management

- Abstractions for PX4 Autopilot available with [auto-apms-px4](https://github.com/autoapms/auto-apms-px4)

- Comprehensive [user guide](https://autoapms.github.io/auto-apms-guide/) and [API documentation](https://autoapms.github.io/auto-apms/)

## 🚀 Getting Started

The following installation guide helps you getting started with AutoAPMS.

1. Create a [ROS 2 workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) and clone this repository

    ```bash
    mkdir ros2_ws && cd ros2_ws
    (mkdir src && cd src && git clone https://github.com/autoapms/auto-apms.git)
    ```

1. Install all required dependencies. We assume that you already installed ROS 2 on your system

    ```bash
    rosdep init  # Skip this if rosdep has already been initialized
    rosdep update
    rosdep install --from-paths src --ignore-src -y
    ```

1. Build and install all packages required for `auto_apms_examples`

    ```bash
    colcon build --packages-up-to auto_apms_examples --symlink-install
    ```

1. Run your first behavior using `ros2 behavior`. This is an extension of the ROS 2 CLI introduced by the `auto_apms_ros2behavior` package

    ```bash
    source install/setup.bash
    ros2 behavior run auto_apms_examples::demo::HelloWorld --blackboard name:=Turtle
    ```

### Check out the demo using [pyrobosim](https://github.com/sea-bass/pyrobosim)

We provide a guide for running a cool **visual demonstration** on complex behaviors created with AutoAPMS in the [auto_apms_simulation](https://github.com/AutoAPMS/auto_apms_simulation) repository. The simulation shows multiple "robots" moving in a magical hall adjusting their behavior dynamically according to the following policy: Approach the goal as long as the hallway is not occupied - if it is, retreat. *Piertotum Locomotor!*

https://github.com/user-attachments/assets/adbb7cab-1a9b-424b-af61-61c351986287

## 🎓 Documentation

Make sure to visit the [User Guide](https://autoapms.github.io/auto-apms-guide/introduction/about) for tutorials and best practices when writing software using AutoAPMS.

We also offer an extensive [API Documentation](https://autoapms.github.io/auto-apms/) which is created using Doxygen >= 1.10. To generate the documentation run the following from the repository's root:

```bash
doxygen doc/Doxyfile
```

## ⭐ Star History

<a href="https://www.star-history.com/#AutoAPMS/auto-apms&type=date&legend=top-left">
 <picture>
   <source media="(prefers-color-scheme: dark)" srcset="https://api.star-history.com/svg?repos=AutoAPMS/auto-apms&type=date&theme=dark&legend=top-left" />
   <source media="(prefers-color-scheme: light)" srcset="https://api.star-history.com/svg?repos=AutoAPMS/auto-apms&type=date&legend=top-left" />
   <img alt="Star History Chart" src="https://api.star-history.com/svg?repos=AutoAPMS/auto-apms&type=date&legend=top-left" />
 </picture>
</a>

## 🙏 Acknowledgements

This repository builds on:

- [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) by Davide Faconti
- [pyrobosim](https://github.com/sea-bass/pyrobosim) by Sebastian Castro

Special thanks to the maintainers of these projects. It was their contributions to the robotic community that made AutoAPMS possible in the first place! 🚀
