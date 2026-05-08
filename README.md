# 🌊 CoUGARs Development Environment

[![arXiv](https://img.shields.io/badge/arXiv-2511.08822-b31b1b.svg)](https://arxiv.org/pdf/2511.08822)
[![ROS 2 Build & Test](https://github.com/cougars-auv/cougars-dev/actions/workflows/ros2_build_and_test.yml/badge.svg)](https://github.com/cougars-auv/cougars-dev/actions/workflows/ros2_build_and_test.yml)
[![Docker Build](https://github.com/cougars-auv/cougars-dev/actions/workflows/docker_build.yml/badge.svg)](https://github.com/cougars-auv/cougars-dev/actions/workflows/docker_build.yml)
[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/cougars-auv/cougars-dev/main.svg)](https://results.pre-commit.ci/latest/github/cougars-auv/cougars-dev/main)
[![codecov](https://codecov.io/gh/cougars-auv/cougars-dev/graph/badge.svg?token=5OBYXUBZR8)](https://codecov.io/gh/cougars-auv/cougars-dev)

<p align="left">
  <img src=".github/assets/mapviz.gif" width="600">
</p>

## 🚀 Get Started

> **Prerequisites:** 64-bit Linux, free disk space (10+ GB recommended), and a dedicated NVIDIA GPU (for HoloOcean simulation).

- Install [Docker](https://www.docker.com/get-started/) and [VSCode Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers).

- Add a [GitHub SSH key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent?platform=linux) and clone the `cougars-dev` repository.

  ```bash
  git clone git@github.com:cougars-auv/cougars-dev.git
  ```

- Choose a development workflow:

  **Simulation (HoloOcean):**

  <p align="left">
    <img src=".github/assets/holoocean.gif" width="400">
  </p>

  - Build a runtime image for [HoloOcean-ROS](https://github.com/snelsondurrant/holoocean-ros/tree/main/docker/runtime) on `snelsondurrant`'s fork. When prompted to run `./build_container.sh`, specify the branch `nelson/fgo-dev` using `./build_container.sh -b nelson/fgo-dev`.

  - Open the `cougars-dev` repository in VSCode and use the Command Palette (`Ctrl + Shift + P`) to select "Dev Containers: Reopen in Container." When prompted to choose a `devcontainer.json` file, click `CoUGARs Dev (HoloOcean)`.

  - Once the containers load, open a new terminal window using `` Ctrl + Alt + Shift + ` `` and launch a HoloOcean scenario in the `holoocean-ct` container using `./holoocean_launch.sh`.

    ```bash
    cd ~/cougars-dev/scripts && ./holoocean_launch.sh
    ```

  - Open a new terminal, build the `ros2_ws` workspace, and select the matching launch configuration using `./sim_launch.sh`.

    ```bash
    cd ~/cougars-dev/ros2_ws && colcon build
    cd ~/cougars-dev/scripts && ./sim_launch.sh
    ```

  **Recorded Data (`rosbag2`):**

  <p align="left">
    <img src=".github/assets/rviz.gif" width="400">
  </p>

  - Open the `cougars-dev` repository in VSCode and use the Command Palette (`Ctrl + Shift + P`) to select "Dev Containers: Reopen in Container." When prompted to choose a `devcontainer.json` file, click `CoUGARs Dev`.

  - Once the containers load, copy your `rosbag2` bag into the `bags` folder at the root of the repository.

  - Open a new terminal window using `` Ctrl + Alt + Shift + ` ``, build the `ros2_ws` workspace, and select the bag using `./bag_launch.sh`.

    ```bash
    cd ~/cougars-dev/ros2_ws && colcon build
    cd ~/cougars-dev/scripts && ./bag_launch.sh
    ```

> **Note:** This repository uses vcstool to manage nested repositories. If not all repositories appear in the Git sidebar, open settings (`Ctrl + ,`), set "Git: Repository Scan Max Depth" to 3, and reload the window.

## 🤝 Contributing

- **Create a Branch:** Create a new branch using the format `name/feature` (e.g., `nelson/repo-docs`).

- **Make Changes:** Develop and debug your new feature. Add good documentation.

  > If you need to add dependencies, update the `package.xml`, `Dockerfile`, `cougars.repos`, or `dependencies.repos` in your branch and test building the image locally. The CI will automatically build and push the new image to Docker Hub upon merge.

- **Sync Frequently:** Regularly rebase your branch against `main` (or merge `main` into your branch) to prevent conflicts.

- **Submit a PR:** Open a pull request, ensure required tests pass, and merge once approved.

## 📚 Citations

Please cite our relevant publications if you find this repository useful for your research:

### CoUGARs
```bibtex
@misc{durrant2025lowcostmultiagentfleetacoustic,
  title={Low-cost Multi-agent Fleet for Acoustic Cooperative Localization Research},
  author={Nelson Durrant and Braden Meyers and Matthew McMurray and Clayton Smith and Brighton Anderson and Tristan Hodgins and Kalliyan Velasco and Joshua G. Mangelson},
  year={2025},
  eprint={2511.08822},
  archivePrefix={arXiv},
  primaryClass={cs.RO},
  url={https://arxiv.org/abs/2511.08822},
}
```

### HoloOcean-ROS
```bibtex
@misc{meyers2025testingevaluationunderwatervehicle,
  title={Testing and Evaluation of Underwater Vehicle Using Hardware-In-The-Loop Simulation with HoloOcean},
  author={Braden Meyers and Joshua G. Mangelson},
  year={2025},
  eprint={2511.07687},
  archivePrefix={arXiv},
  primaryClass={cs.RO},
  url={https://arxiv.org/abs/2511.07687},
}
```

### HoloOcean
```bibtex
@inproceedings{potokar2022holooceanunderwaterroboticssim,
  author={Easton Potokar and Spencer Ashford and Michael Kaess and Joshua G. Mangelson},
  title={Holo{O}cean: An Underwater Robotics Simulator},
  booktitle={Proc. IEEE Intl. Conf. on Robotics and Automation, ICRA},
  address={Philadelphia, PA, USA},
  month={May},
  year={2022}
}
```
