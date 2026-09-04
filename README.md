# 🌊 CoUGARs: Configurable Underwater Group of Autonomous Robots

[![arXiv](https://img.shields.io/badge/arXiv-2511.08822-b31b1b.svg)](https://arxiv.org/pdf/2511.08822)
[![ROS 2 Build & Test](https://github.com/cougars-auv/cougars-dev/actions/workflows/ros2_build_test.yaml/badge.svg)](https://github.com/cougars-auv/cougars-dev/actions/workflows/ros2_build_test.yaml)
[![Docker Build](https://github.com/cougars-auv/cougars-dev/actions/workflows/docker_build.yaml/badge.svg)](https://github.com/cougars-auv/cougars-dev/actions/workflows/docker_build.yaml)
[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/cougars-auv/cougars-dev/main.svg)](https://results.pre-commit.ci/latest/github/cougars-auv/cougars-dev/main)
[![codecov](https://codecov.io/gh/cougars-auv/cougars-dev/graph/badge.svg?token=5OBYXUBZR8)](https://codecov.io/gh/cougars-auv/cougars-dev)

CoUGARs is a low-cost, configurable AUV platform designed for multi-agent autonomy research by the [Field Robotic Systems Lab (FROST Lab)](https://frostlab.byu.edu) at [Brigham Young University](https://byu.edu).

<p align="left">
  <img src=".github/assets/couguv.jpg" width="600">
</p>

## Get Started

> **Prerequisites:** 64-bit Linux, 15+ GB of free disk space, and a dedicated NVIDIA GPU (for HoloOcean simulation).

- Install [Docker](https://www.docker.com/get-started/) and [VSCode Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers).

- Add a [GitHub SSH key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent?platform=linux) and clone the `cougars-dev` repository.

  ```bash
  git clone git@github.com:cougars-auv/cougars-dev.git
  ```

- Choose a development workflow:

  **Simulation (HoloOcean):**

  - Build the runtime image from the `cougars-auv` fork of [HoloOcean-ROS](https://github.com/cougars-auv/holoocean-ros/tree/main/docker/runtime). When prompted to run `./build_container.sh`, specify the `nelson/cougars-dev` branch with `./build_container.sh -b nelson/cougars-dev`.

  - Open the `cougars-dev` repository in VSCode and use the Command Palette (`Ctrl + Shift + P`) to select "Dev Containers: Reopen in Container." When prompted to choose a `devcontainer.json` file, click "CoUGARs Dev (GPU)."

  - Once the containers load, open a new terminal window using `` Ctrl + Alt + Shift + ` `` and launch a HoloOcean scenario in the `cougars-holoocean-ct` container using `./holoocean_launch.sh`.

    ```bash
    cd ~/cougars-dev/scripts && ./holoocean_launch.sh
    ```

  - Open a new terminal, build the `ros2_ws` workspace, and select the matching launch configuration with `./sim_launch.sh`.

    ```bash
    cd ~/cougars-dev/ros2_ws && colcon build
    cd ~/cougars-dev/scripts && ./sim_launch.sh
    ```

  **Recorded Data (`rosbag2`):**

  - Open the `cougars-dev` repository in VSCode and use the Command Palette (`Ctrl + Shift + P`) to select "Dev Containers: Reopen in Container." When prompted to choose a `devcontainer.json` file, click "CoUGARs Dev."

  - Once the containers load, copy your `rosbag2` bag into the repository's `bags` folder.

  - Open a new terminal window using `` Ctrl + Alt + Shift + ` ``, build the `ros2_ws` workspace, and select a bag with `./bag_launch.sh`.

    ```bash
    cd ~/cougars-dev/ros2_ws && colcon build
    cd ~/cougars-dev/scripts && ./bag_launch.sh
    ```

> If you run into crashes or out-of-memory errors while building the workspace, restrict the compiler to a single worker using `colcon build --parallel-workers 1`.

> If not all repositories appear in the Git sidebar in VSCode, open settings (`Ctrl + ,`), set "Git: Repository Scan Max Depth" to 3, and reload the window.

## Documentation

### Fleet Assembly

- Bill of Materials
- Assembly Instructions
- [Base Station Software Setup](https://github.com/cougars-auv/cougars-dev/blob/main/docs/fleet_assembly/base_station_software_setup.md)
- [CougUV Software Setup](https://github.com/cougars-auv/cougars-dev/blob/main/docs/fleet_assembly/couguv_software_setup.md)

### Field Operations

- [Mission Preparation](https://github.com/cougars-auv/cougars-dev/blob/main/docs/field_operations/mission_preparation.md)
- [Fleet Deployment](https://github.com/cougars-auv/cougars-dev/blob/main/docs/field_operations/fleet_deployment.md)
- [Post-Mission Analysis](https://github.com/cougars-auv/cougars-dev/blob/main/docs/field_operations/post_mission_analysis.md)

### Resources

- [Helpful Tutorials](https://github.com/cougars-auv/cougars-dev/blob/main/docs/resources/helpful_tutorials.md)

## Contributing

> For small changes confined to one package, the full `cougars-dev` branch workflow is unnecessary. Simply create a new package branch and PR.

- **Create a Branch:** Create a new `cougars-dev` branch (e.g., `nelson/repo-docs`).

- **Create Package Branches:** For each package you plan to modify, create a new branch with the same name. In your new `cougars-dev` branch, temporarily update the relevant `.repos` files to reference those branches.

- **Make Changes:** Develop, debug, and test your changes.

  > If you need to add dependencies, update the relevant `package.xml` files, Dockerfiles under `.docker/`, `runtime.repos`, `dev.repos`, or `dependencies.repos`. Test building the Docker images locally.

- **Sync Frequently:** Regularly update your branches with the latest `main` (via rebase or merge).

- **Submit Package PRs:** Open a pull request for each package branch. Ensure required tests pass, then merge once approved.

- **Submit Final PR:** After all package PRs have merged, revert the temporary `.repos` changes in your `cougars-dev` branch and open a pull request. Ensure required tests pass, then merge once approved.

  > Once merged, GitHub Actions will build and push the updated Docker images to Docker Hub.

- **Delete Branches:** Remove all the merged branches. Create a new branch from `main` for any follow-up work.

## Citations

If you use this repository in your research, please cite the following publications:

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
