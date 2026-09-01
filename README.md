# 🌊 CoUGARs: Configurable Underwater Group of Autonomous Robots

[![arXiv](https://img.shields.io/badge/arXiv-2511.08822-b31b1b.svg)](https://arxiv.org/pdf/2511.08822)
[![ROS 2 Build & Test](https://github.com/cougars-auv/cougars-dev/actions/workflows/ros2_build_and_test.yml/badge.svg)](https://github.com/cougars-auv/cougars-dev/actions/workflows/ros2_build_and_test.yml)
[![Docker Build](https://github.com/cougars-auv/cougars-dev/actions/workflows/docker_build.yml/badge.svg)](https://github.com/cougars-auv/cougars-dev/actions/workflows/docker_build.yml)
[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/cougars-auv/cougars-dev/main.svg)](https://results.pre-commit.ci/latest/github/cougars-auv/cougars-dev/main)
[![codecov](https://codecov.io/gh/cougars-auv/cougars-dev/graph/badge.svg?token=5OBYXUBZR8)](https://codecov.io/gh/cougars-auv/cougars-dev)

CoUGARs is a low-cost, configurable AUV platform designed for multi-agent autonomy research by the [Field Robotic Systems Lab (FROST Lab)](https://frostlab.byu.edu) at [Brigham Young University](https://byu.edu).

## Get Started

> **Prerequisites:** 64-bit Linux, 15+ GB of free disk space, and a dedicated NVIDIA GPU (for HoloOcean simulation).

- Install [Docker](https://www.docker.com/get-started/) and [VSCode Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers).

- Add a [GitHub SSH key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent?platform=linux) and clone the `cougars-dev` repository.

  ```bash
  git clone git@github.com:cougars-auv/cougars-dev.git
  ```

- Choose a development workflow:

  **Simulation (HoloOcean):**

  - Build a runtime image for [HoloOcean-ROS](https://github.com/cougars-auv/holoocean-ros/tree/main/docker/runtime) on the `cougars-auv` organization's fork. When prompted to run `./build_container.sh`, specify the branch `nelson/cougars-dev` using `./build_container.sh -b nelson/cougars-dev`.

  - Open the `cougars-dev` repository in VSCode and use the Command Palette (`Ctrl + Shift + P`) to select "Dev Containers: Reopen in Container." When prompted to choose a `devcontainer.json` file, click `CoUGARs Dev (GPU)`.

  - Once the containers load, open a new terminal window using `` Ctrl + Alt + Shift + ` `` and launch a HoloOcean scenario in the `cougars-holoocean-ct` container using `./holoocean_launch.sh`.

    ```bash
    cd ~/cougars-dev/scripts && ./holoocean_launch.sh
    ```

  - Open a new terminal, build the `ros2_ws` workspace, and select the matching launch configuration using `./sim_launch.sh`.

    ```bash
    cd ~/cougars-dev/ros2_ws && colcon build
    cd ~/cougars-dev/scripts && ./sim_launch.sh
    ```

  **Recorded Data (`rosbag2`):**

  - Open the `cougars-dev` repository in VSCode and use the Command Palette (`Ctrl + Shift + P`) to select "Dev Containers: Reopen in Container." When prompted to choose a `devcontainer.json` file, click `CoUGARs Dev`.

  - Once the containers load, copy your `rosbag2` bag into the `bags` folder at the root of the repository.

  - Open a new terminal window using `` Ctrl + Alt + Shift + ` ``, build the `ros2_ws` workspace, and select the bag using `./bag_launch.sh`.

    ```bash
    cd ~/cougars-dev/ros2_ws && colcon build
    cd ~/cougars-dev/scripts && ./bag_launch.sh
    ```
> If you run into crashes or out-of-memory errors while building the workspace, restrict the compiler to a single worker using `colcon build --parallel-workers 1`.

> If not all repositories appear in the Git sidebar in VSCode, open settings (`Ctrl + ,`), set "Git: Repository Scan Max Depth" to 3, and reload the window.

## Documentation

### Fleet Assembly
* Bill of Materials
* Assembly Instructions
* [Base Station Software Setup](https://github.com/cougars-auv/cougars-dev/blob/main/docs/fleet_assembly/base_station_software_setup.md)
* [CougUV Software Setup](https://github.com/cougars-auv/cougars-dev/blob/main/docs/fleet_assembly/couguv_software_setup.md)

### Field Operations
* [Mission Preparation](https://github.com/cougars-auv/cougars-dev/blob/main/docs/field_operations/mission_preparation.md)
* [Fleet Deployment](https://github.com/cougars-auv/cougars-dev/blob/main/docs/field_operations/fleet_deployment.md)
* [Post-Mission Analysis](https://github.com/cougars-auv/cougars-dev/blob/main/docs/field_operations/post_mission_analysis.md)

### Resources
* [Helpful Tutorials](https://github.com/cougars-auv/cougars-dev/blob/main/docs/resources/helpful_tutorials.md)

## Contributing

> For small changes confined to one package, the full `cougars-dev` branch workflow is unnecessary. Simply create a new package branch and PR.

- **Create a Branch:** Create a new `cougars-dev` branch (e.g., `nelson/repo-docs`).

- **Create Package Branches:** For each package you plan to modify, create a new branch with the same name. In your new `cougars-dev` branch, temporarily update the relevant `.repos` files to reference those branches.

- **Make Changes:** Develop, debug, and test your changes.

  > If you need to add dependencies, update the relevant `package.xml` files, Dockerfiles under `.docker/`, `runtime.repos`, `dev.repos`, or `dependencies.repos`. Test building the Docker images locally.

- **Sync Frequently:** Regularly update your branches with the latest `main` (via rebase or merge) to catch conflicts early.

- **Submit Package PRs:** Open a pull request for each package branch. Ensure required tests pass, then merge once approved. After merging, delete the branch and create a new branch from `main` for any follow-up work.

  > All pull requests must use [squash and merge](https://docs.github.com/en/pull-requests/reference/pull-request-merges#squash-and-merge-your-commits).

- **Submit Final PR:** After all package PRs have merged, revert the temporary `.repos` changes in your `cougars-dev` branch and open a pull request. Once merged, GitHub Actions will build and push the updated Docker images to Docker Hub. Delete the branch after merging.

## Releasing

We adhere to the **Semantic Versioning (SemVer 2.0.0)** standard to release new versions of this repository:
> Given a version number **`MAJOR.MINOR.PATCH`**, increment the:
> - **MAJOR** version when you make incompatible API changes
> - **MINOR** version when you add functionality in a backward compatible manner
> - **PATCH** version when you make backward compatible bug fixes

- **Create a Release Branch:** Create a dedicated minor release branch (e.g., `release/v1.2.x`) from `main`.

  > Do not create separate branches for patch versions (e.g., `v1.2.0` to `v1.2.1`). Simply merge fixes into the minor release branch and bump the patch version on a new tag when ready to release.

- **Tag Packages:** Check the packages listed in `runtime.repos` and `dev.repos`. If they have untagged updates, update the `<version>` in the `package.xml` files and push new tags (e.g., `v2.3.4`). Since packages version independently of `cougars-dev`, you can either use an existing up-to-date tag or create a new one.

- **Lock Dependencies:** On the release branch, pin all packages in the `.repos` files to their specific tags (instead of branches like `main`). Commit these updates.

- **Tag and Push:** Create and push a version tag (e.g., `v1.2.3`) on your release commit:

  ```bash
  git tag v1.2.3
  git push origin v1.2.3
  ```

  Pushing the tag automatically rebuilds and publishes the Docker images using a `<target>-<version>` format (e.g., `frostlab/cougars:runtime-v1.2.3`) and opens a draft GitHub Release with auto-generated notes.

- **Publish a GitHub Release:** Review the draft release in GitHub and click **Publish**.

## Citations

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
