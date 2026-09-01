# Mission Preparation

## Offline Map Tiles

- On the base station, open the `cougars-dev` repository in VSCode. Use the Command Palette (`Ctrl + Shift + P`) to select "Dev Containers: Reopen in Container."

- Once the containers load, open the [MapTilesDownloader](https://github.com/cougars-auv/MapTilesDownloader) GUI at [localhost:8081](http://localhost:8081/).

- Select and download map tiles for the upcoming mission to make them available offline.

  > These should be configured by default, but leave "Output directory" blank and set the zoom levels from 15 to 19.

## Software Updates

- To run a specific CoUGARs software version, check out its release branch (e.g., `release/v1.2.x`) in `cougars-dev`. Then add the exact release tag (e.g., `VERSION=v1.2.3`) to a `.env` file in `cougars-dev/.devcontainer`.

  ```bash
  cd ~/cougars-dev && git fetch origin && git checkout release/v1.2.x && git pull
  cd ~/cougars-dev/.devcontainer && echo "VERSION=v1.2.3" > .env
  ```

- Update the Docker images. If you specified a version, this will pull the version-specific image tags.

  ```bash
  cd ~/cougars-dev/.devcontainer && docker compose pull
  ```

- Use the Command Palette (`Ctrl + Shift + P`) to select "Dev Containers: Rebuild Container." If you specified a version, this will pull the version-specific package tags from the `.repos` files.

- Open a new terminal window using `` Ctrl + Alt + Shift + ` ``, build the `ros2_ws` workspace, and test the base station software with `./base_launch.sh`.

  ```bash
  cd ~/cougars-dev/ros2_ws && colcon build
  cd ~/cougars-dev/scripts && ./base_launch.sh
  ```

- For each CougUV:

  - Outside of the dev container, attach to a tmux session on the CougUV using `./connect.sh <agent-ns>`.

    ```bash
    cd ~/cougars-dev && ./connect.sh <agent-ns>
    ```

  - To run a specific CoUGARs software version, check out its release branch (e.g., `release/v1.2.x`) in `cougars-dev`.

    ```bash
    cd ~/cougars-dev && git fetch origin && git checkout release/v1.2.x && git pull
    ```

  - Set up the CoUGARs software stack with `./setup.sh $(hostname) <base-station-ip>`. To specify a version, add `--version v1.2.3`.

    ```bash
    cd ~/cougars-dev && ./setup.sh $(hostname) <base-station-ip> # --version v1.2.3
    ```

  - Use the configured aliases to pull the latest code from both remotes, update the Docker image, rebuild the `ros2_ws` workspace, and restart the ROS 2 software.

    ```bash
    pull
    pull-base
    docker-pull
    build
    restart
    logs
    ```
