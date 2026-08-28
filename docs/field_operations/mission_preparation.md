# Mission Preparation

## Offline Map Tiles

- On the base station, open the `cougars-dev` repository in VSCode and use the Command Palette (`Ctrl + Shift + P`) to select "Dev Containers: Reopen in Container."

- Once the containers load, open a web browser and pull up the [MapTilesDownloader](https://github.com/cougars-auv/MapTilesDownloader) GUI by navigating to [localhost:8081](http://localhost:8081/).

- Select and download map tiles for the upcoming mission to make them available offline.

  > These should be configured by default, but leave "Output directory" blank and set the zoom levels from 15 to 19.

## Software Updates

- If you want to run a specific CoUGARs software version, check out the dedicated release branch (e.g., `release/v1.2.x`) on `cougars-dev` and add the exact release tag (e.g., `VERSION=v1.2.3`) to a new `.env` file in `cougars-dev/.devcontainer`.

  ```bash
  cd ~/cougars-dev && git fetch origin && git checkout release/v1.2.x && git pull
  cd ~/cougars-dev/.devcontainer && echo "VERSION=v1.2.3" > .env
  ```

- Update the Docker images. If you specified a version, this will pull the version-specific Docker image.

  ```bash
  cd ~/cougars-dev/.devcontainer && docker compose pull dev router
  ```

- Use the Command Palette (`Ctrl + Shift + P`) to select "Dev Containers: Rebuild Container." If you specified a version, this will pull the pinned tags from the `.repos` files.

- Open a new terminal window using `` Ctrl + Alt + Shift + ` ``, build the `ros2_ws` workspace, and test launching the base station software using `./base_launch.sh`.

  ```bash
  cd ~/cougars-dev/ros2_ws && colcon build
  cd ~/cougars-dev/scripts && ./base_launch.sh
  ```

- For each CougUV:

  - Outside of the dev container, attach to a tmux session on the CougUV using `./connect.sh <agent-ns>`.

    ```bash
    cd ~/cougars-dev && ./connect.sh <agent-ns>
    ```

  - If you want to run a specific CoUGARs software version, check out the dedicated release branch (e.g., `release/v1.2.x`) on `cougars-dev`.

    ```bash
    cd ~/cougars-dev && git fetch origin && git checkout release/v1.2.x && git pull
    ```

  - Set up the CoUGARs software stack using `./setup.sh $(hostname) <base-station-ip>`. To specify a version, run `./setup.sh $(hostname) <base-station-ip> --version v1.2.3`.

    ```bash
    cd ~/cougars-dev && ./setup.sh $(hostname) <base-station-ip> # --version v1.2.3
    ```

  - Use the configured aliases to pull the latest code from both remotes, update the Docker image, and rebuild/restart the ROS 2 software.

    ```bash
    pull
    pull-base
    docker-pull
    restart
    logs
    ```

    > Skip `pull` and `pull-base` when specifying a version -- those repositories are checked out at tags, not branches.
