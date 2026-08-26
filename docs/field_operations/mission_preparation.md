# Mission Preparation

## Offline Map Tiles

- On the base station, open the `cougars-dev` repository in VSCode and use the Command Palette (`Ctrl + Shift + P`) to select "Dev Containers: Reopen in Container."

- Once the containers load, open a web browser and pull up the [MapTilesDownloader](https://github.com/cougars-auv/MapTilesDownloader) GUI by navigating to [localhost:8081](http://localhost:8081/).

- Select and download map tiles for the upcoming mission to make them available offline.

  > These should be configured by default, but leave "Output directory" blank and set the zoom levels from 15 to 19.

## Software Updates

- If you want to run a specific CoUGARs software version, check out the dedicated release branch (e.g., `release/v1.2.x`) on `cougars-dev` and add `VERSION=v1.2.x` to a new `.env` file in `cougars-dev/.devcontainer`.

  ```bash
  cd ~/cougars-dev && git fetch origin && git checkout release/v1.2.x && git pull
  cd ~/cougars-dev/.devcontainer && echo "VERSION=v1.2.x" > .env
  ```

- Use the Command Palette (`Ctrl + Shift + P`) to select "Dev Containers: Rebuild Container Without Cache." If you specified a version, this will pull the version-specific Docker image and package tags.

- Open a new terminal window using `` Ctrl + Alt + Shift + ` ``, build the `ros2_ws` workspace, and test launching the base station software using `./base_launch.sh`.

  ```bash
  cd ~/cougars-dev/ros2_ws && colcon build
  cd ~/cougars-dev/scripts && ./base_launch.sh
  ```

- For each CougUV:

  - Outside of the dev container, attach to the configured tmux session over mosh using `./connect.sh <agent-ns>`.

    ```bash
    cd ~/cougars-dev && ./connect.sh <agent-ns>
    ```

  - If you want to run a specific CoUGARs software version, check out the dedicated release branch (e.g., `release/v1.2.x`) on `cougars-dev`.

    ```bash
    cd ~/cougars-dev && git fetch origin && git checkout release/v1.2.x && git pull
    ```

  - Set up the CoUGARs software stack using `./setup.sh $(hostname) <base-station-ip>`. To specify a version, run `./setup.sh $(hostname) <base-station-ip> --version v1.2.x`.

    ```bash
    cd ~/cougars-dev && ./setup.sh $(hostname) <base-station-ip> # --version v1.2.x
    ```

  - Use the configured aliases to pull the latest code from both remotes and rebuild and relaunch the ROS 2 nodes.

    ```bash
    pull
    pull-base
    restart
    logs
    ```

    > Skip `pull` and `pull-base` on a pinned release — those repositories are checked out at tags, not branches.
