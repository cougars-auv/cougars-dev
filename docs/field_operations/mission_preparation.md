# Mission Preparation

## Offline Map Tiles

- On the base station, open the `cougars-dev` repository in VSCode. Use the Command Palette (`Ctrl + Shift + P`) to select "Dev Containers: Reopen in Container."

- Once the containers load, open the [MapTilesDownloader](https://github.com/cougars-auv/MapTilesDownloader) GUI at [localhost:8081](http://localhost:8081/).

- Select and download map tiles for the upcoming mission to make them available offline.

  > These should be configured by default, but leave "Output directory" blank and set the zoom levels from 15 to 19.

## Software Updates

- Update the Docker images.

  ```bash
  cd ~/cougars-dev/.devcontainer && docker compose pull
  ```

- Use the Command Palette (`Ctrl + Shift + P`) to select "Dev Containers: Rebuild Container."

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

  - Use the configured aliases to pull the latest code from both remotes, import the vcs-pinned packages, update the Docker image, rebuild the `ros2_ws` workspace, and restart the ROS 2 software.

    ```bash
    pull
    pull-base
    vcs-import
    docker-pull
    build
    restart
    logs
    ```
