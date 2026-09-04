# Fleet Deployment

## Startup

- Power on the dedicated fleet WiFi router and each CougUV.

- On the base station, open the `cougars-dev` repository in VSCode. Use the Command Palette (`Ctrl + Shift + P`) to select "Dev Containers: Reopen in Container."

- Once the containers load, open a new terminal window using `` Ctrl + Alt + Shift + ` `` and launch the base station software with `./base_launch.sh`.

  ```bash
  cd ~/cougars-dev/scripts && ./base_launch.sh
  ```

## Field Changes

- Make any code or config changes on the base station in VSCode, then commit them locally.

  > Do not edit code directly on the CougUVs. Make changes on the base station, then roll them out to the fleet with Git.

- Rebuild the `ros2_ws` workspace and relaunch the base station software with `./base_launch.sh`.

  ```bash
  cd ~/cougars-dev/ros2_ws && colcon build
  cd ~/cougars-dev/scripts && ./base_launch.sh
  ```

- For each CougUV:

  - Outside of the dev container, attach to a tmux session on the CougUV using `./connect.sh <agent-ns>`.

    ```bash
    cd ~/cougars-dev && ./connect.sh <agent-ns>
    ```

  - Use the configured aliases to pull the latest code from the base station remote, rebuild the `ros2_ws` workspace, and restart the ROS 2 software.

    ```bash
    pull-base
    build
    restart
    ```
