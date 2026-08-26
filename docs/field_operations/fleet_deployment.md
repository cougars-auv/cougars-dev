# Fleet Deployment

## Startup

- Power on the dedicated fleet WiFi router and each CougUV.

- On the base station, open the `cougars-dev` repository in VSCode and use the Command Palette (`Ctrl + Shift + P`) to select "Dev Containers: Reopen in Container."

- Once the containers load, open a new terminal window using `` Ctrl + Alt + Shift + ` `` and  launch the base station software using `./base_launch.sh`.

  ```bash
  cd ~/cougars-dev/scripts && ./base_launch.sh
  ```

## On-Site Changes

- Make any code or config changes on the base station using VSCode. Commit them locally.

  > Don't edit code on the CougUVs directly. Instead, make changes on the base station and roll them out to the fleet using git.

- Rebuild the `ros2_ws` workspace and relaunch the base station software using `./base_launch.sh`.

  ```bash
  cd ~/cougars-dev/ros2_ws && colcon build
  cd ~/cougars-dev/scripts && ./base_launch.sh
  ```

- For each CougUV:

  - Outside of the dev container, attach to the configured tmux session using `./connect.sh <agent-ns>`.

    ```bash
    cd ~/cougars-dev && ./connect.sh <agent-ns>
    ```

  - Use the configured aliases to pull the latest code from the base station remote and rebuild/restart the ROS 2 software.

    ```bash
    pull-base
    restart
    ```
