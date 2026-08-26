# Base Station Software Setup

- Install `mosh` on the base station.

    ```bash
    sudo apt-get update && sudo apt-get install -y mosh
    ```

- Work through the [Get Started](https://github.com/cougars-auv/cougars-dev/blob/main/README.md#get-started) section of `cougars-dev`.

- Choose the "Recorded Data (`rosbag2`)" workflow, but skip copying in a bag and run `./base_launch.sh` instead of `./bag_launch.sh`.

    ```bash
    cd ~/cougars-dev/ros2_ws && colcon build
    cd ~/cougars-dev/scripts && ./base_launch.sh
    ```
