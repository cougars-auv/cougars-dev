# CougUV Software Setup

- Complete the [Base Station Software Setup](https://github.com/cougars-auv/cougars-dev/blob/main/docs/fleet_assembly/base_station_software_setup.md).

- Choose a new agent namespace (e.g., `coug0`).

- Add a parameter file to `cougars-dev/config` named `<agent-ns>_params.yaml` (e.g., `coug0_params.yaml`). At a minimum, set the following parameters:

  ```yaml
  /<agent-ns>:
    coug_description_launch:
      ros__parameters:
        urdf_file: "<agent-type>.urdf.xacro"
  
    coug_comms_base_launch:
      ros__parameters:
        beacon_id: <beacon-id>
  ```

- Commit and push the new file.

- Install [Raspberry Pi Imager](https://www.raspberrypi.com/software/) on the base station or a personal computer.

- Flash "Raspberry Pi OS Lite (64-bit)" onto a new SD card. During configuration, set the agent namespace as the hostname and `frostlab` as the username and password. Add the dedicated fleet WiFi router details and enable password-based SSH.

- Insert the SD card into the board and power it on. Once the device is online, generate a new SSH key on the base station outside the dev container, then connect.

  ```bash
  ssh-keygen -t ed25519
  ssh-copy-id frostlab@<agent-ns>.local
  ssh frostlab@<agent-ns>.local
  ```

- On the Raspberry Pi 5:

  - Install [Docker Engine](https://docs.docker.com/engine/install/debian/#install-using-the-repository), including the [Linux post-installation steps](https://docs.docker.com/engine/install/linux-postinstall).
  
  - Install project dependencies.
  
    ```bash
    sudo apt-get update && sudo apt-get install -y git vim mosh tmux rsync chrony linuxptp gpsd gpsd-clients python3-setuptools python3-pip
    sudo pip install --break-system-packages tmuxp vcstool
    ```
  
  - Clone the `cougars-dev` repository.
  
    ```bash
    cd ~ && git clone https://github.com/cougars-auv/cougars-dev.git
    ```
  
  - Set up the CoUGARs software stack with `./setup.sh $(hostname) <base-station-ip>`.
  
    ```bash
    cd ~/cougars-dev && ./setup.sh $(hostname) <base-station-ip>
    ```

- Close the direct SSH connection, then attach via tmux with `./connect.sh <agent-ns>`.

  ```bash
  cd ~/cougars-dev && ./connect.sh <agent-ns>
  ```
 
- Use the configured aliases to pull the latest code from both remotes, update the Docker images, rebuild the `ros2_ws` workspace, and restart the ROS 2 software.
  
    ```bash
    pull
    pull-base
    docker-pull
    build
    restart
    logs
    ```
    
