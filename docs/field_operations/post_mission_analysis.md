# Post-Mission Analysis

## Bag Sync

- For each CougUV:

  - Outside the dev container, copy the bags recorded by the agent to the base station with `./sync_bags.sh <agent-ns>`.

    ```bash
    cd ~/cougars-dev && ./sync_bags.sh <agent-ns>
    ```

## Field Changes

- On the base station, open the cougars-dev repository in VSCode. Use the Command Palette (Ctrl + Shift + P) to select "Dev Containers: Reopen in Container."

- Once the containers load, push any code or config changes committed locally on the base station to GitHub.
