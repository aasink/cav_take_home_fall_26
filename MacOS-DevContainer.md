# macOS Instructions (Dev Container):
We will use a [Dev Container](https://containers.dev/) to run Ubuntu with ROS2 Humble on macOS. This is the recommended approach as it automates much of the Docker setup.

## Prerequisites
First, ensure you have the following installed:

1. **Docker Desktop**: Follow the [installation instructions here](https://docs.docker.com/desktop/setup/install/mac-install/).

2. **VS Code**: Download from [code.visualstudio.com](https://code.visualstudio.com/).

3. **Dev Containers Extension**: Install the [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) in VS Code.

## Opening the Dev Container
1. Open Docker Desktop and ensure it is running.

2. Open this repository folder in VS Code.

3. VS Code should detect the `.devcontainer` folder and prompt you to "Reopen in Container". Click that button.
   
   Alternatively, open the Command Palette (`Cmd+Shift+P`) and run:
   ```
   Dev Containers: Reopen in Container
   ```

4. Wait for the container to build. The first build may take several minutes as it downloads the base image and installs dependencies.

5. Once complete, you'll have a fully configured ROS2 Humble environment with all necessary tools pre-installed.

## Using the Dev Container
Once inside the container:

- The integrated terminal in VS Code runs inside the container.
- Your workspace files are automatically mounted and synced.
- ROS2 Humble is already sourced in your shell.
- Extensions for C++, Python, and ROS are pre-installed.

To verify the setup, run:
```bash
source /opt/ros/humble/setup.bash
ros2
```

### Stopping and Restarting
- **Stop**: Close VS Code or run `Dev Containers: Close Remote Connection` from the Command Palette.
- **Restart**: Open the folder in VS Code and select "Reopen in Container" again.

Recap:
Unlike manual Docker setup, Dev Containers handle user creation, volume mounting, and environment configuration automatically. Your work is saved in your local workspace folder, so you won't lose progress if the container is rebuilt.

## Writing Code
Simply edit files directly in VS Code—they are automatically synced with the container.

## Visualization with Foxglove
For visualization, use [Foxglove Studio](https://foxglove.dev/). The Foxglove bridge is pre-installed in the container.

1. Start the Foxglove bridge inside the container:
   ```bash
   source /opt/ros/humble/setup.bash
   # For custom messages source the workspace setup file
   # source /home/${username}/cav_take_home_fall_26/install/setup.bash
   ros2 launch foxglove_bridge foxglove_bridge_launch.xml
   ```

2. Open Foxglove Studio on your Mac and click "Open connection...".

3. Use `ws://localhost:8765` as the WebSocket URL.

## Getting Files into the Container
Since the workspace is mounted, any files you add to the repository folder on your Mac will automatically appear in the container.

### Mounting External Data Folders
To mount additional folders (e.g., a folder containing mcap files), edit the `mounts` setting in `.devcontainer/devcontainer.json`:

```json
"mounts": [
    "source=/path/to/your/data/folder,target=/home/${localEnv:USER}/cav_take_home_fall_26/data,type=bind,consistency=cached"
]
```

Replace `/path/to/your/data/folder` with the absolute path to your data folder on your Mac. After editing, rebuild the container:
1. Open the Command Palette (`Cmd+Shift+P`)
2. Run `Dev Containers: Rebuild Container`

Your data folder will now be accessible at `~/cav_take_home_fall_26/data` inside the container.

## Troubleshooting

### Container fails to build
- Ensure Docker Desktop is running.
- Try rebuilding: `Dev Containers: Rebuild Container` from the Command Palette.

### Permission issues
- The container runs as your local user, so permission issues should be rare.
- If you encounter issues, try rebuilding the container.

### Slow performance on Apple Silicon
- Ensure you're using Docker Desktop 4.16+ which has improved ARM64 support.
- The Dockerfile uses `arm64v8/ros:humble` which is optimized for M-series chips.
