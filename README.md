# mrs_consensus

## Docker Setup

Follow the instructions in the link below to set up the Docker environment:
https://github.com/misiek02/mrs_crazyflies
```bash
./ros2_ws/src/mrs_crazyflies/startup/start.sh
```

## Build and Usage

Run the following commands to build the workspace, source the setup file, and execute the node:

```bash
# Build
colcon build
```
```bash
# Source
source install/setup.bash
```
```bash
# Run
ros2 run mrs_consensus rendezvous_node
```
