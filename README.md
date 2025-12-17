# mrs_consensus

## Docker Setup

Follow the instructions in the link below to set up the Docker environment:
https://github.com/misiek02/mrs_crazyflies
```bash
# Start the container:
docker start -i mrs_crazyflies_cont
```
```bash
# Open the container in another terminal, while it is already started:
docker exec -it mrs_crazyflies_cont bash
```
```bash
# Stop the container
docker stop mrs_crazyflies_cont
```
```bash
# Delete the container (WARNING: this will delete all data inside the container)
docker rm mrs_crazyflies_cont
```


## Build and Usage

Run the following commands to build the workspace, source the setup file, and execute the node:

Build all packages:
```bash
colcon build
```
Build consensus package:
```bash
colcon build --packages-select mrs_consensus
```
Source:
```bash
source install/setup.bash
```
Start sim env:
```bash
cd ~/ros2_ws/src/mrs_crazyflies/startup/
./start.sh
```
In 3rd terminal (bottom):
```bash
# Run
ros2 run mrs_consensus rendezvous_node
```
