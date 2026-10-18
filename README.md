# Wall Follower Submission

| | |
|---|---|
| **Video (unlisted YouTube)** | [youtube.com/watch?v=mB_asxr3uaQ](https://www.youtube.com/watch?v=mB_asxr3uaQ) |
| **Fastest lap (in the video)** | 13.5 s |
| **Fastest lap (observed, not recorded)** | 9.5 s, reached later in a longer run that was not captured on video |

Simulator HUD after that longer run (25 laps, best lap 9.5 s):

![Simulator HUD showing a best lap of 9.5 s after 25 laps](docs/best_lap_hud.png)

The main code is in
[`wall_follow_node.py`](autodrive_devkit/autodrive_roboracer/wall_follow_node.py).

## How it works

The car uses its LiDAR like a set of distance sensors around it.

1. It checks how far away the walls are on the left and right.
2. It steers toward the middle so it does not get too close to either wall.
3. It also looks diagonally ahead. This lets it notice a corner and start
   turning before it reaches the wall.
4. It drives faster when the road ahead is clear and slows down for corners.
5. If it gets too close to something, it brakes and turns toward the side with
   more room.

The car's speed comes from odometry, so the node can compare its current speed
with the speed it wants and brake when needed.

## Running it

With the bridge running, build the package inside the container:

```bash
cd /home/autodrive_devkit
colcon build
source install/setup.bash
```

Then run the wall follower:

```bash
ros2 run autodrive_roboracer wall_follow_node
```

I changed the speeds from the command line while testing. For example:

```bash
ros2 run autodrive_roboracer wall_follow_node --ros-args \
  -p v_max:=4.0 -p v_corner:=1.5 -p a_brake:=3.0
```

- `v_max` is the speed on straights.
- `v_corner` is the speed in hard turns.
- A smaller `a_brake` value makes the car start braking earlier.

---

<details>
<summary>Original AutoDRIVE documentation</summary>

# AutoDRIVE-RoboRacer Sim-Racing

![Github Stars](https://img.shields.io/github/stars/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing?style=flat&color=blue&label=stars&logo=github&logoColor=white)
![Github Forks](https://img.shields.io/github/forks/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing?style=flat&color=blue&label=forks&logo=github&logoColor=white)
![Github Downloads](https://img.shields.io/github/downloads/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing/total?style=flat&color=blue&label=downloads&logo=github&logoColor=white)
![Sim Docker Stars](https://img.shields.io/docker/stars/autodriveecosystem/autodrive_roboracer_sim?style=flat&color=blue&label=sim-stars&logo=docker&logoColor=white)
![Sim Docker Pulls](https://img.shields.io/docker/pulls/autodriveecosystem/autodrive_roboracer_sim?style=flat&color=blue&label=sim-pulls&logo=docker&logoColor=white)
![API Docker Stars](https://img.shields.io/docker/stars/autodriveecosystem/autodrive_roboracer_api?style=flat&color=blue&label=api-stars&logo=docker&logoColor=white)
![API Docker Pulls](https://img.shields.io/docker/pulls/autodriveecosystem/autodrive_roboracer_api?style=flat&color=blue&label=api-pulls&logo=docker&logoColor=white)


RoboRacer Digital Twin Autonomous Sim-Racing League using AutoDRIVE Ecosystem

> [!NOTE]
> - The setup has been only tested on the [Ubuntu](https://ubuntu.com) operating system.
> - It is assumed that [Docker](https://docs.docker.com/engine/install) is installed.
> - It is assumed that if the Docker container is to take advantage of an NVIDIA GPU, the host machine has been properly configured by installing the necessary NVIDIA GPU drivers and the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/index.html).

## AutoDRIVE Simulator Container

> [!NOTE]
> The simulator executable must be placed inside [`/autodrive_simulator`](/autodrive_simulator) directory. This repository ships with an examplar working simulator executable, which can be updated as necessary.

### Build:

```bash
docker build --tag autodriveecosystem/autodrive_roboracer_sim:<TAG> -f autodrive_simulator.Dockerfile .
```

### Run:

```bash
xhost local:root
docker run --name autodrive_roboracer_sim --rm -it --entrypoint /bin/bash --network=host --ipc=host -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env DISPLAY --privileged --gpus all autodriveecosystem/autodrive_roboracer_sim:<TAG>
```

### Push:

1. Run the image you created in the previous step inside a container:
```bash
xhost local:root
docker run --name autodrive_roboracer_sim --rm -it --entrypoint /bin/bash --network=host --ipc=host -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env DISPLAY --privileged --gpus all autodriveecosystem/autodrive_roboracer_sim:<TAG>
```

2. In a new terminal window, list all containers and make a note of the desired `CONTAINER ID`:
```bash
docker ps -a
```

3. Commit changes to Docker Hub:
```bash
docker commit -m "AutoDRIVE-RoboRacer-Sim-Racing" -a "AutoDRIVE Ecosystem" <CONTAINER ID> autodriveecosystem/autodrive_roboracer_sim:<TAG>
```

4. Login to Docker Hub:
```bash
docker login
```

5. Push the container to Docker Hub, once done, you should be able to see your repository on Docker Hub:
```bash
docker push autodriveecosystem/autodrive_roboracer_sim:<TAG>
```

## AutoDRIVE Devkit Container

> [!NOTE]
> The devkit package(s) must be placed inside [`/autodrive_devkit`](/autodrive_devkit) directory. This repository ships with an examplar working devkit package, which can be updated as necessary.

### Build:

```bash
docker build --tag autodriveecosystem/autodrive_roboracer_api:<TAG> -f autodrive_devkit.Dockerfile .
```

### Run:

```bash
xhost local:root
docker run --name autodrive_roboracer_api --rm -it --entrypoint /bin/bash --network=host --ipc=host -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env DISPLAY --privileged --gpus all autodriveecosystem/autodrive_roboracer_api:<TAG>
```

### Push:

1. Run the image you created in the previous step inside a container:
```bash
xhost local:root
docker run --name autodrive_roboracer_api --rm -it --entrypoint /bin/bash --network=host --ipc=host -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env DISPLAY --privileged --gpus all autodriveecosystem/autodrive_roboracer_api:<TAG>
```

2. In a new terminal window, list all containers and make a note of the desired `CONTAINER ID`:
```bash
docker ps -a
```

3. Commit changes to Docker Hub:
```bash
docker commit -m "AutoDRIVE-RoboRacer-Sim-Racing" -a "AutoDRIVE Ecosystem" <CONTAINER ID> autodriveecosystem/autodrive_roboracer_api:<TAG>
```

4. Login to Docker Hub:
```bash
docker login
```

5. Push the container to Docker Hub, once done, you should be able to see your repository on Docker Hub:
```bash
docker push autodriveecosystem/autodrive_roboracer_api:<TAG>
```

## Competition Execution

1. Pull the team's image:
```bash
docker pull <TEAM_USERNAME>/<IMAGE_NAME>:<TAG>
```

2. Run the team's image you pulled in the previous step inside a container:
```bash
xhost local:root
docker run --name autodrive_roboracer_api --rm -it --entrypoint /bin/bash --network=host --ipc=host -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env DISPLAY --privileged --gpus all <TEAM_USERNAME>/<IMAGE_NAME>:<TAG>
```

3. Run the simulator image inside a container, set graphics to `Ultra` quality and vehicle in `Autonomous` mode:
```bash
xhost local:root
docker run --name autodrive_roboracer_sim --rm -it --entrypoint /bin/bash --network=host --ipc=host -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env DISPLAY --privileged --gpus all autodriveecosystem/autodrive_roboracer_sim:<TAG>
```

4. Begin screen recording (tested with [OBS Studio](https://obsproject.com)).

5. Execute a new bash session within the team's container, launch `rqtgraph` for inspection and start recording all data streams:
```bash
docker exec -it autodrive_roboracer_api bash
ros2 bag record -a -o qualification.bag
ros2 bag record -a -o competition.bag
rqtgraph
```

6. Enable the communication bridge between simulator and devkit to start the race.

7. After the race completion, kill the `rqtgraph`, data recording as well as screen recording processes, and copy the `rosgraph` and `rosbag` file to the host workstation:
```bash
docker cp autodrive_roboracer_api:/home/autodrive_devkit/qualification.bag /home/<USERNAME>
docker cp autodrive_roboracer_api:/home/autodrive_devkit/competition.bag /home/<USERNAME>
docker cp autodrive_roboracer_api:/home/autodrive_devkit/rosgraph.png /home/<USERNAME>
```

## Generally Helpful Docker Tips

1. To access the container while it is running, execute the following command in a new terminal window to start a new bash session inside the container:
```bash
docker exec -it <CONTAINER NAME> bash
```

2. To exit the bash session(s), simply execute:
```bash
exit
```

3. To kill the container, execute the following command:
```bash
docker kill <CONTAINER NAME>
```

4. To remove the container, simply execute:
```bash
docker rm <CONTAINER NAME>
```

5. Running or caching multiple docker images, containers, volumes, and networks can quickly consume a lot of disk space. Hence, it is always a good idea to frequently check Docker disk utilization:
```bash
docker system df
```

6. To avoid utilizing a lot of disk space, it is a good idea to frequently purge docker resources such as images, containers, volumes, and networks that are unused or dangling (i.e. not tagged or associated with a container). There are several ways with many options to achieve this, please refer to appropriate documentation. The easiest way (but a potentially dangerous one) is to use a single command to clean up all the docker resources (dangling or otherwise):
```bash
docker system prune -a
```

7. After Docker Desktop is installed, Docker CLI commands are by default forwarded to Docker Desktop instead of Docker Engine, and hence you cannot connect to the Docker daemon without running Docker Desktop. In order to avoid this, just switch to the `default` Docker context:
```bash
docker context ls
docker context use default
```
> [!NOTE]
> It is not recommended to use Docker Desktop on the Linux operating system. This is because Docker Desktop creates a virtual machine based on Linux, which is first of all not needed for native (host) Linux OS, and secondly, it sometimes does not expose the necessary access ports for the containers (e.g., trouble with GPU access).

## Citation

We encourage you to read and cite the following papers if you use any part of the competition framework for your research:

#### [AutoDRIVE: A Comprehensive, Flexible and Integrated Digital Twin Ecosystem for Enhancing Autonomous Driving Research and Education](https://arxiv.org/abs/2212.05241)
```bibtex
@article{AutoDRIVE-Ecosystem-2023,
author = {Samak, Tanmay and Samak, Chinmay and Kandhasamy, Sivanathan and Krovi, Venkat and Xie, Ming},
title = {AutoDRIVE: A Comprehensive, Flexible and Integrated Digital Twin Ecosystem for Autonomous Driving Research & Education},
journal = {Robotics},
volume = {12},
year = {2023},
number = {3},
article-number = {77},
url = {https://www.mdpi.com/2218-6581/12/3/77},
issn = {2218-6581},
doi = {10.3390/robotics12030077}
}
```
This work has been published in **MDPI Robotics.** The open-access publication can be found on [MDPI](https://doi.org/10.3390/robotics12030077).

#### [AutoDRIVE Simulator: A Simulator for Scaled Autonomous Vehicle Research and Education](https://arxiv.org/abs/2103.10030)
```bibtex
@inproceedings{AutoDRIVE-Simulator-2021,
author = {Samak, Tanmay Vilas and Samak, Chinmay Vilas and Xie, Ming},
title = {AutoDRIVE Simulator: A Simulator for Scaled Autonomous Vehicle Research and Education},
year = {2021},
isbn = {9781450390453},
publisher = {Association for Computing Machinery},
address = {New York, NY, USA},
url = {https://doi.org/10.1145/3483845.3483846},
doi = {10.1145/3483845.3483846},
booktitle = {2021 2nd International Conference on Control, Robotics and Intelligent System},
pages = {1–5},
numpages = {5},
location = {Qingdao, China},
series = {CCRIS'21}
}
```
This work has been published at **2021 International Conference on Control, Robotics and Intelligent System (CCRIS).** The publication can be found on [ACM Digital Library](https://dl.acm.org/doi/abs/10.1145/3483845.3483846).

</details>
