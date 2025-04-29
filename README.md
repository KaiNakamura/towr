# towr contact sequence planner

<img align="right" src="https://i.imgur.com/qI1Jfyl.gif" width="55%"/>

[<img src="https://i.imgur.com/qliQVx1.png" />](https://ieeexplore.ieee.org/document/8283570 "Go to RA-L paper")

_A light-weight and extensible C++ library for trajectory optimization for legged robots._

## Setup

Clone this repo.

#### Build the docker container

Due to the complex list of dependencies, we have setup a docker container for this project. Make sure you have the correct [Docker Desktop](https://www.docker.com/products/docker-desktop/) installed for your system

This project has been setup to easily install and run in VSCode using the [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) plugin.

In VSCode, enter the command <strong>`> Dev Containers: Reopen in Container`</strong> and it should do pretty much everything for you.

There are ways to do the same thing using only Docker Compose files, but you'll want to be careful to use the correct `args`, `workspaceFolder`, `workspaceMount`, `mounts`, and `DISPLAY` as prescribed in the devcontainer. Note that some of these may need to be changed for Linux systems

#### Build the source code

```bash
cd /home/catkin_ws
catkin build fpowr
```

#### Run the project

Depending on your computer setup, and what your `DISPLAY` variable is set to, you may need to install some form of [X Server](https://vcxsrv.com/) and launch that before running the code to get GUI output. The included devcontainer.json has the `DISPLAY` variable set up for this. On some machines you can get away with `"DISPLAY": "0"` and avoid having to set this up.

In one terminal, first:

```bash
cd /home/catkin_ws
roslaunch fpowr fpowr.launch
```

In another terminal, after the above:

```bash
cd /home/catkin_ws
rosrun fpowr footstep_plan_client
```

## Original towr docs

[here](./towr_readme.md)
