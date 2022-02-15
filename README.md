# ABC-iRobotics base ROS docker image

A simple way to start working with ROS noetic and UR robots using Docker.


## Install:

Use the docker command line interface to get the latest built docker image:

```bash
docker pull ghcr.io/abc-irobotics/ros_base:main
```


## Usage:

After pulling the docker image run it in interactive mode by:

```docker run -it IMAGE_NAME```

With this you should already be able to use the simulated UR robots with ROS.

### Building your own ROS packages:
The default working directory is `/usr/catkin_ws`. To use your custom ROS packages inside the container, mount a folder containing the source code of your ROS packages on your host machine to the source folder of this working directory like so:

```bash
docker run -it -v PATH/TO/FOLDER/WITH/ROS_PACKAGES:/usr/catkin_ws/src IMAGE_NAME
```
After running the container like this, you should see the contents of the `PATH/TO/FOLDER/WITH/ROS_PACKAGES` inside the container.
Use`catkin build` to build your ROS packages from source. After running `source devel/setup.bash` you should be able to use your own ROS packages inside the container.


### Using the real UR robots:

For using the real UR robots with this container, first follow the instructions from the Universal Robot ROS Driver repository on [how to set up your robot](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_e_series.md). In order to communicate with the real robot the ports 50001-50003 should be bound to the host's respective ports. To do this run the container like so:

```bash
docker run -it -p 50001-50003:50001-50003 IMAGE_NAME
```

After this start the driver, setting the robot ip to the IP of your robot and reverse IP to the IP of your **HOST** machine:
```bash
roslaunch ur_robot_driver <robot_type>_bringup.launch robot_ip:=192.168.56.101 reverse_ip:=192.168.56.123
```

Setting the reverse IP is needed because the driver runs from the docker container and without explicitly specifying this, the robot will not be able to communicate with the driver!

If the driver is running, start the robot program on your robot that contains the ExternalControl URCap. The robot should connect to the driver and in the driver's terminal a message should appear telling that the robot is ready to recieve control commands. (To actually move the robot don't forget to set it to **remote control** mode!)


## Environment specs:

Inside the container there is a working ros-noetic install, and a pre-built catkin workspace (in /usr/underlay_ws). The ROS packages required for moving simulated or real (UR) robots are already installed inside the docker container.

### Minimal RViz-like GUI for windows:

Since on Windows you can't use GUIs from docker, a web interface is provided instead of the RViz GUI. For this purpose, the [rvizweb](https://github.com/karolyartur/rvizweb) package is used. It is already pre-installed in the container. For the web interface the 8001 and 9090 ports have to be bound to the host, so run the container like so:

```bash
docker run -it -p 8001:8001 -P 9090:9090 IMAGE_NAME
```

To use rvizweb simply run:
```bash
roslaunch rvizweb rvizweb.launch
```

inside the container. Then in the host navigate to `localhost/rvizweb/www/index.html` in the browser.

If you run
```bash
roslaunch ur16e_moveit_config demo.launch use_rviz=false
```

and add the Robot model panel in the RVizWeb browser window, you should already be able to see the simulated robot.