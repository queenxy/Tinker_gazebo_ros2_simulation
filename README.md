# tinker_gazebo_sim
Tinker simulation based on ROS2 and gazebo11, adapted from [tinker_nav_sim](https://github.com/tinkerfuroc/tinker_nav_sim)

## Setup
1. Prepration: 
2. Install all dependecies(You should setup `rosdep` first following official documentation):
    ```sh
    rosdep install --from-paths src -y --ignore-src
    ```
3. Install `gazebo_ros` package
    ```sh
    sudo apt install ros-humble-gazebo-ros-pkgs
    ```
4. Install `ros2 control` dependency
    ```sh
    sudo apt-get install ros-humble-ros2-control
    sudo apt-get install ros-humble-ros2-controllers
    sudo apt-get install ros-humble-gazebo-ros2-control
    ```
5. Install `Universal_Robots_ROS2_Driver` package
    ```sh
    sudo apt-get install ros-humble-ur
    ```
5. If you need to do navigation simulation, install `nav2` dependency
    ```sh
    sudo apt install ros-humble-navigation2
    sudo apt install ros-humble-nav2-bringup
    sudo apt install ros-humble-robot-localization
    sudo apt install ros-humble-slam-toolbox
    ```

## Run

### Test Tinker Model
All terminals should `source` the `setup.bash`(`setup.zsh`) in workspace first.

Be sure you are using `gnome-terminal` rather than `xterm`
#### Visualization
Launch the visual model of tinker, which shows the RobotModel

```sh
ros2 launch tinker_sim display.launch.py
```
#### Test Gazebo Simulation
Launch the robot and controller, which test the robot and chassis controller

```sh
ros2 launch tinker_sim gazebo_sim.launch.py
```
Set `use_teleop:=True` to control the base by keyboard

Then run the following command in another ternimal to control the arm by moveit:
```sh
ros2 launch tinker_sim arm_moveit.launch.py
```
The UR simulation refers https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation
To move it to Tinker, please change add the prefix "ur_" to all the controllers yaml except joint_limits.yaml in ```config/ur5```. And set ```use_fake_hardware:=true``` in simulation.

The controller shaking problem and broken joints can be solved in https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/issues/11 and https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/issues/19

```
<joint name="${prefix}wrist_3_joint" type="revolute">
  [...]
  <dynamics damping="0.7" friction="1.0"/>
</joint>

<gazebo reference="${prefix}wrist_3_joint">
  <implicitSpringDamper>true</implicitSpringDamper>
</gazebo>
```

### Nav Simulation
#### Use Lidar to setup map
Launch slam toolbox and localization
```sh
ros2 launch navigation_sim gazebo_robot.launch.py headless:=False use_teleop:=True
```
#### Test navigation by using rivz to publihs a goal
Launch navigation stack
```sh
ros2 launch navigation_sim navigation.launch.py headless:=False 
```
#### Run Navigation
Set 2D pose first, then set a goalpose in rviz.

