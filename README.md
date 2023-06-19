

# kortex2_controllers
Package defining custom controllers created for kinova robots.
- `kortex2_controllers::TwistController`
- `kortex2_controllers::FaultController`


## TwistController
Cartesian twist controller created to use builtin controller on the kinova robot.

### Usage
Example of instance within `ros2_control` can be found in the [configuration file](../kortex2_bringup/config/kortex_controllers.yaml).
Defined interfaces have to exist within the `hardware_interface::SystemInterface` implemented for the robot (e.g. driver).
Stop any commanding controller and then start `TwistController`to use kinova's builtin cartesian twist controller.

#### Note on controllers
Exclusiveness logic is within the driver, and running joint-based and twist controller at the same time is not possible.
Controller Manager (`ros2_control`) will try to activate both on request, but the hardware should reject it.

```
ros2 control switch_controllers --activate streaming_controller --deactivate joint_trajectory_controller
```

By publishing `geometry_msgs::msg::TwistStamped` to `/streaming_controller/commands`, the commands will be piped to the driver
and streamed to the robot controller.

## FaultController
Controller used to reset faults on the kinova arm.

### Usage
Example of instance within `ros2_control` can be found in the [configuration file](../kortex2_bringup/config/kortex_controllers.yaml).
Interfaces are already hardcoded in the `hardware_interface::SystemInterface` implemented for the robot (e.g. driver)

Spawn `FaultController` if it is not already spawned

```
ros2 run controller_manager spawner fault_controller
```

On the topic `/fault_controller/internal_fault` of type `example_interfaces::msg::Bool` the
information if robot is currently faulted can be found.

Deactivate all the controllers using robot.
```
ros2 control swithc_controllers --deactivate joint_trajectory_controller gripper_controller
```

Call the service for resetting the fault (`example_interfaces::srv::Trigger`).

```
ros2 service call /fault_controller/reset_fault example_interfaces/srv/Trigger
```

Activate controllers to enable robot control.
```
ros2 control swithc_controllers --activate joint_trajectory_controller gripper_controller
```

## Build status



ROS2 Distro | Branch | Build status | Documentation | Released packages
:---------: | :----: | :----------: | :-----------: | :---------------:
**Rolling** | [`rolling`](https://github.com/PickNikRobotics/picknik_controllers/tree/rolling) | [![Rolling Binary Build](https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/rolling-binary-build-main.yml/badge.svg?branch=main)](https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/rolling-binary-build-main.yml?branch=main) <br /> [![Rolling Semi-Binary Build](https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/rolling-semi-binary-build-main.yml/badge.svg?branch=main)](https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/rolling-semi-binary-build-main.yml?branch=main) | [![Doxygen Doc Deployment](https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/doxygen-deploy.yml/badge.svg)](https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/doxygen-deploy.yml) <br /> [Generated Doc](https://PickNikRobotics.github.io/picknik_controllers_Documentation/rolling/html/index.html) | [picknik_controllers](https://index.ros.org/p/picknik_controllers/#rolling)


ROS2 Distro | Branch | Build status | Documentation | Released packages
:---------: | :----: | :----------: | :-----------: | :---------------:
**Humble** | [`humble`](https://github.com/PickNikRobotics/picknik_controllers/tree/humble) | [![Humble Binary Build](https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/humble-binary-build-main.yml/badge.svg?branch=main)](https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/humble-binary-build-main.yml?branch=main) <br /> [![Humble Semi-Binary Build](https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/humble-semi-binary-build-main.yml/badge.svg?branch=main)](https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/humble-semi-binary-build-main.yml?branch=main) | [![Doxygen Doc Deployment](https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/doxygen-deploy.yml/badge.svg)](https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/doxygen-deploy.yml) <br /> [Generated Doc](https://PickNikRobotics.github.io/picknik_controllers_Documentation/humble/html/index.html) | [picknik_controllers](https://index.ros.org/p/picknik_controllers/#humble)


ROS2 Distro | Branch | Build status | Documentation | Released packages
:---------: | :----: | :----------: | :-----------: | :---------------:
**Iron** | [`iron`](https://github.com/PickNikRobotics/picknik_controllers/tree/iron) | [![Iron Binary Build](https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/iron-binary-build-main.yml/badge.svg?branch=main)](https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/iron-binary-build-main.yml?branch=main) <br /> [![Iron Semi-Binary Build](https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/iron-semi-binary-build-main.yml/badge.svg?branch=main)](https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/iron-semi-binary-build-main.yml?branch=main) | [![Doxygen Doc Deployment](https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/doxygen-deploy.yml/badge.svg)](https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/doxygen-deploy.yml) <br /> [Generated Doc](https://PickNikRobotics.github.io/picknik_controllers_Documentation/iron/html/index.html) | [picknik_controllers](https://index.ros.org/p/picknik_controllers/#iron)

### Explanation of different build types

**NOTE**: There are three build stages checking current and future compatibility of the package.

[Detailed build status](.github/workflows/README.md)

1. Binary builds - against released packages (main and testing) in ROS distributions. Shows that direct local build is possible.

   Uses repos file: `$NAME$-not-released.<ros-distro>.repos`

1. Semi-binary builds - against released core ROS packages (main and testing), but the immediate dependencies are pulled from source.
   Shows that local build with dependencies is possible and if fails there we can expect that after the next package sync we will not be able to build.

   Uses repos file: `$NAME$.repos`

1. Source build - also core ROS packages are build from source. It shows potential issues in the mid future.
