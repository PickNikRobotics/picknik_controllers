# picknik_controllers

Package defining custom controllers that are generic and we want to contribute them into ros2_controllers.
The controllers have been prefixed with `picknik_` so that when they eventually land upstream the prefix can be removed and the versions here will become deprecated.

- `picknik_twist_controller::PicknikTwistController`
- `picknik_reset_fault_controller::PicknikResetFaultController`


## PicknikTwistController
Generic ROS2 controller to forward cartesian twist commands.
We plan to land this upstream in ros2_controllers: https://github.com/ros-controls/ros2_controllers/pull/300

### Usage
Example of instance within `ros2_control` can be found in the [configuration file](https://github.com/PickNikRobotics/ros2_kortex/blob/main/kortex_description/arms/gen3/7dof/config/ros2_controllers.yaml).
Defined interfaces have to exist within the `hardware_interface::SystemInterface` implemented for the robot (e.g. driver).
Stop any commanding controller and then start `PicknikTwistController`to forward twist msgs to the hardware's built-in cartesian twist controller.

#### Note on controllers
Exclusiveness logic is within the driver, and running joint-based and twist controller at the same time is not possible.
Controller Manager (`ros2_control`) will try to activate both on request, but the hardware should reject it.

```
ros2 control switch_controllers --activate streaming_controller --deactivate joint_trajectory_controller
```

By publishing `geometry_msgs::msg::Twist` to `/streaming_controller/commands`, the commands will be piped to the driver
and streamed to the robot controller.

## PicknikResetFaultController
ROS 2 controller that offers a service to clear faults in a hardware interface

### Usage
Example of instance within `ros2_control` can be found in the [configuration file](https://github.com/PickNikRobotics/ros2_kortex/blob/main/kortex_description/arms/gen3/7dof/config/ros2_controllers.yaml).
Interfaces are already hardcoded in the `hardware_interface::SystemInterface` implemented for the robot (e.g. driver)

Spawn `FaultController` if it is not already spawned

```
ros2 run controller_manager spawner fault_controller
```

On the topic `/fault_controller/internal_fault` of type `example_interfaces::msg::Bool` the
information if robot is currently faulted can be found.

Deactivate all the controllers using robot.
```
ros2 control switch_controllers --deactivate joint_trajectory_controller gripper_controller
```

Call the service for resetting the fault (`example_interfaces::srv::Trigger`).

```
ros2 service call /fault_controller/reset_fault example_interfaces/srv/Trigger
```

Activate controllers to enable robot control.
```
ros2 control switch_controllers --activate joint_trajectory_controller gripper_controller
```

## [Release repository](https://github.com/ros2-gbp/picknik_controllers-release) Status

<!-- https://repo.ros2.org/status_page/ros_rolling_default.html?q=picknik_twist_controller -->

| Package | [Rolling][r_rel_summary] | [Jazzy][j_rel_summary] | [Kilted][k_rel_summary] | [Humble][h_rel_summary] |
|:-------:|:---------------------:|:-------------------:|:--------------------:|:--------------------:|
| [picknik_reset_fault_controller](https://index.ros.org/p/picknik_reset_fault_controller) | [![Release Src Status][Rsrc__prfc_b]][Rsrc__prfc] <br /> [![Release x86_64 Status][Rbin_64__prfc_b]][Rbin_64__prfc] <br /> [![Release arm64 Status][Rbin_v8__prfc_b]][Rbin_v8__prfc] <br /> [Docs][r_prfc_docs] | [![Release Src Status][Jsrc__prfc_b]][Jsrc__prfc] <br /> [![Release x86_64 Status][Jbin_64__prfc_b]][Jbin_64__prfc] <br /> [![Release arm64 Status][Jbin_v8__prfc_b]][Jbin_v8__prfc] <br /> [Docs][j_prfc_docs] | [![Release Src Status][Ksrc__prfc_b]][Ksrc__prfc] <br /> [![Release x86_64 Status][Kbin_64__prfc_b]][Kbin_64__prfc] <br /> [![Release arm64 Status][Kbin_v8__prfc_b]][Kbin_v8__prfc] <br /> [Docs][k_prfc_docs] | [![Release Src Status][Hsrc__prfc_b]][Hsrc__prfc] <br /> [![Release x86_64 Status][Hbin_64__prfc_b]][Hbin_64__prfc] <br /> [![Release arm64 Status][Hbin_v8__prfc_b]][Hbin_v8__prfc] <br /> [Docs][h_prfc_docs] |
| [picknik_twist_controller](https://index.ros.org/p/picknik_twist_controller) | [![Release Src Status][Rsrc__ptc_b]][Rsrc__ptc] <br /> [![Release x86_64 Status][Rbin_64__ptc_b]][Rbin_64__ptc] <br /> [![Release arm64 Status][Rbin_v8__ptc_b]][Rbin_v8__ptc] <br /> [Docs][r_ptc_docs] | [![Release Src Status][Jsrc__ptc_b]][Jsrc__ptc] <br /> [![Release x86_64 Status][Jbin_64__ptc_b]][Jbin_64__ptc] <br /> [![Release arm64 Status][Jbin_v8__ptc_b]][Jbin_v8__ptc] <br />  [Docs][j_ptc_docs]| [![Release Src Status][Ksrc__ptc_b]][Ksrc__ptc] <br /> [![Release x86_64 Status][Kbin_64__ptc_b]][Kbin_64__ptc] <br /> [![Release arm64 Status][Kbin_v8__ptc_b]][Kbin_v8__ptc] <br /> [Docs][k_ptc_docs] | [![Release Src Status][Hsrc__ptc_b]][Hsrc__ptc] <br /> [![Release x86_64 Status][Hbin_64__ptc_b]][Hbin_64__ptc] <br /> [![Release arm64 Status][Hbin_v8__ptc_b]][Hbin_v8__ptc]  <br /> [Docs][r_ptc_docs] |

<!-- repo.ros2.org pages -->
[r_rel_summary]: https://repo.ros2.org/status_page/ros_rolling_default.html?q=picknik_reset_fault_controller%7Cpicknik_twist_controller
[j_rel_summary]: https://repo.ros2.org/status_page/ros_jazzy_default.html?q=picknik_reset_fault_controller%7Cpicknik_twist_controller
[k_rel_summary]: https://repo.ros2.org/status_page/ros_kilted_default.html?q=picknik_reset_fault_controller%7Cpicknik_twist_controller
[h_rel_summary]: https://repo.ros2.org/status_page/ros_humble_default.html?q=picknik_reset_fault_controller%7Cpicknik_twist_controller

<!-- docs.ros.org pages -->
[r_prfc_docs]: https://docs.ros.org/en/rolling/p/picknik_reset_fault_controller
[j_prfc_docs]: https://docs.ros.org/en/jazzy/p/picknik_reset_fault_controller
[k_prfc_docs]: https://docs.ros.org/en/kilted/p/picknik_reset_fault_controller
[h_prfc_docs]: https://docs.ros.org/en/humble/p/picknik_reset_fault_controller
[r_ptc_docs]: https://docs.ros.org/en/rolling/p/picknik_twist_controller
[j_ptc_docs]: https://docs.ros.org/en/jazzy/p/picknik_twist_controller
[k_ptc_docs]: https://docs.ros.org/en/kilted/p/picknik_twist_controller
[h_ptc_docs]: https://docs.ros.org/en/humble/p/picknik_twist_controller

<!-- picknik_reset_fault_controller statuses -->
[Rsrc__prfc]: https://build.ros2.org/job/Rsrc_uN__picknik_reset_fault_controller__ubuntu_noble__source
[Rbin_64__prfc]: https://build.ros2.org/job/Rbin_uN64__picknik_reset_fault_controller__ubuntu_noble_amd64__binary
[Rbin_v8__prfc]: https://build.ros2.org/job/Rbin_unv8_uNv8__picknik_reset_fault_controller__ubuntu_noble_arm64__binary
[Jsrc__prfc]: https://build.ros2.org/job/Jsrc_uN__picknik_reset_fault_controller__ubuntu_noble__source
[Jbin_64__prfc]: https://build.ros2.org/job/Jbin_uN64__picknik_reset_fault_controller__ubuntu_noble_amd64__binary
[Jbin_v8__prfc]: https://build.ros2.org/job/Jbin_unv8_uNv8__picknik_reset_fault_controller__ubuntu_noble_arm64__binary
[Ksrc__prfc]: https://build.ros2.org/job/Ksrc_uN__picknik_reset_fault_controller__ubuntu_noble__source
[Kbin_64__prfc]: https://build.ros2.org/job/Kbin_uN64__picknik_reset_fault_controller__ubuntu_noble_amd64__binary
[Kbin_v8__prfc]: https://build.ros2.org/job/Kbin_unv8_uNv8__picknik_reset_fault_controller__ubuntu_noble_arm64__binary
[Hsrc__prfc]: https://build.ros2.org/job/Hsrc_uJ__picknik_reset_fault_controller__ubuntu_jammy__source
[Hbin_64__prfc]: https://build.ros2.org/job/Hbin_uJ64__picknik_reset_fault_controller__ubuntu_jammy_amd64__binary
[Hbin_v8__prfc]: https://build.ros2.org/job/Hbin_ujv8_uJv8__picknik_reset_fault_controller__ubuntu_jammy_arm64__binary
<!-- picknik_reset_fault_controller status badges -->
[Rsrc__prfc_b]: https://build.ros2.org/buildStatus/icon?job=Rsrc_uN__picknik_reset_fault_controller__ubuntu_noble__source&subject=src
[Rbin_64__prfc_b]: https://build.ros2.org/buildStatus/icon?job=Rbin_uN64__picknik_reset_fault_controller__ubuntu_noble_amd64__binary&subject=x86_64_bin
[Rbin_v8__prfc_b]: https://build.ros2.org/buildStatus/icon?job=Rbin_unv8_uNv8__picknik_reset_fault_controller__ubuntu_noble_arm64__binary&subject=arm64_bin
[Jsrc__prfc_b]: https://build.ros2.org/buildStatus/icon?job=Jsrc_uN__picknik_reset_fault_controller__ubuntu_noble__source&subject=src
[Jbin_64__prfc_b]: https://build.ros2.org/buildStatus/icon?job=Jbin_uN64__picknik_reset_fault_controller__ubuntu_noble_amd64__binary&subject=x86_64_bin
[Jbin_v8__prfc_b]: https://build.ros2.org/buildStatus/icon?job=Jbin_unv8_uNv8__picknik_reset_fault_controller__ubuntu_noble_arm64__binary&subject=arm64_bin
[Ksrc__prfc_b]: https://build.ros2.org/buildStatus/icon?job=Ksrc_uN__picknik_reset_fault_controller__ubuntu_noble__source&subject=src
[Kbin_64__prfc_b]: https://build.ros2.org/buildStatus/icon?job=Kbin_uN64__picknik_reset_fault_controller__ubuntu_noble_amd64__binary&subject=x86_64_bin
[Kbin_v8__prfc_b]: https://build.ros2.org/buildStatus/icon?job=Kbin_unv8_uNv8__picknik_reset_fault_controller__ubuntu_noble_arm64__binary&subject=arm64_bin
[Hsrc__prfc_b]: https://build.ros2.org/buildStatus/icon?job=Hsrc_uJ__picknik_reset_fault_controller__ubuntu_jammy__source&subject=src
[Hbin_64__prfc_b]: https://build.ros2.org/buildStatus/icon?job=Hbin_uJ64__picknik_reset_fault_controller__ubuntu_jammy_amd64__binary&subject=x86_64_bin
[Hbin_v8__prfc_b]: https://build.ros2.org/buildStatus/icon?job=Hbin_ujv8_uJv8__picknik_reset_fault_controller__ubuntu_jammy_arm64__binary&subject=arm64_bin
<!-- picknik_twist_controller statuses -->
[Rsrc__ptc]: https://build.ros2.org/job/Rsrc_uN__picknik_twist_controller__ubuntu_noble__source
[Rbin_64__ptc]: https://build.ros2.org/job/Rbin_uN64__picknik_twist_controller__ubuntu_noble_amd64__binary
[Rbin_v8__ptc]: https://build.ros2.org/job/Rbin_unv8_uNv8__picknik_twist_controller__ubuntu_noble_arm64__binary
[Jsrc__ptc]: https://build.ros2.org/job/Jsrc_uN__picknik_twist_controller__ubuntu_noble__source
[Jbin_64__ptc]: https://build.ros2.org/job/Jbin_uN64__picknik_twist_controller__ubuntu_noble_amd64__binary
[Jbin_v8__ptc]: https://build.ros2.org/job/Jbin_unv8_uNv8__picknik_twist_controller__ubuntu_noble_arm64__binary
[Ksrc__ptc]: https://build.ros2.org/job/Ksrc_uN__picknik_twist_controller__ubuntu_noble__source
[Kbin_64__ptc]: https://build.ros2.org/job/Kbin_uN64__picknik_twist_controller__ubuntu_noble_amd64__binary
[Kbin_v8__ptc]: https://build.ros2.org/job/Kbin_unv8_uNv8__picknik_twist_controller__ubuntu_noble_arm64__binary
[Hsrc__ptc]: https://build.ros2.org/job/Hsrc_uJ__picknik_twist_controller__ubuntu_jammy__source
[Hbin_64__ptc]: https://build.ros2.org/job/Hbin_uJ64__picknik_twist_controller__ubuntu_jammy_amd64__binary
[Hbin_v8__ptc]: https://build.ros2.org/job/Hbin_ujv8_uJv8__picknik_twist_controller__ubuntu_jammy_arm64__binary
<!-- picknik_twist_controller status badges -->
[Rsrc__ptc_b]: https://build.ros2.org/buildStatus/icon?job=Rsrc_uN__picknik_twist_controller__ubuntu_noble__source&subject=src
[Rbin_64__ptc_b]: https://build.ros2.org/buildStatus/icon?job=Rbin_uN64__picknik_twist_controller__ubuntu_noble_amd64__binary&subject=x86_64_bin
[Rbin_v8__ptc_b]: https://build.ros2.org/buildStatus/icon?job=Rbin_unv8_uNv8__picknik_twist_controller__ubuntu_noble_arm64__binary&subject=arm64_bin
[Jsrc__ptc_b]: https://build.ros2.org/buildStatus/icon?job=Jsrc_uN__picknik_twist_controller__ubuntu_noble__source&subject=src
[Jbin_64__ptc_b]: https://build.ros2.org/buildStatus/icon?job=Jbin_uN64__picknik_twist_controller__ubuntu_noble_amd64__binary&subject=x86_64_bin
[Jbin_v8__ptc_b]: https://build.ros2.org/buildStatus/icon?job=Jbin_unv8_uNv8__picknik_twist_controller__ubuntu_noble_arm64__binary&subject=arm64_bin
[Ksrc__ptc_b]: https://build.ros2.org/buildStatus/icon?job=Ksrc_uN__picknik_twist_controller__ubuntu_noble__source&subject=src
[Kbin_64__ptc_b]: https://build.ros2.org/buildStatus/icon?job=Kbin_uN64__picknik_twist_controller__ubuntu_noble_amd64__binary&subject=x86_64_bin
[Kbin_v8__ptc_b]: https://build.ros2.org/buildStatus/icon?job=Kbin_unv8_uNv8__picknik_twist_controller__ubuntu_noble_arm64__binary&subject=arm64_bin
[Hsrc__ptc_b]: https://build.ros2.org/buildStatus/icon?job=Hsrc_uJ__picknik_twist_controller__ubuntu_jammy__source&subject=src
[Hbin_64__ptc_b]: https://build.ros2.org/buildStatus/icon?job=Hbin_uJ64__picknik_twist_controller__ubuntu_jammy_amd64__binary&subject=x86_64_bin
[Hbin_v8__ptc_b]: https://build.ros2.org/buildStatus/icon?job=Hbin_ujv8_uJv8__picknik_twist_controller__ubuntu_jammy_arm64__binary&subject=arm64_bin

## [Source repository](https://github.com/PickNikRobotics/picknik_controllers) Status



[h_bin_main]: 
[h_bin_testing]
[h_semi_bin_main]
[h_semi_bin_testing]
[h_src_testing]


| Dependencies Repository | [Rolling][r_src_url] | [Jazzy][j_src_url] | [Kilted][k_src_url] | [Humble][h_src_url] |
|:-----------------:|:---------------------:|:-------------------:|:--------------------:|:--------------------:|
| Release-Main Bin |
| Release-Testing Bin |
| Mixed Release-Main Bin & Upstream Src |
| Mixed Release-Testing Bin & Upstream Src |
| Upstream Src |


[r_src_url]: https://github.com/PickNikRobotics/picknik_controllers/tree/rolling
[j_src_url]: https://github.com/PickNikRobotics/picknik_controllers/tree/jazzy
[k_src_url]: https://github.com/PickNikRobotics/picknik_controllers/tree/kilted
[h_src_url]: https://github.com/PickNikRobotics/picknik_controllers/tree/humble

| ROS2 Distro | 

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


1. Binary builds - against released packages (main and testing) in ROS distributions. Shows that direct local build is possible.

   Uses repos file: `$NAME$-not-released.<ros-distro>.repos`

1. Semi-binary builds - against released core ROS packages (main and testing), but the immediate dependencies are pulled from source.
   Shows that local build with dependencies is possible and if fails there we can expect that after the next package sync we will not be able to build.

   Uses repos file: `$NAME$.repos`

1. Source build - also core ROS packages are build from source. It shows potential issues in the mid future.
