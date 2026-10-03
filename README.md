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
## [Release Repository](https://github.com/ros2-gbp/picknik_controllers-release) Status

<!-- https://repo.ros2.org/status_page/ros_rolling_default.html?q=picknik_twist_controller -->

| Package | [Rolling][r_rel_summary] | [Jazzy][j_rel_summary] | [Kilted][k_rel_summary] | [Humble][h_rel_summary] |
|:-------:|:---------------------:|:-------------------:|:--------------------:|:--------------------:|
| [picknik_reset_fault_controller](https://index.ros.org/p/picknik_reset_fault_controller) | [![Release Src Status][prfc_r_src_b]][prfc_r_src] <br /> [![Release x86_64 Status][prfc_r_bin_64_b]][prfc_r_bin_64] <br /> [![Release arm64 Status][prfc_r_bin_v8_b]][prfc_r_bin_v8] <br /> [Docs][r_prfc_docs] | [![Release Src Status][prfc_j_src_b]][prfc_j_src] <br /> [![Release x86_64 Status][prfc_j_bin_64_b]][prfc_j_bin_64] <br /> [![Release arm64 Status][prfc_j_bin_v8_b]][prfc_j_bin_v8] <br /> [Docs][j_prfc_docs] | [![Release Src Status][prfc_k_src_b]][prfc_k_src] <br /> [![Release x86_64 Status][prfc_k_bin_64_b]][prfc_k_bin_64] <br /> [![Release arm64 Status][prfc_k_bin_v8_b]][prfc_k_bin_v8] <br /> [Docs][k_prfc_docs] | [![Release Src Status][prfc_h_src_b]][prfc_h_src] <br /> [![Release x86_64 Status][prfc_h_bin_64_b]][prfc_h_bin_64] <br /> [![Release arm64 Status][prfc_h_bin_v8_b]][prfc_h_bin_v8] <br /> [Docs][h_prfc_docs] |
| [picknik_twist_controller](https://index.ros.org/p/picknik_twist_controller) | [![Release Src Status][ptc_r_src_b]][ptc_r_src] <br /> [![Release x86_64 Status][ptc_r_bin_64_b]][ptc_r_bin_64] <br /> [![Release arm64 Status][ptc_r_bin_v8_b]][ptc_r_bin_v8] <br /> [Docs][r_ptc_docs] | [![Release Src Status][ptc_j_src_b]][ptc_j_src] <br /> [![Release x86_64 Status][ptc_j_bin_64_b]][ptc_j_bin_64] <br /> [![Release arm64 Status][ptc_j_bin_v8_b]][ptc_j_bin_v8] <br />  [Docs][j_ptc_docs]| [![Release Src Status][ptc_k_src_b]][ptc_k_src] <br /> [![Release x86_64 Status][ptc_k_bin_64_b]][ptc_k_bin_64] <br /> [![Release arm64 Status][ptc_k_bin_v8_b]][ptc_k_bin_v8] <br /> [Docs][k_ptc_docs] | [![Release Src Status][ptc_h_src_b]][ptc_h_src] <br /> [![Release x86_64 Status][ptc_h_bin_64_b]][ptc_h_bin_64] <br /> [![Release arm64 Status][ptc_h_bin_v8_b]][ptc_h_bin_v8]  <br /> [Docs][r_ptc_docs] |

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

<!-- picknik_reset_fault_controller release CI -->
<!-- Rolling -->
[prfc_r_src]: https://build.ros2.org/job/Rsrc_uN__picknik_reset_fault_controller__ubuntu_noble__source
[prfc_r_src_b]: https://build.ros2.org/buildStatus/icon?job=Rsrc_uN__picknik_reset_fault_controller__ubuntu_noble__source&subject=src
[prfc_r_bin_64]: https://build.ros2.org/job/Rbin_uN64__picknik_reset_fault_controller__ubuntu_noble_amd64__binary
[prfc_r_bin_64_b]: https://build.ros2.org/buildStatus/icon?job=Rbin_uN64__picknik_reset_fault_controller__ubuntu_noble_amd64__binary&subject=x86_64_bin
[prfc_r_bin_v8]: https://build.ros2.org/job/Rbin_unv8_uNv8__picknik_reset_fault_controller__ubuntu_noble_arm64__binary
[prfc_r_bin_v8_b]: https://build.ros2.org/buildStatus/icon?job=Rbin_unv8_uNv8__picknik_reset_fault_controller__ubuntu_noble_arm64__binary&subject=arm64_bin
<!-- Jazzy -->
[prfc_j_src]: https://build.ros2.org/job/Jsrc_uN__picknik_reset_fault_controller__ubuntu_noble__source
[prfc_j_src_b]: https://build.ros2.org/buildStatus/icon?job=Jsrc_uN__picknik_reset_fault_controller__ubuntu_noble__source&subject=src
[prfc_j_bin_64]: https://build.ros2.org/job/Jbin_uN64__picknik_reset_fault_controller__ubuntu_noble_amd64__binary
[prfc_j_bin_64_b]: https://build.ros2.org/buildStatus/icon?job=Jbin_uN64__picknik_reset_fault_controller__ubuntu_noble_amd64__binary&subject=x86_64_bin
[prfc_j_bin_v8]: https://build.ros2.org/job/Jbin_unv8_uNv8__picknik_reset_fault_controller__ubuntu_noble_arm64__binary
[prfc_j_bin_v8_b]: https://build.ros2.org/buildStatus/icon?job=Jbin_unv8_uNv8__picknik_reset_fault_controller__ubuntu_noble_arm64__binary&subject=arm64_bin
<!-- Kilted -->
[prfc_k_src]: https://build.ros2.org/job/Ksrc_uN__picknik_reset_fault_controller__ubuntu_noble__source
[prfc_k_src_b]: https://build.ros2.org/buildStatus/icon?job=Ksrc_uN__picknik_reset_fault_controller__ubuntu_noble__source&subject=src
[prfc_k_bin_64]: https://build.ros2.org/job/Kbin_uN64__picknik_reset_fault_controller__ubuntu_noble_amd64__binary
[prfc_k_bin_64_b]: https://build.ros2.org/buildStatus/icon?job=Kbin_uN64__picknik_reset_fault_controller__ubuntu_noble_amd64__binary&subject=x86_64_bin
[prfc_k_bin_v8]: https://build.ros2.org/job/Kbin_unv8_uNv8__picknik_reset_fault_controller__ubuntu_noble_arm64__binary
[prfc_k_bin_v8_b]: https://build.ros2.org/buildStatus/icon?job=Kbin_unv8_uNv8__picknik_reset_fault_controller__ubuntu_noble_arm64__binary&subject=arm64_bin
<!-- Humble -->
[prfc_h_src]: https://build.ros2.org/job/Hsrc_uJ__picknik_reset_fault_controller__ubuntu_jammy__source
[prfc_h_src_b]: https://build.ros2.org/buildStatus/icon?job=Hsrc_uJ__picknik_reset_fault_controller__ubuntu_jammy__source&subject=src
[prfc_h_bin_64]: https://build.ros2.org/job/Hbin_uJ64__picknik_reset_fault_controller__ubuntu_jammy_amd64__binary
[prfc_h_bin_64_b]: https://build.ros2.org/buildStatus/icon?job=Hbin_uJ64__picknik_reset_fault_controller__ubuntu_jammy_amd64__binary&subject=x86_64_bin
[prfc_h_bin_v8]: https://build.ros2.org/job/Hbin_ujv8_uJv8__picknik_reset_fault_controller__ubuntu_jammy_arm64__binary
[prfc_h_bin_v8_b]: https://build.ros2.org/buildStatus/icon?job=Hbin_ujv8_uJv8__picknik_reset_fault_controller__ubuntu_jammy_arm64__binary&subject=arm64_bin

<!-- picknik_twist_controller release CI -->
<!-- Rolling -->
[ptc_r_src]: https://build.ros2.org/job/Rsrc_uN__picknik_twist_controller__ubuntu_noble__source
[ptc_r_src_b]: https://build.ros2.org/buildStatus/icon?job=Rsrc_uN__picknik_twist_controller__ubuntu_noble__source&subject=src
[ptc_r_bin_64]: https://build.ros2.org/job/Rbin_uN64__picknik_twist_controller__ubuntu_noble_amd64__binary
[ptc_r_bin_64_b]: https://build.ros2.org/buildStatus/icon?job=Rbin_uN64__picknik_twist_controller__ubuntu_noble_amd64__binary&subject=x86_64_bin
[ptc_r_bin_v8]: https://build.ros2.org/job/Rbin_unv8_uNv8__picknik_twist_controller__ubuntu_noble_arm64__binary
[ptc_r_bin_v8_b]: https://build.ros2.org/buildStatus/icon?job=Rbin_unv8_uNv8__picknik_twist_controller__ubuntu_noble_arm64__binary&subject=arm64_bin
<!-- Jazzy -->
[ptc_j_src]: https://build.ros2.org/job/Jsrc_uN__picknik_twist_controller__ubuntu_noble__source
[ptc_j_src_b]: https://build.ros2.org/buildStatus/icon?job=Jsrc_uN__picknik_twist_controller__ubuntu_noble__source&subject=src
[ptc_j_bin_64]: https://build.ros2.org/job/Jbin_uN64__picknik_twist_controller__ubuntu_noble_amd64__binary
[ptc_j_bin_64_b]: https://build.ros2.org/buildStatus/icon?job=Jbin_uN64__picknik_twist_controller__ubuntu_noble_amd64__binary&subject=x86_64_bin
[ptc_j_bin_v8]: https://build.ros2.org/job/Jbin_unv8_uNv8__picknik_twist_controller__ubuntu_noble_arm64__binary
[ptc_j_bin_v8_b]: https://build.ros2.org/buildStatus/icon?job=Jbin_unv8_uNv8__picknik_twist_controller__ubuntu_noble_arm64__binary&subject=arm64_bin
<!-- Kilted -->
[ptc_k_src]: https://build.ros2.org/job/Ksrc_uN__picknik_twist_controller__ubuntu_noble__source
[ptc_k_src_b]: https://build.ros2.org/buildStatus/icon?job=Ksrc_uN__picknik_twist_controller__ubuntu_noble__source&subject=src
[ptc_k_bin_64]: https://build.ros2.org/job/Kbin_uN64__picknik_twist_controller__ubuntu_noble_amd64__binary
[ptc_k_bin_64_b]: https://build.ros2.org/buildStatus/icon?job=Kbin_uN64__picknik_twist_controller__ubuntu_noble_amd64__binary&subject=x86_64_bin
[ptc_k_bin_v8]: https://build.ros2.org/job/Kbin_unv8_uNv8__picknik_twist_controller__ubuntu_noble_arm64__binary
[ptc_k_bin_v8_b]: https://build.ros2.org/buildStatus/icon?job=Kbin_unv8_uNv8__picknik_twist_controller__ubuntu_noble_arm64__binary&subject=arm64_bin
<!-- Humble -->
[ptc_h_src]: https://build.ros2.org/job/Hsrc_uJ__picknik_twist_controller__ubuntu_jammy__source
[ptc_h_src_b]: https://build.ros2.org/buildStatus/icon?job=Hsrc_uJ__picknik_twist_controller__ubuntu_jammy__source&subject=src
[ptc_h_bin_64]: https://build.ros2.org/job/Hbin_uJ64__picknik_twist_controller__ubuntu_jammy_amd64__binary
[ptc_h_bin_64_b]: https://build.ros2.org/buildStatus/icon?job=Hbin_uJ64__picknik_twist_controller__ubuntu_jammy_amd64__binary&subject=x86_64_bin
[ptc_h_bin_v8]: https://build.ros2.org/job/Hbin_ujv8_uJv8__picknik_twist_controller__ubuntu_jammy_arm64__binary
[ptc_h_bin_v8_b]: https://build.ros2.org/buildStatus/icon?job=Hbin_ujv8_uJv8__picknik_twist_controller__ubuntu_jammy_arm64__binary&subject=arm64_bin

## [Source Repository](https://github.com/PickNikRobotics/picknik_controllers) Status

| Dependencies Repository | [Rolling][r_src_url] | [Jazzy][j_src_url] | [Kilted][k_src_url] | [Humble][h_src_url] |
|:-----------------:|:---------------------:|:-------------------:|:--------------------:|:--------------------:|
| Release-Main Bin | [![Rolling][r_bin_main_b]][r_bin_main] | [![Jazzy][j_bin_main_b]][j_bin_main] | [![Kilted][k_bin_main_b]][k_bin_main] | [![Humble][h_bin_main_b]][h_bin_main] |
| Release-Testing Bin | [![Rolling][r_bin_testing_b]][r_bin_testing] | [![Jazzy][j_bin_testing_b]][j_bin_testing] | [![Kilted][k_bin_testing_b]][k_bin_testing] | [![Humble][h_bin_testing_b]][h_bin_testing] |
| Mixed Release: Main Bin & Upstream Src | [![Rolling][r_semi_bin_main_b]][r_semi_bin_main] | [![Jazzy][j_semi_bin_main_b]][j_semi_bin_main] | [![Kilted][k_semi_bin_main_b]][k_semi_bin_main] | [![Humble][h_semi_bin_main_b]][h_bin_main] |
| Mixed Release: Testing Bin & Upstream Src | [![Rolling][r_semi_bin_testing_b]][r_semi_bin_testing] | [![Jazzy][j_semi_bin_testing_b]][j_semi_bin_testing] | [![Kilted][k_semi_bin_testing_b]][k_semi_bin_testing] | [![Humble][h_semi_bin_testing_b]][h_bin_testing] |
| Upstream Src | [![Rolling][r_src_b]][r_src] | [![Jazzy][j_src_b]][j_src] | [![Kilted][k_src_b]][k_src] | [![Humble][h_src_b]][h_src] |

<!-- Source repo -->
[r_src_url]: https://github.com/PickNikRobotics/picknik_controllers/tree/rolling
[j_src_url]: https://github.com/PickNikRobotics/picknik_controllers/tree/jazzy
[k_src_url]: https://github.com/PickNikRobotics/picknik_controllers/tree/kilted
[h_src_url]: https://github.com/PickNikRobotics/picknik_controllers/tree/humble

<!-- Source repository CI -->
<!-- Rolling -->
[r_bin_main]: https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/rolling-binary-build-main.yml
[r_bin_main_b]: https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/rolling-binary-build-main.yml/badge.svg
[r_bin_testing]: https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/rolling-binary-build-testing.yml
[r_bin_testing_b]: https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/rolling-binary-build-testing.yml/badge.svg
[r_semi_bin_main]: https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/rolling-semi-binary-build-main.yml
[r_semi_bin_main_b]: https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/rolling-semi-binary-build-main.yml/badge.svg
[r_semi_bin_testing]: https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/rolling-semi-binary-build-testing.yml
[r_semi_bin_testing_b]: https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/rolling-semi-binary-build-testing.yml/badge.svg
[r_src]: https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/rolling-source-build.yml
[r_src_b]: https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/rolling-source-build.yml/badge.svg
<!-- Jazzy -->
[j_bin_main]: https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/jazzy-binary-build-main.yml
[j_bin_main_b]: https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/jazzy-binary-build-main.yml/badge.svg
[j_bin_testing]: https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/jazzy-binary-build-testing.yml
[j_bin_testing_b]: https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/jazzy-binary-build-testing.yml/badge.svg
[j_semi_bin_main]: https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/jazzy-semi-binary-build-main.yml
[j_semi_bin_main_b]: https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/jazzy-semi-binary-build-main.yml/badge.svg
[j_semi_bin_testing]: https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/jazzy-semi-binary-build-testing.yml
[j_semi_bin_testing_b]: https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/jazzy-semi-binary-build-testing.yml/badge.svg
[j_src]: https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/jazzy-source-build.yml
[j_src_b]: https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/jazzy-source-build.yml/badge.svg
<!-- Kilted -->
[k_bin_main]: https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/kilted-binary-build-main.yml
[k_bin_main_b]: https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/kilted-binary-build-main.yml/badge.svg
[k_bin_testing]: https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/kilted-binary-build-testing.yml
[k_bin_testing_b]: https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/kilted-binary-build-testing.yml/badge.svg
[k_semi_bin_main]: https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/kilted-semi-binary-build-main.yml
[k_semi_bin_main_b]: https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/kilted-semi-binary-build-main.yml/badge.svg
[k_semi_bin_testing]: https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/kilted-semi-binary-build-testing.yml
[k_semi_bin_testing_b]: https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/kilted-semi-binary-build-testing.yml/badge.svg
[k_src]: https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/kilted-source-build.yml
[k_src_b]: https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/kilted-source-build.yml/badge.svg
<!-- Humble -->
[h_bin_main]: https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/humble-binary-build-main.yml
[h_bin_main_b]: https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/humble-binary-build-main.yml/badge.svg
[h_bin_testing]: https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/humble-binary-build-testing.yml
[h_bin_testing_b]: https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/humble-binary-build-testing.yml/badge.svg
[h_semi_bin_main]: https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/humble-semi-binary-build-main.yml
[h_semi_bin_main_b]: https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/humble-semi-binary-build-main.yml/badge.svg
[h_semi_bin_testing]: https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/humble-semi-binary-build-testing.yml
[h_semi_bin_testing_b]: https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/humble-semi-binary-build-testing.yml/badge.svg
[h_src]: https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/humble-source-build.yml
[h_src_b]: https://github.com/PickNikRobotics/picknik_controllers/actions/workflows/humble-source-build.yml/badge.svg

- **Release-Main Bin**: Builds source plus `picknik_controllers-not-released.<ros-distro>.repos` with dependencies retrieved from ROS 2 "main" release repository.
- **Release-Testing Bin**: Builds source plus `picknik_controllers-not-released.<ros-distro>.repos` with dependencies retrieved from ROS 2 "testing" release repository.
- **Mixed Release**: Main Bin & Upstream Src: Builds source plus `picknik_controllers.<ros-distro>.repos` (intended to list additional repositories than `picknik_controllers-not-released.<ros-distro>.repos`) with dependencies retrieved from ROS 2 "main" release repository.
- **Mixed Release**: Testing Bin & Upstream Src: Builds source plus `picknik_controllers.<ros-distro>.repos` with dependencies retrieved from ROS 2 "testing" release repository.
- **Upstream Src**: Builds source plus all dependencies from source - no ROS 2 release repositories are used.

### `main` Branch Compatibility

The `main` source branch is currently compatible, with ABI/API breaking changes, with the following distros:
- Rolling
- Jazzy

Releases for distros are built from distro specific branches and do not break ABI/API (except for Rolling).