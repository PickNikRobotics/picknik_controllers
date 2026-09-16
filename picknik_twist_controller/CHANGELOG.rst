^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package picknik_twist_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.5 (2026-09-16)
------------------
* Declare the hardware_interface dependency explicitly (`#39 <https://github.com/PickNikRobotics/picknik_controllers/issues/39>`_)
  Both controllers include <hardware_interface/loaned_command_interface.hpp> and
  name hardware_interface::LoanedCommandInterface directly, but neither declared
  hardware_interface in package.xml or found it in CMake. They relied on
  controller_interface to pull it in transitively:
  picknik\_*_controller -> controller_interface -> hardware_interface
  That works today and is not a build break, but it means a dependency we use
  directly is invisible to rosdep and to anyone reading the manifest, and it would
  break silently if controller_interface ever stopped re-exporting it.
  Adds, for both packages:
  - <depend>hardware_interface</depend> in package.xml
  - find_package(hardware_interface REQUIRED)
  - hardware_interface in THIS_PACKAGE_INCLUDE_DEPENDS, so it is re-exported to
  consumers alongside the other direct deps
  - ${hardware_interface_TARGETS} in target_link_libraries, matching the
  ${..._TARGETS} style already used for controller_interface. That variable
  resolves to hardware_interface::hardware_interface only -- mock_components
  lives in a separate export set and is not pulled in.
  Reported by @Plumezz in `#36 <https://github.com/PickNikRobotics/picknik_controllers/issues/36>`_ against the humble branch; the same gap is present
  on main and on humble, and both packages are affected, not just
  picknik_reset_fault_controller.
  Verified by building both packages from a clean workspace in a
  ros:lyrical-ros-base container: rosdep resolves the new dependency and colcon
  build succeeds.
  Fixes `#36 <https://github.com/PickNikRobotics/picknik_controllers/issues/36>`_
  Co-authored-by: Nathan Brooks <nathan.brooks@picknik.ai>
  Co-authored-by: Claude Opus 5 (1M context) <noreply@anthropic.com>
* Merge pull request `#28 <https://github.com/PickNikRobotics/picknik_controllers/issues/28>`_ from ahcorde/ahcorde/replace_atd_with_tll
  Replace ament_target_dependencies with target_link_libraries
* Replace ament_target_dependencies with target_link_libraries
* Contributors: Alejandro Hernandez Cordero, Nathan Brooks

0.0.4 (2025-02-09)
------------------
* Fix deprecated realtime_tools header imports (`#14 <https://github.com/PickNikRobotics/picknik_controllers/issues/14>`_)
* Contributors: Sebastian Castro

0.0.3 (2023-07-24)
------------------
* Use Twist not TwistStamped (`#11 <https://github.com/PickNikRobotics/picknik_controllers/issues/11>`_)
* Contributors: Alex Moriarty

0.0.2 (2023-07-14)
------------------
* fix typo (`#10 <https://github.com/PickNikRobotics/picknik_controllers/issues/10>`_)
  ABI breaking change fixes a typo from original internal package rename for open sourcing
  PicknikTwistControler -> PicknikTwistController
* Contributors: Anthony Baker

0.0.1 (2023-07-11)
------------------
* Initial Release of picknik_twist_controller
  * Originally this was used internally and there was an attempt to release it to ros2_controllers here: https://github.com/ros-controls/ros2_controllers/pull/300
  * The goal is to still move this upstream but it needs to be refactored before going upstream.
* twist_controller -> picknik_twist_controller (`#3 <https://github.com/PickNikRobotics/picknik_controllers/issues/3>`_)
  * twist_controller -> picknik_twist_controller
  1. prefix twist_controller with picknik_twist_controller
  When we merge twist_controller into ros2_controllers we can depricate
  this one and not have naming conflicts as users migrate
  * cmake: 3.8 -> 3.16
  bump to oldest version used on Ubuntu Focal + ROS 2 Humble
  https://www.ros.org/reps/rep-2000.html#humble-hawksbill-may-2022-may-2027
  ---------
* Contributors: Alexander Moriarty @moriarty, Anthony Baker @abake48, @livanov93, @destogl, @MarqRazz, @Abishalini, @JafarAbdi
