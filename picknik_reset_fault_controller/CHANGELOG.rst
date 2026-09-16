^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package picknik_reset_fault_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Merge pull request `#23 <https://github.com/PickNikRobotics/picknik_controllers/issues/23>`_ from christophfroehlich/fix/API
  Fix upstream API
* Use correct state interface again
* Fix potential bad optional access
* Fix upstream API
* Contributors: Alejandro Hernandez Cordero, Christoph Froehlich, Nathan Brooks

0.0.4 (2025-02-09)
------------------
* Fix deprecated realtime_tools header imports (`#14 <https://github.com/PickNikRobotics/picknik_controllers/issues/14>`_)
* Contributors: Sebastian Castro

0.0.3 (2023-07-24)
------------------

0.0.2 (2023-07-14)
------------------

0.0.1 (2023-07-11)
------------------
* Initial Release of picknik_reset_fault_controller
  * Originally this was used internally and there was an attempt to release it to ros2_controllers
  * After discussion with the ros2 controllers WG over slack we have decided to open source it here first
  * The goal is to still move this upstream but it can be worked on here first and moved in the future
* fault_controller -> picknik_reset_fault_controller (`#2 <https://github.com/PickNikRobotics/picknik_controllers/issues/2>`_)
  * fault_controller -> picknik_reset_fault_controller
  This commit does two things:
  1) renames fault_controller to reset_fault_controller
  2) prefixes with picknik\_
  The first change, is to be more specific what this controller is used
  for.
  The second change is because we want to move this controller into
  ros2_controllers and when that is complete we can drop the picknik\_ and
  depricate this version allowing for a transition period.
  ---------
* Contributors: Alexander Moriarty @moriarty, Anthony Baker @abake48, @livanov93, @destogl, @MarqRazz, @Abishalini, @JafarAbdi
