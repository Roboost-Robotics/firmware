
This directory is intended for project specific (private) libraries.
PlatformIO will compile them to static libraries and link into executable file.

The source code of each library should be placed in a an own separate directory
("lib/your_library_name/[here are source files]").

For example, see a structure of the following two libraries `Foo` and `Bar`:

|--entorhinal-cortex
|  |-README.md
|  |--conf
|  |  |-README.md
|  |  |-conf_hardware.h
|  |  |-conf_network_example.h
|  |  |-conf_network.h
|  |--doc
|  |  |-README.md
|  |--include
|  |  |-README.md
|  |  |--kinematics
|  |  |  |-kinematics.hpp
|  |  |--motor-control
|  |  |  |--motor-drivers
|  |  |  |  |-motor_driver.hpp
|  |  |  |  |-l298n_motor_driver.hpp
|  |  |  |-encoder.hpp
|  |  |  |-motor_control_manager.hpp
|  |  |  |-motor_controller.hpp
|  |  |  |-pid.hpp
|  |  |  |-simple_motor_controller.hpp
|  |  |-rcl_checks.h
|  |  |-robot_controller.hpp
|  |--lib
|  |  |-README.md
|  |--res
|  |  |-README.md
|  |  |-kinematics_calculations.mcdx
|  |--src
|  |  |-README.md
|  |  |--kinematics
|  |  |  |-mecanum_kinematics_4w.cpp
|  |  |--motor-control
|  |  |  |--motor-drivers
|  |  |  |  |-l298n_motor_driver.cpp
|  |  |  |-encoder.cpp
|  |  |  |-motor_control_manager.cpp
|  |  |  |-pid.cpp
|  |  |  |-simple_motor_controller.cpp
|  |  |-core.cpp
|  |  |-robot_controller.cpp
|  |-.clang-format
|  |-.gitignore
|  |-Doxyfile
|  |-platformio.ini
|--primary-motor-cortex
|  |-README.md
|  |--conf
|  |  |-README.md
|  |  |-conf_hardware.h
|  |  |-conf_network_example.h
|  |  |-conf_network.h
|  |--doc
|  |  |-README.md
|  |--include
|  |  |-README.md
|  |  |--kinematics
|  |  |  |-kinematics.hpp
|  |  |--motor-control
|  |  |  |--motor-drivers
|  |  |  |  |-motor_driver.hpp
|  |  |  |  |-l298n_motor_driver.hpp
|  |  |  |-encoder.hpp
|  |  |  |-motor_control_manager.hpp
|  |  |  |-motor_controller.hpp
|  |  |  |-pid.hpp
|  |  |  |-simple_motor_controller.hpp
|  |  |-rcl_checks.h
|  |  |-robot_controller.hpp
|  |--lib
|  |  |-README.md
|  |--res
|  |  |-README.md
|  |  |-kinematics_calculations.mcdx
|  |--src
|  |  |-README.md
|  |  |--kinematics
|  |  |  |-mecanum_kinematics_4w.cpp
|  |  |--motor-control
|  |  |  |--motor-drivers
|  |  |  |  |-l298n_motor_driver.cpp
|  |  |  |-encoder.cpp
|  |  |  |-motor_control_manager.cpp
|  |  |  |-pid.cpp
|  |  |  |-simple_motor_controller.cpp
|  |  |-core.cpp
|  |  |-robot_controller.cpp
|  |-.clang-format
|  |-.gitignore
|  |-Doxyfile
|  |-platformio.ini
|-README.md

and a contents of `src/main.c`:

```
#include <Foo.h>
#include <Bar.h>

int main (void)
{
  ...
}

```

PlatformIO Library Dependency Finder will find automatically dependent
libraries scanning project source files.

More information about PlatformIO Library Dependency Finder

- <https://docs.platformio.org/page/librarymanager/ldf.html>
