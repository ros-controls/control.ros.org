.. _diff_to_ros1:

Differences to ros_control (ROS 1)
==================================

Hardware Structures - classes
-----------------------------

The ros_control framework uses the ``RobotHW`` class as a rigid structure to handle any hardware.
This makes it impossible to extend the existing robot with additional hardware, like sensors, actuators, and tools, without coding.
The ``CombinedRobotHardware`` corrects this drawback.
Still, this solution is not optimal, especially when combining robots with external sensors.

The ros2_control framework defines three types of hardware ``Actuator``, ``Sensor`` and ``System``.
Using a combination (composition) of those basic components, any physical robotic cell (robot and its surrounding) can be described.
This also means that multi-robot, robot-sensor, robot-gripper combinations are supported out of the box.
Section `Hardware Components <#hardware-components>`__ describes this in detail.

Hardware Interfaces
-------------------

The ros_control framework allows only three types of interfaces (joints), i.e., ``position``, ``velocity``, and ``effort``. The ``RobotHW`` class makes it very hard to use any other data to control the robot.

The ros2_control approach does not enforce a fixed set of interface types, but they are defined as strings in `hardware's description <#hardware-description-in-urdf>`__.
To ensure compatibility of standard controllers, standard interfaces are defined as constants in `hardware_interface package <https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/include/hardware_interface/types/hardware_interface_type_values.hpp>`__.

Controller's Access to Hardware
-------------------------------

In ros_control, the controllers had direct access to the ``RobotHW`` class requesting access to its interfaces (joints).
The hardware itself then took care of registered interfaces and resource conflicts.

In ros2_control, ``ResourceManager`` takes care of the state of available interfaces in the framework and enables controllers to access the hardware.
Also, the controllers do not have direct access to hardware anymore, but they register their interfaces to the ``ControllerManager``.

Migration Guide to ros2_control
===============================

RobotHardware to Components
---------------------------
#. The implementation of ``RobotHW`` is not used anymore.
   This should be migrated to `SystemInterface`_ class or, for more granularity, `SensorInterface`_ and `ActuatorInterface`_.
   See the above description of "Hardware Components" to choose a suitable strategy.
#. Decide which component type is suitable for your case. Maybe it makes sense to separate ``RobotHW`` into multiple components.
#. Implement `ActuatorInterface`_, `SensorInterface`_ or `SystemInterface`_ classes as follows:

   #. In the constructor, initialize all variables needed for communication with your hardware or define the default one.
   #. In the configure function, read all the parameters your hardware needs from the parsed URDF snippet (i.e., from the `HardwareInfo`_ structure). Here you can cross-check if all joints and interfaces in URDF have allowed values or a combination of values.
   #. Define interfaces to and from your hardware using ``export_*_interfaces`` functions.
      The names are ``<joint>/<interface>`` (e.g., ``joint_a2/position``).
      This can be extracted from the `HardwareInfo`_ structure or be hard-coded if sensible.
   #. Implement ``start()`` and ``stop()`` methods for your hardware.
      This usually includes changing the hardware state to receive commands or set it into a safe state before interrupting the command stream.
      It can also include starting and stopping communication.
   #. Implement ``read()`` and ``write()`` methods to exchange commands with the hardware.
      This method is equivalent to those from ``RobotHW``-class in ROS 1.
   #. Do not forget the ``PLUGINLIB_EXPORT_CLASS`` macro at the end of the .cpp file.
#. Create .xml library description for the pluginlib, for example see `RRBotSystemPositionOnlyHardware <https://github.com/ros-controls/ros2_control_demos/blob/master/example_1/ros2_control_demo_example_1.xml>`_.


Controller Migration
--------------------
An excellent example of a migrated controller is the `JointTrajectoryController`_.
The real-time critical methods are marked as such.

#. Implement `ControllerInterface`_ class as follows:

   #. If there are any member variables, initialized those in the constructor.
   #. In the ``init()`` method, first call ``ControllerInterface::init()`` to initialize the lifecycle of the controller. Following this, declare all parameters defining their default values.
   #. Implement the ``state_interface_configuration()`` and ``command_interface_configuration()`` methods.
   #. Design the ``update()`` function for the controller. (**real-time**)
   #. Add the required lifecycle management methods (others are optional):
         * ``on_configure()`` - reads parameters and configures controller.
         * ``on_activate()`` - called when controller is activated (started) (**real-time**)
         * ``on_deactivate()`` - called when controller is deactivated (stopped) (**real-time**)
   #. Finally, do not forget to add the ``PLUGINLIB_EXPORT_CLASS`` macro at the end of the .cpp file.
#. Create .xml library description for the pluginlib, for example see `joint_trajectory_plugin.xml <https://github.com/ros-controls/ros2_controllers/blob/master/joint_trajectory_controller/joint_trajectory_plugin.xml>`_.


.. _ActuatorInterface: https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/include/hardware_interface/actuator_interface.hpp
.. _SensorInterface: https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/include/hardware_interface/sensor_interface.hpp
.. _SystemInterface: https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/include/hardware_interface/system_interface.hpp
.. _HardwareInfo: https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/include/hardware_interface/hardware_info.hpp
.. _JointTrajectoryController: https://github.com/ros-controls/ros2_controllers/blob/master/joint_trajectory_controller/src/joint_trajectory_controller.cpp
.. _ControllerInterface: https://github.com/ros-controls/ros2_control/blob/master/controller_interface/include/controller_interface/controller_interface.hpp
