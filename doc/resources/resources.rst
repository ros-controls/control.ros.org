.. _resources:

Resources
=========

The resources provided in the ``resources`` folder are available for use under CC-BY license |CC-BY|_.
The original authors are named either in the documents or in the list down below.

Any files submitted to the documentation should be "licensed" by stating your name and ``ros2_control`` organization if no company name applicable, e.g., ``CC-BY My Name (ros2_control/company_name)``.

Presentations
---------------

2023-02 ROS Meetup Munich #5
,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
`Presentation: Tricycle Controller with ros2_control <https://github.com/ros-controls/control.ros.org/blob/master/doc/resources/presentations/pixel_robotics_tricycle_controller_with_ros2_control.pdf>`_

`Meetup event link <https://www.meetup.com/robot-operating-system-ros/events/290966049/>`_

  Summary:
    In this presentation Pixel Robotics presents the contributed the Tricycle controller to ros2_controllers, prefaced by an introduction to ros2_control.

  *Presenters: Johannes Plapp & Tony Najjar*

  Authors:
    - Johannes Plapp (Pixel Robotics)
    - Tony Najjar (Pixel Robotics)


2022-12 ROS-Industrial Conference 2022
,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
`Presentation: ros2_control - Kernel for ROS 2 controlled robots <https://github.com/ros-controls/control.ros.org/blob/master/doc/resources/presentations/2022-12_ROS-I_Conference-ros2_control_-_Kernel_for_ROS_2_controlled_robots.pdf>`_

  Summary:
    ros2_control is a hardware-agnostic control framework focusing on the modular composition of control systems for robots, sharing of controllers as well as real-time performance. The framework provides “kernel” functionality for robots by abstracting the hardware and doing heavy low-level management, for example, hardware lifecycle, communication and access control.

  *Presenter: Dr. Denis Stogl*

  Authors:
    - Dr. Denis Stogl (Stogl Robotics Consulting)


2022-10 ROSCon 2022
,,,,,,,,,,,,,,,,,,,
`Presentation: A practitioner's guide to ros2_control <https://raw.githubusercontent.com/ros-controls/control.ros.org/master/doc/resources/presentations/2022-10_ROSCon2022_A_practitioners_guide_to_ros2_control.pdf>`_

  Summary:
    ros2_control is a hardware-agnostic control framework focusing on the modular composition of control systems for robots, sharing of controllers as well as real-time performance. The framework provides controller-lifecycle and hardware management on top of abstractions of real or virtual hardware interfaces.

    This talk delves deeper into ros2_control, showcasing new features and what they could be used for, such as explicit lifecycle management, chaining controllers, emergency-stop handlers and mock components. Finally, we showcase different usages of ros2_control on openly accessible examples.

`Recording <https://vimeo.com/showcase/9954564/video/767139648>`__

  *Presenter: Dr. Bence Magyar*

  Authors:
    - Dr. Bence Magyar (FiveAI Ltd)
    - Dr. Denis Stogl (Stogl Robotics Consulting)


2022-06 ROSCon Fr 2022
,,,,,,,,,,,,,,,,,,,,,,,
`Presentation: What is new in the best (and only) control framework for ROS2 - ros2_control <https://raw.githubusercontent.com/ros-controls/control.ros.org/master/doc/resources/presentations/2022-06_ROSConFr_What-is-new-in-ros2_control.pdf>`_

  Summary:
    ros2_control is a hardware-agnostic control framework with a focus on both real-time performance and sharing of controllers. The framework has become one of the main utilities for abstracting hardware and low-level control for 3rd party solutions like `MoveIt2` and `Nav2` systems.

    The presentation provides practical tips to use ros2_control, from creating a robot description, writing hardware drivers to configuring standard controllers. Some hot-new features, like controller chaining, will be shown. Furthermore, you will get introduced to concepts like modular reuse of hardware drivers, multi-robot architectures and parameters injection for controllers.

`Recording <https://peertube.laas.fr/w/dAmVEo9GrJLrcwLpashtZe>`__

  *Presenter: Dr. Denis Stogl*

  Authors:
    - Dr. Denis Stogl (Stogl Robotics Consulting)


2021-10 ROS World 2021
,,,,,,,,,,,,,,,,,,,,,,,,
`Presentation: ros2_control - The future of ros_control <https://raw.githubusercontent.com/ros-controls/control.ros.org/master/doc/resources/presentations/2021-10_ROS_World_2021-ros2_control_The_future_of_ros_control.pdf>`_

  Summary:
    ros2_control is a robot-agnostic control framework with a focus on both real-time performance and sharing of controllers. The framework offers controller lifecycle and hardware resource management, as well as abstractions on hardware interfaces.

    Controllers expose ROS interfaces for 3rd party solutions to robotics problems like manipulation path planning (`moveit2`) and autonomous navigation (`nav2`). The modular design makes it ideal for both research and industrial use. A robot made up of a mobile base and an arm that supports ros2_control needs no extra code, only a few controller configuration files and it is ready to go.

`Recording <https://vimeo.com/649654948>`__

  *Presenter: Dr. Bence Magyar*

  Authors:
    - Dr. Bence Magyar (FiveAI Ltd)
    - Denis Stogl (Stogl Robotics Consulting)


`Presentation: Making a robot ROS 2 powered - a case study using the UR manipulators <https://raw.githubusercontent.com/ros-controls/control.ros.org/master/doc/resources/presentations/2021-10_ROS_World-Making_a_robot_ROS_2_powered.pdf>`_

  Summary:
    With the release of ros2_control and MoveIt 2, ROS 2 Foxy finally has all the “ingredients” needed to power a robot with similar features as in ROS 1. We present the driver for Universal Robot’s manipulators as a real-world example of how robots can be run using ROS 2. We show how to realize multi-interface support for position and velocity commands in the driver and how to support scaling controllers while respecting factors set on the teach pendant. Finally, we show how this real-world example influences development of ros2_control to support non-joint related inputs and outputs in its real-time control loop.

`Recording <https://vimeo.com/649651707/46a3be27ed>`__

  *Presenter: Denis Štogl*

  Authors:
    - Denis Štogl (PickNik Inc.)
    - Dr. Nathan Brooks (PickNik Inc.)
    - Lovro Ivanov (PickNik Inc.)
    - Dr. Andy Zelenak (PickNik Inc.)
    - Rune Søe-Knudsen (Universal Robots)


`Presentation: Online Trajectory Generation and Admittance Control in ROS2 <https://raw.githubusercontent.com/ros-controls/control.ros.org/master/doc/resources/presentations/2021-10_ROS_World-Admittance_Control_in_ROS2.pdf>`_

  Summary:
    One of the top reasons to upgrade from ROS1 to ROS2 is better suitability for realtime tasks. We discuss the development of a new ROS2 controller to handle realtime contact tasks such as tool insertion with industrial robots. The admittance controller handles trajectories and single-waypoint streaming commands, making it compatible with MoveIt and many teleoperation frameworks. Part of the work involved ensuring kinematic limits (position/velocity/acceleration/jerk) are obeyed while limiting interaction forces with the environment. Finally, we give practical recommendations and examples of the admittance controller. A live demo will be shown at our booth.

`Recording <https://vimeo.com/649652452/682bd92e95>`__

  *Presenter: Dr. Andy Zelenak*

  Authors:
    - Dr. Andy Zeleank (PickNik Inc.)
    - Denis Štogl (PickNik Inc.)


2021-10-07 Weekly Robotics Meetup #13
,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
`Meetup presentation: Getting started with ros2_control <https://raw.githubusercontent.com/ros-controls/control.ros.org/master/doc/resources/presentations/2021-1_WR_Meetup_Getting_started_with_ros2_control.pdf>`_

  Summary:
    ros2_control is a robot-agnostic control framework with a focus on both real-time performance and sharing of controllers. The framework offers controller lifecycle and hardware-resource management, as well as abstractions on hardware interfaces.

    Controllers expose ROS interfaces for 3rd party solutions to robotics problems like manipulation path planning (`moveit2`) and autonomous navigation (the ROS2 navigation stack). Hardware components on the other side provide a unified interface for robotic hardware, enabling standardized life-cycle and access management. The modular design makes ros2_control ideal for both research and industrial use. For example, a robot made up of a mobile base and an arm that supports ros2_control needs no extra code, only a few controller configuration files, and it is ready to go.

    In this talk, we will discuss concepts of ros2_control framework compared to ros(1)_control framework and show examples of their use in the wild.

`Recording <https://www.youtube.com/watch?v=9AsDmPJWcnQ>`__

  *Presenters: Dr. Bence Magyar and Denis Štogl*

  Authors:
    - Dr. Bence Magyar (FiveAI Ltd)
    - Denis Stogl (Stogl Robotics Consulting)


2021-06 ROSDevDay 2021
,,,,,,,,,,,,,,,,,,,,,,,,
`Presentation materials <https://github.com/bmagyar/rosdevday-presentation>`_

`Recording <https://www.youtube.com/watch?v=5OfOPcu8Erw>`_

  *Presenters: Dr. Bence Magyar and Denis Štogl*

  Authors:
    - Dr. Bence Magyar (FiveAI Ltd)
    - Denis Stogl (Stogl Robotics Consulting)

2021-05 ROSCon Fr 2021
,,,,,,,,,,,,,,,,,,,,,,,
`Presentation: Getting started with ros2_control <https://raw.githubusercontent.com/ros-controls/control.ros.org/master/doc/resources/presentations/2021-06_RosConFR_Getting_started_with_ros2_control.pdf>`_

  Summary:
    The presentation gives a quick overview on the basic concepts and some simple implementation examples. We show implementing a simple Hardware Abstraction Layer (aka SystemComponent) and a forwarding controller. Once done, we also look into modifying the controller with the example goal of changing the type of the command topic.

`Recording <https://peertube.laas.fr/w/sDPKwTWP6gAr5h1CcZPnbJ>`__

  *Presenter: Dr. Bence Magyar*

Diagrams
---------
Folder with diagrams and sources for the images.
Simply use `diagrams.net <http://diagrams.net>`_ for editing.

`ros2_control <https://github.com/ros-controls/control.ros.org/blob/master/doc/resources/diagrams/ros2_control.drawio>`_ - a collection of ``ros2_control``-related diagrams.

  - overview diagrams
  - integration with MoveIt2
  - class diagrams
  - lifecycle diagrams
  - command and state interfaces examples
  - mobile manipulator architectures
  - Force-Control architectures


Images
-------
Generated images for the presentation which can be useful also for the documentation.

Overview of ros2_control
  .. image:: images/ros2_control_overview.png

ros2_control robot integration with MoveIt2
  .. image:: images/ros2_control_robot_integration_with_moveit2.png

Architecture of complex controller and semantic components:
  .. image:: images/complex_controllers_and_semantic_components.png

Architecture of command and state interfaces:
  .. image:: images/command_and_state_interfaces.png

Lifecycle of hardware interfaces:
  .. image:: images/hardware_interface_lifecycle.png

ros2_control integration with MoveIt2
  .. image:: images/ros2_control_robot_integration_with_moveit2.png

Controllers architecture with chained controllers - admittance controller example
  .. image:: images/ros2_control_mobile_manipulator_control_arch_admittance_chaining.png

Controllers architecture with chained controllers - admittance controller example (URDF)
  .. image:: images/ros2_control_mobile_manipulator_controllers_admittance_chaining.png

Controllers architecture without chained controllers - admittance controller example
  .. image:: images/ros2_control_mobile_manipulator_control_arch_admittance_without_chaining.png

Controllers architecture with chained controllers - mobile base controller example
  .. image:: images/ros2_control_mobile_manipulator_control_arch_base_chaining.png

Controllers architecture with chained controllers - mobile base controller example (URDF)
  .. image:: images/ros2_control_mobile_manipulator_controllers_base_chaining.png

Controllers architecture without chained controllers - admittance controller example
  .. image:: images/ros2_control_mobile_manipulator_control_arch_base_without_chaining.png

Controllers architecture - overview
  .. image:: images/ros2_control_mobile_manipulator_control_arch_convoluted_controllers.png

Controllers architecture - URDF
  .. image:: images/ros2_control_mobile_manipulator_controllers_convoluted_controllers.png

Hardware architecture - independent communication to the hardware (modular hardware)
  .. image:: images/ros2_control_mobile_manipulator_control_arch_independent_hardware.png

Hardware architecture - independent communication to the hardware (modular hardware) (URDF)
  .. image:: images/ros2_control_mobile_manipulator_control_arch_independent_hardware_urdf.png

Hardware architecture - gripper communication through Arm
  .. image:: images/ros2_control_mobile_manipulator_control_arch_gripper_through_arm_comms.png

Hardware architecture - gripper communication through Arm (URDF)
  .. image:: images/ros2_control_mobile_manipulator_control_arch_gripper_through_arm_comms_urdf.png

Hardware architecture - monolitic communication to hardware
  .. image:: images/ros2_control_mobile_manipulator_control_arch_monolitic_hardware.png

Hardware architecture - monolitic communication to hardware (URDF)
  .. image:: images/ros2_control_mobile_manipulator_control_arch_monolitic_hardware_urdf.png

Hardware architecture - multiple hardware in one controller manager
  .. image:: images/ros2_control_mobile_manipulator_control_arch_multi_robots_in_one_controller_manager.png

Example files - ros2_control - "Controlko" mobile manipulator
  .. image:: images/ros2_control_mobile_manipulator.png

Example files - ros2_control - "Controlko" mobile manipulator (URDF)
  .. image:: images/ros2_control_mobile_manipulator_controllers.png

.. |CC-BY| image:: https://i.creativecommons.org/l/by/4.0/88x31.png
.. _CC-BY: http://creativecommons.org/licenses/by/4.0/
