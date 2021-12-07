.. _resources:

Resources
================
The resources provided in the ``resources`` folder are available for use under CC-BY license |CC-BY|_.
The original authors are named either in the documents or in the list down below.

Any files submitted to the documentation should be "licensed" by stating your name and ``ros2_control`` organization if no company name applicable, e.g., ``CC-BY My Name (ros2_control/company_name)``.

Presentations
---------------

2021-10 ROS World 2021
,,,,,,,,,,,,,,,,,,,,,,,,
`Presentation: ros2_control - The future of ros_control <presentations/2021-10_ROS_World_2021-ros2_control_The_future_of_ros_control.pdf>`_

  Summary:
    ros2_control is a robot-agnostic control framework with a focus on both real-time performance and sharing of controllers. The framework offers controller lifecycle and hardware resource management, as well as abstractions on hardware interfaces.

    Controllers expose ROS interfaces for 3rd party solutions to robotics problems like manipulation path planning (`moveit2`) and autonomous navigation (`nav2`). The modular design makes it ideal for both research and industrial use. A robot made up of a mobile base and an arm that supports ros2_control needs no extra code, only a few controller configuration files and it is ready to

`Recording <https://vimeo.com/649654948>`_

  *Presenter: Dr. Bence Magyar*

  Authors:
    - Dr. Bence Magyar (FiveAI Ltd)
    - Denis Stogl (Stogl Robotics Consulting)


`Presentation: Making a robot ROS 2 powered - a case study using the UR manipulators <presentations/2021-10_ROS_World-Making_a_robot_ROS_2_powered.pdf>`_

  Summary:
    With the release of ros2_control and MoveIt 2, ROS 2 Foxy finally has all the “ingredients” needed to power a robot with similar features as in ROS 1. We present the driver for Universal Robot’s manipulators as a real-world example of how robots can be run using ROS 2. We show how to realize multi-interface support for position and velocity commands in the driver and how to support scaling controllers while respecting factors set on the teach pendant. Finally, we show how this real-world example influences development of ros2_control to support non-joint related inputs and outputs in its real-time control loop.

`Recording <https://vimeo.com/649651707/46a3be27ed>`_

  *Presenter: Denis Štogl*

  Authors:
    - Denis Štogl (PickNik Inc.)
    - Dr. Nathan Brooks (PickNik Inc.)
    - Lovro Ivanov (PickNik Inc.)
    - Dr. Andy Zelenak (PickNik Inc.)
    - Rune Søe-Knudsen (Universal Robots)


`Presentation: Online Trajectory Generation and Admittance Control in ROS2 <presentations/2021-10_ROS_World-Admittance_Control_in_ROS2.pdf>`_

  Summary:
    One of the top reasons to upgrade from ROS1 to ROS2 is better suitability for realtime tasks. We discuss the development of a new ROS2 controller to handle realtime contact tasks such as tool insertion with industrial robots. The admittance controller handles trajectories and single-waypoint streaming commands, making it compatible with MoveIt and many teleoperation frameworks. Part of the work involved ensuring kinematic limits (position/velocity/acceleration/jerk) are obeyed while limiting interaction forces with the environment. Finally, we give practical recommendations and examples of the admittance controller. A live demo will be shown at our booth.

`Recording <https://vimeo.com/649652452/682bd92e95>`_

  *Presenter: Dr. Andy Zelenak*

  Authors:
    - Dr. Andy Zeleank (PickNik Inc.)
    - Denis Štogl (PickNik Inc.)


2021-10-07 Weekly Robotics Meetup #13
,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
`Presentation: Getting started with ros2_control <presentations/2021-1_WR_Meetup_Getting_started_with_ros2_control.pdf>`_

  Summary:
    ros2_control is a robot-agnostic control framework with a focus on both real-time performance and sharing of controllers. The framework offers controller lifecycle and hardware-resource management, as well as abstractions on hardware interfaces.

    Controllers expose ROS interfaces for 3rd party solutions to robotics problems like manipulation path planning (`moveit2`) and autonomous navigation (the ROS2 navigation stack). Hardware components on the other side provide a unified interface for robotic hardware, enabling standardized life-cycle and access management. The modular design makes ros2_control ideal for both research and industrial use. For example, a robot made up of a mobile base and an arm that supports ros2_control needs no extra code, only a few controller configuration files, and it is ready to go.

    In this talk, we will discuss concepts of ros2_control framework compared to ros(1)_control framework and show examples of their use in the wild.

`Recording <https://www.youtube.com/watch?v=9AsDmPJWcnQ>`_

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


Diagrams
---------
Folder with diagrams and sources for the images.
Simply use `diagrams.net <http://diagrams.net>`_ for editing.

`ros2_control <diagrams/ros2_control.drawio>`_ - a collection of ``ros2_control``-related diagrams.

  - overview diagrams
  - integration with MoveIt2
  - class diagrams
  - lifecycle diagrams


Images
-------
Generated images for the presentation which can be useful also for the documentation.

Overview of ros2_control
  .. image:: images/ros2_control_overview.png
     :target: images/ros2_control_overview.png


Architecture of complex controller and semantic components:
  .. image:: images/complex_controllers_and_semantic_components.png
     :target: images/complex_controllers_and_semantic_components.png


Architecture of command and state interfaces:
  .. image:: images/command_and_state_interfaces.png
     :target: images/command_and_state_interfaces.png


Lifecycle of hardware interfaces:
  .. image:: images/hardware_interface_lifecycle.png
     :target: images/hardware_interface_lifecycle.png


ros2_control integration with MoveIt2
  .. image:: images/ros2_control_robot_integration_with_moveit2.png
     :target: images/ros2_control_robot_integration_with_moveit2.png



.. |CC-BY| image:: https://i.creativecommons.org/l/by/4.0/88x31.png
.. _CC-BY: http://creativecommons.org/licenses/by/4.0/
