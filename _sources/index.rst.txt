.. _documentation_home:

Welcome to the ros2_control documentation!
==========================================

.. toctree::
   :hidden:

   getting_started.rst
   ros2_controllers/doc/index.rst
   differences_to_ros1.rst
   acknowledgements.rst

The ros2_control is a framework for (real-time) control of robots using (`ROS2 <https://index.ros.org/doc/ros2/>`_).
Its packages are a rewrite of `ros_control <http://wiki.ros.org/ros_control>`_ packages used in `ROS` (`Robot Operating System <https://wiki.ros.org>`_).
ros2_control's goal is to simplify integrating new hardware and overcome some drawbacks.

If you are not familiar with the control theory, please get some idea about it (e.g., at `Wikipedia <https://en.wikipedia.org/wiki/Control_theory>`_) to get familiar with the terms used in this manual.


Overview
=========
The ros2_control framework consists of the following Github repositories:

* `ros2_control`_ - the main interfaces and components of the framework;
* `ros2_controllers`_ - widely used controllers, such as forward command controller, joint trajectory controller, differential drive controller;
* `control_toolbox`_ - some widely-used control theory implementations (e.g. PID) used by controllers;
* `realtime_tools`_ - general toolkit for realtime support, e.g., realtime buffers and publishers;
* `control_msgs`_ - common messages.


Additionally, there are following (unreleased) packages are relevant for getting-started and project management:

* `ros2_control_demos`_ - examples implementations of common use-cases for a smooth start;
* `roadmap`_ - planning and design docs for the project.

Development Organisation and Communication
-------------------------------------------

WG Meeting
   Every second Wednesday there is a Working Group meeting.
   To join the meeting check the announcement on `ROS Discourse`_.
   You can joint the meeting through `google groups <https://groups.google.com/forum/#!forum/ros-control-working-group-invites>`_ or directly on Google Meet (check the announcement).
   To propose new discussion points, or review notes from previous meetings, check `this document <https://docs.google.com/document/d/1818AoYucI2z82awL_-8sAA5pMCV_g_wXCJiM6SQmhSQ/edit?usp=sharing>`_.

Projects
   GitHub `projects under ros-control organization <https://github.com/orgs/ros-controls/projects>`_ are used to track the work.

Bug reports and feature requests
   Use the issue tracker in the corresponding repository for this.
   Give a short summary of the problem
   Make sure to provide a minimal list of steps one can follow to reproduce the issue you found
   Provide relevant information regarding the operating system, ROS distribution, etc.

Questions
   Please use `ROS Answers <https://answers.ros.org/>`_ and tag your questions with ``ros2_control``.

General discussions
   Please use `ROS Discourse`_.


.. _ros2_control: https://github.com/ros-controls/ros2_control
.. _ros2_controllers: https://github.com/ros-controls/ros2_controllers
.. _control_msgs: https://github.com/ros-controls/control_msgs
.. _realtime_tools: https://github.com/ros-controls/realtime_tools
.. _control_toolbox: https://github.com/ros-controls/control_toolbox
.. _ros2_control_demos: https://github.com/ros-controls/ros2_control_demos
.. _controller_manager_msgs: https://github.com/ros-controls/ros2_control/tree/master/controller_manager_msgs
.. _Controller Manager: https://github.com/ros-controls/ros2_control/blob/master/controller_manager/src/controller_manager.cpp
.. _ControllerInterface: https://github.com/ros-controls/ros2_control/blob/master/controller_interface/include/controller_interface/controller_interface.hpp
.. _ros2_control node: https://github.com/ros-controls/ros2_control/blob/master/controller_manager/src/ros2_control_node.cpp
.. _ForwardCommandController implementation: https://github.com/ros-controls/ros2_controllers/blob/master/forward_command_controller/src/forward_command_controller.cpp
.. _Resource Manager: https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/src/resource_manager.cpp
.. _JointTrajectoryController: https://github.com/ros-controls/ros2_controllers/blob/master/joint_trajectory_controller/src/joint_trajectory_controller.cpp
.. _Node Lifecycle Design: https://design.ros2.org/articles/node_lifecycle.html
.. _ros2controlcli: https://github.com/ros-controls/ros2_control/tree/master/ros2controlcli
.. _Hardware Access through Controllers design document: https://github.com/ros-controls/roadmap/blob/master/design_drafts/hardware_access.md
.. _ROS2 Control Components URDF Examples design document: https://github.com/ros-controls/roadmap/blob/master/design_drafts/components_architecture_and_urdf_examples.md
.. _roadmap: https://github.com/ros-controls/roadmap
.. _ROS Discourse: https://discourse.ros.org


----

.. |date| date::
.. |time| date:: %H:%M

Built on |date| at |time| GMT

