.. _documentation_home:

Welcome to the ros2_control documentation!
==========================================

.. toctree::
   :hidden:

   doc/roscon2024_workshop.rst
   doc/getting_started/getting_started.rst
   doc/ros2_control/doc/index.rst
   doc/ros2_controllers/doc/controllers_index.rst
   doc/ros2_control_demos/doc/index.rst
   doc/ros2_control/ros2controlcli/doc/userdoc.rst
   doc/simulators/simulators.rst
   doc/release_notes/release_notes.rst
   doc/migration/migration.rst
   doc/api_list/api_list.rst
   doc/supported_robots/supported_robots.rst
   doc/resources/resources.rst
   doc/contributing/contributing.rst
   doc/project_ideas.rst
   doc/acknowledgements/acknowledgements.rst

The ros2_control is a framework for (real-time) control of robots using (`ROS 2 <https://docs.ros.org/en/rolling/>`_).
Its packages are a rewrite of `ros_control <http://wiki.ros.org/ros_control>`_ packages used in ``ROS`` (`Robot Operating System <https://wiki.ros.org>`_).
ros2_control's goal is to simplify integrating new hardware and overcome some drawbacks.

If you are not familiar with the control theory, please get some idea about it (e.g., at `Wikipedia <https://en.wikipedia.org/wiki/Control_theory>`_) to get familiar with the terms used in this manual.


``ros2_control`` Repositories
-------------------------------------------
The framework consists of the following Github repositories hosted under the `ros-controls`_ Github organization:

* `ros2_control`_ - the main interfaces and components of the framework;
* `ros2_controllers`_ - widely used controllers, such as forward command controller, joint trajectory controller, differential drive controller;
* `control_toolbox`_ - some widely-used control theory implementations (e.g. PID) used by controllers;
* `realtime_tools`_ - general toolkit for realtime support, e.g., realtime buffers and publishers;
* `control_msgs`_ - common messages;
* `kinematics_interface`_ - for using C++ kinematics frameworks;
* `gz_ros2_control`_ - Plugin for Gazebo;


Additionally, the following (unreleased) packages are relevant for documentation and project management:

* `ros2_control_demos`_ - example implementations of common use-cases for a smooth start;
* `roadmap`_ - planning and design docs for the project;
* `ros2_control_ci`_ - reusable Github actions;
* `ros2_debian`_ - Docker images for Debian CI jobs;
* `ros2_rhel`_ - Docker images for RHEL CI jobs;
* `ros2_control_cmake`_ - CMake macros for the project;
* `ros2_control_ci`_ - reusable Github actions;
* `control.ros.org`_ - this documentation page.

Development Organisation and Communication
-------------------------------------------

Questions
   Please use `Robotics Stack Exchange <https://robotics.stackexchange.com>`_ and tag your questions with ``ros2_control``.

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

General discussions
   Please use `ROS Discourse`_.


.. _ros2_control: https://github.com/ros-controls/ros2_control
.. _ros2_controllers: https://github.com/ros-controls/ros2_controllers
.. _control_msgs: https://github.com/ros-controls/control_msgs
.. _realtime_tools: https://github.com/ros-controls/realtime_tools
.. _control_toolbox: https://github.com/ros-controls/control_toolbox
.. _kinematics_interface: https://github.com/ros-controls/kinematics_interface
.. _ros2_control_demos: https://github.com/ros-controls/ros2_control_demos
.. _gz_ros2_control: https://github.com/ros-controls/gz_ros2_control
.. _ros2_control_ci: https://github.com/ros-controls/ros2_control_ci
.. _ros2_control_cmake: https://github.com/ros-controls/ros2_control_cmake
.. _ros2_debian: https://github.com/ros-controls/ros2_debian
.. _ros2_rhel: https://github.com/ros-controls/ros2_rhel
.. _control.ros.org: https://github.com/ros-controls/control.ros.org
.. _ros-controls: https://github.com/ros-controls
.. _controller_manager_msgs: https://github.com/ros-controls/ros2_control/tree/{REPOS_FILE_BRANCH}/controller_manager_msgs
.. _Controller Manager: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/controller_manager/src/controller_manager.cpp
.. _ControllerInterface: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/controller_interface/include/controller_interface/controller_interface.hpp
.. _ros2_control node: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/controller_manager/src/ros2_control_node.cpp

.. _Resource Manager: https://github.com/ros-controls/ros2_control/blob/{REPOS_FILE_BRANCH}/hardware_interface/src/resource_manager.cpp
.. _Node Lifecycle Design: https://design.ros2.org/articles/node_lifecycle.html
.. _ros2controlcli: https://github.com/ros-controls/ros2_control/tree/{REPOS_FILE_BRANCH}/ros2controlcli
.. _Hardware Access through Controllers design document: https://github.com/ros-controls/roadmap/blob/master/design_drafts/hardware_access.md
.. _ROS2 Control Components URDF Examples design document: https://github.com/ros-controls/roadmap/blob/master/design_drafts/components_architecture_and_urdf_examples.md
.. _roadmap: https://github.com/ros-controls/roadmap
.. _ROS Discourse: https://discourse.ros.org


----

.. |date| date::
.. |time| date:: %H:%M

Built on |date| at |time| GMT
