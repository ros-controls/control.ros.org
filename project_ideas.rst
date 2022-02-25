.. _project_ideas:

Project Ideas for GSoC 2022
=============================

Asynchronous Control Components
--------------------------------
This project orbits around everything asynchronous: hardware components and controllers.
The goal is to support long-running controller tasks by providing a simple and clean interface for users implementing such controllers.
This will include addition of multi-threading support with protected memory access, e.g., through buffers that are implemented in `control_toolbox repository <https://github.com/ros-controls/control_toolbox>`_.

The second part enables asynchronous access to hardware. This should address issues our users are having when hardware-communication frequency has to be stable at high rate, e.g. 1 kHz, or when they are running multi-hardware setup on various frequencies.
Also, another use-case of this functionality is to enable update of different interfaces of the same hardware on variable rate. For example, "heartbeat" signal should be sent only once per second, but the robot's motors should be controlled at 500 Hz.
Related issues for this functionality should address are:

- `Communication Failures with Universal Robots <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/210>`_
- `Writing interfaces at different rates <https://github.com/ros-controls/ros2_control/issues/649>`_

This project is defined by but not limited to the relevant design drafts:

- `Asynchronous Controllers <https://github.com/ros-controls/roadmap/blob/master/design_drafts/async_controller.md>`_
- `Asynchronous and Trigger Interfaces <https://github.com/ros-controls/roadmap/pull/52>`_

Skills required/preferred:

- Good C++ skills
- C++ asynchronous libraries / tricks

Possible mentors: Bence Magyar, Denis Štogl 
Expected size of project: 350 hours
Difficulty: hard


Tutorials and Demos for ros2_control
------------------------------------

This project is about extending our tutorials and examples on how to use the ros2_control framework.
Also, existing demos should be integrated with our documentation at `control.ros.org <https://control.ros.org>`_.
In general, the following tasks are envisioned:

- Examples on simulators' integration (Gazebo and Ignition) and their use; 
- Showcase multi-robot and multi-controller manager use-case with examples and by using simulators
- Showcase: How to do Error Management with ros2_control (graceful error handling and degradation)
- Implementing different example robot-hardware architectures for industrial and service robotics
- Implementing use-cases from `roadmap <https://github.com/ros-controls/roadmap>`_ repository

The all code will be placed into our `demo repository <https://github.com/ros-controls/ros2_control_demos/>`_.

Related design drafts:

- `Robot-Hardware Architectures <https://github.com/ros-controls/roadmap/blob/master/design_drafts/components_architecture_and_urdf_examples.md>`_
- `GPIO Interfaces <https://github.com/ros-controls/roadmap/blob/master/design_drafts/non_joint_command_interfaces.md>`_
- `Showcase mode-switching and conflict check <https://github.com/ros-controls/roadmap/blob/master/design_drafts/mode_switching_and_conflict_check.md>`_
- `Movement-/Safety-critical Interfaces <https://github.com/ros-controls/roadmap/pull/51>`_


Skills required/preferred:

- Good C++ skills
- Basic understanding of ROS and/or ROS2
- Familiarity with the Gazebo simulator

Possible mentors: Denis Štogl
Expected size of project: 350 hours
Difficulty: medium


Mission-Control for ros2_control
----------------------------------

The ros2_control framework focuses on direct management of hardware and their controllers to enable real-time capabilities with ROS.
Although those interfaces are easy to use, they provide only "manual" control over the system's state.
Therefore, very often in more complex systems our users have to implement an external, state-machine like component that serves as an orchestrator for ros2_control.
The main purpose of such a component is to serve as conductor of ros2_control by taking care of scenarios and making sure that at the appropriate moment the right controllers and hardware are in expected states.

This functionality should replace some high-level components currently used, e.g., MoveIt2-"SimpleControllerManager".

The main functionalities for the components and goals of the project are:

- Defining a scenario in form of a multi-robot and multi-tool configuration and its behavior that serves as a benchmark.
- Extending controller_manager with status publisher, providing all needed data to a mission-control component.
- Adding status topics to all standard controllers.
- Defining format of a YAML file where users can configure controller presets.
- Implementing the mission-control module/script that sets the controller_manager, i.e., the ros2_control framework, in a specific configuration/state.

Skills required/preferred:

- Good C++ skills
- Basic understanding of ROS and/or ROS2

Possible mentors: Bence Magyar, Denis Štogl
Expected size of project: 350 hours
Difficulty: medium


Add support for hardware semantic components
--------------------------------------------

The ros2_control framework relies on simple command and state interfaces in the form of double values to exchange data between hardware components and controllers. It is desired however to provide good C++ data structures both on the hardware component and the controller side which improves code readability and maintainability. 

Earlier in the project, the concept of semantic components were introduced which essentially provides a grouping for these values and a semantic-specific API to use them. For instance, an IMU sensor will typically report 9 values, 3 values for each axis of the acelerometer, gyroscope and compass sensor parts respectively. Such values can be grouped and served through an API as a ROS IMU message or as a C++ struct for both input and output of these values while the ros2_control framework maintains it's low-profile communication interfaces internally.

The goal of this project is to add semantic components that are relevant for hardware components.

ADD DESCRIPTION HERE DENIS

Additionally, this project includes extending the existing simulation tools with a set of common semantic components to support different sensors and actuators.

Skills required/preferred:

- Good C++ skills
- Basic understanding of ROS and/or ROS2
- Basic understanding of the Gazebo simulator

Possible mentors: Bence Magyar, Denis Štogl
Expected size of project: 175 hours
Difficulty: medium


Feature-parity for controllers from ROS1
----------------------------------------

The ros2_control framework in ROS2 is a rewrite of the ros_control framework from ROS1.
Our rich set of standard controllers was one of the main motivations for users to adopt ros_control in ROS1 and while we ported most of them, there are quite a few features missing for the two main controllers of this set, the diff_drive_controller and the joint_trajectory_controller.

This work will consist of reviewing the two versions of the two controllers and comparing for feature parity. Once the missing parts are identified, port them over from ROS1 with as much test support as possible.

Related existing issues are:
- https://github.com/ros-controls/ros2_controllers/issues/303
- https://github.com/ros-controls/ros2_controllers/issues/304

Stretch goals:
- https://github.com/ros-controls/realtime_tools/issues/81
- https://github.com/ros-controls/ros2_controllers/issues/302

Skills required/preferred:

- Good C++ skills
- Basic understanding of ROS and/or ROS2
- Basic understanding of unit testing with gmock

Possible mentors: Bence Magyar
Expected size of project: 350 hours
Difficulty: medium
