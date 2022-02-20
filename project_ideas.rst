.. _project_ideas:

Project Ideas
=============

Asynchronous Components
-----------------------

This project orbits around everything asynchronous: hardware components and controllers.
It is made up of 2 distinct parts mentioned above.
This project is defined by but not limited to the relevant design drafts:
- TBD


Tutorials and Demos for ros2_control
------------------------------------

This project is about extending our simulator integration, suite of implemented examples and demoing industrial use-cases.
The main repo to work into will be https://github.com/ros-controls/ros2_control_demos/

- Simulators integration
- Implement examples from roadmap repo
- Multi-robot, Multi-controller manager simulation
- Use-cases: Error management
- Different Robot-Hardware Architectures

Related design drafts:
- TBD

Mission-Control for ros2_control (ros2c)
----------------------------------------

Main goals are
- Should replace MoveIt “SimpleControllerManager”
- Add topic of events to controller_manager
- Report status of controller_manager internals
- Allow defining presets of controllers for mode
