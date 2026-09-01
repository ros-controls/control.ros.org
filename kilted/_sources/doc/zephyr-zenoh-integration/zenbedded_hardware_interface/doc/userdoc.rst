:github_url: https://github.com/ros-controls/zephyr-zenoh-integration/blob/{REPOS_FILE_BRANCH}/zenbedded_hardware_interface/doc/userdoc.rst

.. _zenbedded_hardware_interface_userdoc:

zenbedded_hardware_interface
============================

A ``ros2_control`` ``SystemInterface`` implementation that communicates with microcontrollers via Zenoh.
Reads sensor state from a Zenoh subscriber and writes joint commands to a Zenoh publisher.


Setup
-----

Build the workspace from source with ``colcon``:

.. code:: bash

   mkdir -p ~/zephyr_zenoh_ws/src
   cd ~/zephyr_zenoh_ws/src
   git clone git@github.com:ros-controls/zephyr-zenoh-integration.git
   cd ~/zephyr_zenoh_ws
   rosdep install --from-paths src --ignore-src -r -y
   colcon build


Features
--------

- Agent-less communication -- no ROS 2 bridge process required on the MCU
- Lock-free state buffer using ``realtime_tools::RealtimeBuffer``
- YAML-driven interface schema (via ``zenbedded_transport``) for wire-format
  compatibility between host and firmware
- Configurable Zenoh endpoint, mode (client/peer), and topic keys
- Interfaces declared dynamically from the schema YAML


Parameters
----------

The following parameters are read from the ``ros2_control`` URDF ``<param>``
tags:

.. list-table::
   :header-rows: 1

   * - Parameter
     - Type
     - Required
     - Default
     - Description
   * - ``zenoh_endpoint``
     - string
     - yes
     - --
     - Zenoh endpoint (e.g. ``tcp/127.0.0.1:7447``)
   * - ``state_topic``
     - string
     - yes
     - --
     - Zenoh key expression for MCU state publications
   * - ``command_topic``
     - string
     - yes
     - --
     - Zenoh key expression for host command publications
   * - ``schema_path``
     - string
     - yes
     - --
     - Path to the interface schema YAML file
   * - ``zenoh_mode``
     - string
     - no
     - ``"client"``
     - Zenoh session mode (``"client"`` or ``"peer"``)

Example URDF snippet:

.. code:: xml

   <ros2_control name="ZenbeddedSystem" type="system">
     <hardware>
       <plugin>zenbedded::ZenbeddedHardware</plugin>
       <param name="zenoh_endpoint">tcp/192.168.1.100:7447</param>
       <param name="zenoh_mode">client</param>
       <param name="state_topic">rt/robot/state</param>
       <param name="command_topic">rt/robot/command</param>
       <param name="schema_path">$(find zenbedded_hardware_interface)/config/interface_schema.yaml</param>
     </hardware>
     <joint name="motor_arm">
       <state_interface name="position"/>
       <command_interface name="position"/>
     </joint>
     <joint name="pendulum_axis">
       <state_interface name="position"/>
     </joint>
   </ros2_control>


Example schema (``config/interface_schema.yaml``)
--------------------------------------------------

.. code:: yaml

   state_interfaces:
     motor_arm:
       position: float32
     pendulum_axis:
       position: float32
   command_interfaces:
     motor_arm:
       position: float32

This generates the following packed C struct via ``zenbedded_transport``:

.. code:: c

   typedef struct {
     float motor_arm_position;
     float pendulum_axis_position;
   } zenbedded_state_t;

   typedef struct {
     float motor_arm_position;
   } zenbedded_command_t;


Lock-free state access
----------------------

State updates from the Zenoh callback are written into a
``realtime_tools::RealtimeBuffer<std::vector<uint8_t>>``, allowing the
real-time control loop to read the latest state snapshot without
blocking on the network callback.
