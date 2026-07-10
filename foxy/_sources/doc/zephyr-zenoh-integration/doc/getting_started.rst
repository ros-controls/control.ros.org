:github_url: https://github.com/ros-controls/zephyr-zenoh-integration/blob/{REPOS_FILE_BRANCH}/doc/getting_started.rst

.. _zephyr_zenoh_getting_started:

Getting Started
===============

This guide walks you through setting up the Zephyr Zenoh Integration end-to-end.


Prerequisites
-------------

- ROS 2 Jazzy (or later)
- A Zephyr-supported board (ESP32-S3 reference target)
- ``west`` installed and configured


1. Build the ROS 2 workspace
----------------------------

.. code:: bash

   mkdir -p ~/zephyr_zenoh_ws/src
   cd ~/zephyr_zenoh_ws/src
   git clone git@github.com:ros-controls/zephyr-zenoh-integration.git
   cd ~/zephyr_zenoh_ws
   rosdep install --from-paths src --ignore-src -r -y
   colcon build


2. Build and flash a demo firmware
----------------------------------

.. code:: bash

   cd ~/zephyr_zenoh_ws/src/zephyr-zenoh-integration/demos/inverted_pendulum
   west init -l .
   west update
   west build -p always -b esp32s3_devkitc/esp32s3/procpu
   west flash --esp-device /dev/ttyUSB0


3. Run the hardware interface (ROS 2 host)
------------------------------------------

The hardware interface is loaded via the ``ros2_control`` plugin system.
You can test it with the included test suite:

.. code:: bash

   source ~/zephyr_zenoh_ws/install/setup.bash
   colcon test --packages-select zenbedded_hardware_interface

For integration with a real MCU, the ``zenbedded_hardware_interface`` must be instantiated in a ``ros2_control`` URDF with the required parameters. 
See the :ref:`zenbedded_hardware_interface_userdoc` for parameter details.


4. Verify
---------

.. code:: bash

   # Verify the plugin is registered
   ros2 control list_hardware_interfaces

   # Monitor Zenoh topics (if MCU is publishing)
   zenohc sub rt/robot/state
