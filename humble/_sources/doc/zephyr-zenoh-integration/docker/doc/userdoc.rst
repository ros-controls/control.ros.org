:github_url: https://github.com/ros-controls/zephyr-zenoh-integration/blob/{REPOS_FILE_BRANCH}/docker/doc/userdoc.rst

.. _docker_userdoc:

Docker
======

A Docker-based development environment for working with Zenbedded without installing dependencies locally.


Usage
-----

.. code:: bash

   docker compose up -d
   docker compose exec -it dev-env bash


Includes
--------

The image is based on ``ros:lyrical-ros-base`` and adds:

- ``ros-lyrical-ros2-control``, ``ros-lyrical-ros2-controllers``
- ``rmw_zenoh_cpp`` (ROS 2 middleware)
- Zephyr SDK 1.0.1 with ESP32 (``xtensa-espressif_esp32_zephyr-elf``,
  ``xtensa-espressif_esp32s3_zephyr-elf``) and ARM toolchains
- ``west`` and Zephyr Python dependencies
- ``zenoh-pico`` (via ``west.yml`` manifest)

Environment variables set in the container:

- ``ZEPHYR_BASE=/zephyr_ws/zephyr``
- ``ZEPHYR_SDK_INSTALL_DIR=/zephyr_ws/zephyr-sdk-1.0.1``
- ``RMW_IMPLEMENTATION=rmw_zenoh_cpp``


Volume mounts
-------------

- ``zenbedded_transport`` → ROS 2 workspace ``/ros2_ws/src/`` and Zephyr
  workspace ``/zephyr_ws/``
- ``zenbedded_hardware_interface`` → ROS 2 workspace ``/ros2_ws/src/``
- ``zenbedded_rcl`` → Zephyr workspace ``/zephyr_ws/``
- ``demos/`` → Zephyr workspace ``/zephyr_ws/``

Networking uses ``host`` mode and the container runs privileged
(for USB-based flashing).
