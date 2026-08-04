:github_url: https://github.com/ros-controls/zephyr-zenoh-integration/blob/{REPOS_FILE_BRANCH}/zenbedded_rcl/doc/userdoc.rst

.. _zenbedded_rcl_userdoc:

zenbedded_rcl
=============

The embedded client library for Zephyr RTOS that provides a high-level API for ROS 2 communication.

.. warning::

   This package is **work in progress**.


Features
--------

- **Agent-less**: Communicates directly with ROS 2 via Zenoh, with no bridge
  process running on the MCU.
- **Double-buffered**: Lock-free state/command buffers with ABA-prevention
  versioning, safe for the control thread.
- **ROS-like API**: ``state()`` / ``command()`` accessors plus ``sync()`` to
  publish and consume buffers; an optional control thread is started via
  ``start_thread(control_freq)``.
- **Optimized**: Designed for a low memory footprint and real-time execution
  within the Zephyr thread model.


Setup
-----

``zenbedded_rcl`` is a Zephyr module; build it as part of a demo application
in a west workspace that also provides ``zenoh-pico`` (e.g. the Docker
development environment mounts ``demos/`` into a ready-made workspace):

.. code:: bash

   cd demos/inverted_pendulum
   west build -p always -b esp32s3_devkitc/esp32s3/procpu #your appropriate board
   west flash --esp-device /dev/ttyUSB0
