:github_url: https://github.com/ros-controls/zephyr-zenoh-integration/blob/{REPOS_FILE_BRANCH}/demos/doc/userdoc.rst

.. _demos_userdoc:

Demos
=====

.. warning::

   This package is **work in progress**.

inverted_pendulum
-----------------

The zenbedded Zephyr modules will communicate to ``zenbedded_hardware_interface`` via Zenoh, where a controller from ``ros2_control`` balances the pendulum.

rcl_cpp_test
------------

A C++ demo that exercises the ``zenbedded_rcl`` client together with the
Zephyr network stack and Zenoh publish/subscribe.

zenbedded_test_node
-------------------

Validates CDR ``JointState`` / ``JointCommand`` payload extraction using the
``zenbedded_transport`` serialization helpers. Buildable for ``native_sim``
and ``esp32s3_devkitc/esp32s3/procpu``; run via ``twister``.
