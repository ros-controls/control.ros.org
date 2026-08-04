:github_url: https://github.com/ros-controls/zephyr-zenoh-integration/blob/{REPOS_FILE_BRANCH}/zenbedded_transport/doc/userdoc.rst

.. _zenbedded_transport_userdoc:

zenbedded_transport
===================

Shared MicroCDR wire-format serialization for ``JointState`` / ``JointCommand``
payloads, so the ROS 2 host can unpack MCU publications directly (e.g. via
``topic_based_hardware_interface``).

.. warning::

   This package is **work in progress**.


Contents
--------

Implemented:

- **Serialization**: MicroCDR-style ``JointState`` and ``JointCommand``
  serialization via the ``zcdr_*`` API in ``serialization.h``, shared by the
  C (Zephyr) and C++ (ROS 2) builds.

Planned:

- **IDL Definitions**: Shared ``.msg`` files defining the control interface.
- **Type Mapping**: Logic that maps ROS 2 interface names to Zenoh resource
  keys.
- **Zenoh transport**: A ``zenoh_transport`` wrapper around the Zenoh session.


Usage
-----

The ``zcdr_*`` API in ``serialization.h`` covers the CDR payload lifecycle:

- ``zcdr_init_joint_state`` / ``zcdr_init_joint_command`` lay out the static
  CDR buffer and precompute the field offsets.
- ``zcdr_serialize_joint_state`` / ``zcdr_serialize_joint_command`` fill the
  buffer with a ``JointState`` / ``JointCommand`` payload.
- ``zcdr_deserialize_joint_state`` / ``zcdr_deserialize_joint_command`` unpack
  a CDR buffer back into primitive arrays.

``demos/zenbedded_test_node`` exercises the full serialize/publish path.
