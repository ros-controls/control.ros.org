:github_url: https://github.com/ros-controls/zephyr-zenoh-integration/blob/{REPOS_FILE_BRANCH}/doc/architecture.rst

.. _architecture:

Architecture
============

The integration provides two architecture variants.

No-CDR architecture
-------------------

.. image:: images/nocdr_architecture.png
   :width: 300
   :align: center
   :alt: No-CDR architecture — uses zenbedded_schema, zenbedded_hardware_interface,
         and zenbedded_rcl with zero-copy packed binary over Zenoh

No intermediate serialization. ``zenbedded_schema`` defines the wire format,
``zenbedded_hardware_interface`` runs on the host, ``zenbedded_rcl`` runs on
the MCU — all sharing the same packed struct layout.

CDR architecture
----------------

.. image:: images/cdr_architecture.png
   :width: 300
   :align: center
   :alt: CDR architecture — uses topic_based_hardware_interface and
         zenbedded_transport with CDR serialization over Zenoh

Uses standard CDR serialization. ``topic_based_hardware_interface`` on the host
communicates with ``zenbedded_transport`` on the MCU over Zenoh topics.

Package responsibilities
------------------------

zenbedded_hardware_interface **[stable]**
    A ``ros2_control`` ``SystemInterface`` that connects to remote MCUs over
    Zenoh. It reads sensor state from a Zenoh subscriber and writes joint
    commands to a Zenoh publisher. Configuration (endpoint, topics, interface
    schema) is done via ROS 2 parameters. Uses ``realtime_tools::RealtimeBuffer``
    for lock-free state access.

zenbedded_schema **[stable]**
    A YAML-driven schema parser and C/C++ header generator that produces packed
    binary struct descriptions. Both the hardware interface (host-side) and the
    firmware (MCU-side) use the same schema to ensure wire-format compatibility.
    Has no ROS dependencies -- builds standalone or with ``colcon``.

zenbedded_transport **[WIP]**
    Zephyr module infrastructure for shared transport logic between the MCU
    and the ROS 2 host. Implements MicroCDR-style ``JointState`` and
    ``JointCommand`` serialization (the ``zcdr_*`` API in ``serialization.h``).
    A ``zenoh_transport`` wrapper, shared ``.msg`` definitions, and ROS
    name-to-Zenoh resource key mapping are planned.

zenbedded_rcl **[WIP]**
    An embedded client library for Zephyr RTOS that provides a high-level,
    ROS-like API for hardware components running on the MCU. The
    ``ZenbeddedClient`` (double-buffered state/command buffers with
    ABA-prevention versioning, Zenoh publish/subscribe, optional control
    thread) is implemented and ready for use. A ``HardwareComponent`` base
    class (lifecycle-managed sensor/actuator) and ``ComponentManager`` are
    planned but not yet implemented. Communicates directly with the host over
    Zenoh without any intermediate agent.
