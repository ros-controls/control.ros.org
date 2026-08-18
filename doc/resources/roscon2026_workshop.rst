.. _roscon2026_workshop:

ROSCon 2026 Workshop
=======================

.. figure:: images/ROSCon2026.png
    :figwidth: 40%
    :align: center

Scaling ros2_control: From Async Hardware Drivers to RL Inference Engines
-------------------------------------------------------------------------

This hands-on workshop breaks the synchronicity barrier. You will learn to architect Asynchronous Hardware Interfaces that prevent I/O bottlenecks and deploy RL Policy Models (via ONNX/Torch) as non-blocking controllers. We move beyond basic tutorials to tackle production-grade challenges: thread-safe data exchange, managing inference jitter, synchronizing drivers and controllers to the robot controllers, and maintaining real-time stability.

This workshop shows how to integrate ros2_control into your production systems as well as to deploy RL policies on to hardware.

Prerequisites - Before coming to the conference
-----------------------------------------------

1. It is recommended to have a **Linux-based OS** installed on your laptop (Recommended: Ubuntu 24.04). No ROS setup is required locally, as everything will run in Docker containers.

2. We need attendees to have `docker engine <https://docs.docker.com/engine/install/>`_ and the `docker compose <https://docs.docker.com/compose/install/linux/>`_ plugin installed. Installation instructions for various platforms can be found on the linked pages.

Pull as soon as you can to verify your setup and get the majority of the download but also try re-pulling closer to the date to make sure you have the latest updates!

.. code::

  wget https://tinyurl.com/roscontrol2026 -O docker-compose.yaml
  docker compose pull

For optimal copy&paste experience, you can pull the github repository. Some things are not yet finalized but pulling early and often is a good idea.

.. code::

  git clone https://github.com/ros-controls/roscon2026_control_workshop

People
------

This workshop was brought to you by

* Dr. Bence Magyar, Principal Software Engineer, `Locus Robotics <https://locusrobotics.com>`_
* Sai Kishor Kothakota, Robotics Engineer, `PAL Robotics <https://pal-robotics.com>`_
* Christoph Fröhlich, `AIT Austrian Institute of Technology GmbH <https://www.ait.ac.at>`_
* Dr.-Ing. Denis Stogl, `b>>robotized <https://en.b-robotized.com/>`_
* Marq Rasmussen, `KUKA Robotics <https://www.kuka.com>`_
