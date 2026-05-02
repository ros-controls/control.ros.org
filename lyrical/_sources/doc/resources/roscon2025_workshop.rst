.. _roscon2025_workshop:

ROSCon 2025 Workshop
=======================


.. list-table::
    :widths: 50 50
    :align: center

    * - .. figure:: images/ROSCon2025.png

      - .. figure:: images/ROSConUK2025.png
          :figwidth: 75%

ros2_control: Writing Custom Robot Drivers
------------------------------------------

ros2_control is a hardware-agnostic control framework for abstracting hardware and low-level control for 3rd party solutions like ``MoveIt2`` and ``Nav2`` systems.

This workshop provides a practical deep dive into writing robot drivers with ros2_control. You will be introduced to hands-on integration of an embedded board that implements a differential drive robot.

Additionally, we'll demonstrate examples from different domains and best practices for using ros2_control for ease of use, increased flexibility and robustness.

Prerequisites - Before coming to the conference
-----------------------------------------------

1. Please bring a **USB-C cable you can plug to your laptop**! It should be power- and data-capable.

2. It is recommended to have a **Linux-based OS** installed on your laptop (Recommended: Ubuntu 22.04 or 24.04). No ROS setup is required locally, as everything will run in Docker containers.

3. We need attendees to have `docker engine <https://docs.docker.com/engine/install/>`_ and the `docker compose <https://docs.docker.com/compose/install/linux/>`_ plugin installed. Installation instructions for various platforms can be found on the linked pages.

Pull as soon as you can to verify your setup and get the majority of the download but also try re-pulling closer to the date to make sure you have the latest updates!

.. code::

  wget https://tinyurl.com/roscontrol2025 -O docker-compose.yaml
  docker compose pull

For optimal copy&paste experience, you can pull the github repository. Some things are not yet finalized but pulling early and often is a good idea.

.. code::

  git clone https://github.com/ros-controls/roscon2025_control_workshop

Slides
------

:download:`Slides for: ros2_control: Fun with Robot Drivers <presentations/ROSCon2025-Workshop_Fun_with_Robot_Drivers.pdf>`

People
------

This workshop was brought to you by

* Dr.-Ing. Denis Stogl, `b>>robotized <https://en.b-robotized.com/>`_
* Dr. Bence Magyar, `Locus Robotics <https://locusrobotics.com>`_
* Marq Rasmussen, `Locus Robotics <https://locusrobotics.com>`_
* Sai Kishor Kothakota, `PAL Robotics <https://pal-robotics.com>`_
* Christoph Fröhlich, `Austrian Institute Of Technology <https://www.ait.ac.at>`_
