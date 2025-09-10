.. _roscon2025_workshop:

ROSConUK 2025 Workshop
=======================

  .. image:: images/ROSConUK2025.png
      :width: 50%

ros2_control: Writing Custom Robot Drivers
------------------------------------------

ros2_control is a hardware-agnostic control framework for abstracting hardware and low-level control for 3rd party solutions like ``MoveIt2`` and ``Nav2`` systems. This workshop provides a practical deep dive into writing robot drivers with ros2_control. You will be introduced to hands-on integration of an embedded board that implements a differential drive robot. Additionally, we'll demonstrate examples from different domains and best practices for using ros2_control for ease of use, increased flexibility and robustness.

Before coming to the conference
-------------------------------
Please bring a *USB-C cable you can plug to your laptop*! It should be power- and data-capable. I'll need attendees to have docker and the docker-compose plugin installed. Installation instructions for various platforms are `here <https://docs.docker.com/engine/install/>`__.

Pull as soon as you can to verify your setup and get the majority of the download but also try re-pulling closer to the date to make sure you have the latest updates!

.. code::

  wget https://tinyurl.com/roscontrol2025 -O docker-compose.yaml
  docker compose pull

For optimal copy&paste experience, you can pull the github repository. Some things are not yet finalized but pulling early and often is a good idea.

.. code::

  git pull https://github.com/ros-controls/roscon2025_control_workshop


People
------

This workshop was brought to you by

* Dr. Bence Magyar, `Locus Robotics <https://locusrobotics.com>`_
