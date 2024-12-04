.. _roscon2024_workshop:

ROSCon 2024 Workshop
====================

**Location: Room 200**

**Time: 13:00-17:00, 21. October 2024**

ros2_control: Fun with Controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  .. image:: resources/images/ROSCon2024.png
      :scale: 50%

Summary
-------

If you already know that the ros2_control framework acts as a Kernel for ROS 2 robotics systems you are using but need help with application complexity, then this workshop is for you. The workshop covers the use of ros2_control controllers in products from various industries and shows solutions for all the little issues when running 24/7.

You will get a practical overview of concepts like controller chaining - used for cascade control and real-time state estimators; and asynchronous and "side-load" controllers that enable you to run complex calculations without jitter in your control loops. We expect your active involvement!

Before coming to the conference
-------------------------------

Recommended system setup:

* Ubuntu 22.04 or Ubuntu 24.04
* docker engine & docker compose installed

If you don't have docker, follow `the docker instructions <https://docs.docker.com/engine/install/ubuntu>`_ to install it.

If you don't have docker compose, run ``sudo apt-get install docker-compose-plugin`` or follow `the docker compose instructions <https://docs.docker.com/compose/install/linux/>`_ to install it.

Slides are available in `pdf <https://tinyurl.com/ros2control-pdf>`_ and `pptx <https://tinyurl.com/ros2control-pptx>`_ .

Try pulling the container we use prior to coming to the workshop:

.. code::

   docker pull bmagyar/roscon2024_workshop

People
------

This workshop was brought to you by

* Dr. Denis Stogl, `Stogl Robotics Consulting <https://stoglrobotics.de>`_
* Dr. Bence Magyar, `Locus Robotics <https://locusrobotics.com>`_
