.. _roscon2023_workshop:

ROSCon 2023 Workshop
====================

Slides

`Presentation: ros2_control - ros2_control on Steroids <https://github.com/ros-controls/control.ros.org/blob/master/doc/resources/ROSCon2023_Workshop_ros2_control_on_Steroids.pdf>`_

**Location: Imperial 9**

**Time: 13:00-17:00, 18. October 2023**

ros2_control on Steroids
^^^^^^^^^^^^^^^^^^^^^^^^

  .. image:: ../images/ROSCon2023.jpg
      :scale: 50%

Summary
-------

If you already know that the ros2_control framework acts as a Kernel for ROS 2 robotics systems, you are using it but struggling with application complexity, then this workshop is for you. The workshop covers the use of ros2_control in products from various industries and shows solutions for all the little issues when running 24/7.

You will get a practical overview of concepts like controller chaining, hardware modularization, multi-robot architectures, parameter injection, and debugging of complex systems. On top of showcasing these functionalities, we expect your involvement in the discussion by bringing your complex application and discussing existing and potentially missing tooling in ros2_control.

Before coming to the conference
-------------------------------

Recommended system setup:

* Ubuntu 20.04 or Ubuntu 22.04
* docker engine & docker compose installed

If you don't have docker, follow `the docker instructions <https://docs.docker.com/engine/install/ubuntu>`_ to install it.

If you don't have docker compose, run ``sudo apt-get install docker-compose-plugin`` or follow `the docker compose instructions <https://docs.docker.com/compose/install/linux/>`_ to install it.

Once done, grab the latest version of the workshop container by running:

``docker pull bmagyar/roscon2023_workshop:latest``

Now, to set up a workspace, run the following commands where you want this to be placed:

  .. code-block:: shell

    mkdir -p ws/src
    cd ws/src
    git clone https://github.com/ros-controls/roscon2023_control_workshop
    vcs import --input roscon2023_control_workshop/roscon2023_control_workshop.ci.repos .


You can run things locally if you have all dependencies set up.
The alternative is using the container which includes all dependencies & comes ready to compile the workspace. Using the same terminal as before (or a new one parked at ``ws/src``) run:

  .. code-block:: shell

    docker compose -f roscon2023_control_workshop/docker/docker-compose.yaml run dev
    tmux
    source /opt/ros/rolling/setup.bash
    colcon build --symlink-install
    source install/setup.bash


Open 2 more terminals in ``tmux`` by using ``CTRL+B`` and ``"`` and ``CTRL+B`` and ``%``.

You can navigate in tmux using ``CTRL+B`` and ``ARROW`` keys.

People
------

This workshop was brought to you by

* Denis Stogl
* Bence Magyar
