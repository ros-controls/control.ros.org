.. _roscon2023_workshop:

ROSCon 2023 Workshop
====================

**Location: Room 202**

**Time: 13:00-17:00, 18. October 2023**

ros2_control: Fun with Controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  .. image:: resources/images/ROSCon2024.jpg
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

Once done, grab the latest version of the workshop container by running:

``docker pull bmagyar/roscon2024_workshop:latest``

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

* Dr. Denis Stogl
* Dr. Bence Magyar
