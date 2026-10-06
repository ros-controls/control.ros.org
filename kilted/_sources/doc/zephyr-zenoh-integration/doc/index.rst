:github_url: https://github.com/ros-controls/zenbedded/blob/{REPOS_FILE_BRANCH}/doc/index.rst

.. _zenbedded:

Zenbedded
=========

Better embedded integration for ROS 2 robots: bringing microcontrollers running `Zephyr RTOS`_ into ``ros2_control`` as first-class participants, with agent-less, low-latency communication over `Zenoh`_.

`Link to GitHub Repository <https://github.com/ros-controls/zenbedded>`_


Packages
********

.. toctree::
   :titlesonly:

   Hardware Interface <../zenbedded_hardware_interface/doc/userdoc.rst>
   Transport <../zenbedded_transport/doc/userdoc.rst>
   Firmware Client Library <../zenbedded_rcl/doc/userdoc.rst>


Guides and Examples
*******************

.. toctree::
   :titlesonly:

   Getting Started <getting_started.rst>
   Architecture <architecture.rst>
   Demos <../demos/doc/userdoc.rst>
   Docker <../docker/doc/userdoc.rst>
   Benchmarking <benchmarking.rst>


.. _Zephyr RTOS: https://zephyrproject.org/
.. _Zenoh: https://zenoh.io/
