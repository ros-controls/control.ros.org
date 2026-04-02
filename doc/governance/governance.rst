
.. _Governance:

Project Governance
==================

.. contents:: Table of Contents
   :depth: 2
   :local:

Since 2025, the ros-controls project has been governed by the `Open Source Robotics Alliance (OSRA) <https://osralliance.org/>`__.
The information below is meant to give a quick overview of the project governance, but for full information please see `the OSRA's website <https://osralliance.org/how-it-works/>`__.

The ros-controls Project Management Committee is responsible for the day-to-day operations of the ros-controls project.
The ros-controls PMC consists of the Project Leader, the ros-controls PMC Members (who have full voting rights), a Supporting Individual Representative, and the Chair of the TGC.
The project also has Committers, who help manage one or more repositories but are not a part of the PMC.
The Project Leader, all PMC Members, and all Committers are chosen on a meritocratic basis.

The day-to-day operations of the ros-controls PMC include managing the members and committers, managing the repositories that make up ros-controls, reviewing and merging code from the ros-controls community, maintaining the repositories, and making technical decisions that decide the direction of the project.

For more details about the ros-controls PMC, please see the `Charter for the ros-controls Project <https://github.com/openrobotics/osra-policies-and-procedures/pull/7>`__.

Current ros-controls PMC Constituents
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The ros-controls PMC currently consists of the following constituents:

.. list-table::
   :header-rows: 1

   * - Name
     - Affiliation
     - GitHub handle
     - PMC role
     - Time Zone (optional)
   * - Bence Maygar
     - `Locus Robotics <https://locusrobotics.com/>`_
     - `bmagyar <https://github.com/bmagyar>`_
     - Project Leader
     - GMT (UTC+0)
   * - Denis Stogl
     - `b»robotized <https://www.b-robotized.com/>`_
     - `destogl <https://github.com/destogl>`_
     - Project Co-Leader
     - CET (UTC+1)/CEST (UTC+2)
   * - Christoph Fröhlich
     - `AIT - Austrian Institute of Technology GmbH <https://www.ait.ac.at/>`_
     - `christophfroehlich <https://github.com/christophfroehlich>`_
     - Member
     - CET (UTC+1)/CEST (UTC+2)
   * -  Sai Kishor Kothakota
     - `PAL Robotics S.L <https://pal-robotics.com/>`_
     - `saikishor <https://github.com/saikishor>`_
     - Member
     - CET (UTC+1)/CEST (UTC+2)
   * - Marq Rasmussen
     - `Locus Robotics <https://locusrobotics.com/>`_
     - `MarqRazz <https://github.com/MarqRazz>`_
     - Member
     - MST (UTC-7)/MDT (UTC-6)

Current ros-controls Committers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The ros-controls committers (who are not also part of the ros-controls PMC) consists of the following constituents:

.. list-table::
   :header-rows: 1

   * - Name
     - Affiliation
     - GitHub handle
     - Time Zone (optional)
   * - Alejandro Hernandez Cordero
     - `Honu Robotics <https://www.honurobotics.com/>`_
     - `ahcorde <https://github.com/ahcorde>`_
     - CET (UTC+1)/CEST (UTC+2)
   * - Julia Jia
     - Independent
     - `Juliaj <https://github.com/Juliaj>`_
     - PST (UTC-8)/PDT (UTC-7)
   * - Nathan Dunkelberger
     - `NASA JSC Robotics <https://www.nasa.gov/reference/jsc-robotics/>`_
     - `ndunkelb-nasa <https://github.com/ndunkelb-nasa>`_
     - CST (UTC-6)/CDT (UTC-5)
   * - Emma Zemler
     - `NASA JSC Robotics <https://www.nasa.gov/reference/jsc-robotics/>`_
     - `ezemler-nasa <https://github.com/ezemler-nasa>`_
     - CST (UTC-6)/CDT (UTC-5)
   * - Michael Tobia
     - `NASA JSC Robotics <https://www.nasa.gov/reference/jsc-robotics/>`_
     - `mtobia-nasa <https://github.com/mtobia-nasa>`_
     - CST (UTC-6)/CDT (UTC-5)
   * - Erik Holum
     - `NASA JSC Robotics <https://www.nasa.gov/reference/jsc-robotics/>`_
     - `eholum-nasa <https://github.com/eholum-nasa>`_
     - EST (UTC-5)/EDT (UTC-4)

Past ros-controls PMC Constituents
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The ros-controls PMC thanks the following past constituents for their service:

.. list-table::
   :header-rows: 1

   * - Name
     - PMC role
     - GitHub handle (optional)
   * - None yet
     - None yet
     - None yet

Repositories managed by the ros-controls PMC
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The following repositories are managed by the ros-controls PMC:

.. list-table::
   :header-rows: 1

   * - Repository URL
     - Committers
   * - https://github.com/ros-controls/ros2_control
     - None yet
   * - https://github.com/ros-controls/ros2_controllers
     - None yet
   * - https://github.com/ros-controls/ros2_control_cmake
     - None yet
   * - https://github.com/ros-controls/ros2_control_ci
     - None yet
   * - https://github.com/ros-controls/ros2_control_demos
     - None yet
   * - https://github.com/ros-controls/control_msgs
     - None yet
   * - https://github.com/ros-controls/control_toolbox
     - None yet
   * - https://github.com/ros-controls/control.ros.org
     - None yet
   * - https://github.com/ros-controls/gz_ros2_control
     - Alejandro Hernandez Cordero
   * - https://github.com/ros-controls/kinematics_interface
     - None yet
   * - https://github.com/ros-controls/realtime_tools
     - None yet
   * - https://github.com/ros-controls/topic_based_hardware_interfaces
     - Marq Rasmussen
   * - https://github.com/ros-controls/onnxruntime_vendor
     - Julia Jia
   * - https://github.com/ros-controls/.github
     - None yet
   * - https://github.com/ros-controls/mujoco_ros2_control
     - Nathan Dunkelberger, Emma Zemler, Michael Tobia, Erik Holum

Releases, Versioning, and Public API
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
As ros-controls PMC is independent of the ROS PMC we thrive to follow `its strategy <https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Developer-Guide.html#quality-practices>`__
but have to adapt it according to our needs. This includes an asynchronous release cycle, where ros-controls repositories have reached a stable state for a ROS distro on **1st of October after an official ROS distribution release**.
It is very likely that ros2_control packages will be available via the ROS build farm earlier than this date.

This stable release for a ROS distro, called **stable ros2_control release** from now, will be announced on `ROS discourse <https://discourse.openrobotics.org/c/ros-controls/ros-controls-announce-news/107>`__ on time.

Versioning
~~~~~~~~~~

We will use the ROS-specific rules on top of ``semver's`` for versioning, but also adhere to some ros-controls-specific rules:

* Major version increments (i.e. API breaking changes) should not be made within a **stable ros2_control release**.

* ros2_control heavily relies on the usage of `pluginlib <https://index.ros.org/p/pluginlib/>`__. Therefore, we distinguish two types of compiled code: non-plugin code together with plugin base classes, and plugins itself.

  * ABI of plugins may change at every release, i.e., also within a **stable ros2_control release**. Plugins built by the buildfarm will still be loaded by pluginlib's class loader, but code linking against the exported libraries will break.
  * For non-plugin code, ABI breaks within a **stable ros2_control release** are less likely but still unavoidable to fix code, which is critical regarding safety aspects of robot control.

  .. important::

    * Always update your full ROS installation, not only a single package. For example, on Ubuntu run ``sudo apt update && sudo apt upgrade`` after you install a new package.
    * Recompile your custom code after updating any upstream ROS packages.

* The same applies for run-time behavior changes: Changes within a **stable ros2_control release** are less likely but still unavoidable to fix safety-critical behavior. Where possible, we try to keep the legacy behavior configurable together with a deprecation warning, see section below.

The ros2_control maintainer team considers safety a top priority and bugs or issues found in the framework may be back ported to distros regardless of API stability.
These issues will be discussed at the biweekly PMC meeting where the community can decide the best route forward.

Public API declaration
~~~~~~~~~~~~~~~~~~~~~~

According to ``semver``, every package must clearly declare a public API.

* For most C++ packages the declaration is any header that it installs. Private class members and methods are not part of the public API.

* For other languages like Python, a public API must be explicitly defined, so that it is clear what symbols can be relied on with respect to the versioning guidelines.

If something you are using is not explicitly listed as part of the public API in the package's documentation, then you cannot depend on it not changing between minor or patch versions.

Deprecation strategy
~~~~~~~~~~~~~~~~~~~~

Where possible, we will use the tick-tock deprecation and migration strategy for breaking changes (API or behavior-breaking changes).

* New deprecations can be run-time messages or compiler warnings expressing that the functionality is being deprecated. The functionality will be completely removed in any future release, or at latest in the next **stable ros2_control release** (there may be details in the deprecation note).

* New deprecations can also come in every release of **stable ros2_control release** by performing backports of changes from the rolling version. These are meant to help users migrate early, however the functionality will remain available in that specific ROS distribution.

Have a look at :ref:`release_notes` and :ref:`migration`, where we will highlight necessary changes within every ros2_control version of a ROS distro.

.. important::

  Don't use compiler flags like ``-Wall -Werror`` in your development environment, as they may cause unnecessary build failures if deprecation notes are added.

Example of function ``foo`` deprecated and replaced by function ``bar``:

.. list-table::
   :header-rows: 1

   * - Package version
     - Description
     - API
   * - x.y.z, <x-1>.y.z
     - Original version
     - ``void foo();``
   * - x.<y+1>.z
     - New feature including deprecation
     - | ``[[deprecated("use bar()")]] void foo();``
       | ``void bar();``
   * - <x-1>.<y+1>.z
     - Backport to **stable ros2_control release**
     - | ``[[deprecated("use bar()")]] void foo();``
       | ``void bar();``
   * - x.<y+2>.z
     - New release of development version
     - ``void bar();``
   * - <x-1>.<y+2>.z
     - New release of **stable ros2_control release**
     - | ``[[deprecated("use bar()")]] void foo();``
       | ``void bar();``
