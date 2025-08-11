
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
     - Not Yet Available
   * - https://github.com/ros-controls/ros2_controllers
     - Not Yet Available
   * - https://github.com/ros-controls/ros2_control_cmake
     - Not Yet Available
   * - https://github.com/ros-controls/ros2_control_ci
     - Not Yet Available
   * - https://github.com/ros-controls/ros2_control_demos
     - Not Yet Available
   * - https://github.com/ros-controls/control_msgs
     - Not Yet Available
   * - https://github.com/ros-controls/control_toolbox
     - Not Yet Available
   * - https://github.com/ros-controls/control.ros.org
     - Not Yet Available
   * - https://github.com/ros-controls/gz_ros2_control
     - Alejandro Hernandez Cordero
   * - https://github.com/ros-controls/kinematics_interface
     - Not Yet Available
   * - https://github.com/ros-controls/realtime_tools
     - Not Yet Available
   * - https://github.com/ros-controls/topic_based_hardware_interfaces
     - Marq Rasmussen
   * - https://github.com/ros-controls/.github
     - Not Yet Available

Versioning, Releases and Public API
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

We will use the `Semantic Versioning guidelines <http://semver.org/>`__ (``semver``) for versioning.

We will also adhere to some ROS-specific rules built on top of ``semver's`` full meaning:

* Major version increments (i.e. breaking changes) should not be made within a released ROS distribution.

  * Patch (interface-preserving) and minor (non-breaking) version increments do not break compatibility, so these sorts of changes *are* allowed within a release.

  * Major ROS releases are the best time to release breaking changes.
    If a core package needs multiple breaking changes, they should be merged into their integration branch (e.g. rolling) to allow catching problems in CI quickly, but released together to reduce the number of major releases for ROS users.

  * Though major increments require a new distribution, a new distribution does not necessarily require a major bump (if development and release can happen without breaking API).

* For compiled code, the ABI is considered part of the public interface.
  Any change that requires recompiling dependent code is considered major (breaking).

  * ABI breaking changes *can* be made in a minor version bump *before* a distribution release (getting added to the rolling release).

* We enforce API stability for core packages in Dashing and Eloquent even though their major version components are ``0``, despite `SemVer's specification <https://semver.org/#spec-item-4>`_ regarding initial development.

  * Subsequently, packages should strive to reach a mature state and increase to version ``1.0.0`` so to match ``semver's`` specifications.

Caveats
~~~~~~~

These rules are *best-effort*.
In unlikely, extreme cases, it may be necessary to break API within a major version/distribution.
Whether an unplanned break increments the major or minor version will be assessed on a case-by-case basis.

For example, consider a situation involving released X-turtle, corresponding to major version ``1.0.0``, and released Y-turtle, corresponding to major version ``2.0.0``.

If an API-breaking fix is identified to be absolutely necessary in X-turtle, bumping to ``2.0.0`` is obviously not an option because ``2.0.0`` already exists.

The solutions for handling X-turtle's version in such a case, both non-ideal, are:

1. Bumping X-turtle's minor version: non-ideal because it violates SemVer's principle that breaking changes must bump the major version.

2. Bumping X-turtle's major version past Y-turtle (to ``3.0.0``): non-ideal because the older distro's version would become higher than the already-available version of a newer distro, which would invalidate/break version-specific conditional code.

The developer will have to decide which solution to use, or more importantly, which principle they are willing to break.
We cannot suggest one or the other, but in either case we do require that explicit measures be taken to communicate the disruption and its explanation to users manually (beyond just the version increment).

If there were no Y-turtle, even though the fix would technically just be a patch, X-turtle would have to bump to ``2.0.0``.
This case adheres to SemVer, but breaks from our own rule that major increments should not be introduced in a released distribution.

This is why we consider the versioning rules *best-effort*.
As unlikely as the examples above are, it is important to accurately define our versioning system.

Public API declaration
~~~~~~~~~~~~~~~~~~~~~~

According to ``semver``, every package must clearly declare a public API.
We will use the "Public API Declaration" section of the quality declaration of a package to declare what symbols are part of the public API.

For most C and C++ packages the declaration is any header that it installs.
However, it is acceptable to define a set of symbols which are considered private.
Avoiding private symbols in headers can help with ABI stability, but is not required.

For other languages like Python, a public API must be explicitly defined, so that it is clear what symbols can be relied on with respect to the versioning guidelines.
The public API can also be extended to build artifacts like configuration variables, CMake config files, etc. as well as executables and command-line options and output.
Any elements of the public API should be clearly stated in the package's documentation.
If something you are using is not explicitly listed as part of the public API in the package's documentation, then you cannot depend on it not changing between minor or patch versions.

Deprecation strategy
~~~~~~~~~~~~~~~~~~~~

Where possible, we will also use the tick-tock deprecation and migration strategy for major version increments.
New deprecations will come in a new distribution release, accompanied by compiler warnings expressing that the functionality is being deprecated.
In the next release, the functionality will be completely removed (no warnings).

Example of function ``foo`` deprecated and replaced by function ``bar``:

=========  ========================================================
 Version    API
=========  ========================================================
X-turtle   void foo();
Y-turtle   [[deprecated("use bar()")]] void foo(); <br> void bar();
Z-turtle   void bar();
=========  ========================================================

We must not add deprecations after a distribution is released.
Deprecations do not necessarily require a major version bump, though.
A deprecation can be introduced in a minor version bump if the bump happens before the distro is released (similar to ABI breaking changes).

For example, if X-turtle begins development as ``2.0.0``, a deprecation can be added in ``2.1.0`` before X-turtle is released.

We will attempt to maintain compatibility across distros as much as possible.
However, like the caveats associated with SemVer, tick-tock or even deprecation in general may be impossible to completely adhere to in certain cases.
