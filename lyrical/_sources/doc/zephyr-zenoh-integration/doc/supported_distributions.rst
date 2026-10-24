:github_url: https://github.com/ros-controls/zenbedded/blob/{REPOS_FILE_BRANCH}/doc/supported_distributions.rst

.. _supported_distributions:

Supported ROS 2 Distributions
=============================

Zenbedded is built from a single ``master`` branch. Host packages and firmware are
versioned together, so the ROS 2 distribution only affects the host side; the Zephyr
side is pinned by the development image (Zephyr v4.4.2 and the ``zenbedded``
zenoh-pico fork).

.. list-table::
   :header-rows: 1
   :widths: 18 12 24 24 22

   * - ROS 2 distro
     - Branch
     - Host (ROS 2)
     - Firmware (Zephyr)
     - Status
   * - **Lyrical**
     - ``master``
     - CI: binary build against ``main`` and ``testing``
     - CI: ``native_sim`` run, ESP32-S3 build
     - **Supported**
   * - **Kilted**
     - ``master``
     - Source build only
     - Source build only
     - **Not validated**
   * - **Jazzy**
     - ``master``
     - Source build only
     - Source build only
     - **Not validated**
   * - Humble and older
     - --
     - Not built
     - Not built
     - **Unsupported**

Support levels
--------------

**Supported**
   Covered by CI and exercised end-to-end. Regressions are treated as bugs.

**Not validated**
   Builds from the same sources, but no CI lane exists yet and the combination has
   not been validated end-to-end. CI coverage for Kilted and Jazzy is planned for
   after the first release.

**Unsupported**
   Not built or tested. Pull requests that add support are welcome, but no support
   is implied.


Lyrical
-------

The validated target.

- Host: binary build against the Lyrical ``main`` and ``testing`` repositories,
  plus the pre-commit job.
- Firmware: twister builds the demos and tests for
  ``esp32s3_devkitc/esp32s3/procpu`` and runs the pytest suites on
  ``native_sim/native/64``.
- Development image: ``ros:lyrical-ros-base`` with ``ros2_control``,
  ``rmw_zenoh_cpp``, Zephyr, and zenoh-pico.

Build from source:

.. code-block:: bash

   mkdir -p ~/zenbedded_ws/src
   cd ~/zenbedded_ws/src
   git clone https://github.com/ros-controls/zenbedded.git
   cd ~/zenbedded_ws
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --packages-select zenbedded_hardware_interface zenbedded_transport
   source install/setup.bash


Kilted
------

Not validated. The host packages use ``ros2_control`` interfaces that are present in
Kilted, so a source build is expected to work; it is not covered by CI, and no
end-to-end run is part of the test matrix. Set ``ROS_DISTRO`` to the distribution
you are building for (``kilted`` here):

.. code-block:: bash

   source /opt/ros/${ROS_DISTRO}/setup.bash
   mkdir -p ~/zenbedded_ws/src
   cd ~/zenbedded_ws/src
   git clone https://github.com/ros-controls/zenbedded.git
   cd ~/zenbedded_ws
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --packages-select zenbedded_hardware_interface zenbedded_transport
   source install/setup.bash

The Docker development image is Lyrical-based. For Kilted, build the host packages
natively or derive an image from ``ros:kilted-ros-base`` with the packages listed in
:ref:`docker_userdoc`.


Jazzy
-----

Not validated, with the same caveats as Kilted: source build expected, no CI lane,
no end-to-end validation. The firmware side is unaffected.

Follow the `Kilted`_ build steps above with ``ROS_DISTRO`` set to ``jazzy``.


Firmware on any distro
----------------------

The Zephyr workspace is independent of the host distribution. Use the development
container, which pins Zephyr v4.4.2, the Zephyr SDK, and the zenoh-pico fork, then
build the application:

.. code-block:: bash

   cd /zephyr_ws/demos/inverted_pendulum/inverted_pendulum_tier2
   west build -p always -b esp32s3_devkitc/esp32s3/procpu
   west flash

See :ref:`docker_userdoc` for the workspace layout.


Adding a distribution
---------------------

To promote a distribution from *not validated* to *supported*:

1. Add the distribution to the binary build matrix (see
   ``.github/workflows/lyrical-binary-build.yml``) and add a pre-commit lane.
2. Add an end-to-end run on the distribution, or document the manual test that was
   performed.
3. Update the table above and the matrix in ``README.md``.
4. Add a note here describing any distribution-specific limitation.

Keep the firmware column unchanged unless the Zephyr or zenoh-pico pins change.
