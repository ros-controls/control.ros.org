================================
Zenbedded Inverted Pendulum Demo
================================

A stepper-driven Furuta (rotary) inverted pendulum balancing system. An ESP32-S3
running Zephyr RTOS samples an AS5600 magnetic angle encoder and commands a
stepper motor, while the non-linear energy-shaping swing-up and LQR balancing
controller runs on the host PC within the ``ros2_control`` framework.

Architecture
------------

The demo supports three execution tiers under a unified ``ros2_control`` stack:

- **Tier 1 (ROS 2 CDR Topics)**: The ESP32-S3 firmware uses ``ZenbeddedClient``
  with ``JointStateCodec`` and ``JointCommandCodec``, communicating over standard
  ROS 2 CDR topics (``/zenbedded/inverted_pendulum/state`` and
  ``/zenbedded/inverted_pendulum/cmd``). On the host,
  ``InvertedPendulumTier1Hardware`` relays these topics directly into
  ``ros2_control``.
- **Tier 2 (High-Performance Packed Binary)**: The firmware exchanges compact
  binary C-struct payloads governed by ``interface_schema.yaml`` directly over
  Zenoh. On the host, ``ZenbeddedHardware`` bridges these binary buffers into
  ``ros2_control``.
- **Mock Hardware (In-Process Simulation)**: Simulates coupled 2-DOF Lagrangian
  dynamics with Coulomb friction and motor torque saturation in software,
  enabling development and automated testing without physical hardware.

In all tiers, the exact same controller (``inverted_pendulum_controller``),
joint state broadcaster, robot state publisher, and RViz 3D visualization run
identically.

.. code-block:: text

    ┌──────────────────────┐   USB CDC-ECM / Wi-Fi   ┌─────────────────────────────┐
    │ ESP32-S3 (Zephyr)    │ <─────────────────────> │ Host PC (ROS 2)             │
    │ • AS5600 Encoder     │         Zenoh           │ • zenohd (tcp/7447)         │
    │ • Stepper (TMC/A4988)│     (Tier 1 or 2)       │ • ros2_control_node         │
    │ • NeoPixel Status LED│                         │ • inverted_pendulum_ctrl    │
    └──────────────────────┘                         │ • robot_state_pub + RViz2   │
                                                     └─────────────────────────────┘

Prerequisites
-------------

1. Git LFS 3D Meshes
~~~~~~~~~~~~~~~~~~~~

The URDF model uses detailed STL meshes for visual rendering in RViz. Ensure
binary meshes are pulled:

.. code-block:: bash

    git lfs pull

Verify that the files in ``demos/inverted_pendulum/inverted_pendulum_ros/meshes/``
are binary STLs (several megabytes each) rather than 132-byte pointer text files.

2. Zenoh Router
~~~~~~~~~~~~~~~

When running ROS 2 with ``RMW_IMPLEMENTATION=rmw_zenoh_cpp``, a Zenoh router must
be running on the network:

.. code-block:: bash

    zenohd -l tcp/0.0.0.0:7447

Alternatively:

.. code-block:: bash

    export ZENOH_CONFIG_OVERRIDE='mode="peer";listen/endpoints=["tcp/0.0.0.0:7447"]'
    ros2 run rmw_zenoh_cpp rmw_zenohd

3. Host NTP Time Synchronization (Chrony)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To ensure microsecond-level timestamp synchronization between the host and MCU,
configure the host Chrony daemon to serve local subnets. In ``/etc/chrony/chrony.conf``:

.. code-block:: text

    allow 192.0.2.0/24
    allow 127.0.0.1

Replace ``127.0.0.1`` with your Wi-Fi subnet (shown by ``ip addr``) so the ESP32 can
reach the NTP server.

Then restart the service:

.. code-block:: bash

    sudo systemctl restart chrony

Firmware Setup (ESP32-S3)
-------------------------

The firmware supports both USB CDC-ECM (wired Ethernet over USB) and Wi-Fi
transports via Zephyr snippets.

.. note::

   For Wi-Fi builds, set ``CONFIG_ZENBEDDED_ZENOH_LOCATOR="<HOST_IP>:7447"`` in ``prj.conf``.

1. Build Firmware
~~~~~~~~~~~~~~~~~

Tier 1 (ROS 2 Topics over USB CDC-ECM):

.. code-block:: bash

    west build -b esp32s3_devkitc/esp32s3/procpu -p always \
      demos/inverted_pendulum/inverted_pendulum_tier1 -S cdc-ecm

Tier 2 (Zenbedded Binary over USB CDC-ECM):

.. code-block:: bash

    west build -b esp32s3_devkitc/esp32s3/procpu -p always \
      demos/inverted_pendulum/inverted_pendulum_tier2 -S cdc-ecm

To enable Wi-Fi, use ``-S wifi`` instead. For concurrent dual transport
(USB CDC-ECM priority with Wi-Fi fallback), pass both snippets:

.. code-block:: bash

    # Dual Transport (USB CDC-ECM priority with Wi-Fi fallback)
    west build -b esp32s3_devkitc/esp32s3/procpu -p always \
      demos/inverted_pendulum/inverted_pendulum_tier2 -S cdc-ecm -S wifi

2. Flash and Monitor
~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

    west flash
    west espressif monitor

3. Wi-Fi Credentials (if using Wi-Fi)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In the serial console, save network credentials to NVS:

.. code-block:: text

    wifi cred add -s "YourSSID" -k 1 -p "YourPassword"

4. LED Status Codes
~~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 25 75

   * - LED Color
     - Meaning
   * - Blue (solid)
     - Initialization: peripherals up, acquiring network IP and boot-time
       encoder zero. If the LED stays solid blue, the Zenoh connection never
       completed (see Troubleshooting).
   * - Flashing Green (toggles every 250 ms)
     - Zenoh session live; publishing state and executing control commands.

Host Bringup (ROS 2)
--------------------

1. Build the ROS 2 Workspace
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

    source /opt/ros/lyrical/setup.bash
    colcon build --packages-select zenbedded_inverted_pendulum zenbedded_hardware_interface --symlink-install
    source install/setup.bash

2. Launch Stack
~~~~~~~~~~~~~~~

Tier 1 (Topic-Based Hardware Interface + RViz):

.. code-block:: bash

    ros2 launch zenbedded_inverted_pendulum inverted_pendulum.launch.py tier:=1

Tier 2 (Zenoh Binary Hardware Interface + RViz):

.. code-block:: bash

    ros2 launch zenbedded_inverted_pendulum inverted_pendulum.launch.py tier:=2

In-Process Simulation (Mock Hardware + RViz with Periodic Random Kicks):

.. code-block:: bash

    # Periodic random disturbance kicks enabled by default:
    ros2 launch zenbedded_inverted_pendulum inverted_pendulum.launch.py use_mock_hardware:=true

    # Disable disturbance kicks:
    ros2 launch zenbedded_inverted_pendulum inverted_pendulum.launch.py use_mock_hardware:=true enable_kicking:=false

Headless Mode (disable RViz):

.. code-block:: bash

    ros2 launch zenbedded_inverted_pendulum inverted_pendulum.launch.py tier:=2 use_rviz:=false

Hardware Wiring & Pinout
------------------------

AS5600 12-Bit Magnetic Encoder (I2C)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 20 25 55

   * - Signal
     - ESP32 GPIO
     - Notes
   * - SDA
     - GPIO1
     - Internal pull-up enabled, 400 kHz Fast-Mode
   * - SCL
     - GPIO2
     - Internal pull-up enabled, 400 kHz Fast-Mode
   * - VCC
     - 3.3V
     - Connect to 3V3 rail
   * - GND
     - GND
     - Common ground

Stepper Driver (TMC2209)
~~~~~~~~~~~~~~~~~~~~~~~~~

The demo wires a TMC2209 driver with GPIO-controlled microstepping pins,
resolved to 32 microsteps in the overlay (matching ``micro-step-res = <32>``).

.. list-table::
   :header-rows: 1
   :widths: 20 25 55

   * - Signal
     - ESP32 GPIO
     - Notes
   * - EN
     - GPIO4
     - Enable output (active low)
   * - M0
     - GPIO5
     - Microstepping select 0
   * - M1
     - GPIO6
     - Microstepping select 1
   * - STEP
     - GPIO7
     - Step pulse output
   * - DIR
     - GPIO8
     - Direction output (inverted in overlay via ``invert-direction``)
   * - VMOT
     - Motor V+
     - External 12V-24V power with 100 µF capacitor
   * - GND
     - GND
     - Common ground with ESP32

The legacy A4988 mapping (EN = GPIO13, STEP = GPIO3, DIR = GPIO8) remains
available as a commented-out ``a4988`` node in the board overlay.

Status LED (WS2812 NeoPixel)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 20 25 55

   * - Signal
     - ESP32 GPIO
     - Notes
   * - DIN
     - GPIO48
     - On-board NeoPixel data line on ESP32-S3 DevKitC

Troubleshooting
---------------

- **Robot model is invisible or white in RViz**:
  Ensure Git LFS binary STLs are pulled (``git lfs pull``) and the RobotModel
  display has ``Durability Policy`` set to ``Transient Local``.
- **Controller manager hangs waiting for robot_description**:
  When using ``rmw_zenoh_cpp``, make sure ``zenohd -l tcp/0.0.0.0:7447`` is running.
- **LED stays Blue on MCU**:
  MCU cannot resolve or connect to the Zenoh router. For USB CDC-ECM, verify
  the host interface (``enx...`` or ``usb0``) has IP ``192.0.2.1/24``.
