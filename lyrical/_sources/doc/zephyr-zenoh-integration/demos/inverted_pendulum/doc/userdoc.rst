================================
Zenbedded Inverted Pendulum Demo
================================

A stepper-driven inverted pendulum. An ESP32 running Zephyr reads an AS5600
magnetic encoder and drives a stepper motor, while the balancing controller
runs on a PC as a ROS 2 node. The two talk over `Zenoh <https://zenoh.io/>`__
via Zenbedded.

.. code-block:: text

    ┌──────────────────────┐   Wi-Fi / Zenoh   ┌─────────────────────┐
    │ ESP32 (Zephyr)       │ <───────────────> │ PC                  │
    │ AS5600 + stepper     │     tcp/7447      │ zenohd + ROS 2      │
    └──────────────────────┘                   └─────────────────────┘

Setup
-----

1. Start the Zenoh router
~~~~~~~~~~~~~~~~~~~~~~~~~

On your PC:

.. code-block:: bash

    export ZENOH_CONFIG_OVERRIDE='mode="peer";listen/endpoints=["tcp/0.0.0.0:7447"]'
    export RUST_LOG=zenoh=debug
    ros2 run rmw_zenoh_cpp rmw_zenohd

Leave it running.

2. Configure the firmware
~~~~~~~~~~~~~~~~~~~~~~~~~

Find your PC's IP address on the same network the ESP32 will join
(``ip addr`` on Linux/macOS, ``ipconfig`` on Windows). Then set it in
``prj.conf``:

.. code-block:: ini

    CONFIG_ZENBEDDED_ZENOH_IP_PORT="<IP>:7447"

For example: ``CONFIG_ZENBEDDED_ZENOH_IP_PORT="192.168.1.50:7447"``

3. Build and flash
~~~~~~~~~~~~~~~~~~

Build and flash the firmware with west.

.. code-block:: bash

    west build -p always -b esp32s3_devkitc/esp32s3/procpu inverted_pendulum_tier2
    west flash

4. Add your Wi-Fi credentials
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Open a serial terminal/monitor on the board's serial port, and run:

.. code-block:: text

    wifi cred add -s "<SSID>" -k 1 -p <PASSWORD>


It is normal to see a FATAL EXCEPTION here.

Then restart the MCU. Credentials are stored in flash, so you only need to do
this once (or when your network changes).

You can keep the serial monitor open to watch the logs.

5. Check the LED status
~~~~~~~~~~~~~~~~~~~~~~~

If your board has a NeoPixel LED:

.. list-table::
   :header-rows: 1
   :widths: 25 75

   * - LED
     - Meaning
   * - Blue
     - Starting up / connecting
   * - Flashing green
     - Wi-Fi and Zenoh are configured and connected

The firmware needs the encoder to be connected or reading data to finish setup,
so you can verify the network side with the logs.

Controller (ROS 2)
------------------

Build
~~~~~

Clone or symlink these packages into your ROS 2 workspace (``ros2_ws/src``):

- ``zenbedded_hardware_interface``
- ``zenbedded_transport``
- ``inverted_pendulum_ros``

Then build with symlinks:

.. code-block:: bash

    cd ~/ros2_ws
    mkdir src && ln -s /zephyr_ws/demos/inverted_pendulum/inverted_pendulum_ros src/
    colcon build --symlink-install
    source install/setup.bash

Run
~~~

.. code-block:: bash

    ros2 launch zenbedded_inverted_pendulum inverted_pendulum.launch.py

Connection Guide
----------------

Pins are taken from the devicetree overlay.

Stepper driver (A4988, 1/16 microstepping)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 20 25 55

   * - Signal
     - ESP32 GPIO
     - Notes
   * - STEP
     - GPIO3
     -
   * - DIR
     - GPIO8
     -
   * - EN
     - GPIO13
     - Active low

Microstepping is set to 16 in the overlay. The pins are not controlled by the
firmware, so tie MS1, MS2 and MS3 high on the A4988 to get 1/16 steps. Connect
the driver's logic supply to 3.3V (VDD), and share GND between the driver,
motor supply and ESP32.

AS5600 encoder (I2C)
~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 20 25 55

   * - Signal
     - ESP32 GPIO
     - Notes
   * - SDA
     - GPIO1
     - Internal pull-up enabled
   * - SCL
     - GPIO2
     - Internal pull-up enabled

I2C address ``0x36``, running at fast mode (400 kHz). Power the AS5600 from 3.3V
and mount the diametric magnet centered over the chip on the pendulum axis.

Status LED (WS2812 / NeoPixel)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 20 25 55

   * - Signal
     - ESP32 GPIO
     - Notes
   * - DIN
     - GPIO48
     - Driven via I2S, 1 LED

This is the on-board NeoPixel on many ESP32-S3 dev boards. If your board has
none, the demo still works; use the serial logs instead.

Wiring summary
~~~~~~~~~~~~~~

.. code-block:: text

    ESP32            A4988
    GPIO3   ───────> STEP
    GPIO8   ───────> DIR
    GPIO13  ───────> EN
    3V3     ───────> VDD
    GND     ───────> GND

    ESP32            AS5600
    GPIO1   <──────> SDA
    GPIO2   ───────> SCL
    3V3     ───────> VCC
    GND     ───────> GND

    ESP32            NeoPixel (optional)
    GPIO48  ───────> DIN   (on-board on many boards)

Power the stepper through the A4988's VMOT/GND pins from a separate motor
supply, with a bulk capacitor (~100 µF) across them.

Optional: TMC2209 / DRV-style driver
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The overlay includes a commented-out configuration for drivers that allow
microstepping control. To use it, uncomment the ``tmc2209_driver`` and
``tmc2209_ctrl`` nodes and point the ``stepper-driver`` and ``stepper-ctrl``
aliases at them instead of the A4988 nodes.

.. list-table::
   :header-rows: 1
   :widths: 20 25 55

   * - Signal
     - ESP32 GPIO
     - Notes
   * - EN
     - GPIO4
     - Active low
   * - M0
     - GPIO5
     -
   * - M1
     - GPIO6
     -
   * - STEP
     - GPIO7
     -
   * - DIR
     - GPIO8
     - Direction is inverted in the overlay

Microstepping resolution is set to 32 in this configuration.

Troubleshooting
---------------

- **LED stays blue:** the board can't join Wi-Fi or reach the router. Check the
  SSID/password in the serial logs, and that your PC IP in ``prj.conf`` is
  correct and port 7447 isn't blocked by a firewall.
- **No Wi-Fi commands in the shell:** make sure you're connected to the right
  serial port and the Zephyr Wi-Fi shell is enabled.
- **Controller sees no data:** confirm ``zenohd`` is running and the board is
  connected (check the router logs), and verify the AS5600 wiring.
