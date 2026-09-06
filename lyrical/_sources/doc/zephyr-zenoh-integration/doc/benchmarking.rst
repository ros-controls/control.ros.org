:github_url: https://github.com/ros-controls/zephyr-zenoh-integration/blob/{REPOS_FILE_BRANCH}/doc/benchmarking.rst

.. _benchmarking:

Benchmarking
============

Benchmarks for the Zenbedded transport - Zenoh vs DDS (micro-ROS), Tier 1 vs
Tier 2, and max control frequency across data sizes.

Default test setup: ESP32-S3, WiFi, Zenoh client mode, Best Effort QoS,
ROS 2 Lyrical. All values TBD until measured.

Data sizes are expressed in float64 values, state to command at a 2:1
ratio. Tier 2 schemas carry exactly the fields they define.


Tier 1 vs micro-ROS (Zenoh vs DDS)
-----------------------------------

Same CDR messages (``JointState``, ``JointCommand``), different wire protocol.

.. list-table::
   :header-rows: 1
   :widths: 40 30 30

   * - Metric
     - micro-ROS (DDS)
     - zenbedded Tier 1 (Zenoh)
   * - MCU to ROS 2 latency (ms)
     - TBD
     - TBD
   * - ROS 2 to MCU latency (ms)
     - TBD
     - TBD
   * - Round-trip latency (ms)
     - TBD
     - TBD
   * - Jitter p99-p50 (us)
     - TBD
     - TBD
   * - Max control frequency (Hz)
     - TBD
     - TBD


Tier 1 vs Tier 2 (serialization)
---------------------------------

Same user data, different wire cost. Tier 1 sends standard CDR messages;
Tier 2 sends ``#pragma pack(push, 1)`` flat structs for the same values.
Grid below matches the max frequency table.

.. list-table::
   :header-rows: 1
   :widths: 15 20 20 20 20

   * - State doubles
     - Tier 1 payload (bytes)
     - Tier 2 payload (bytes)
     - Tier 1 serialize (ns/op)
     - Tier 2 serialize (ns/op)
   * - 4
     - TBD
     - 32
     - TBD
     - TBD
   * - 8
     - TBD
     - 64
     - TBD
     - TBD
   * - 16
     - TBD
     - 128
     - TBD
     - TBD
   * - 32
     - TBD
     - 256
     - TBD
     - TBD
   * - 64
     - TBD
     - 512
     - TBD
     - TBD

Both tiers carry the same Zenoh transport framing on top of the payload, so
the framing cancels out of this comparison.


Max frequency vs data size
---------------------------

Publish at 50, 100, 200, 500, 1000 Hz for 10 s each. Max freq = highest
rate with ≥ 99.9 % delivery. 1000+ samples per measurement, 100 warmup.

.. list-table::
   :header-rows: 1
   :widths: 15 20 30 30

   * - State doubles
     - Command doubles
     - Tier 1 max freq (Hz)
     - Tier 2 max freq (Hz)
   * - 4
     - 2
     - TBD
     - TBD
   * - 8
     - 4
     - TBD
     - TBD
   * - 16
     - 8
     - TBD
     - TBD
   * - 32
     - 16
     - TBD
     - TBD
   * - 64
     - 32
     - TBD
     - TBD


Notes
-----

Timestamps: ``k_uptime_get_32()`` on firmware (ms), ``rclpy.now()`` on host.
One-way latencies assume synced clocks (NTP/PTP); round-trip needs no sync.
ESP32-S3 is the primary target; native_sim results are supplementary.
