:github_url: https://github.com/ros-controls/zenbedded/blob/{REPOS_FILE_BRANCH}/zenbedded_common/doc/userdoc.rst

.. _zenbedded_common_userdoc:

zenbedded_common
================

Common network bringup, USB CDC ECM device stack, and multi-transport connectivity
utilities for Zenbedded applications.

Contents
--------

- **Unified Network Initialization**: Simple ``init_network()`` call supporting standalone
  USB CDC ECM, standalone Wi-Fi STA, and resilient dual-transport operation.
- **Native USB Device Stack Next (USBD)**: Zero-copy USB CDC ECM Ethernet emulation with
  built-in DHCPv4 server leasing an IP to the host PC for plug-and-play operation.
- **Wi-Fi Station Management**: Automated connection via Zephyr credentials storage (NVS),
  automatic DHCPv4 client startup, and modem power management.
- **Transport Redirection Helper**: Dynamic locator adaptation via ``get_usb_peer_locator()``
  to automatically direct Zenoh traffic over USB when attached, with seamless fallback to Wi-Fi.

Transport Architecture & Priority
---------------------------------

When both transports are enabled (``CONFIG_ZENBEDDED_COMMON_USB_CDC=y`` and
``CONFIG_ZENBEDDED_COMMON_WIFI=y``):

1. **USB CDC ECM Priority**: The USB device controller is initialized and enabled first.
   If a host cable is connected, the Ethernet carrier comes up within 500 ms and static IP /
   DHCPv4 server are configured immediately.
2. **Fast Carrier Check**: If the USB cable is disconnected, the driver detects that the carrier
   is down within 2 seconds and skips waiting, falling back immediately to Wi-Fi.
3. **Wi-Fi Fallback**: Wi-Fi station connects to the stored SSID in NVS, starts DHCPv4 discovery,
   and acquires an IPv4 address.
4. **Resilience**: ``init_network()`` succeeds if either transport becomes ready.

Kconfig Options
---------------

.. list-table::
   :widths: 35 15 50
   :header-rows: 1

   * - Kconfig Symbol
     - Default
     - Description
   * - ``CONFIG_ZENBEDDED_COMMON``
     - ``y``
     - Enable common Zenbedded utilities.
   * - ``CONFIG_ZENBEDDED_COMMON_USB_CDC``
     - ``n``
     - Enable USB CDC ECM Ethernet emulation interface.
   * - ``CONFIG_ZENBEDDED_COMMON_WIFI``
     - ``n``
     - Enable 802.11 Wi-Fi station interface using stored credentials.
   * - ``CONFIG_ZENBEDDED_USB_CDC_STATIC_IP``
     - ``y``
     - Assign deterministic static IP (``192.0.2.2``) to device USB interface.
   * - ``CONFIG_ZENBEDDED_USB_CDC_DHCPV4_SERVER``
     - ``y``
     - Run embedded DHCPv4 server to lease ``192.0.2.1`` to the host PC.

Usage
-----

Include the header:

.. code-block:: cpp

   #include <zenbedded_common/network.hpp>

Bring up network connectivity in ``main()``:

.. code-block:: cpp

   if (int err = zenbedded::common::init_network(); err != 0) {
       LOG_ERR("Network bringup failed: %d", err);
       return err;
   }

Inspect readiness and select locator:

.. code-block:: cpp

   char locator[64];
   if (zenbedded::common::is_usbd_ready()) {
       zenbedded::common::get_usb_peer_locator(DEFAULT_LOCATOR, locator, sizeof(locator));
   } else if (zenbedded::common::is_wifi_ready()) {
       strncpy(locator, DEFAULT_LOCATOR, sizeof(locator));
   }
