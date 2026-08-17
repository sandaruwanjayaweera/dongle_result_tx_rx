nRF52840 Dongle BLE Extended-Advertising UART Bridge
####################################################

An experimental bidirectional bridge that transports fixed-size application
frames between a host computer and Bluetooth Low Energy (BLE) extended
advertisements.

The application targets the Nordic Semiconductor nRF52840 Dongle and is built
with Zephyr through the nRF Connect SDK. Each dongle can simultaneously:

* receive a frame from a host through asynchronous UART;
* place the frame in BLE Manufacturer Specific Data and advertise it;
* passively scan for a selected peer; and
* forward an accepted BLE frame to the host through UART.

The firmware was developed for research experiments carrying custom
Cooperative Awareness Message (CAM)-style data between two devices without a
BLE connection.

.. note::

   This is a research prototype, not a complete or standards-compliant ETSI
   ITS CAM implementation. It sends raw application bytes and does not perform
   ASN.1 encoding, authentication, acknowledgements, or retransmission.


Data flow
*********

For a two-device experiment, the same application is installed on both
dongles. Traffic can flow in either direction:

1. A host writes a framed payload to ``uart0``.
2. The attached dongle copies the payload into an extended-advertising packet.
3. The peer dongle detects the advertisement while scanning.
4. The peer forwards the received payload to its host through ``uart0``.

Advertising is non-connectable and non-scannable. No GATT service or BLE
connection is created.


Features
********

* Concurrent BLE extended advertising and passive scanning.
* Asynchronous UART receive and transmit using Zephyr callbacks.
* FIFO-backed UART buffering with dynamically allocated buffers.
* Manufacturer Specific Data transport for CAM-style binary frames.
* Continuous scanning with a 30 ms scan interval and 30 ms scan window.
* Configurable constants for buffer capacity and accepted frame size.
* Persistent Bluetooth settings through flash/NVS support.


Protocol used by the current firmware
*************************************

UART-to-BLE path
================

UART reception is completed when a carriage return or line-feed byte is
detected. The current application recognizes an input frame when its first two
bytes are:

.. list-table:: Frame header
   :header-rows: 1
   :widths: 25 35 40

   * - Offset
     - Required value
     - Meaning
   * - ``0``
     - ``0x01``
     - Protocol version used by this experiment
   * - ``1``
     - ``0x02``
     - CAM message identifier
   * - ``2`` to ``64``
     - Application-defined
     - Remaining CAM-style payload

The active frame size is ``65`` bytes (``RECEIVED_DATA_SIZE``). If a received
UART buffer is longer, only the first 65 bytes are copied. Shorter accepted
frames update only the corresponding part of the advertising buffer.

The Manufacturer Specific Data array is statically allocated for 229 bytes:

* bytes 0--1 are reserved for the Bluetooth company identifier and currently
  remain ``0x0000``;
* the accepted application frame begins at byte 2; and
* unused bytes remain zero or retain their previous value.

After updating the advertising data, the application starts the extended
advertising set, waits 20 ms, and stops it.

BLE-to-UART path
================

The scanner currently accepts only advertisements that satisfy both of these
conditions:

* advertiser address: ``F2:45:C2:1D:B3:46``; and
* first two application bytes: ``0x01 0x02``.

For an accepted report, the firmware forwards exactly 65 bytes from the
application payload to UART.

.. warning::

   The source assigns ``F2:45:C2:1D:B3:46`` as the local controller address and
   also uses it as the peer-address filter. This reflects the original
   controlled experiment. Do not deploy multiple copies unchanged on a shared
   network. Introduce separate local and peer addresses before general use.


Important configuration values
******************************

.. list-table:: Current defaults
   :header-rows: 1
   :widths: 35 20 45

   * - Setting
     - Value
     - Purpose
   * - ``RECEIVED_DATA_SIZE``
     - ``65`` bytes
     - Frame length forwarded between BLE and UART
   * - ``UART_BUF_SIZE``
     - ``600`` bytes
     - Size of each asynchronous UART buffer
   * - ``BLE_ARRAY_MAX``
     - ``227`` bytes
     - Application capacity following the two-byte manufacturer prefix
   * - Scan interval
     - ``0x0030``
     - 30 ms in BLE 0.625 ms units
   * - Scan window
     - ``0x0030``
     - 30 ms, resulting in continuous scanning
   * - UART RX timeout
     - ``50``
     - Timeout supplied to the Zephyr asynchronous UART API
   * - BLE device name
     - ``CAM extended``
     - Name configured in ``prj.conf``

Change these values only after checking the corresponding Zephyr buffer and
controller limits.


Repository structure
********************

::

   dongle_result_tx_rx/
   |-- CMakeLists.txt
   |-- prj.conf
   |-- sample.yaml
   |-- child_image/
   |   `-- hci_rpmsg.conf
   `-- src/
       |-- main.c
       |-- broadcaster_multiple.c
       `-- cam.h

``main.c``
   Starts the application and calls ``broadcaster_multiple()``.

``broadcaster_multiple.c``
   Implements Bluetooth initialization, extended advertising, passive
   scanning, address filtering, and the asynchronous UART bridge.

``cam.h``
   Defines CAM-related experimental fields and the static manufacturer-data
   buffer. The current transport treats the received frame as raw bytes; these
   field definitions are not automatically serialized into the transmitted
   frame.

``prj.conf``
   Enables the Zephyr Bluetooth broadcaster/observer roles, extended
   advertising and scanning, UART asynchronous API, heap, settings, and
   controller buffers.

``child_image/hci_rpmsg.conf``
   Contains controller configuration for targets that use an HCI RPMsg child
   image. It is normally not used by the single-core nRF52840 Dongle build.


Requirements
************

Hardware
========

* One nRF52840 Dongle (PCA10059) for development or reception tests.
* Two dongles for a complete bidirectional link.
* A suitable connection between each host and the dongle's ``uart0``.
  Depending on the selected board definition, this can require an external
  3.3 V USB-to-UART adapter and a board overlay defining the UART pins.

The dongle has no onboard J-Link debugger. An external SWD debugger is optional
but is required for direct ``west flash`` and source-level debugging.

Software
========

The original development environment used:

* nRF Connect SDK 2.3.0;
* nRF Connect for VS Code; and
* nRF Connect Programmer 3.0.7.

Later SDK releases may require source, Kconfig, or board-name changes. In
particular, older nRF Connect SDK releases use the board target
``nrf52840dongle_nrf52840``; current Zephyr releases use
``nrf52840dongle/nrf52840``.

See the `official nRF52840 Dongle board documentation
<https://docs.zephyrproject.org/latest/boards/nordic/nrf52840dongle/doc/index.html>`__
for hardware, bootloader, flashing, and debugging details.


Building
********

nRF Connect for VS Code
=======================

1. Install the nRF Connect SDK and the nRF Connect extension pack for VS Code.
2. Open this repository as an existing application.
3. Add a build configuration.
4. Select ``nrf52840dongle_nrf52840`` when using nRF Connect SDK 2.3.0.
5. Build the application.

The firmware image is normally generated at
``build/zephyr/zephyr.hex``.

Command line
============

From an initialized nRF Connect SDK 2.3.0 environment:

.. code-block:: bash

   git clone https://github.com/sandaruwanjayaweera/dongle_result_tx_rx.git
   cd dongle_result_tx_rx
   west build -b nrf52840dongle_nrf52840 -p always .

If using a newer Zephyr version, consult its board documentation for the
current target name. Compatibility with versions other than the original SDK
has not been verified in this repository.


Programming the dongle
**********************

Using the factory USB bootloader
================================

1. Connect the nRF52840 Dongle to the computer.
2. Press its reset button to enter bootloader mode.
3. Open nRF Connect Programmer.
4. Select the dongle and add ``build/zephyr/zephyr.hex``.
5. Write the image to the device.

The standard dongle board target reserves space for the factory USB
bootloader. Be careful when programming through an external SWD probe: a full
chip erase can remove the bootloader.

Using an external debugger
==========================

Connect a compatible SWD probe to the dongle's debug pads and run:

.. code-block:: bash

   west flash

Refer to the official board documentation before using this method.


UART setup and operation
************************

The application binds directly to ``DT_NODELABEL(uart0)``. It does not include
a devicetree overlay that selects UART pins, and it does not explicitly
configure USB CDC ACM as the host transport. Confirm the generated devicetree
and the ``uart0`` pin mapping for your SDK/board configuration before wiring the
host.

The UART baud rate and framing are inherited from the selected board and any
overlay. Configure the host serial port to match those settings.

To test a two-dongle setup:

1. Build and program both dongles.
2. Connect each dongle's ``uart0`` to a host serial interface.
3. Open both host serial ports with matching UART settings.
4. Send a 65-byte binary frame beginning with ``0x01 0x02`` and terminate the
   UART input with carriage return or line feed.
5. Verify that the peer host receives the 65-byte frame.

Because the current address is hard-coded, use this procedure only in a
controlled environment or first separate the local address from the peer
filter in the source.


Useful source customizations
****************************

Before reusing the firmware in another experiment, consider making the
following values configurable:

* local BLE address and peer address;
* protocol version and message identifier;
* payload length;
* advertising duration and interval;
* scan interval, window, duplicate filtering, and PHY;
* UART device, pins, baud rate, and frame delimiter; and
* Bluetooth company identifier.


Known limitations
*****************

* The local and filtered peer addresses are hard-coded to the same value.
* The receiver assumes a 65-byte application frame and does not fully validate
  the advertising-report length before copying it.
* Short UART frames can leave older bytes in the static advertising buffer.
* Duplicate filtering may suppress identical reports from the same address.
* There is no acknowledgement, retry, fragmentation, sequence-number, or
  application-level integrity mechanism.
* Advertising data is broadcast without encryption or authentication.
* The Bluetooth company-identifier bytes remain ``0x0000``.
* ``cam.h`` does not provide a packed, portable serialization format; byte
  order and field encoding remain the host application's responsibility.
* ``sample.yaml`` is inherited from the original Zephyr sample and does not
  describe the present hardware test setup.
* The repository has no automated hardware-in-the-loop tests.


Safety and security
*******************

The firmware is intended for controlled research experiments. Do not transmit
sensitive information or use unauthenticated received data directly for
safety-critical robot or UAV control. Validate message length, origin,
freshness, and integrity at the application layer before acting on a frame.


License
*******

The C source files carry ``SPDX-License-Identifier: Apache-2.0`` notices. The
HCI RPMsg child-image configuration carries a Nordic five-clause license
notice. This repository does not currently contain a standalone project-level
``LICENSE`` file; verify the applicable terms before redistribution.


Author
******

`Sandaruwan Jayaweera <https://github.com/sandaruwanjayaweera>`__
