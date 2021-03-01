# ISP4520-examples
LoRa and LoRaWan implementation examples using ISP4520

## Overview

The aim of this archive is to show examples of raw LoRa and LoRaWAN stack implementation using the ISP4520 module.

The ISP4520 offers a unique combination of two leading IoT radio technologies in one class leading miniaturized package. With integrated BLE and LoRa connectivities, this module offers the long-range capability of LoRa for data transmission over distance, combined with the high-throughput flexible service of BLE for a more local connection that can be used to carry out configuration, commissioning and update via smartphone or tablet applications.
 
The module incorporates chips from the leading semiconductor vendors for each technology – Nordic Semiconductor for BLE and Semtech for LoRa. Processing power is provided by the Nordic nRF52’s onboard M4 floating point processor, with 512K of flash memory available for advanced applications. The Semtech SX1261 (for EU and AS versions) or SX1262 (for US version) provide the LoRa radio function. Both semiconductors offer best in class low power consumption, coupled with an array of power saving features, allowing for multi-year coin-cell operation. The module integrates antennas for both LoRa and BLE transmission, thus allowing this device to be a pre-certified complete radio and application core solution, requiring only external sensors, or connection to a customers existing device.

For each ISP4520 variant (Europe, Asia, US) the following examples are provided:

* **lorawan_class_A**: Class A end-device example application.

* **lorawan_class_C**: Class C end-device example application.

* **ble_lorawan_AT_commands**: Example of LoRaWan AT commands set.

* **lora_ping_pong**: Point to point LoRa link example application.

* **lora_tx_cw**: Transmits an RF Continuous Wave.

* **lora_tx_temperature**: Transmits local temperature using Point to point LoRa link.

* **lora_rx_temperature**: Receives temperature from lora_tx_temperature and transmits it to a COM port.

The LoRaWan implementation is based on the official LoRaWAN stack (http://stackforce.github.io/LoRaMac-doc/).
All Loramac/Region files are fetched from the StackForce LoRaMac-node master branch (4.4.3 release)
https://github.com/Lora-net/LoRaMac-node/tree/master

## Environment

The examples are ready to use with the Segger Embedded Studio (SES) IDE.

SES provide a free license for nRF52832 development. Therefore it can be used freely for ISP4520 development.
Licenses can be requested at https://license.segger.com/Nordic.cgi

For more information regarding Segger Embedded Studio, please visit https://www.segger.com/products/development-tools/embedded-studio/

## Test

For Europeran regional parameters, the LoRaWan Class A example has been tested using the LoRa-alliance pre-certification tool LCTT (https://lora-alliance.org/lorawan-certification-test-tool).
Tests for US and Asia regional parameters are ongoing. The gateways used with the pre-certification tool are the Semtech Picocell SX1308P868GW and SX1308P915GW.

Additionally, Class A & C basic LoRaWan functionalities (OTAA, uplinks, downlinks) have been tested with the AAEON-ILRA01 gateway (https://www.aaeon.com/en/p/intel-lora-gateway-system-server).

## Changelog

### 2021-01-01, v3.0.2

Fixed error in the parameter check of AT+CLASS command
Fixed Key handling when using at commands

### 2020-05-26, v3.0.0

New implementation based on v4.4.3 stackforce implementation.

### 2019-12-12, v2.0.0

Release for customer

### 2018-06-28, v1.0.0

Initial version.
