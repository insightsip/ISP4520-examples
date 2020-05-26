# ISP4520-examples
LoRa and LoRaWan implementation examples using ISP4520

## Overview

The aim of this archive is to show examples of raw LoRa and LoRaWAN stack implementation using the ISP4520 module.

For each ISP4520 variant (Europe, Asia, US) the following examples are provided:

* **lorawan_class_A**: Class A end-device example application.

* **lorawan_class_C**: Class C end-device example application.

* **ble_lorawan_AT_commands**: Class C end-device example application.

* **lora_ping_pong**: Point to point LoRa link example application.

* **lora_tx_cw**: Transmits an RF Continuous Wave.

* **lora_tx_temperature**: Transmits local temperature using Point to point LoRa link.

* **lora_rx_temperature**: Receives temperature from lora_tx_temperature and transmits it to a COM port.

The LoRaWan implementation is based on the official LoRaWAN stack (http://stackforce.github.io/LoRaMac-doc/*)
All Loramac/Region files are fetched from the StackForce LoRaMac-node master branch (4.4.3 release)
https://github.com/Lora-net/LoRaMac-node/tree/master

## Environment

The examples are ready to use with the Segger Embedded Studio (SES) IDE.

SES provide a free license for nRF52832 development. Therefore it can be used freely for ISP4520 development.
Licenses can be requested at https://license.segger.com/Nordic.cgi

For more information regarding Segger Embedded Studio, please visit https://www.segger.com/products/development-tools/embedded-studio/

## Test

The LoRaWan examples are tested using the LoRa-alliance pre-certification tool LCTT.

## Changelog

### 2020-05-26, v3.0.0

New implementation based on v4.4.3 stackforce implementation.

### 2019-12-12, v2.0.0

Release for customer

### 2018-06-28, v1.0.0

Initial version.