# ISP4520-examples
LoRa and LoRaWan implementation examples using ISP4520

## Overview

The aim of this archive is to show examples of raw LoRa and LoRaWAN stack implementation using the ISP4520 module.

The ISP4520 offers a unique combination of two leading IoT radio technologies in one class leading miniaturized package. With integrated BLE and LoRa connectivities, this module offers the long-range capability of LoRa for data transmission over distance, combined with the high-throughput flexible service of BLE for a more local connection that can be used to carry out configuration, commissioning and update via smartphone or tablet applications.
 
The module incorporates chips from the leading semiconductor vendors for each technology – Nordic Semiconductor for BLE and Semtech for LoRa. Processing power is provided by the Nordic nRF52’s onboard M4 floating point processor, with 512K of flash memory available for advanced applications. The Semtech SX1261 (for EU and AS versions) or SX1262 (for US version) provide the LoRa radio function. Both semiconductors offer best in class low power consumption, coupled with an array of power saving features, allowing for multi-year coin-cell operation. The module integrates antennas for both LoRa and BLE transmission, thus allowing this device to be a pre-certified complete radio and application core solution, requiring only external sensors, or connection to a customers existing device.

For each ISP4520 variant (Europe, Asia, US) the following examples are provided:

* **lorawan/end-device**: LoRaWan end-device example application.

* **lorawan/class_A**: LoRaWan Class A end-device example application (deprecated - Please do not use).

* **lorawan/at_commands**: Example of LoRaWan AT commands set.

* **lora/ping_pong**: Point to point LoRa link example application.

* **lora/tx_cw**: Transmits an RF Continuous Wave.

* **lora/tx_temperature**: Transmits local temperature using Point to point LoRa link.

* **lora/rx_temperature**: Receives temperature from lora_tx_temperature and transmits it to a COM port.

The LoRaWan implementation is based on the official LoRaWAN stack (http://stackforce.github.io/LoRaMac-doc/).
All Loramac/Region files are fetched from the StackForce LoRaMac-node master branch (4.5.1 release)
https://github.com/Lora-net/LoRaMac-node/tree/master

## Environment

The examples are ready to use with the Segger Embedded Studio (SES) IDE.

SES provide a free license for nRF52832 development. Therefore it can be used freely for ISP4520 development.
Licenses can be requested at https://license.segger.com/Nordic.cgi

For more information regarding Segger Embedded Studio, please visit https://www.segger.com/products/development-tools/embedded-studio/

## Test

For all 3 regional version (EU,868, AS923 and US915), the LoRaWan end-device example has been tested using the LoRa-alliance pre-certification tool LCTT (https://lora-alliance.org/lorawan-certification-test-tool).
The gateways used with the pre-certification tool are the Semtech Picocell SX1308P868GW and SX1308P915GW.

## Changelog

### 2023-07-19, v3.1.14

- Change PA configuration

### 2023-04-17, v3.1.13

- Add missing RF frequency configuration in "TX temperature" & "RX temperature" example

### 2023-03-14, v3.1.12

- Fix a buffer issue when sending AT commands with a size larger than 256
- Fix issue with AT+JOINDLY1? and AT+JOINDLY2? commands
- Add new error code "no channel found"

### 2023-03-01, v3.1.11

- Add ISP4580 project

### 2023-02-14, v3.1.10

- Fix SES projects

### 2022-05-13, v3.1.8

- Fix certification issues and re-transmission handling.

### 2021-11-23, v3.1.7

- Fix Eeprom issue on AS & US variant.
- Fix timeout on tx-cw example.

### 2021-07-27, v3.1.6

- Fixed a timing issue.

### 2021-04-08, v3.1.5

- Fixed issue with context storage on flash.

### 2021-03-11, v3.1.4

- Reduced current consumption on some LoRa examples.
- AT+SEND and AT+JOINRQ returns more responses (BUSY, DUTY CYCLE etc..).

### 2021-02-05, v3.1.3

- Fixed issues related to at-commands & unwanted LoRaWAN context erase. 
- Enabled DCDC regulator and set BLE TX power to 4dBm for the at-commands example.

### 2021-02-05, v3.1.1

- New implementation based on v4.5.1 stackforce implementation.

### 2020-05-26, v3.0.0

- New implementation based on v4.4.3 stackforce implementation.

### 2019-12-12, v2.0.0

- Release for customer

### 2018-06-28, v1.0.0

- Initial version.
