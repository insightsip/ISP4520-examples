## Changelog

### 2025-03-26, v3.2.0

### Added

- tx and rx lora demo for isp4580
- US version of lorawan demo for isp4580

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