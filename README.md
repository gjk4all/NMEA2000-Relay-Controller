# NMEA2000--Relay-Controller
A remote switch operated bij NMEA2000 PGN 127502 - Switch Bank Control messages

The software is written for and tested on a Blue Pill (stm32f103c8t6)\
It is written in STM32CubeIDE (version 1.19)

For the switching output is use a [12 volt 8 channel relay board from AliExpress](https://nl.aliexpress.com/item/1005007003070916.html)

## Schematic
V1.0 [schematic](NMEA2000%20Relay%20Controller%20V1.0.pdf) added to the repository

## Compliance
- Reacts on PGN 59904 - ISO Request messages (for all supported PGN's)
- Sends NMEA2000 PGN 60928 - ISO Address Claim messages to join the bus
- Reacts on NMEA2000 PGN 60928 - ISO Address Claim messages to resolve address conflicts
- Reacts on NMEA2000 PGN 127502 - Switch Bank Control messages to switch relays
- Reacts to request for PGN 126996 Product Information
- Reacts to request for PGN 126998 Configuration Information
- Reacts on request for PGN 126464 PGN List, sends TX list and RX list (2 responces)
- Reacts to PGN 60160 ISO Transport Protocol DT (for PGN 65240)
- Reacts to PGN 60416 ISO Transport Protocol CM (BAM announcement for PGN 65240)
- Reacts to PGN 65240 ISO Commanded Address
- Broadcasts NMEA2000 PGN 127501 - Binary Switch Bank Status messages every second with current relay status
- Broadcasts NMEA2000 PGN 126993 Heartbeat messages

## Compliance todo (in future release)
- PGN 59392 ISO Acknowledgment (TX/RX) (implemented but no use yet)

## Electronics
See schematic, relay coils ar pulled to earth with a ULN2803 driver. 
- Operating voltage: 12V (9-16V through NMEA2000 bus)
- Max coil voltage: 12V (NMEA2000 bus voltage)
- Max current per channel: 500mA
- Max current total: 2.5A
### Safety overrule
All relays can be activated manually by pulling there control lines to ground with a switch, this can be used as backup for critical circuits (navigation lights etc.)

## Links
[Canboat.github.com](https://canboat.github.io/canboat/canboat.html)\
[mgenergysystems.eu](https://docs.mgenergysystems.eu/en/application-notes/Tracking-MG-device-on-NMEA2000-CAN-bus#:~:text=Address%20Claim%20procedure%20(ACL),send%20by%20this%20device%20first.)\
[embeddedflakes.com](https://embeddedflakes.com/network-management-in-sae-j1939/)

## Disclamer
I try my best to make the device as compliance to NMEA2000 as possible bot don't have the means to certify it with NMEA nor to test it on a real NMEA2000 network.
It shows all it's messages as valid in CanBoats N2K analyzer. Use of this device and the software is at your own risk! (Don't say i didn't warn you).

The purpose of this project is to learn how the NMEA2000 protocol operates and to share my experiences with the world. Suggestions for improvement are welcome.
