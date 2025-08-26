# NMEA2000-Switch-Device
A remote switch operated bij NMEA2000 PGN 127502 - Switch Bank Control messages

The software is written for and tested on a Blue Pill (stm32f103c8t6)
It is written in STM32CubeIDE (version 1.19)

## Schematic
V1.0 [schematic](NMEA2000 Relay Controller V1.0.pdf) added to the repository

## Compliance
- Reacts on PGN 59904 - ISO Request messages (only for PGN 60928)
- Sends NMEA2000 PGN 60928 - ISO Address Claim messages to join the bus
- Reacts on NMEA2000 PGN 60928 - ISO Address Claim messages to resolve address conflicts
- Reacts on NMEA2000 PGN 127502 - Switch Bank Control messages to switch relays
- Broadcasts NMEA2000 PGN 127501 - Binary Switch Bank Status messages every second with current relay status

## Compliance todo (in future release)
- PGN 59392 ISO Acknowledgment (TX/RX)
- PGN 59904 ISO Request (RX, for all supported PGN's)
- PGN 60160 ISO Transport Protocol DT (RX, for PGN 65240)
- PGN 60416 ISO Transport Protocol CM (RX, for PGN 65240)
- PGN 65240 ISO Commanded Address (RX)
- PGN 126464 PGN List (TX)
- PGN 126993 Heartbeat (TX)
- PGN 126996 Product Information (TX)
- PGN 126998 Configuration Information (TX)

## Electronics
See schematic, relay coils ar pulled to earth with a ULN2803 driver. 
- Operating voltage: 12V (Via NMEA2000 bus)
- Max coil voltage: 12V
- Max current per channel: 500mA
- Max current total: 2.5A

## Links
[Canboat.github.com](https://canboat.github.io/canboat/canboat.html)
[mgenergysystems.eu](https://docs.mgenergysystems.eu/en/application-notes/Tracking-MG-device-on-NMEA2000-CAN-bus#:~:text=Address%20Claim%20procedure%20(ACL),send%20by%20this%20device%20first.)

## Disclamer
I try my best to make the device as compliance to NMEA2000 as possible bot don't have the means to certify it with NMEA nor to test it on a real NMEA2000 network. Use of this device and the software is at your own risk! (Don't say i didn't warn you).

The purpose of this project is to learn how the NMEA2000 protocol operates and to share my experiences with the world. Suggestions for improvement are welcome.
