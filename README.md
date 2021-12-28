# tc35661-spp-mcu-driver
A simple driver for the TC35661 bluetooth chip which only supports SPP mode.

Currently it hasn't made possible for the device to send connection request initiatively performing as a master device. And it only accepts connection request from one master device. In demo program `main.c` for STM32F103C8T6, the link key is stored in flash memory after pairing with a master device, then it will not accept connection request from any other device. This can be changed, though.

## Hardware
The TC35661 chip is on the DCBT V1.3 module, it's pin definition is compatible with BM77, but without Pins 13~19. Pin 2, 3, 5 (and probably 4) are connected together internally (3.3 V), and Pin 1 (GND), Pin 20 (RXD), Pin 21 (TXD) should be connected.
