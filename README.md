# tc35661-spp-mcu-driver
A simple driver for the TC35661 bluetooth chip which has an USART interface and only supports SPP mode.

This program is designed based on information provided in <https://github.com/yuht/TC35661_Bluetooth>.

Currently it hasn't made possible for the device to send connection request initiatively performing as a master device. A unique SSP mode that doesn't require user confirmation (of the 6 digit pairing numbers) is set by this program.

## Hardware
The TC35661 chip is on the DCBT V1.3 module, it's pin definition is compatible with BM77, but without Pins 13~19. Pin 2, 3, 5 (and probably 4) are connected together internally (3.3 V), and Pin 1 (GND), Pin 20 (RXD), Pin 21 (TXD) should be connected (RXD to the MCU's USART TXD, TXD to the MCU's RXD).
