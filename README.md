# multi-key-knob
![3key1knob-picture](/images/3key1knob.png "3key1knob-picture.")
Alternative firmware for CH552 based [3key-1knob COTS HW](https://amzn.eu/d/6oloNJh). It is Based on [3keys-1knob-firmware](https://github.com/biemster/3keys_1knob) and it allows changing different input-key for rotary knob

- when key-1 is pressed, corresponding Red-LED will turn on and the knob function will be VOL-UP/VOL-DN
- when key-2 is pressed, corresponding Green-LED will turn on and the knob function will be KEY-UP/KEY-DN
- when key-3 is pressed, corresponding Blue-LED will turn on and the knob function will be PAGE-UP/PAGE-DN

### compile:
`$ make bin`

### compile & flash to pad:
- if on original firmware of OEM: connect P1.5 to GND and connect USB
- if on this firmware: press key1 while connecting USB
- `$ make flash`
```
user@machine:~/multi-key-knob$ make flash
Compiling multi-key-knob.c ...
Compiling include/usb_handler.c ...
Compiling include/usb_hid.c ...
Compiling include/delay.c ...
Compiling include/neo.c ...
Compiling include/usb_conkbd.c ...
Compiling include/usb_descr.c ...
Building multi-key-knob.ihx ...
Building multi-key-knob.bin ...
------------------
FLASH: 3205 bytes
IRAM:  26 bytes
XRAM:  274 bytes
------------------
Removing temporary files ...
Uploading to CH55x ...
Connecting to device ...
FOUND: CH552 with bootloader v2.50.
Erasing chip ...
Flashing multi-key-knob.bin to CH552 ...
SUCCESS: 3205 bytes written.
Verifying ...
SUCCESS: 3205 bytes verified.
DONE.
```
