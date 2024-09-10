//modified ch552 fmw of Stefan Wagner to turn 3buttonKnob hw knob to send different key-codes based on selected-key

// ===================================================================================
// Project:   MacroPad Mini for CH551, CH552 and CH554
// Version:   v1.1
// Year:      2023
// Author:    Stefan Wagner
// Github:    https://github.com/wagiminator
// EasyEDA:   https://easyeda.com/wagiminator
// License:   http://creativecommons.org/licenses/by-sa/3.0/
// ===================================================================================
//
// Description:
// ------------
// Firmware example implementation for the MacroPad Mini.
//
// References:
// -----------
// - Blinkinlabs: https://github.com/Blinkinlabs/ch554_sdcc
// - Deqing Sun: https://github.com/DeqingSun/ch55xduino
// - Ralph Doncaster: https://github.com/nerdralph/ch554_sdcc
// - WCH Nanjing Qinheng Microelectronics: http://wch.cn
//
// Compilation Instructions:
// -------------------------
// - Chip:  CH551, CH552 or CH554
// - Clock: min. 12 MHz internal
// - Adjust the firmware parameters in include/config.h if necessary.
// - Make sure SDCC toolchain and Python3 with PyUSB is installed.
// - Press BOOT button on the board and keep it pressed while connecting it via USB
//   with your PC.
// - Run 'make flash'.
//
// Operating Instructions:
// -----------------------
// - Connect the board via USB to your PC. It should be detected as a HID keyboard.
// - Press a macro key and see what happens.
// - To enter bootloader hold down key 1 while connecting the MacroPad to USB. All
//   NeoPixels will light up white as long as the device is in bootloader mode 
//   (about 10 seconds).


// ===================================================================================
// Libraries, Definitions and Macros
// ===================================================================================

// Libraries
#include <config.h>                         // user configurations
#include <system.h>                         // system functions
#include <delay.h>                          // delay functions
#include <neo.h>                            // NeoPixel functions
#include <usb_conkbd.h>                     // USB HID consumer keyboard functions

#define KEY1UP '0' //CON_VOL_UP
#define KEY1DN '1' //CON_VOL_DOWN
#define KEY2UP '2' //KBD_KEY_UP_ARROW
#define KEY2DN '3' //KBD_KEY_DOWN_ARROW
#define KEY3UP '4' //KBD_KEY_PAGE_UP
#define KEY3DN '5' //KBD_KEY_PAGE_DOWN

// Prototypes for used interrupts
void USB_interrupt(void);
void USB_ISR(void) __interrupt(INT_NO_USB) {
  USB_interrupt();
}

// ===================================================================================
// NeoPixel Functions
// ===================================================================================

// NeoPixel variables
__idata uint8_t neo1 = 127;                 // brightness of NeoPixel 1
__idata uint8_t neo2 = 127;                 // brightness of NeoPixel 2
__idata uint8_t neo3 = 127;                 // brightness of NeoPixel 3

// Update NeoPixels
void NEO_update(void) {
  EA = 0;                                   // disable interrupts
  NEO_writeColor(neo1, 0, 0);               // NeoPixel 1 lights up red
  NEO_writeColor(0, neo2, 0);               // NeoPixel 2 lights up green
  NEO_writeColor(0, 0, neo3);               // NeoPixel 3 lights up blue
  EA = 1;                                   // enable interrupts
}

// Read EEPROM (stolen from https://github.com/DeqingSun/ch55xduino/blob/ch55xduino/ch55xduino/ch55x/cores/ch55xduino/eeprom.c)
uint8_t eeprom_read_byte (uint8_t addr){
  ROM_ADDR_H = DATA_FLASH_ADDR >> 8;
  ROM_ADDR_L = addr << 1; //Addr must be even
  ROM_CTRL = ROM_CMD_READ;
  return ROM_DATA_L;
}

// ===================================================================================
// Main Function
// ===================================================================================
void main(void)
{
  // Variables
  __bit key1last = 0;                       // last state of key 1
  __bit key2last = 0;                       // last state of key 2
  __bit key3last = 0;                       // last state of key 3
  __bit knobswitchlast = 0;                 // last state of knob switch
  __idata uint8_t i;                        // temp variable
  uint8_t currentKnobKey;                   // current key to be sent by knob

  // Enter bootloader if key-1 is pressed(the key far from the rotary knob)
  NEO_init();                               // init NeoPixels
  if(!PIN_read(PIN_KEY1)) {                 // key 1 pressed?
    NEO_latch();                            // make sure pixels are ready
    for(i=9; i; i--) NEO_sendByte(127);     // light up all pixels
    BOOT_now();                             // enter bootloader
  }

  // Setup
  CLK_config();                             // configure system clock
  DLY_ms(5);                                // wait for clock to settle
  KBD_init();                               // init USB HID keyboard
  WDT_start();                              // start watchdog timer

  //initially select key-1(volup/voldn)
  neo1=127;// light up NeoPixel
  neo2=neo3=0;// light up NeoPixel
  NEO_update();      // update NeoPixels NOW!

  // Loop
  while(1)
  {
	//when Key-1 is pressed, turn on its LED to RED showing that Key-1 is being selcted for knob operation
	if(!PIN_read(PIN_KEY1) != key1last)// key 1 state changed?
	{
		key1last = !key1last;      // update last state flag
		if(key1last)               // key was pressed?
		{
			neo1=127;// light up NeoPixel to RED
			neo2=neo3=0;// turn OFF remaining two LEDs
		        NEO_update();// update NeoPixels NOW!
		}
	}
	//when Key-2 is pressed, turn on its LED to GREEN showing that Key-2 is being selcted for knob operation
        if(!PIN_read(PIN_KEY2) != key2last)// key 2 state changed?
        {
                key2last = !key2last;      // update last state flag
                if(key2last)               // key was pressed?
                {
                        neo2=127;// light up NeoPixel to GREEN
                        neo1=neo3=0;// turn OFF remaining two LEDs
                        NEO_update();// update NeoPixels NOW!
                }
        }	

	//when Key-3 is pressed, turn on its LED to BLUE showing that Key-3 is being selcted for knob operation
        if(!PIN_read(PIN_KEY3) != key3last)// key 3 state changed?
        {
                key3last = !key3last;      // update last state flag
                if(key3last)               // key was pressed?
                {
                        neo3=127;// light up NeoPixel to BLUE
                        neo1=neo2=0;// turn OFF remaining two LEDs
                        NEO_update();      // update NeoPixels NOW!
                }
        }
	
	// Handle knob clockwise/counter-clockwise action and send the corresponding KEY-event to HID host
	currentKnobKey = 0;      // clear key variable
	if(!PIN_read(PIN_ENC_A)) // encoder turned ?
	{
		if(PIN_read(PIN_ENC_B))
		{
			if(neo1==127)
				currentKnobKey = KEY1UP;//clockwise
			else if(neo2==127)
				currentKnobKey = KEY2UP;
			else
				currentKnobKey = KEY3UP;
		}
		else
		{
                        if(neo1==127)
                                currentKnobKey = KEY1DN;//counter-clockwise
                        else if(neo2==127)
                                currentKnobKey = KEY2DN;
                        else
                                currentKnobKey = KEY3DN;
		}
		DLY_ms(10);                           // debounce
		while(!PIN_read(PIN_ENC_A));          // wait until next detent
		KBD_type(currentKnobKey);
		//if(neo1==127)
		//{
		//	CON_press(currentKnobKey);
		//	DLY_ms(1);
		//	CON_release(currentKnobKey);
		//}
		//else //if(neo2==127)
		//{
		//	KBD_press(currentKnobKey);
		//	DLY_ms(1);
		//	KBD_release(currentKnobKey);
		//}
	}
	DLY_ms(5); // latch and debounce
	WDT_reset();// reset watchdog
  }
}
