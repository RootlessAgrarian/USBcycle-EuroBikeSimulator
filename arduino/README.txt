
The following Arduino sketches are published here:

--------------------------------------------------------------------------------
Simple game controller (mouse/kbrd WASD walking/driving games) using 1 Arduino:

USBcycle_WASD.ino 
	Simple WASD game controller (kbrd/mouse only):  includes support
	for Crash Drive 2 (could be tweaked for other driving games with
        keyboard controls).

USBcycle_Diagnostic.ino
	A stripped version of USBcycle_WASD for diagnostic purposes:  
	writes debug messages to Arduino IDE serial monitor.

--------------------------------------------------------------------------------
Joystick driving game controller, as in Euro Bike Simulator, using 2 Arduinos

USBcycle_Leo1.ino
	Master arduino code for 2-duino version.  The master controls the
	i2c bus and does all kbrd/mouse stuff (1st HID).

USBcycle_Leo2.ino
	Slave arduino code for 2-duino version.  The slave does all the
	joystick input (2nd HID).

USBcycle_Leo1_Test.ino
	A stripped version of USBcycle_Leo1 which does no USB sending, just
	writes to IDE serial monitor.  Used to verify basic functions.  Does
	talk to Leo2 over I2C.

USBcycle_Leo2_Test.ino
	A verbose version of USBcycle_Leo2, as above.  Does joystick axis
	sends, which can be verified by a utility such as JoystickShow.
	Prints out a chatty log to Serial Monitor.

