
The following Arduino sketches are published here:

--------------------------------------------------------------------------------
Simple game controller (mouse/kbrd WASD walking/driving games) using 1 Arduino:

Leo_WASD_v3.ino 
	Simple WASD game controller (kbrd/mouse only):  includes support
	for Crash Drive 2 (could be tweaked for other driving games with
        keyboard controls).

USBcycle_Diagnostic.ino
	A stripped version of Leo_WASD for diagnostic purposes:  
	writes debug messages to Arduino IDE serial monitor.

--------------------------------------------------------------------------------
Joystick driving game controller, as in Euro Bike Simulator, using 2 Arduinos

Leo1_Current.ino
	Master arduino code for 2-duino version.  The master controls the
	i2c bus and does all kbrd/mouse stuff (1st HID).

Leo2_Current.ino
	Slave arduino code for 2-duino version.  The slave does all the
	joystick input (2nd HID).

Leo1_Diagnostic.ino
	A stripped version of Leo1_Current which does no USB sending, just
	writes to IDE serial monitor.  Used to verify basic functions.  Does
	talk to Leo2 over I2C.

Leo2_Diagnostic.ino
	A stripped version of Leo2_Current, as above.  Does joystick axis
	sends, which can be verified by a utility such as JoystickShow.

