getduino:	
		cp /Users/de/Work/ARDUINO/devel/PROJECTS/USBcycle_Leo1/USBcycle_Leo1.ino ./arduino
		cp /Users/de/Work/ARDUINO/devel/PROJECTS/USBcycle_Leo2/USBcycle_Leo2.ino ./arduino
		cp /Users/de/Work/ARDUINO/devel/PROJECTS/USBcycle_WASD/USBcycle_WASD.ino ./arduino
		cp /Users/de/Work/ARDUINO/devel/PROJECTS/USBcycle_Diagnostic/USBcycle_Diagnostic.ino ./arduino
		cp /Users/de/Work/ARDUINO/devel/PROJECTS/USBcycle_Leo1_Test/USBcycle_Leo1_Test.ino ./arduino
		cp /Users/de/Work/ARDUINO/devel/PROJECTS/USBcycle_Leo2_Test/USBcycle_Leo2_Test.ino ./arduino

getprof:
		cp -r /Users/de/Library/Application\ Support/Euro\ Truck\ Simulator\ 2/profiles/4379636C697374 ETS2_profiles
		find . -name '*autosave_job*' -exec /bin/rm -rf {} \;

getmods:
		cp /Users/de/Library/Application\ Support/Euro\ Truck\ Simulator\ 2/mod/BikeView*.scs ETS2_mods

push:		
	git push origin master

pull:		
	git pull origin master

init:
	git remote add origin https://github.com/RootlessAgrarian/USBcycle\-EuroBikeSimulator.git
