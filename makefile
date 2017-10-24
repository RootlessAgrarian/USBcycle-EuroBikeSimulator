getduino:	
		cp /Users/de/Work/ARDUINO/devel/PROJECTS/Leo1_Current/Leo1_Current.ino ./arduino
		cp /Users/de/Work/ARDUINO/devel/PROJECTS/Leo2_Current/Leo2_Current.ino ./arduino
		cp /Users/de/Work/ARDUINO/devel/PROJECTS/Leo_WASD_v3/Leo_WASD_v3.ino ./arduino
		cp /Users/de/Work/ARDUINO/devel/PROJECTS/USBcycle_Diagnostic/USBcycle_Diagnostic.ino ./arduino
		cp /Users/de/Work/ARDUINO/devel/PROJECTS/Leo1_Diagnostic/Leo1_Diagnostic.ino ./arduino
		cp /Users/de/Work/ARDUINO/devel/PROJECTS/Leo2_Diagnostic/Leo2_Diagnostic.ino ./arduino

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
