getduino:	
		cp /Ume '*autosave*.sii' -exec /bin/rm {} \;
		sers/de/Work/ARDUINO/devel/PROJECTS/Leo1_beta/Leo1_beta.ino ./arduino
		cp /Users/de/Work/ARDUINO/devel/PROJECTS/Leo2_beta/Leo2_beta.ino ./arduino

getprof:
		cp -r /Users/de/Library/Application\ Support/Euro\ Truck\ Simulator\ 2/profiles/4379636C697374 ETS2_profiles
		find . -name '*autosave_job*' -exec /bin/rm -rf {} \;

getmods:
		cp /ers/de/Library/Application\ Support/Euro\ Truck\ Simulator\ 2/mod/BikeView*.scs ETS2_mods

