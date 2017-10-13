# EuroBikeSimulator
Arduino code and other bits and pieces required to ride a bike through ETS2

NOTE (Sep 20 2017):  this project is just about ready for public download... 
The project is complete (this version) and working, but could do with improved documentation
and packaging.  Youtube video is probably the fastest way to understand what it's about:

https://youtu.be/vI55Mqzf5uQ

The project in a nutshell is this:  Arduino code and related Euro Truck Simulator 2
mods to make it possible to ride a stationary bike (in my case my regular mtb on a trainer stand) through 
ETS2, using the bike's own brake, steering and pedals as a controller, with a control panel (buttons, LEDs 
etc) which mounts on the bike handlebars so no keyboard is required.  So ETS2 becomes a HUGE open world 
virtual cycling app.  Here's a long, elaborate writeup explaining the rationale, design decisions, etc.

https://create.arduino.cc/projecthub/Tazling/usbcycle-ride-through-your-virtual-world-8ff961

This project does not qualify as a pro trainer (yet).  There is as yet no telemetry/feedback from
the game (for realistic effort on grades etc) nor is there any provision for advanced trainer stands
with lean-steering.  It is a simple USB game controller.  There are two versions:  one interfaces with
basic WASD games, the other is the full-blown kbrd/mouse/3-axis version suitable for driving sims.

This version is a prototype.  If there is sufficient interest I might eventually make a packaged 
"USB Bike Controller kit" ... but it is pretty easy to DIY and I am happy to share my code and notes with
anyone who wants to pursue this excellent off-season fitness strategy.  Riding through the endless
midsummer of ETS2 is not only good exercise but good winter light therapy to offset to gloom of the long
dark days!

Huge thanks and appreciation to the team at SCS, authors of the core ETS2 game, and the ProMods team,
authors of imho the best map extension available.  Both teams have created a world of startling beauty
and richness which is a pleasure to visit.  Also thanks to a few pioneer makers who have created similar
bike game controllers over the last couple of years:  your projects really inspired me.
