# EuroBikeSimulator
# Arduino code and other bits and pieces required to ride a bike through ETS2

NOTE (Oct 14 2017):  USBcycle is just about ready for public download... and about to be tagged 1.0.

The project is complete (this version) and working, but could do with improved documentation
and packaging (ongoing).  Youtube video is probably the fastest way to understand what it's about:

https://youtu.be/vI55Mqzf5uQ

The project in a nutshell is this:  USBcycle Arduino hw/sw and related Euro Truck Simulator 2
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

What you need to make this work:

-- a gaming computer, and a game that accepts (at most) 3 axis control plus mouse and keyboard, or just 
WASD keyboard.  I built this project to work with Euro Truck Simulator (and American Truck Simulator), 
but you could adapt it to other driving, walking, or endless-runner games.

-- a bicycle + trainer-stand or other device (rotary stepper, treadmill, etc) with a rotating or reciprocating 
mechanism that would enable you to detect fwd (and reverse if desired) motion using reed switches and magnets.

-- 2 Arduinos with USB HID capability, i.e. Leos, Pro Micros, Dues.

-- associated parts (see detailed project writeup above):   LEDs, switches, sensors, wire, proto boards, etc

-- a few evenings of your time to attach sensors to bike, make a harness, build a control box for the duinos, etc.

-- (preferably) a basic USB debug/monitor tool to help you verify the functioning of your control box

-- a few more evenings of your time to test with your game and tune your USBcycle for your specific application:
if you are using ETS2, you'll want to install my Cyclist profile and BikeView mod (tweaks to the Mercedes Actros
truck to suppress truck sfx and move camera 5 to a cyclist's head position ahead of the front bumper).


*** Future Plans:

-- One immediate plan is to build a WASD version customised for CrashDrive (one of the silliest, but most entertaining
simple driving games around).  CrashDrive is well suited to USBcycle and I hope to make a nice responsive (and crazy)
biking experience out of it.

-- Improving ETS2 and ATS support with more appopriate base vehicle, better sound mods, etc.

-- Improving WASD version for walking games like Dear Esther, Portal etc.


*** NOTE as of Fall 2017:

For some reason, USBcycle used to work with Unity games such as Off Peak (which was one of my favourite test games early
in development, April/May 2017).  However, as of October 2017 Unity games are consistently refusing to recognise any 
Mouse X/Y input from the Leonardo.  They do recognise mouse button presses, just not movement.  I have posted queries to
Unity Answers and to Arduino forum and am waiting hopefully for a clue.  In the mean time, I have to warn users that
Unity-based games probably will not work with USBcycle, and this is very disappointing (because I do a little virtual
world building in Unity myself).
