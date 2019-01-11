// This config file is all about the Nextion.
// It defines handy arrays and counters that
// describe the Nextion interface, sorta.
// It's pretty crude so far.  There could be 
// a lot more abstraction, and I could lean
// more heavily on the Nextion's internal smarts.
// But that would tie me even more tightly to
// the proprietary hardware.  I'd like to keep
// it as flexible as possible.  ITEAD could go
// out of business.  A better touchscreen could
// come along.

// 
// ================ Global variables for Nextion housekeeping ============
//
//  for data, the Nextion is essentially a write-only device.
//  all it sends us is button press events of various flavours,
//  and we respond to those (sometimes) by sending it commands.
//

String nexti_msg;
String nexti_cmd;
int nexti_page;
int nexti_id;
boolean nexti_dump;
byte nexti_ffct;
char nexti_expect = '?';

boolean ackWaiting=0;
boolean valWaiting=0;

// ====================== structs and arrays for Nextion state ==============

struct nex_return {
  int code;
  String msg;
};

struct nex_return nexti_back[36] {
  {0,"Invalid instruction"},
  {1,"Successful execution"},
  {2,"Invalid Component ID"},
  {3,"Invalid Page ID"},
  {4,"Invalid Picture ID"},
  {5,"Invalid Font ID"},
  {6,""},{7,""},{8,""},{9,""},{10,""},{11,""},{12,""},{13,""},{14,""},{15,""},{16,""},
  {0x11,"Invalid Baud Rate"},
  {0x12,"Invalid curve ctrl/channel"},
  {0x13,""},{0x14,""},{0x15,""},{0x16,""},{0x17,""},{0x18,""},{0x19,""},
  {0x1a,"Invalid variable name"},
  {0x1b,"Invalid variable operation"},
  {0x1c,"Failed to assign"},
  {0x1d,"EEPROM operation failed"},
  {0x1e,"Invalid Parameter Qty"},
  {0x1f,"I/O Failure"},
  {0x20,"Undefined escape chars"},
  {0x21,""},{0x22,""},
  {0x23,"Variable name too long"}
};

//
///  Images:  array of background images for Nextion pages
//
const int PageCt = 7;     // this must be set in advance.  a priori knowledge required.
// This array is indexed like so:  Images(1,0) is Page 1, Empty;  Images(1,1) is Page 1, Full.
// and so on.  It's important to get this right! as this is how we map picqs for bar graphs.
// also how we turn checkbuttons on and off, etc.
int Images [PageCt] [2] = {
  {0,0},{2,1},{3,3},{5,4},{7,6},{8,8},{10,9}
};


// struct defining a nextion item (component)
struct nex_item {
  int page;
  int id;
  int type;
  int x;
  int y;
  int w;
  int h;
  float val;
  String alias;
};
//
// make this oversized so you can cope with new widgets?
// nope, we are generating it so it is always just the right size.
// NOTE! this array is GENERATED code.  There's a sketch
// called NextionBasic_Due and it contains code that uses
// Patrick (ITEAD's) handy component lister.  It's a real
// kluge but beats typing in the following lines by hand.
// Instructions will be in the README.
// ** YOU HAVE TO RUN THIS CODE (at least for the affected page)
// ** EVERY TIME YOU CREATE A NEW HOTSPOT OR OTHER COMPONENT.
// ** THERE WILL BE DIRE CONSEQUENCES IF YOU FORGET TO DO THIS!
// (like accvio, segv, memory trampling).
//
// BEGIN GENERATED CODE
//
struct nex_item gizmos[] = {
{1,1,116,109,30,100,30,0,"t0"},
{1,2,116,288,31,100,30,0,"t1"},
{1,3,109,104,77,55,55,0,"m0"},
{1,4,109,161,78,55,55,0,"m1"},
{1,5,109,221,79,55,55,0,"m2"},
{1,6,109,281,78,55,55,0,"m3"},
{1,7,109,341,79,55,55,0,"m4"},
{1,8,109,104,139,55,55,0,"m5"},
{1,9,109,163,138,55,55,0,"m6"},
{1,10,109,221,137,55,55,0,"m7"},
{1,11,109,283,138,55,55,0,"m8"},
{1,12,109,341,137,55,55,0,"m9"},
{1,13,109,103,196,55,55,0,"m10"},
{1,14,109,161,197,55,55,0,"m11"},
{1,15,109,221,196,55,55,0,"m12"},
{1,16,109,281,196,55,55,0,"m13"},
{1,17,109,339,197,55,55,0,"m14"},
{1,18,109,430,84,40,40,0,"m15"},
{1,19,109,432,131,40,40,0,"m16"},
{1,20,109,431,178,40,40,0,"m17"},
{1,21,109,433,223,40,40,0,"m18"},
{1,22,109,52,72,30,160,0,"m19"},
{1,23,109,15,72,30,160,0,"m20"},
{1,24,109,207,6,80,40,0,"m21"},
{1,25,116,6,36,80,30,0,"t2"},
{2,1,116,0,93,100,30,0,"t0"},
{2,2,116,1,152,100,30,0,"t1"},
{2,3,109,104,20,55,55,0,"m0"},
{2,4,109,162,21,55,55,0,"m1"},
{2,5,109,222,22,55,55,0,"m2"},
{2,6,109,280,24,55,55,0,"m3"},
{2,7,109,340,21,55,55,0,"m4"},
{2,8,109,105,80,55,55,0,"m5"},
{2,9,109,164,82,55,55,0,"m6"},
{2,10,109,221,81,55,55,0,"m7"},
{2,11,109,282,81,55,55,0,"m8"},
{2,12,109,341,81,55,55,0,"m9"},
{2,13,109,105,140,55,55,0,"m10"},
{2,14,109,163,141,55,55,0,"m11"},
{2,15,109,222,141,55,55,0,"m12"},
{2,16,109,283,140,55,55,0,"m13"},
{2,17,109,341,141,55,55,0,"m14"},
{2,18,109,105,199,55,55,0,"m15"},
{2,19,109,164,200,55,55,0,"m16"},
{2,20,109,224,198,55,55,0,"m17"},
{2,21,109,282,200,55,55,0,"m18"},
{2,22,109,341,199,55,55,0,"m19"},
{2,23,109,434,40,40,40,0,"m20"},
{2,24,109,432,131,40,40,0,"m21"},
{2,25,109,433,177,40,40,0,"m22"},
{2,26,109,432,223,40,40,0,"m23"},
{3,1,109,433,39,40,40,0,"m0"},
{3,2,109,39,54,50,50,0,"m1"},
{3,3,109,41,164,50,50,0,"m2"},
{3,4,109,356,164,50,50,0,"m3"},
{3,5,109,181,225,90,40,0,"m4"},
{3,6,109,242,66,165,40,0,"m5"},
{3,7,109,246,117,165,40,0,"m6"},
{3,8,109,38,116,165,40,0,"m7"},
{3,9,116,99,178,250,30,0,"t0"},
{3,10,109,432,87,40,40,0,"m8"},
{3,11,109,431,177,40,40,0,"m9"},
{3,12,109,433,224,40,40,0,"m10"},
{3,13,109,128,54,50,50,0,"m11"},
{4,1,116,300,43,100,30,0,"t0"},
{4,2,116,300,89,100,30,0,"t1"},
{4,3,116,300,146,100,30,0,"t2"},
{4,4,116,130,42,65,30,0,"t3"},
{4,5,116,130,92,65,30,0,"t4"},
{4,6,116,130,145,65,30,0,"t5"},
{4,7,109,166,206,90,45,0,"m0"},
{4,8,109,434,39,40,40,0,"m1"},
{4,9,109,433,88,40,40,0,"m2"},
{4,10,109,432,130,40,40,0,"m3"},
{4,11,109,432,224,40,40,0,"m4"},
{4,12,109,10,47,115,20,0,"m5"},
{4,13,109,13,98,115,20,0,"m6"},
{4,14,109,10,151,115,20,0,"m7"},
{5,1,109,433,40,40,40,0,"m0"},
{5,2,109,433,87,40,40,0,"m1"},
{5,3,109,433,131,40,40,0,"m2"},
{5,4,109,433,175,40,40,0,"m3"},
{6,1,116,337,123,25,30,0,"t0"},
{6,2,116,362,123,25,30,0,"t1"},
{6,3,116,385,124,20,30,0,"t2"},
{6,4,116,405,124,25,30,0,"t3"},
{6,5,116,430,124,25,30,0,"t4"},
{6,6,116,139,61,80,30,0,"t5"},
{6,7,116,139,93,80,30,0,"t6"},
{6,8,116,139,125,80,30,0,"t7"},
{6,9,116,139,157,80,30,0,"t8"},
{6,10,116,139,189,80,30,0,"t9"},
{6,11,116,139,225,80,30,0,"t10"},
{6,12,109,344,198,110,60,0,"m0"},
{6,13,109,398,12,60,60,0,"m1"},
{6,14,109,334,94,30,30,0,"m2"},
{6,15,109,363,91,30,30,0,"m3"},
{6,16,109,401,93,30,30,0,"m4"},
{6,17,109,426,94,30,30,0,"m5"},
{6,18,109,332,154,30,30,0,"m6"},
{6,19,109,361,153,30,30,0,"m7"},
{6,20,109,399,155,30,30,0,"m8"},
{6,21,109,429,155,30,30,0,"m9"},
{6,22,109,254,170,75,90,0,"m10"},
{6,23,109,243,113,90,50,0,"m11"},
};
int gizmo_ct = 105;

// END GENERATED CODE 

