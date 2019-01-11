
#include "Arduino.h"
#include "String.h"

boolean parseDump(String msg) {
  // we know the order of the fields in the csv string called 'msg'.
  // they are:  page, id, x, y, w, h, type, (char) alias.
  // see above (top of code, globals):
  // struct nex_item {
  // int page;
  // int id;
  // void *alias;
  // int type;
  // int x;
  // int y;
  // int w;
  // int h;
  // };
  // nex_item gizmos[];
  // int gizmo_ct = 0;
  // 
  String foo;
  int i = 0;
  // Serial.print(F("Trying to parse msg: "));Serial.println(msg);
  foo = getValue(msg, ',' , i);  i++;
  if (!foo.toInt()) {return 1;}
  gizmos[gizmo_ct].page = foo.toInt();
  foo = getValue(msg, ',' , i);  i++;
  gizmos[gizmo_ct].id = foo.toInt();
  foo = getValue(msg, ',' , i);  i++;
  gizmos[gizmo_ct].x = foo.toInt();
  foo = getValue(msg, ',' , i);  i++;
  gizmos[gizmo_ct].y = foo.toInt();
  foo = getValue(msg, ',' , i);  i++;
  gizmos[gizmo_ct].w = foo. toInt();
  foo = getValue(msg, ',' , i);  i++;
  gizmos[gizmo_ct].h = foo.toInt();
  foo = getValue(msg, ',' , i);  i++;
  gizmos[gizmo_ct].type = foo.toInt();
  foo = getValue(msg, ',' , i);  i++;  // SKIP the type string field
  foo = getValue(msg, ',' , i);  i++;
  gizmos[gizmo_ct].alias = foo;
  gizmo_ct++;

// I had an error code here but not sure how I would ever know if
// there was an error.  I guess foo would be void?
  return 0;
 
}

void printGizmos() {

// print out the Gizmos we have collected
// in a useful struct array init form which can be pasted into
// our .h file... but don't exec this code in this sketch.
// weird things may happen!

  Serial.println(F("struct nex_item gizmos[] = {"));
  for (int i = 0; i< gizmo_ct; i++) {
    Serial.print("{");
    Serial.print(String(gizmos[i].page) + "," + String(gizmos[i].id) + "," + String(gizmos[i].type) + ",");
    Serial.print(String(gizmos[i].x) + "," + String(gizmos[i].y) + "," + String(gizmos[i].w) + "," + String(gizmos[i].h) + ",");
    Serial.print("0,");Serial.print("\"");Serial.print(gizmos[i].alias),Serial.print("\"");
    Serial.println("},");
  }
  Serial.println("}");
  Serial.print("int gizmo_ct = ");  Serial.print(gizmo_ct);  Serial.println(";");
}

int findGizmo(int pg, int id) {

  int pg2, id2;
  
  for (int i=0; i< gizmo_ct; i++) {
    pg2 = gizmos[i].page;
    id2 = gizmos[i].id;
    if ((pg == pg2) && (id == id2)) {
      return i;
    }
   }
   return -1;
}


