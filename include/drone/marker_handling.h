/**
 * This is a template for a header of a general node
 */

#ifndef MARKER_HANDLING_H
#define MARKER_HANDLING_H

#include "drone/ARMarkers.h"
#include "drone/ARMarker.h"

typedef struct Marker{
  int type;

  float dist_x;
  float dist_y;
  float dist_z;
  
  float pose_x;
  float pose_y;
  float pose_z;
  float pose_w;

  struct Marker* link;

} Marker;

typedef struct AllMarkers{
  int seq;
  int markerNum;

  struct Marker* head;

} AllMarkers;

AllMarkers* msg2marker(const drone::ARMarkers::ConstPtr &msg);
std::string marker2str(AllMarkers* markers);
AllMarkers* str2marker(char* str);
void printMarkers(AllMarkers* markers);
void printTypes(int type);

#endif //END_IF_MARKER_HANDLING_H