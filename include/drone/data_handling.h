/**
 * This file (data_handling.h) provides functions for processing 
 * and communicating drone data, implemented in data_handling.cpp
 *
 * Author: David Sanders <david.sanders@mail.utoronto.ca>
 */

#ifndef DATA_HANDLING_H
#define DATA_HANDLING_H

#include "network/network.h"
#include <string>

using namespace std;

// Data passed to drone
class DroneData{
public:
  DroneData();
  DroneData(ByteStream& stream);
  ~DroneData();

  // Distance from Camera
  double dist_x;
  double dist_y;
  double dist_z;

  //double rm[3][3]; // Rotation Matrix
  double roll, pitch, yaw;

  // Extra Data
  string comment;


  // Accessors
  int getSeq();

  // Mutators
  void setSeq(int _seq);

  // Converts DroneData to/from a byte stream
  ByteStream serialize();
  void deserialize(ByteStream& stream);

  void print();

private:
  // Sequence Number
  int seq;

};



#endif //END_IF_DATA_HANDLING_H