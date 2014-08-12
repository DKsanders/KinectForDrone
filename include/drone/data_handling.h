/**
 * This file (data_handling.h) provides functions for processing 
 * and communicating drone data, implemented in data_handling.cpp
 *
 * Author: David Sanders <david.sanders@mail.utoronto.ca>
 */

#ifndef DATA_HANDLING_H
#define DATA_HANDLING_H

#include <string>

using namespace std;

// Data passed to drone
struct DroneData{
public:
  DroneData();
  ~DroneData();

  // Sequence Number
  int seq;
  
  // Distance from Camera
  double dist_x;
  double dist_y;
  double dist_z;

  // Rotation Matrix
  //double rm[3][3];
  double roll, pitch, yaw;

  // Extra Data
  string comment;

  // Functions
  void print();

};

/**
 * Converts DroneData into a byte stream
 * Arguments:
 *  data(INPUT) - DroneData that needs to be serialized
 *  buf(OUTPUT) - buffer for holding byte stream
 *  buf_size(OUTPUT) - number of bytes in byte stream
 */
void serialize(const DroneData* data, char*& buf, int & buf_size);

/**
 * Converts a byte stream into DroneData
 * Arguments:
 *  msg(INPUT) - byte stream
 * Return:
 *  a pointer to a DroneData structure, created using new
 */
DroneData* deserialize(const char* msg);

#endif //END_IF_DATA_HANDLING_H