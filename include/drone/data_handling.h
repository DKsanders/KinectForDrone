/**
 * This file (data_handling.h) provides functions for processing 
 * and communicating drone data, implemented in data_handling.cpp
 *
 * Author: David Sanders <david.sanders@mail.utoronto.ca>
 */

#ifndef DATA_HANDLING_H
#define DATA_HANDLING_H

#include "drone/network.h"
#include <string>

using namespace std;

class CalibrationData {
public:
  CalibrationData();
  ~CalibrationData();

  // Accessors
  double getX() { return xOffset; };
  double getY() { return yOffset; };
  double getZ() { return zOffset; };
  double getRoll() { return rollOffset; };
  double getPitch() { return pitchOffset; };
  double getYaw() { return yawOffset; };

  // Callibrating
  void calibrate(double x, double y, double z, double roll, double pitch, double yaw);

private:
  double xOffset;
  double yOffset;
  double zOffset;

  double rollOffset;
  double pitchOffset;
  double yawOffset;

  int sampleNum; // number of current samples processed
};

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
  int getSeq() { return seq; };

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