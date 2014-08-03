#ifndef DATA_HANDLING_H
#define DATA_HANDLING_H

#include <string>

using namespace std;

// Rotation Matrix
typedef struct RotationMatrix{
  double matrix[3][3];
} RotationMatrix;

// Quaternion
typedef struct Quaternion{
  double w, x, y, z;
} Quaternion;

// Data passed to drone
typedef struct DroneData{
public:
  // Ensure full initialization when created
  DroneData(){rm = new RotationMatrix;}

  // Sequence Number
  int seq;
  
  // Distance from Camera
  double dist_x;
  double dist_y;
  double dist_z;

  // Rotation Matrix
  RotationMatrix* rm;

  // Extra Data
  string comment;

} DroneData;

// Structure for holding data needed by both server client
typedef struct SharedData{
  int id; // use to check if new data or not
  string etc;
} SharedData;

// Converts a Quaternion to a Rotatino Matrix
RotationMatrix* quat2rm(Quaternion* quat); 

// Converts DroneData into a string of bytes
void serialize(const DroneData* data, char*& buf, int & buf_size);
// Converts a string of bytes into DroneData
DroneData* deserialize(const char* msg);
// Prints the type DroneData to the screen
void printData(const DroneData* data);

#endif //END_IF_DATA_HANDLING_H