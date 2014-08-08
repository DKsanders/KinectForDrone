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

// Rotation Matrix
typedef class RotationMatrix{
public:
  RotationMatrix();
  ~RotationMatrix();

  double matrix[3][3];

  double getDeterminant();
  RotationMatrix getInverse();
  RotationMatrix operator*(const RotationMatrix& rhs);
private:

} RotationMatrix;

// Homogeneous Matrix
typedef class HomogeneousMatrix{
public:
  HomogeneousMatrix();
  ~HomogeneousMatrix();
  
  double matrix[4][4];

  double getDeterminant();
  HomogeneousMatrix getInverse();
  HomogeneousMatrix operator*(const HomogeneousMatrix& rhs);
  HomogeneousMatrix operator=(const RotationMatrix& rhs);

private:

} HomogeneousMatrix;

// Quaternion
typedef struct Quaternion{
  double w, x, y, z;
  void print();
} Quaternion;

// Data passed to drone
typedef struct DroneData{
public:
  DroneData(){rm = new RotationMatrix;}
  ~DroneData(){delete rm;}

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

/**
 * Converts a Quaternion to a Rotatino Matrix
 * Arguments:
 *  quat(INPUT) - a pointer to a quaternion structure to be converted
 * Return:
 *  a pointer to a rotation matrix struct, created using new
 */
RotationMatrix* quat2rm(Quaternion* quat); 

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

/**
 * Prints the type DroneData to the screen
 * Arguments:
 *  data(INPUT) - a pointer to the data you want printed
 */
void printData(const DroneData* data);

#endif //END_IF_DATA_HANDLING_H