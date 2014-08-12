/**
 * This file (my_matrix.h) provides basic matrices and
 * matrix operations, implemented in my_matrix.cpp
 *
 * SHOULD BE REPLACED BY PROPER MATRIX LIBRARY!!
 *
 * Author: David Sanders <david.sanders@mail.utoronto.ca>
 */

#ifndef MY_MATRIX_H
#define MY_MATRIX_H

#include "data_handling.h"
#include "markers.h"

using namespace std;

// Quaternion
struct Quaternion{
  double w, x, y, z;
  void print();
};

// Rotation Matrix
class RotationMatrix{
public:
  RotationMatrix();
  RotationMatrix(const Quaternion& quat);
  RotationMatrix(const MarkerData& rhs);
  ~RotationMatrix();

  double matrix[3][3];

  double getDeterminant();
  void transpose();
  RotationMatrix getTranspose();
  void invert();
  RotationMatrix getInverse();
  void print();

  RotationMatrix operator+(const RotationMatrix& rhs);
  RotationMatrix operator-(const RotationMatrix& rhs);
  RotationMatrix operator*(const RotationMatrix& rhs);
  RotationMatrix operator+(const double rhs);
  RotationMatrix operator-(const double rhs);
  RotationMatrix operator*(const double rhs);
  RotationMatrix operator/(const double rhs);
  RotationMatrix& operator=(const Quaternion& quat);
  RotationMatrix& operator=(const MarkerData& rhs);
private:

};

// Homogeneous Matrix
class HomogeneousMatrix{
public:
  HomogeneousMatrix();
  HomogeneousMatrix(const Quaternion& quat);
  HomogeneousMatrix(const RotationMatrix& rhs);
  HomogeneousMatrix(const MarkerData& rhs);
  ~HomogeneousMatrix();
  
  double matrix[4][4];
  double roll;
  double pitch;
  double yaw;

  double getDeterminant();
  void transpose();
  HomogeneousMatrix getTranspose();
  void invert();
  HomogeneousMatrix getInverse();
  void print();
  DroneData toData();
  void hm2rpy();


  HomogeneousMatrix operator+(const HomogeneousMatrix& rhs);
  HomogeneousMatrix operator-(const HomogeneousMatrix& rhs);
  HomogeneousMatrix operator*(const HomogeneousMatrix& rhs);
  HomogeneousMatrix operator+(const double rhs);
  HomogeneousMatrix operator-(const double rhs);
  HomogeneousMatrix operator*(const double rhs);
  HomogeneousMatrix operator/(const double rhs);
  HomogeneousMatrix& operator=(const RotationMatrix& rhs);
  HomogeneousMatrix& operator=(const Quaternion& quat);
  HomogeneousMatrix& operator=(const MarkerData& rh);

private:

};

/**
 * Converts a Quaternion to a Rotatino Matrix
 * Arguments:
 *  quat(INPUT) - a pointer to a quaternion structure to be converted
 * Return:
 *  a pointer to a rotation matrix struct, created using new
 */
//RotationMatrix* quat2rm(Quaternion* quat); 

//DroneData& DroneData::operator=(const RotationMatrix& rhs);
//DroneData& DroneData::operator=(const HomogeneousMatrix& rhs);

#endif //END_IF_MY_MATRIX_H