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

#include "drone/data_handling.h"
#include "drone/parsers.h"

using namespace std;

// Vector
class Vector{
public:
  double* vector;

  Vector();
  Vector(int _size);
  Vector(const MarkerData& data);
  Vector(const Vector& other);
  ~Vector();

  // Accessors
  int getSize() { return size; };

  double getNorm();
  void normalize();
  
  void fromMarkerData(const MarkerData& rhs);
  void print();

  Vector& operator=(const Vector& rhs);
  Vector operator*(const Vector& rhs); // cross-product
  Vector operator+(const Vector& rhs);
  Vector operator-(const Vector& rhs); 
  Vector operator*(double rhs); 
  Vector operator/(double rhs);
  Vector operator+(double rhs);
  Vector operator-(double rhs); 

private:
  int size;
};

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

  // Accessors
  double getRoll() { return roll; };
  double getPitch() { return pitch; };
  double getYaw() { return yaw; };

  // Mutators
  void setRoll(double _roll);
  void setPitch(double _pitch);
  void setYaw(double _yaw);

  // Matrix operations
  double getDeterminant();
  void transpose();
  HomogeneousMatrix getTranspose();
  void calc_rpy();

  // Prints the matrix; rpy printed in degrees
  void print();
  // Calibrates the rpy and distances
  void calibrate(CalibrationData calib);

  // Data conversions
  DroneData toDroneData();
  void fromMarkerData(const MarkerData& rhs);

  HomogeneousMatrix operator+(const HomogeneousMatrix& rhs);
  HomogeneousMatrix operator-(const HomogeneousMatrix& rhs);
  HomogeneousMatrix operator*(const HomogeneousMatrix& rhs);
  HomogeneousMatrix operator+(const double rhs);
  HomogeneousMatrix operator-(const double rhs);
  HomogeneousMatrix operator*(const double rhs);
  HomogeneousMatrix operator/(const double rhs);
  HomogeneousMatrix& operator=(const RotationMatrix& rhs);
  HomogeneousMatrix& operator=(const Quaternion& quat);

private:
  double roll;
  double pitch;
  double yaw;

};

#endif //END_IF_MY_MATRIX_H