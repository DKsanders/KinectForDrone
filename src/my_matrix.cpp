/**
 * This file (my_matrix.cpp) implements basic matrices and 
 * matrix operations, declared in my_matrix.h
 *
 * SHOULD BE REPLACED BY PROPER MATRIX LIBRARY!!
 *
 * Author: David Sanders <david.sanders@mail.utoronto.ca>
 */

#include "drone/my_matrix.h"
#include <iostream>
#include <math.h>

using namespace std;

void Quaternion::print(){
    cout << "---------------------------------------------------------------------"<< endl;
    cout << "Quaternion: " << endl;
    cout << "  w: " << w << endl;
    cout << "  x: " << x << endl;
    cout << "  y: " << y << endl;
    cout << "  z: " << z << endl;
}

RotationMatrix::RotationMatrix(){
    matrix[0][0] = 0;
    matrix[0][1] = 0;
    matrix[0][2] = 0;
    matrix[1][0] = 0;
    matrix[1][1] = 0;
    matrix[1][2] = 0;
    matrix[2][0] = 0;
    matrix[2][1] = 0;
    matrix[2][2] = 0;
}

RotationMatrix::RotationMatrix(const Quaternion& quat){
    // Convert
    double z = sqrt(quat.w * quat.w + quat.x * quat.x + quat.y * quat.y + quat.z * quat.z);
    double a = quat.w/z;
    double b = quat.x/z;
    double c = quat.y/z;
    double d = quat.z/z;

    matrix[0][0] = a*a + b*b - c*c - d*d;
    matrix[0][1] = 2*b*c - 2*a*d;
    matrix[0][2] = 2*b*d + 2*a*c;
    matrix[1][0] = 2*b*c + 2*a*d;
    matrix[1][1] = a*a - b*b + c*c - d*d;
    matrix[1][2] = 2*c*d - 2*a*b;
    matrix[2][0] = 2*b*d - 2*a*c;
    matrix[2][1] = 2*c*d + 2*a*b;
    matrix[2][2] = a*a - b*b - c*c + d*d;
}

RotationMatrix::RotationMatrix(const MarkerData& rhs){
    RotationMatrix temp;
    temp = rhs;
    *this = temp;
}

RotationMatrix::~RotationMatrix(){
    ;
}

double RotationMatrix::getDeterminant(){
    return matrix[0][0]* (matrix[1][1]*matrix[2][2] - matrix[2][1]*matrix[1][2])
           - matrix[0][1]* (matrix[1][0]*matrix[2][2] - matrix[2][0]*matrix[1][2])
           + matrix[0][2]* (matrix[1][0]*matrix[2][1] - matrix[2][0]*matrix[1][1]);
}

void RotationMatrix::transpose(){
    double temp = matrix[0][1];
    matrix[0][1] = matrix[1][0];
    matrix[1][0] = temp;
    temp = matrix[0][2];
    matrix[0][2] = matrix[2][0];
    matrix[2][0] = temp;
    temp = matrix[1][2];
    matrix[1][2] = matrix[2][1];
    matrix[2][1] = temp;
}

RotationMatrix RotationMatrix::getTranspose(){
    RotationMatrix temp = *this;
    temp.transpose();
    return temp;
}

void RotationMatrix::invert(){
    this->transpose();
}

RotationMatrix RotationMatrix::getInverse(){
    RotationMatrix temp = *this;
    temp.invert();
    return temp;
}

void RotationMatrix::print(){
    cout << "---------------------------------------------------------------------"<< endl;
    cout << "Rotation Matrix: "  << endl;
    cout << "  " << this->matrix[0][0]  << "  " << this->matrix[0][1]  << "  " << this->matrix[0][2] << endl;
    cout << "  " << this->matrix[1][0]  << "  " << this->matrix[1][1]  << "  " << this->matrix[1][2] << endl;
    cout << "  " << this->matrix[2][0]  << "  " << this->matrix[2][1]  << "  " << this->matrix[2][2] << endl;
}

RotationMatrix RotationMatrix::operator+(const RotationMatrix& rhs){
    RotationMatrix rtn;
    int SIZE = 3;
    int i,j;
    for(i=0; i<SIZE; i++){
        for(j=0; j<SIZE; j++){
            rtn.matrix[i][j] = this->matrix[i][j] + rhs.matrix[i][j];
        }
    }
    return rtn;
}

RotationMatrix RotationMatrix::operator-(const RotationMatrix& rhs){
    RotationMatrix rtn;
    int SIZE = 3;
    int i,j;
    for(i=0; i<SIZE; i++){
        for(j=0; j<SIZE; j++){
            rtn.matrix[i][j] = this->matrix[i][j] - rhs.matrix[i][j];
        }
    }
    return rtn;
}

RotationMatrix RotationMatrix::operator*(const RotationMatrix& rhs){
    RotationMatrix rtn;
    double sum;
    int i,j,k;
    int SIZE = 3;
    for ( i = 0 ; i < SIZE ; i++) {
      for ( j = 0 ; j < SIZE ; j++) {
        sum = 0;
        for ( k = 0 ; k < SIZE ; k++) {
          sum += this->matrix[i][k] * rhs.matrix[k][j];
        }
        rtn.matrix[i][j] = sum;
      }
    }
    return rtn;
}

RotationMatrix RotationMatrix::operator+(const double rhs){
    RotationMatrix rtn;
    int SIZE = 3;
    int i,j;
    for(i=0; i<SIZE; i++){
        for(j=0; j<SIZE; j++){
            rtn.matrix[i][j] = this->matrix[i][j] + rhs;
        }
    }
    return rtn;
}

RotationMatrix RotationMatrix::operator-(const double rhs){
    RotationMatrix rtn;
    int SIZE = 3;
    int i,j;
    for(i=0; i<SIZE; i++){
        for(j=0; j<SIZE; j++){
            rtn.matrix[i][j] = this->matrix[i][j] - rhs;
        }
    }
    return rtn;
}

RotationMatrix RotationMatrix::operator*(const double rhs){
    RotationMatrix rtn;
    int SIZE = 3;
    int i,j;
    for(i=0; i<SIZE; i++){
        for(j=0; j<SIZE; j++){
            rtn.matrix[i][j] = this->matrix[i][j] * rhs;
        }
    }
    return rtn;
}

RotationMatrix RotationMatrix::operator/(const double rhs){
    RotationMatrix rtn;
    int SIZE = 3;
    int i,j;
    for(i=0; i<SIZE; i++){
        for(j=0; j<SIZE; j++){
            rtn.matrix[i][j] = this->matrix[i][j] / rhs;
        }
    }
    return rtn;
}

RotationMatrix& RotationMatrix::operator=(const Quaternion& quat){
     // Convert
    double z = sqrt(quat.w * quat.w + quat.x * quat.x + quat.y * quat.y + quat.z * quat.z);
    double a = quat.w/z;
    double b = quat.x/z;
    double c = quat.y/z;
    double d = quat.z/z;

    this->matrix[0][0] = a*a + b*b - c*c - d*d;
    this->matrix[0][1] = 2*b*c - 2*a*d;
    this->matrix[0][2] = 2*b*d + 2*a*c;
    this->matrix[1][0] = 2*b*c + 2*a*d;
    this->matrix[1][1] = a*a - b*b + c*c - d*d;
    this->matrix[1][2] = 2*c*d - 2*a*b;
    this->matrix[2][0] = 2*b*d - 2*a*c;
    this->matrix[2][1] = 2*c*d + 2*a*b;
    this->matrix[2][2] = a*a - b*b - c*c + d*d;
    
    return *this;
}

RotationMatrix& RotationMatrix::operator=(const MarkerData& rhs){
    this->matrix[0][0] = rhs.rm[0][0];
    this->matrix[0][1] = rhs.rm[0][1];
    this->matrix[0][2] = rhs.rm[0][2];
    this->matrix[1][0] = rhs.rm[1][0];
    this->matrix[1][1] = rhs.rm[1][1];
    this->matrix[1][2] = rhs.rm[1][2];
    this->matrix[2][0] = rhs.rm[2][0];
    this->matrix[2][1] = rhs.rm[2][1];
    this->matrix[2][2] = rhs.rm[2][2];
    return *this;
}

HomogeneousMatrix::HomogeneousMatrix(){
    matrix[0][0] = 0;
    matrix[0][1] = 0;
    matrix[0][2] = 0;
    matrix[1][0] = 0;
    matrix[1][1] = 0;
    matrix[1][2] = 0;
    matrix[2][0] = 0;
    matrix[2][1] = 0;
    matrix[2][2] = 0;
    matrix[3][0] = 0;
    matrix[3][1] = 0;
    matrix[3][2] = 0;
    matrix[3][3] = 1;
}

HomogeneousMatrix::HomogeneousMatrix(const Quaternion& quat){
    HomogeneousMatrix temp;
    temp = quat;
    *this = temp;
}

HomogeneousMatrix::HomogeneousMatrix(const RotationMatrix& rhs){
    HomogeneousMatrix temp;
    temp = rhs;
    *this = temp;
}

HomogeneousMatrix::HomogeneousMatrix(const MarkerData& rhs){
    HomogeneousMatrix temp;
    temp = rhs;
    *this = temp;
}

HomogeneousMatrix::~HomogeneousMatrix(){
    ;
}

double HomogeneousMatrix::getDeterminant(){
    return matrix[0][0]* (matrix[1][1]*matrix[2][2] - matrix[2][1]*matrix[1][2])
           - matrix[0][1]* (matrix[1][0]*matrix[2][2] - matrix[2][0]*matrix[1][2])
           + matrix[0][2]* (matrix[1][0]*matrix[2][1] - matrix[2][0]*matrix[1][1]);
}
void HomogeneousMatrix::transpose(){
    double temp = matrix[0][1];
    matrix[0][1] = matrix[1][0];
    matrix[1][0] = temp;
    temp = matrix[0][2];
    matrix[0][2] = matrix[2][0];
    matrix[2][0] = temp;
    temp = matrix[1][2];
    matrix[1][2] = matrix[2][1];
    matrix[2][1] = temp;

    matrix[3][0] = matrix[0][3];
    matrix[3][1] = matrix[1][3];
    matrix[3][2] = matrix[2][3];
    matrix[0][3] = 0;
    matrix[1][3] = 0;
    matrix[2][3] = 0;
}

HomogeneousMatrix HomogeneousMatrix::getTranspose(){
    HomogeneousMatrix temp = *this;
    temp.transpose();
    return temp;
}

void HomogeneousMatrix::invert(){
    this->transpose();
}

HomogeneousMatrix HomogeneousMatrix::getInverse(){
    HomogeneousMatrix temp = *this;
    temp.invert();
    return temp;
}

void HomogeneousMatrix::print(){
    cout << "---------------------------------------------------------------------"<< endl;
    cout << "Homogeneous Matrix: "  << endl;
    cout << "  " << this->matrix[0][0]  << "  " << this->matrix[0][1]  << "  " << this->matrix[0][2] << "  " << this->matrix[0][3] << endl;
    cout << "  " << this->matrix[1][0]  << "  " << this->matrix[1][1]  << "  " << this->matrix[1][2] << "  " << this->matrix[1][3] << endl;
    cout << "  " << this->matrix[2][0]  << "  " << this->matrix[2][1]  << "  " << this->matrix[2][2] << "  " << this->matrix[2][3] << endl;
    cout << "  " << this->matrix[3][0]  << "  " << this->matrix[3][1]  << "  " << this->matrix[3][2] << "  " << this->matrix[3][3] << endl;
    cout << "Roll: "  << roll << endl;
    cout << "Pitch: "  << pitch << endl;
    cout << "Yaw: "  << yaw << endl;
}

HomogeneousMatrix HomogeneousMatrix::operator+(const HomogeneousMatrix& rhs){
    HomogeneousMatrix rtn;
    int SIZE = 4;
    int i,j;
    for(i=0; i<SIZE; i++){
        for(j=0; j<SIZE; j++){
            rtn.matrix[i][j] = this->matrix[i][j] + rhs.matrix[i][j];
        }
    }
    return rtn;
}

HomogeneousMatrix HomogeneousMatrix::operator-(const HomogeneousMatrix& rhs){
    HomogeneousMatrix rtn;
    int SIZE = 4;
    int i,j;
    for(i=0; i<SIZE; i++){
        for(j=0; j<SIZE; j++){
            rtn.matrix[i][j] = this->matrix[i][j] - rhs.matrix[i][j];
        }
    }
    return rtn;
}

HomogeneousMatrix HomogeneousMatrix::operator*(const HomogeneousMatrix& rhs){
    HomogeneousMatrix rtn;
    double sum;
    int i,j,k;
    int SIZE = 4;
    for ( i = 0 ; i < SIZE ; i++) {
      for ( j = 0 ; j < SIZE ; j++) {
        sum = 0;
        for ( k = 0 ; k < SIZE ; k++) {
          sum += this->matrix[i][k] * rhs.matrix[k][j];
        }
        rtn.matrix[i][j] = sum;
      }
    }
    return rtn;
}

HomogeneousMatrix HomogeneousMatrix::operator+(const double rhs){
    HomogeneousMatrix rtn;
    int SIZE = 4;
    int i,j;
    for(i=0; i<SIZE; i++){
        for(j=0; j<SIZE; j++){
            rtn.matrix[i][j] = this->matrix[i][j] + rhs;
        }
    }
    return rtn;
}

HomogeneousMatrix HomogeneousMatrix::operator-(const double rhs){
    HomogeneousMatrix rtn;
    int SIZE = 4;
    int i,j;
    for(i=0; i<SIZE; i++){
        for(j=0; j<SIZE; j++){
            rtn.matrix[i][j] = this->matrix[i][j] - rhs;
        }
    }
    return rtn;
}

HomogeneousMatrix HomogeneousMatrix::operator*(const double rhs){
    HomogeneousMatrix rtn;
    int SIZE = 4;
    int i,j;
    for(i=0; i<SIZE; i++){
        for(j=0; j<SIZE; j++){
            rtn.matrix[i][j] = this->matrix[i][j] * rhs;
        }
    }
    return rtn;
}

HomogeneousMatrix HomogeneousMatrix::operator/(const double rhs){
    HomogeneousMatrix rtn;
    int SIZE = 4;
    int i,j;
    for(i=0; i<SIZE; i++){
        for(j=0; j<SIZE; j++){
            rtn.matrix[i][j] = this->matrix[i][j] / rhs;
        }
    }
    return rtn;
}

HomogeneousMatrix& HomogeneousMatrix::operator=(const RotationMatrix& rhs){
    this->matrix[0][0] = rhs.matrix[0][0];
    this->matrix[0][1] = rhs.matrix[0][1];
    this->matrix[0][2] = rhs.matrix[0][2];
    this->matrix[1][0] = rhs.matrix[1][0];
    this->matrix[1][1] = rhs.matrix[1][1];
    this->matrix[1][2] = rhs.matrix[1][2];
    this->matrix[2][0] = rhs.matrix[2][0];
    this->matrix[2][1] = rhs.matrix[2][1];
    this->matrix[2][2] = rhs.matrix[2][2];
    return *this;
}

HomogeneousMatrix& HomogeneousMatrix::operator=(const Quaternion& quat){
    RotationMatrix rm = quat;
    *this = rm;
    return *this;
}

HomogeneousMatrix& HomogeneousMatrix::operator=(const MarkerData& rhs){
    this->matrix[0][0] = rhs.rm[0][0];
    this->matrix[0][1] = rhs.rm[0][1];
    this->matrix[0][2] = rhs.rm[0][2];
    this->matrix[1][0] = rhs.rm[1][0];
    this->matrix[1][1] = rhs.rm[1][1];
    this->matrix[1][2] = rhs.rm[1][2];
    this->matrix[2][0] = rhs.rm[2][0];
    this->matrix[2][1] = rhs.rm[2][1];
    this->matrix[2][2] = rhs.rm[2][2];
    this->matrix[0][3] = rhs.dist_x;
    this->matrix[1][3] = rhs.dist_y;
    this->matrix[2][3] = rhs.dist_z;
    return *this;
}

DroneData HomogeneousMatrix::toData(){
    DroneData rtn;
    /*
    rtn.rm[0][0] = this->matrix[0][0];
    rtn.rm[0][1] = this->matrix[0][1];
    rtn.rm[0][2] = this->matrix[0][2];
    rtn.rm[1][0] = this->matrix[1][0];
    rtn.rm[1][1] = this->matrix[1][1];
    rtn.rm[1][2] = this->matrix[1][2];
    rtn.rm[2][0] = this->matrix[2][0];
    rtn.rm[2][1] = this->matrix[2][1];
    rtn.rm[2][2] = this->matrix[2][2];
    */
    rtn.roll = this->roll;
    rtn.pitch = this->pitch;
    rtn.yaw = this->yaw;

    rtn.dist_x = this->matrix[0][3];
    rtn.dist_y = this->matrix[1][3];
    rtn.dist_z = this->matrix[2][3];
    return rtn;
}


void HomogeneousMatrix::hm2rpy(){
    roll = atan2(matrix[2][1], matrix[2][2]);//*180/3.1415926535897;
    pitch = atan2(-1*matrix[2][0], sqrt(matrix[1][0]*matrix[1][0]+matrix[0][0]*matrix[0][0]));//*180/3.1415926535897;
    yaw = atan2(matrix[1][0], matrix[0][0]);//*180/3.1415926535897;
}

/*
DroneData& DroneData::operator=(const RotationMatrix& rhs){
    DroneData rtn(*this);
    rtn.rm[0][0] = rhs.matrix[0][0];
    rtn.rm[0][1] = rhs.matrix[0][1];
    rtn.rm[0][2] = rhs.matrix[0][2];
    rtn.rm[1][0] = rhs.matrix[1][0];
    rtn.rm[1][1] = rhs.matrix[1][1];
    rtn.rm[1][2] = rhs.matrix[1][2];
    rtn.rm[2][0] = rhs.matrix[2][0];
    rtn.rm[2][1] = rhs.matrix[2][1];
    rtn.rm[2][2] = rhs.matrix[2][2];
    return rtn;
}

DroneData& DroneData::operator=(const HomogeneousMatrix& rhs){
    DroneData rtn(*this);
    rtn.rm[0][0] = rhs.matrix[0][0];
    rtn.rm[0][1] = rhs.matrix[0][1];
    rtn.rm[0][2] = rhs.matrix[0][2];
    rtn.rm[1][0] = rhs.matrix[1][0];
    rtn.rm[1][1] = rhs.matrix[1][1];
    rtn.rm[1][2] = rhs.matrix[1][2];
    rtn.rm[2][0] = rhs.matrix[2][0];
    rtn.rm[2][1] = rhs.matrix[2][1];
    rtn.rm[2][2] = rhs.matrix[2][2];
    rtn.dist_x = rhs.matrix[0][3];
    rtn.dist_y = rhs.matrix[1][3];
    rtn.dist_z = rhs.matrix[2][3];
    return rtn;
}
*/