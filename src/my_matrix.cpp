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

Vector::Vector() {
    size = 3;
    vector = new double[size];
    int i;
    for(i=0; i<size; i++) {
        vector[i] = 0;
    }
}

Vector::Vector(int _size) {
    size = _size;
    vector = new double[size];
    int i;
    for(i=0; i<size; i++) {
        vector[i] = 0;
    }
}

Vector::Vector(const MarkerData& data) {
    size = 3;
    vector = new double[size];
    this->vector[0] = data.dist_x;
    this->vector[1] = data.dist_y;
    this->vector[2] = data.dist_z;
}

Vector::Vector(const Vector& other) {
    size = other.size;
    vector = new double[size];
    int i;
    for(i=0; i<size; i++) {
        vector[i] = other.vector[i];
    }
}

Vector::~Vector() {
    delete [] vector;
}

double Vector::getNorm() {
    int i;
    double sum = 0;
    for(i=0; i<size; i++) {
        sum += (vector[i]*vector[i]);
    }
    return sqrt(sum);
}

void Vector::normalize() {
    double norm = getNorm();
    int i;
    for(i=0; i<size; i++) {
        vector[i] = vector[i]/norm;
    }
}

void Vector::print() {
    int i;
    cout << "Vector:" << endl;
    for (i=0; i<size; i++) {
        cout << "  " << vector[i] << endl;
    }
}

Vector& Vector::operator=(const Vector& rhs) {
    delete [] this->vector;
    this->size = rhs.size;
    this->vector = new double[size];
    int i;
    for(i=0; i<size; i++) {
        this->vector[i] = rhs.vector[i];
    }
    return *this;
}

Vector Vector::operator*(const Vector& rhs) {
    Vector rtn(3);
    // Check that both are size 3
    if(this->size != 3 || rhs.size != 3) {
        return rtn;
    }
    rtn.vector[0] = this->vector[1]*rhs.vector[2] - this->vector[2]*rhs.vector[1];
    rtn.vector[1] = this->vector[2]*rhs.vector[0] - this->vector[0]*rhs.vector[2];
    rtn.vector[2] = this->vector[0]*rhs.vector[1] - this->vector[1]*rhs.vector[0];
    return rtn;
}

Vector Vector::operator+(const Vector& rhs) {
    Vector rtn;
    // Check that both are size 3
    if(this->size != rhs.size) {
        return rtn;
    }
    int i;
    for(i=0; i<size; i++) {
        rtn.vector[i] = this->vector[i] + rhs.vector[i];
    }
    return rtn;
}

Vector Vector::operator-(const Vector& rhs) {
    Vector rtn;
    // Check that both are same size
    if(this->size != rhs.size) {
        return rtn;
    }
    int i;
    for(i=0; i<size; i++) {
        rtn.vector[i] = this->vector[i] - rhs.vector[i];
    }
    return rtn;
}

Vector Vector::operator*(double rhs) {
    Vector rtn;
    int i;
    for(i=0; i<size; i++) {
        rtn.vector[i] = this->vector[i] * rhs;
    }
    return rtn;
}

Vector Vector::operator/(double rhs) {
    Vector rtn;
    int i;
    for(i=0; i<size; i++) {
        rtn.vector[i] = this->vector[i] / rhs;
    }
    return rtn;
}

Vector Vector::operator+(double rhs) {
    Vector rtn;
    int i;
    for(i=0; i<size; i++) {
        rtn.vector[i] = this->vector[i] + rhs;
    }
    return rtn;
}

Vector Vector::operator-(double rhs) {
    Vector rtn;
    int i;
    for(i=0; i<size; i++) {
        rtn.vector[i] = this->vector[i] - rhs;
    }
    return rtn;
}

void Vector::fromMarkerData(const MarkerData& rhs) {
    delete [] this->vector;
    this->size = 3;
    this->vector = new double[size];
    this->vector[0] = rhs.dist_x;
    this->vector[1] = rhs.dist_y;
    this->vector[2] = rhs.dist_z;
}


void Quaternion::print() {
    cout << "---------------------------------------------------------------------"<< endl;
    cout << "Quaternion: " << endl;
    cout << "  w: " << w << endl;
    cout << "  x: " << x << endl;
    cout << "  y: " << y << endl;
    cout << "  z: " << z << endl;
}

RotationMatrix::RotationMatrix() {
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

RotationMatrix::RotationMatrix(const Quaternion& quat) {
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

RotationMatrix::RotationMatrix(const MarkerData& rhs) {
    *this = rhs;
}

RotationMatrix::~RotationMatrix() {
    ;
}

double RotationMatrix::getDeterminant() {
    return matrix[0][0]* (matrix[1][1]*matrix[2][2] - matrix[2][1]*matrix[1][2])
           - matrix[0][1]* (matrix[1][0]*matrix[2][2] - matrix[2][0]*matrix[1][2])
           + matrix[0][2]* (matrix[1][0]*matrix[2][1] - matrix[2][0]*matrix[1][1]);
}

void RotationMatrix::transpose() {
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

RotationMatrix RotationMatrix::getTranspose() {
    RotationMatrix temp = *this;
    temp.transpose();
    return temp;
}

void RotationMatrix::invert() {
    this->transpose();
}

RotationMatrix RotationMatrix::getInverse() {
    RotationMatrix temp = *this;
    temp.invert();
    return temp;
}

void RotationMatrix::print() {
    cout << "---------------------------------------------------------------------"<< endl;
    cout << "Rotation Matrix: "  << endl;
    cout << "  " << this->matrix[0][0]  << "  " << this->matrix[0][1]  << "  " << this->matrix[0][2] << endl;
    cout << "  " << this->matrix[1][0]  << "  " << this->matrix[1][1]  << "  " << this->matrix[1][2] << endl;
    cout << "  " << this->matrix[2][0]  << "  " << this->matrix[2][1]  << "  " << this->matrix[2][2] << endl;
}

RotationMatrix RotationMatrix::operator+(const RotationMatrix& rhs) {
    RotationMatrix rtn;
    int SIZE = 3;
    int i,j;
    for(i=0; i<SIZE; i++) {
        for(j=0; j<SIZE; j++) {
            rtn.matrix[i][j] = this->matrix[i][j] + rhs.matrix[i][j];
        }
    }
    return rtn;
}

RotationMatrix RotationMatrix::operator-(const RotationMatrix& rhs) {
    RotationMatrix rtn;
    int SIZE = 3;
    int i,j;
    for(i=0; i<SIZE; i++) {
        for(j=0; j<SIZE; j++) {
            rtn.matrix[i][j] = this->matrix[i][j] - rhs.matrix[i][j];
        }
    }
    return rtn;
}

RotationMatrix RotationMatrix::operator*(const RotationMatrix& rhs) {
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

RotationMatrix RotationMatrix::operator+(const double rhs) {
    RotationMatrix rtn;
    int SIZE = 3;
    int i,j;
    for(i=0; i<SIZE; i++) {
        for(j=0; j<SIZE; j++) {
            rtn.matrix[i][j] = this->matrix[i][j] + rhs;
        }
    }
    return rtn;
}

RotationMatrix RotationMatrix::operator-(const double rhs) {
    RotationMatrix rtn;
    int SIZE = 3;
    int i,j;
    for(i=0; i<SIZE; i++) {
        for(j=0; j<SIZE; j++) {
            rtn.matrix[i][j] = this->matrix[i][j] - rhs;
        }
    }
    return rtn;
}

RotationMatrix RotationMatrix::operator*(const double rhs) {
    RotationMatrix rtn;
    int SIZE = 3;
    int i,j;
    for(i=0; i<SIZE; i++) {
        for(j=0; j<SIZE; j++) {
            rtn.matrix[i][j] = this->matrix[i][j] * rhs;
        }
    }
    return rtn;
}

RotationMatrix RotationMatrix::operator/(const double rhs) {
    RotationMatrix rtn;
    int SIZE = 3;
    int i,j;
    for(i=0; i<SIZE; i++) {
        for(j=0; j<SIZE; j++) {
            rtn.matrix[i][j] = this->matrix[i][j] / rhs;
        }
    }
    return rtn;
}

RotationMatrix& RotationMatrix::operator=(const Quaternion& quat) {
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

RotationMatrix& RotationMatrix::operator=(const MarkerData& rhs) {
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

HomogeneousMatrix::HomogeneousMatrix() {
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
    roll = 0;
    pitch = 0;
    yaw = 0;
}

HomogeneousMatrix::HomogeneousMatrix(const Quaternion& quat) {
    *this = quat;
}

HomogeneousMatrix::HomogeneousMatrix(const RotationMatrix& rhs) {
    *this = rhs;
}

HomogeneousMatrix::HomogeneousMatrix(const MarkerData& rhs) {
    this->fromMarkerData(rhs);
}

HomogeneousMatrix::~HomogeneousMatrix() {
    ;
}

void HomogeneousMatrix::setRoll(double _roll) {
    roll = _roll;
}

void HomogeneousMatrix::setPitch(double _pitch) {
    pitch = _pitch;
}

void HomogeneousMatrix::setYaw(double _yaw) {
    yaw = _yaw;
}

double HomogeneousMatrix::getDeterminant() {
    return matrix[0][0]* (matrix[1][1]*matrix[2][2] - matrix[2][1]*matrix[1][2])
           - matrix[0][1]* (matrix[1][0]*matrix[2][2] - matrix[2][0]*matrix[1][2])
           + matrix[0][2]* (matrix[1][0]*matrix[2][1] - matrix[2][0]*matrix[1][1]);
}
void HomogeneousMatrix::transpose() {
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

HomogeneousMatrix HomogeneousMatrix::getTranspose() {
    HomogeneousMatrix temp = *this;
    temp.transpose();
    return temp;
}

void HomogeneousMatrix::print() {
    cout << "---------------------------------------------------------------------"<< endl;
    cout << "Homogeneous Matrix: "  << endl;
    cout << "  " << this->matrix[0][0]  << "  " << this->matrix[0][1]  << "  " << this->matrix[0][2] << "  " << this->matrix[0][3] << endl;
    cout << "  " << this->matrix[1][0]  << "  " << this->matrix[1][1]  << "  " << this->matrix[1][2] << "  " << this->matrix[1][3] << endl;
    cout << "  " << this->matrix[2][0]  << "  " << this->matrix[2][1]  << "  " << this->matrix[2][2] << "  " << this->matrix[2][3] << endl;
    cout << "  " << this->matrix[3][0]  << "  " << this->matrix[3][1]  << "  " << this->matrix[3][2] << "  " << this->matrix[3][3] << endl;
    cout << "Roll: "  << roll*180/3.14159265358979323846 << endl;
    cout << "Pitch: "  << pitch*180/3.14159265358979323846 << endl;
    cout << "Yaw: "  << yaw*180/3.14159265358979323846 << endl;
}

void HomogeneousMatrix::calc_rpy() {
    if(matrix[2][0] == 1 || matrix[2][0] == -1 ) {
        // Singularity in the homogeneous matrix - unable to calculate roll/pitch/yaw
        return;
    }
    roll = atan2(matrix[2][1], matrix[2][2]);//*180/3.1415926535897;
    pitch = atan2(-1*matrix[2][0], sqrt(matrix[1][0]*matrix[1][0]+matrix[0][0]*matrix[0][0]));//*180/3.1415926535897;
    yaw = atan2(matrix[1][0], matrix[0][0]);//*180/3.1415926535897;
}

void HomogeneousMatrix::calibrate(CalibrationData calib) {
    roll = roll - calib.getRoll();
    pitch = pitch - calib.getPitch();
    yaw = yaw - calib.getYaw();
    matrix[0][3] = matrix[0][3] - calib.getX();
    matrix[1][3] = matrix[1][3] - calib.getY();
    matrix[2][3] = matrix[2][3] - calib.getZ();
}

DroneData HomogeneousMatrix::toDroneData() {
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

void HomogeneousMatrix::fromMarkerData(const MarkerData& rhs) {
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
    this->matrix[3][0] = 0;
    this->matrix[3][1] = 0;
    this->matrix[3][2] = 0;
    this->matrix[3][3] = 1;
}

HomogeneousMatrix HomogeneousMatrix::operator+(const HomogeneousMatrix& rhs) {
    HomogeneousMatrix rtn;
    int SIZE = 4;
    int i,j;
    for(i=0; i<SIZE; i++) {
        for(j=0; j<SIZE; j++) {
            rtn.matrix[i][j] = this->matrix[i][j] + rhs.matrix[i][j];
        }
    }
    rtn.roll = roll + rhs.roll;
    rtn.pitch = pitch + rhs.pitch;
    rtn.yaw = yaw + rhs.yaw;
    return rtn;
}

HomogeneousMatrix HomogeneousMatrix::operator-(const HomogeneousMatrix& rhs) {
    HomogeneousMatrix rtn;
    int SIZE = 4;
    int i,j;
    for(i=0; i<SIZE; i++) {
        for(j=0; j<SIZE; j++) {
            rtn.matrix[i][j] = this->matrix[i][j] - rhs.matrix[i][j];
        }
    }
    rtn.roll = roll - rhs.roll;
    rtn.pitch = pitch - rhs.pitch;
    rtn.yaw = yaw - rhs.yaw;
    return rtn;
}

HomogeneousMatrix HomogeneousMatrix::operator*(const HomogeneousMatrix& rhs) {
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
    rtn.calc_rpy();
    return rtn;
}

HomogeneousMatrix HomogeneousMatrix::operator+(const double rhs) {
    HomogeneousMatrix rtn;
    int SIZE = 4;
    int i,j;
    for(i=0; i<SIZE; i++) {
        for(j=0; j<SIZE; j++) {
            rtn.matrix[i][j] = this->matrix[i][j] + rhs;
        }
    }
    rtn.roll = roll + rhs;
    rtn.pitch = pitch + rhs;
    rtn.yaw = yaw + rhs;
    return rtn;
}

HomogeneousMatrix HomogeneousMatrix::operator-(const double rhs) {
    HomogeneousMatrix rtn;
    int SIZE = 4;
    int i,j;
    for(i=0; i<SIZE; i++) {
        for(j=0; j<SIZE; j++) {
            rtn.matrix[i][j] = this->matrix[i][j] - rhs;
        }
    }
    rtn.roll = roll - rhs;
    rtn.pitch = pitch - rhs;
    rtn.yaw = yaw - rhs;
    return rtn;
}

HomogeneousMatrix HomogeneousMatrix::operator*(const double rhs) {
    HomogeneousMatrix rtn;
    int SIZE = 4;
    int i,j;
    for(i=0; i<SIZE; i++) {
        for(j=0; j<SIZE; j++) {
            rtn.matrix[i][j] = this->matrix[i][j] * rhs;
        }
    }
    rtn.roll = roll * rhs;
    rtn.pitch = pitch * rhs;
    rtn.yaw = yaw * rhs;
    return rtn;
}

HomogeneousMatrix HomogeneousMatrix::operator/(const double rhs) {
    HomogeneousMatrix rtn;
    int SIZE = 4;
    int i,j;
    for(i=0; i<SIZE; i++) {
        for(j=0; j<SIZE; j++) {
            rtn.matrix[i][j] = this->matrix[i][j] / rhs;
        }
    }
    rtn.roll = roll / rhs;
    rtn.pitch = pitch / rhs;
    rtn.yaw = yaw / rhs;
    return rtn;
}

HomogeneousMatrix& HomogeneousMatrix::operator=(const RotationMatrix& rhs) {
    this->matrix[0][0] = rhs.matrix[0][0];
    this->matrix[0][1] = rhs.matrix[0][1];
    this->matrix[0][2] = rhs.matrix[0][2];
    this->matrix[1][0] = rhs.matrix[1][0];
    this->matrix[1][1] = rhs.matrix[1][1];
    this->matrix[1][2] = rhs.matrix[1][2];
    this->matrix[2][0] = rhs.matrix[2][0];
    this->matrix[2][1] = rhs.matrix[2][1];
    this->matrix[2][2] = rhs.matrix[2][2];
    matrix[3][0] = 0;
    matrix[3][1] = 0;
    matrix[3][2] = 0;
    matrix[3][3] = 1;
    this->calc_rpy();
    return *this;
}

HomogeneousMatrix& HomogeneousMatrix::operator=(const Quaternion& quat ) {
    RotationMatrix rm = quat;
    *this = rm;
    matrix[3][0] = 0;
    matrix[3][1] = 0;
    matrix[3][2] = 0;
    matrix[3][3] = 1;
    return *this;
}
