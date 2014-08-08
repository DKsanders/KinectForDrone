/**
 * This file (data_handling.cpp) implements functions for processing 
 * and communicating drone data, declared in data_handling.h
 *
 * Author: David Sanders <david.sanders@mail.utoronto.ca>
 */

#include "drone/data_handling.h"
#include "network/network.h"
#include <iostream>
#include <math.h>

using namespace std;

RotationMatrix::RotationMatrix(){
    ;
}

RotationMatrix::~RotationMatrix(){
    ;
}

double RotationMatrix::getDeterminant(){
    return matrix[0][0]* (matrix[1][1]*matrix[2][2] - matrix[2][1]*matrix[1][2])
           - matrix[0][1]* (matrix[1][0]*matrix[2][2] - matrix[2][0]*matrix[1][2])
           + matrix[0][2]* (matrix[1][0]*matrix[2][1] - matrix[2][0]*matrix[1][1]);
}

RotationMatrix RotationMatrix::getInverse(){
    ;
}

RotationMatrix RotationMatrix::operator*(const RotationMatrix& rhs){
    ;
}

HomogeneousMatrix::HomogeneousMatrix(){
    ;
}

HomogeneousMatrix::~HomogeneousMatrix(){
    ;
}

double HomogeneousMatrix::getDeterminant(){
    ;
}

HomogeneousMatrix HomogeneousMatrix::getInverse(){
    ;
}

HomogeneousMatrix HomogeneousMatrix::operator*(const HomogeneousMatrix& rhs){
    ;
}

HomogeneousMatrix HomogeneousMatrix::operator=(const RotationMatrix& rhs){
    ;
}

void Quaternion::print(){
    cout << "Quaternion: " << endl;
    cout << "w: " << w << endl;
    cout << "x: " << x << endl;
    cout << "y: " << y << endl;
    cout << "z: " << z << endl;
}

RotationMatrix* quat2rm(Quaternion* quat){
    // Create Rotation Matrix
    RotationMatrix* rm = new RotationMatrix;

    // Convert
    double z = sqrt(quat->w * quat->w + quat->x * quat->x + quat->y * quat->y + quat->z * quat->z);
    double a = quat->w/z;
    double b = quat->x/z;
    double c = quat->y/z;
    double d = quat->z/z;

    rm->matrix[0][0] = a*a + b*b - c*c - d*d;
    rm->matrix[0][1] = 2*b*c - 2*a*d;
    rm->matrix[0][2] = 2*b*d + 2*a*c;
    rm->matrix[1][0] = 2*b*c + 2*a*d;
    rm->matrix[1][1] = a*a - b*b + c*c - d*d;
    rm->matrix[1][2] = 2*c*d - 2*a*b;
    rm->matrix[2][0] = 2*b*d - 2*a*c;
    rm->matrix[2][1] = 2*c*d + 2*a*b;
    rm->matrix[2][2] = a*a - b*b - c*c + d*d;
    
    return rm;
}

void serialize(const DroneData* data, char*& buf, int & buf_size){
    // Initialize
    buf = new char[MAX_BUF_SIZE]; // buffer for storing data
    char* current = buf; // keeps track of where to write in buffer

    // Write to stream
    current += write2stream(current, data->seq);
    current += write2stream(current, data->dist_x);
    current += write2stream(current, data->dist_y);
    current += write2stream(current, data->dist_z);
    current += write2stream(current, data->rm->matrix[0][0]);
    current += write2stream(current, data->rm->matrix[0][1]);
    current += write2stream(current, data->rm->matrix[0][2]);
    current += write2stream(current, data->rm->matrix[1][0]);
    current += write2stream(current, data->rm->matrix[1][1]);
    current += write2stream(current, data->rm->matrix[1][2]);
    current += write2stream(current, data->rm->matrix[2][0]);
    current += write2stream(current, data->rm->matrix[2][1]);
    current += write2stream(current, data->rm->matrix[2][2]);
    current += write2stream(current, data->comment.c_str());

    // Calculate buffer size
    buf_size = (int) (current-buf);
    return;
}

DroneData* deserialize(const char* msg){
    // Initialize
    char* current = const_cast<char*>(msg);
    DroneData* data = new DroneData;

    // Read from stream
    current += readFromStream(current, data->seq);
    current += readFromStream(current, data->dist_x);
    current += readFromStream(current, data->dist_y);
    current += readFromStream(current, data->dist_z);
    current += readFromStream(current, data->rm->matrix[0][0]);
    current += readFromStream(current, data->rm->matrix[0][1]);
    current += readFromStream(current, data->rm->matrix[0][2]);
    current += readFromStream(current, data->rm->matrix[1][0]);
    current += readFromStream(current, data->rm->matrix[1][1]);
    current += readFromStream(current, data->rm->matrix[1][2]);
    current += readFromStream(current, data->rm->matrix[2][0]);
    current += readFromStream(current, data->rm->matrix[2][1]);
    current += readFromStream(current, data->rm->matrix[2][2]);
    current += readFromStream(current, data->comment);

    return data;
}

void printData(const DroneData* data){
    cout << "---------------------------------------------------------------------"<< endl;
    cout << "Sequence Number: " << endl;
    cout << "  " << data -> seq << endl;
    cout << "Position: "  << endl;
    cout << "  x "  << data -> dist_x << endl;
    cout << "  y "  << data -> dist_y << endl;
    cout << "  z "  << data -> dist_z << endl;
    cout << "Rotation Matrix: "  << endl;
    cout << "  " << data->rm->matrix[0][0]  << "  " << data->rm->matrix[0][1]  << "  " << data->rm->matrix[0][2] << endl;
    cout << "  " << data->rm->matrix[1][0]  << "  " << data->rm->matrix[1][1]  << "  " << data->rm->matrix[1][2] << endl;
    cout << "  " << data->rm->matrix[2][0]  << "  " << data->rm->matrix[2][1]  << "  " << data->rm->matrix[2][2] << endl;
    cout << "Comment: " << endl;
    cout << "  " << data-> comment << endl;
    return;
}