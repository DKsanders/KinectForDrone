#include "drone/data_handling.h"
#include "network/network.h"
#include <iostream>
#include <sstream>
#include <cstdlib>
#include <cstring>
#include <stdint.h>
#include <endian.h>

using namespace std;

// Converts a Quaternion to a Rotatino Matrix
RotationMatrix* quat2rm(Quaternion* quat){
    // Create Rotation Matrix
    RotationMatrix* rm = new RotationMatrix;

    // Convert
    double a = quat->w;
    double b = quat->x;
    double c = quat->y;
    double d = quat->z;

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

// Converts DroneData into a string of bytes; return string must be freed
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

    buf_size = current-buf;
    return;
}

// Converts a string of bytes into DroneData
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

// Prints the type DroneData to the screen
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