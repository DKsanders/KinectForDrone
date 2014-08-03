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
char* data2str(const DroneData* data, int & buf_size){
    // Initialize
    char* buf = (char*)malloc(sizeof(char)*MAX_BUF_SIZE); // buffer for storing data
    int offset = 0; // offset to keep track of where to write to

    // Write to stream
    offset += write2stream(buf+offset, data->seq);
    offset += write2stream(buf+offset, data->dist_x);
    offset += write2stream(buf+offset, data->dist_y);
    offset += write2stream(buf+offset, data->dist_z);
    offset += write2stream(buf+offset, data->rm->matrix[0][0]);
    offset += write2stream(buf+offset, data->rm->matrix[0][1]);
    offset += write2stream(buf+offset, data->rm->matrix[0][2]);
    offset += write2stream(buf+offset, data->rm->matrix[1][0]);
    offset += write2stream(buf+offset, data->rm->matrix[1][1]);
    offset += write2stream(buf+offset, data->rm->matrix[1][2]);
    offset += write2stream(buf+offset, data->rm->matrix[2][0]);
    offset += write2stream(buf+offset, data->rm->matrix[2][1]);
    offset += write2stream(buf+offset, data->rm->matrix[2][2]);
    offset += write2stream(buf+offset, data->comment.c_str());

    buf_size = offset;
    return buf;
}

// Converts a string of bytes into DroneData
DroneData* str2data(const char* msg){
    // Initialize
    char* buf = (char*)malloc(sizeof(char)*MAX_BUF_SIZE);
    memcpy(buf, msg, MAX_BUF_SIZE);
    int offset = 0;
    DroneData* data = new DroneData;

    // Read from stream
    offset += readFromStream(buf+offset, data->seq);
    offset += readFromStream(buf+offset, data->dist_x);
    offset += readFromStream(buf+offset, data->dist_y);
    offset += readFromStream(buf+offset, data->dist_z);
    offset += readFromStream(buf+offset, data->rm->matrix[0][0]);
    offset += readFromStream(buf+offset, data->rm->matrix[0][1]);
    offset += readFromStream(buf+offset, data->rm->matrix[0][2]);
    offset += readFromStream(buf+offset, data->rm->matrix[1][0]);
    offset += readFromStream(buf+offset, data->rm->matrix[1][1]);
    offset += readFromStream(buf+offset, data->rm->matrix[1][2]);
    offset += readFromStream(buf+offset, data->rm->matrix[2][0]);
    offset += readFromStream(buf+offset, data->rm->matrix[2][1]);
    offset += readFromStream(buf+offset, data->rm->matrix[2][2]);
    offset += readFromStream(buf+offset, data->comment);

    free(buf);
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