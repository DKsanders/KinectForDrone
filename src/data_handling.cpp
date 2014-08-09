/**
 * This file (data_handling.cpp) implements functions for processing 
 * and communicating drone data, declared in data_handling.h
 *
 * Author: David Sanders <david.sanders@mail.utoronto.ca>
 */

#include "drone/data_handling.h"
#include "network/network.h"
#include <iostream>

using namespace std;

DroneData::DroneData(){
    ;
}

DroneData::~DroneData(){
    ;
}

void DroneData::print(){
    cout << "---------------------------------------------------------------------"<< endl;
    cout << "Sequence Number: " << endl;
    cout << "  " << seq << endl;
    cout << "Position: "  << endl;
    cout << "  x "  << dist_x << endl;
    cout << "  y "  << dist_y << endl;
    cout << "  z "  << dist_z << endl;
    cout << "Rotation Matrix: "  << endl;
    cout << "  " << rm[0][0]  << "  " << rm[0][1]  << "  " << rm[0][2] << endl;
    cout << "  " << rm[1][0]  << "  " << rm[1][1]  << "  " << rm[1][2] << endl;
    cout << "  " << rm[2][0]  << "  " << rm[2][1]  << "  " << rm[2][2] << endl;
    cout << "Comment: " << endl;
    cout << "  " << comment << endl;
    return;
}

void serialize(const DroneData* data, char*& buf, int & buf_size){
    // Initialize
    char* current = buf; // keeps track of where to write in buffer

    // Write to stream
    current += write2stream(current, data->seq);
    current += write2stream(current, data->dist_x);
    current += write2stream(current, data->dist_y);
    current += write2stream(current, data->dist_z);
    current += write2stream(current, data->rm[0][0]);
    current += write2stream(current, data->rm[0][1]);
    current += write2stream(current, data->rm[0][2]);
    current += write2stream(current, data->rm[1][0]);
    current += write2stream(current, data->rm[1][1]);
    current += write2stream(current, data->rm[1][2]);
    current += write2stream(current, data->rm[2][0]);
    current += write2stream(current, data->rm[2][1]);
    current += write2stream(current, data->rm[2][2]);
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
    current += readFromStream(current, data->rm[0][0]);
    current += readFromStream(current, data->rm[0][1]);
    current += readFromStream(current, data->rm[0][2]);
    current += readFromStream(current, data->rm[1][0]);
    current += readFromStream(current, data->rm[1][1]);
    current += readFromStream(current, data->rm[1][2]);
    current += readFromStream(current, data->rm[2][0]);
    current += readFromStream(current, data->rm[2][1]);
    current += readFromStream(current, data->rm[2][2]);
    current += readFromStream(current, data->comment);

    return data;
}
