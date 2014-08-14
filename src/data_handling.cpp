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
    comment = "";
}

DroneData::DroneData(ByteStream& stream){
    deserialize(stream);
}

DroneData::~DroneData() {
    ;
}

int DroneData::getSeq() {
    return seq;
}

void DroneData::setSeq(int _seq) {
    seq = _seq;
}

ByteStream DroneData::serialize() {
    // Initialize
    ByteStream stream; // keeps track of where to write in buffer

    // Write to stream
    stream.write(seq);
    stream.write(dist_x);
    stream.write(dist_y);
    stream.write(dist_z);

    /*
    stream.write(rm[0][0]);
    stream.write(rm[0][1]);
    stream.write(rm[0][2]);
    stream.write(rm[1][0]);
    stream.write(rm[1][1]);
    stream.write(rm[1][2]);
    stream.write(rm[2][0]);
    stream.write(rm[2][1]);
    stream.write(rm[2][2]);
    */

    stream.write(roll);
    stream.write(pitch);
    stream.write(yaw);

    stream.write(comment.c_str());
    
    return stream;
}

void DroneData::deserialize(ByteStream& stream) {
    // Read from stream
    int status = 0;
    status += stream.read(seq);
    status += stream.read(dist_x);
    status += stream.read(dist_y);
    status += stream.read(dist_z);
    
    /*
    status += stream.read(rm[0][0]);
    status += stream.read(rm[0][1]);
    status += stream.read(rm[0][2]);
    status += stream.read(rm[1][0]);
    status += stream.read(rm[1][1]);
    status += stream.read(rm[1][2]);
    status += stream.read(rm[2][0]);
    status += stream.read(rm[2][1]);
    status += stream.read(rm[2][2]);
    */

    status += stream.read(roll);
    status += stream.read(pitch);
    status += stream.read(yaw);

    status += stream.read(comment);

    return;
}

void DroneData::print(){
    
    cout << "---------------------------------------------------------------------"<< endl;
    cout << "Sequence Number: " << endl;
    cout << "  " << seq << endl;
    cout << "Position: "  << endl;
    cout << "  x "  << dist_x << endl;
    cout << "  y "  << dist_y << endl;
    cout << "  z "  << dist_z << endl;
    /*
    cout << "Rotation Matrix: "  << endl;
    cout << "  " << rm[0][0]  << "  " << rm[0][1]  << "  " << rm[0][2] << endl;
    cout << "  " << rm[1][0]  << "  " << rm[1][1]  << "  " << rm[1][2] << endl;
    cout << "  " << rm[2][0]  << "  " << rm[2][1]  << "  " << rm[2][2] << endl;
    */
    cout << "Roll: "  << roll*180/3.1415926535897932 << endl;
    cout << "Pitch: "  << pitch*180/3.1415926535897932 << endl;
    cout << "yaw: "  << yaw*180/3.1415926535897932 << endl;

    cout << "Comment: " << endl;
    cout << "  " << comment << endl;
    
    return;
}

DroneData* deserialize(const char* msg){
}
