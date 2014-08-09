/**
 * This file (data_processor.h) provides the main functionality of
 * the tracking system.
 *
 * It tracks markers on the drone using the Microsoft Kinect (c),
 * proccesses the data to estimate the attitude of the drone,
 * and sends the data to the drone via wifi. 
 *
 * Author: David Sanders <david.sanders@mail.utoronto.ca>
 */

#ifndef DATA_PROCESSOR_H
#define DATA_PROCESSOR_H

// ROS libs
#include "ros/ros.h"
#include "drone/ARMarkers.h"
#include "drone/ARMarker.h"

// Libraries
#include <sstream>
#include <pthread.h>
#include "network/connection.h"
#include "data_handling.h"

// Constants
#define PUB_BUFFER_SIZE 1
#define SUB_BUFFER_SIZE 0

using namespace std;

// Structure for holding data needed by both server and client
typedef struct SharedData{
  int id; // use to check if new data or not

  // Flags
  int serverReady;
  int clientReady;
  int done;

  string etc; // data
} SharedData;

namespace drone
{
  class DataProcessor
  {
  public:
    DataProcessor (ros::NodeHandle & _n);
    ~DataProcessor ();

    // Communication interface
    Client* client; // to drone
    Server* server; // from drone
    
    // 
    int getCurrentID();
    SharedData* getSharedData();
    ConfigParams* getParams();

  private:
    // Variables
    ros::NodeHandle n;
    int seq; // sequence number for data sent
    int currentSharedDataID;
    ConfigParams* params;
    SharedData* sharedData;

    // Subscribing to topics
    void arPoseMarkersCallback(const drone::ARMarkers::ConstPtr &msg);
    ros::Subscriber ar_pose_markers_sub;

    // Functions
    DroneData processDataToDrone(const drone::ARMarkers::ConstPtr &msg, const int seq);

    // Parameters
    string network_config;

  };

}

// Functions for running servers and clients using pthread
void* runServer(void* dataProcessor);
void* runClient(void* dataProcessor);

#endif //END_IF_DATA_PROCESSOR_H