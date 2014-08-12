/**
 * This file (marker_tracker.h) provides the main functionality of
 * the marker tracking system.
 *
 * It tracks markers on the drone using the Microsoft Kinect (c),
 * proccesses the data to estimate the attitude of the drone,
 * and sends the data to the drone via wifi. 
 *
 * Author: David Sanders <david.sanders@mail.utoronto.ca>
 */

#ifndef MARKER_TRACKER_H
#define MARKER_TRACKER_H

// ROS libs
#include "ros/ros.h"
#include "drone/ARMarkers.h"
#include "drone/ARMarker.h"

// Libraries
#include <sstream>
#include <pthread.h>
#include "network/connection.h"
#include "data_handling.h"
#include "markers.h"

// Constants
#define PUB_BUFFER_SIZE 1
#define SUB_BUFFER_SIZE 0

using namespace std;

// Structure for holding data needed by both server and client
struct SharedData{
  SharedData();

  int id; // use to check if new data or not

  // Flags
  int serverReady;
  int clientReady;
  int done;

  string etc; // data
};

namespace drone
{
  class DataProcessor
  {
  public:
    DataProcessor (ros::NodeHandle & _n);
    ~DataProcessor ();
    
    // Accessors
    int getSeqFromDrone();
    Client* getClient();
    Server* getServer();
    ConfigParams* getClientParams();
    ConfigParams* getServerParams();
    SharedData* getSharedData();
    MarkerDataSet* getMarkerDataSet();

    // Mutators
    void setClient(Client* _client);
    void setServer(Server* _server);

  private:
    // Variables
    ros::NodeHandle n;
    int seqToDrone; // sequence number for data sent
    int seqFromDrone; // sequence number for data recieved
    Client* client; // to drone
    Server* server; // from drone
    ConfigParams* clientParams;
    ConfigParams* serverParams;
    MarkerDataSet* markers;
    SharedData* sharedData;

    // Subscribe to topics
    void arPoseMarkersCallback(const drone::ARMarkers::ConstPtr &msg);
    ros::Subscriber ar_pose_markers_sub;

    // Functions
    /**
     * Processes the raw data of markers into DroneData
     * Arguments:
     *  msg(INPUT) - DroneData that needs to be serialized
     *  seq(INPUT) - buffer for holding byte stream
     *  DroneData(OUTPUT) - number of bytes in byte stream
     * Return:
     *  0 if successful
     */
    int processDataToDrone(const drone::ARMarkers::ConstPtr &msg, const int seq, DroneData& data);

    // Parameters
    string client_config;
    string server_config;
    string marker_data;

  };

}

// Functions for running servers and clients using pthread
void* runServer(void* dataProcessor);
void* runClient(void* dataProcessor);

#endif //END_IF_MARKER_TRACKER_H