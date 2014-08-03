/**
 * This is a template for a header of a general node
 */

#ifndef DATA_PROCESSOR_H
#define DATA_PROCESSOR_H

// ROS libs
#include "ros/ros.h"
#include "drone/ARMarkers.h"
#include "drone/ARMarker.h"
#include "visualization_msgs/Marker.h"
#include "network/connection.h"
#include "data_handling.h"

// Libraries
#include <sstream>
#include <pthread.h>

// Constants
#define PUB_BUFFER_SIZE 1
#define SUB_BUFFER_SIZE 0
#define LOOP_RATE 10 // in Hz

using namespace std;

namespace drone
{
  class DataProcessor
  {
  public:
    DataProcessor (ros::NodeHandle & _n);
    ~DataProcessor ();
    /************ For interacting outside of calss ******/
    // Communication interface
    Client* client; // to drone
    Server* server; // from drone
    // Functions
    int getCurrentID();
    SharedData* getSharedData();
    ConfigParams* getParams();
  private:
    /************ For interacting within of calss ******/
    // Variables
    ros::NodeHandle n;
    int seq; // sequence number for data sent
    int currentSharedDataID;
    ConfigParams* params;
    SharedData* sharedData;

    // Publishing to topics
    //ros::Publisher TOPIC_pub;

    // Subscribing to topics
    void arPoseMarkersCallback(const drone::ARMarkers::ConstPtr &msg);
    ros::Subscriber ar_pose_markers_sub;

    // Requesting services

    // Responding to services

    // Functions
    DroneData* processDataToDrone(const drone::ARMarkers::ConstPtr &msg, const int seq);
    // Parameters

  };

}

void* processDataFromDrone(void* dataProcessor);
void* spin(void* obj);

#endif //END_IF_DATA_PROCESSOR_H