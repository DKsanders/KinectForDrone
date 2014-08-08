/**
 * This file (data_processor.cpp) is the main file for the tracking system.
 *
 * It tracks markers on the drone using the Microsoft Kinect (c),
 * proccesses the data to estimate the attitude of the drone,
 * and sends the data to the drone via wifi. 
 *
 * Author: David Sanders <david.sanders@mail.utoronto.ca>
 */


// ROS libs
#include "drone/data_processor.h"
#include <sstream>
#include <pthread.h>
#include <ros/package.h>
using namespace std;

// Lock for shared data
pthread_mutex_t sharedDataLock;

int main (int argc, char **argv)
{   
    // Initialize
    ros::init (argc, argv, "data_processor");
    ros::NodeHandle nh; // Access point for communication with ROS system
    drone::DataProcessor dp (nh);

    cout << "Press Any Key to Exit" << endl;
    cin.get();
    return 0;
}

namespace drone {
    DataProcessor::DataProcessor(ros::NodeHandle & _n):n (_n){
        // Initialize
        pthread_mutex_init(&sharedDataLock, NULL);
        sharedData = new SharedData;
        sharedData -> id = 0;
        seq = 0;
        currentSharedDataID = 0;
        server = NULL;
        client = NULL;
        pthread_t clientThreadID, serverThreadID; // thread IDs
        
        // Get parameters
        string package_path = ros::package::getPath (ROS_PACKAGE_NAME);
        ros::NodeHandle n_param ("~");
        XmlRpc::XmlRpcValue param_center;

        string path;
        if (!n_param.getParam ("network_configuration", path)){
            network_config = package_path + "/config/network.config";
        }else{
            network_config = path;
        }

        // Subscribe to topics
        ar_pose_markers_sub = n.subscribe("ar_pose_markers", SUB_BUFFER_SIZE, &DataProcessor::arPoseMarkersCallback, this);

        // Run server and client
        params = readConfig(network_config.c_str());
        if(params == NULL){
            cout << "error reading config file" << endl;
        }
        if(!params->clientOff){
            pthread_create(&clientThreadID, NULL, runClient, this);
        } else {
            cout << "CLIENT: OFF" << endl;
        }
        if(!params->serverOff){
            pthread_create(&serverThreadID, NULL, runServer, this);
        } else {
            cout << "SERVER: OFF" << endl;
        }
    }

    DataProcessor::~DataProcessor(){
        delete sharedData;
        delete server;
        delete client;
        delete params;
    }

    void DataProcessor::arPoseMarkersCallback(const drone::ARMarkers::ConstPtr &msg)
    {
        // Handle updated data from drone
        pthread_mutex_lock(&sharedDataLock);
        if (currentSharedDataID != sharedData -> id){
            currentSharedDataID = sharedData -> id; // update id
            // process new data from drone
            cout << sharedData -> id << endl;
        }
        pthread_mutex_unlock(&sharedDataLock);

        // Handle msg
        if(msg->markers.empty()){
            // Not identifying any markers

        } else {
            // Process data
            DroneData* data = processDataToDrone(msg, seq+1);
            int buf_size;
            char* buf;
            serialize(data, buf, buf_size);
            // Send the message
            if (client->send(buf, buf_size)) {
                // Couldn't send
                ROS_INFO("failed to send to drone");
            }
            delete data;
            delete [] buf;
            seq += 1;    
        }
    }

    DroneData* DataProcessor::processDataToDrone(const drone::ARMarkers::ConstPtr &msg, const int seq){
        // Interpret marker data
        DroneData* data = new DroneData;
        data -> seq = seq;
        int markerNum = msg -> markers.size();

        // Process data of each marker
        int i;
        for (i = 0; i < markerNum; i++){
            Quaternion* quat = new Quaternion;
            int type = (int) (msg -> markers[i].id); // starts from 0

            data -> dist_x = msg -> markers[i].pose.pose.position.x;
            data -> dist_y = msg -> markers[i].pose.pose.position.y;
            data -> dist_z = msg -> markers[i].pose.pose.position.z;

            quat -> x = msg -> markers[i].pose.pose.orientation.x;
            quat -> y = msg -> markers[i].pose.pose.orientation.y;
            quat -> z = msg -> markers[i].pose.pose.orientation.z;
            quat -> w = msg -> markers[i].pose.pose.orientation.w;

            delete data->rm;
            data->rm = quat2rm(quat);
            printData(data);
            delete quat;
        }
        return data;
    }

    int DataProcessor::getCurrentID(){
        return currentSharedDataID;
    }

    SharedData* DataProcessor::getSharedData(){
        return sharedData;
    }

    ConfigParams* DataProcessor::getParams(){
        return params;
    }
}

void* runServer(void* dataProcessor){
    // Initialize server
    drone::DataProcessor* dp = (drone::DataProcessor*) dataProcessor;
    int status = 0;
    status = serverInit(dp->server, dp->getParams()->serverHost, dp->getParams()->serverPort, dp->getParams()->serverType);
    if (status != 0) {
        cout << "error initializing server" << endl;
        pthread_detach(pthread_self());
        pthread_exit(NULL);
    }

    // Process data from drone, store it in sharedData
    SharedData* sharedData = dp->getSharedData();
    dp->server->listen();
    while(1){
        status = dp->server->receive();
        if (status != 0){
            // Error receiving
            cout << "recv() failed, exiting processDataFromDrone()" << endl;
            break;
        }
        // Handle dp -> server-> buf
        cout << dp -> server->buf << endl;

        pthread_mutex_lock(&sharedDataLock);
        // Modify sharedData
        sharedData -> id = sharedData -> id + 1;

        pthread_mutex_unlock(&sharedDataLock);
    }
    pthread_detach(pthread_self());
    pthread_exit(NULL);
}

void* runClient(void* dataProcessor){
    // Initialize client
    drone::DataProcessor* dp = (drone::DataProcessor*) dataProcessor;
    int status = 0;
    status = clientInit(dp->client, dp->getParams()->clientHost, dp->getParams()->clientPort, dp->getParams()->clientType);
    if(status != 0) {
        cout << "error initializing client" << endl;
        pthread_detach(pthread_self());
        pthread_exit(NULL);
    }

    // Call callback functions
    ros::spin();
    
    pthread_detach(pthread_self());
    pthread_exit(NULL);
}