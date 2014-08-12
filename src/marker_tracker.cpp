/**
 * This file (marker_tracker.cpp) is the main file for the marker tracking system.
 *
 * It tracks markers on the drone using the Microsoft Kinect (c),
 * proccesses the data to estimate the attitude of the drone,
 * and sends the data to the drone via wifi. 
 *
 * Author: David Sanders <david.sanders@mail.utoronto.ca>
 */


// Libraries
#include <sstream>
#include <pthread.h>

#include <ros/package.h>
#include "drone/marker_tracker.h"

#include "drone/my_matrix.h" // Should be replaced with proper matrix library

using namespace std;

// Locks
pthread_mutex_t sharedDataLock; // for accessing shared data
pthread_mutex_t printLock; // for printing to screen

int main (int argc, char **argv)
{   
    // Initialize
    ros::init (argc, argv, "data_processor");
    ros::NodeHandle nh; // Access point for communication with ROS system
    drone::DataProcessor dp (nh);

    pthread_mutex_lock(&printLock);
    cout << "Press Any Key to Exit" << endl;
    pthread_mutex_unlock(&printLock);
    cin.get();
    return 0;
}


SharedData::SharedData(){
    id = 0;
    serverReady = 0;
    clientReady = 0;
    done = 0;
}

namespace drone {
    DataProcessor::DataProcessor(ros::NodeHandle & _n):n (_n){
        // Initialize
        int status = 0;
        pthread_mutex_init(&sharedDataLock, NULL);
        pthread_mutex_init(&printLock, NULL);
        sharedData = new SharedData;
        seq = 0;
        currentSharedDataID = 0;
        pthread_t clientThreadID, serverThreadID;
        
        // Get parameters
        string package_path = ros::package::getPath (ROS_PACKAGE_NAME);
        ros::NodeHandle n_param ("~");
        XmlRpc::XmlRpcValue param_center;

        string path;
        // Client parameters
        if (!n_param.getParam ("client_configuration", path)){
            client_config = package_path + "/config/client.config";
        }else{
            client_config = path;
        }
        // Server parameters
        if (!n_param.getParam ("server_configuration", path)){
            server_config = package_path + "/config/server.config";
        }else{
            server_config = path;
        }
        // Marker data
        if (!n_param.getParam ("marker_data", path)){
            marker_data = package_path + "/data/marker_data";
        }else{
            marker_data = path;
        }

        // Subscribe to topics
        ar_pose_markers_sub = n.subscribe("ar_pose_markers", SUB_BUFFER_SIZE, &DataProcessor::arPoseMarkersCallback, this);

        // Read config file for network
        serverParams = readConfig(server_config.c_str());
        clientParams = readConfig(client_config.c_str());
        MarkerFileParser markerParser;
        status = markerParser.readMarkers(marker_data.c_str());
        markers = markerParser.getMarkers();
        if(serverParams == NULL || clientParams == NULL || status != 0){
            //Error reading config file
            cout << "Error reading config file" << endl;
        }

        // Initialize client
        pthread_create(&clientThreadID, NULL, runClient, this);

        // Wait until client has been initialized
        int clientReady = 0;
        while(!clientReady){
            pthread_mutex_lock(&sharedDataLock);
            clientReady = sharedData -> clientReady;
            pthread_mutex_unlock(&sharedDataLock);
            sleep(1);
        }

        // Initialize server
        pthread_create(&serverThreadID, NULL, runServer, this);
    }

    DataProcessor::~DataProcessor(){
        delete sharedData;
        if(server != NULL) delete server;
        if(client != NULL) delete client;
        if(clientParams != NULL) delete clientParams;
        if(serverParams != NULL) delete serverParams;
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
            return;
        } else {
            // Process data
            DroneData data = processDataToDrone(msg, seq);
            char* buf = new char[MAX_BUF_SIZE]; // buffer for storing data
            int buf_size;
            serialize(&data, buf, buf_size);
            // Send the message
            if (client->send(buf, buf_size)) {
                // Couldn't send
                ROS_INFO("failed to send to drone");
            }
            delete [] buf;
            seq += 1;    
        }
    }

    DroneData DataProcessor::processDataToDrone(const drone::ARMarkers::ConstPtr &msg, const int seq){
        // Interpret marker data
        DroneData data;
        int markerNum = msg -> markers.size();
        HomogeneousMatrix final;
        final.matrix[3][3] = 0;
        HomogeneousMatrix h02offset; // Auxilary matrix, h02 when roll/pitch/yaw = 0
        h02offset.matrix[0][2] = 1;
        h02offset.matrix[1][0] = 1;
        h02offset.matrix[2][1] = 1;

        // Process data of each marker
        int i;
        for (i = 0; i < markerNum; i++){
            //int markerType = (int) (msg -> markers[i].id); // starts from 0

            // Obtain Quaternion info
            Quaternion quat;
            quat.x = msg -> markers[i].pose.pose.orientation.x;
            quat.y = msg -> markers[i].pose.pose.orientation.y;
            quat.z = msg -> markers[i].pose.pose.orientation.z;
            quat.w = msg -> markers[i].pose.pose.orientation.w;

            // Create Homogeneous Matrices
            HomogeneousMatrix h10 = quat; // kinect -> marker
            h10.matrix[0][3] = msg -> markers[i].pose.pose.position.x;
            h10.matrix[1][3] = msg -> markers[i].pose.pose.position.y;
            h10.matrix[2][3] = msg -> markers[i].pose.pose.position.z;

            // Get marker data and transform it to a homogeneous matrix
            MarkerData* markerData = markers->getMarker(msg -> markers[i].id); // marker -> drone
            HomogeneousMatrix h21 = *markerData;
            /*
            h21.matrix[0][0] = 0;
            h21.matrix[0][1] = 1;
            h21.matrix[0][2] = 0;
            h21.matrix[0][3] = -0.04;
            h21.matrix[1][0] = 0;
            h21.matrix[1][1] = 0;
            h21.matrix[1][2] = -1;
            h21.matrix[1][3] = 0;
            h21.matrix[2][0] = -1;
            h21.matrix[2][1] = 0;
            h21.matrix[2][2] = 0;
            h21.matrix[2][3] = -0.23;
            */
            //double h21[4][4] = { { 0, 0, -1, -4 }, { 0, -1, 0, 0}, { -1, 0, 0, -23 }, { 0, 0, 0, 1 }};
            //HomogeneousMatrix h21 = (*markerData);
            HomogeneousMatrix h20 = h10*h21; // kinect -> drone
            final = final + h20;
        }
        final = final/markerNum; // get average of markers
        HomogeneousMatrix rpy = h02offset*final;
        if(final.matrix[2][0] == 1 || final.matrix[2][0] == -1 ){
            // Singularity in the homogeneous matrix - unable to calculate roll/pitch/yaw
            //return NULL;
        } else {
            rpy.hm2rpy();
        }
        data = rpy.toData();
        data.seq = seq;
        data.print();
        return data;
    }

    int DataProcessor::getCurrentID(){
        return currentSharedDataID;
    }

    SharedData* DataProcessor::getSharedData(){
        return sharedData;
    }

    ConfigParams* DataProcessor::getClientParams(){
        return clientParams;
    }

    ConfigParams* DataProcessor::getServerParams(){
        return serverParams;
    }
}

void* runServer(void* dataProcessor){
    // Initialize server
    drone::DataProcessor* dp = (drone::DataProcessor*) dataProcessor;
    int status = 0;
    status = serverInit(dp->server, dp->getServerParams());
    if (status != 0) {
        cout << "error initializing server" << endl;
        pthread_detach(pthread_self());
        pthread_exit(NULL);
    }

    // Raise flag for initialized server
    SharedData* sharedData = dp->getSharedData();
    pthread_mutex_lock(&sharedDataLock);
    sharedData->serverReady = 1;
    pthread_mutex_unlock(&sharedDataLock);

    if (dp->server == NULL){
        // Server off
        pthread_detach(pthread_self());
        pthread_exit(NULL);
    }

    // Accept loop
    while(1){
        status = dp->server->accept();
        if(status != 0){
            pthread_detach(pthread_self());
            pthread_exit(NULL);
        }
        // Process data from drone, store it in sharedData
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
    }
    pthread_detach(pthread_self());
    pthread_exit(NULL);
}

void* runClient(void* dataProcessor){
    // Initialize client
    drone::DataProcessor* dp = (drone::DataProcessor*) dataProcessor;
    int status = 0;
    pthread_mutex_lock(&printLock);
    status = clientInit(dp->client, dp->getClientParams());
    pthread_mutex_unlock(&printLock);
    if(status != 0) {
        cout << "error initializing client" << endl;
        pthread_detach(pthread_self());
        pthread_exit(NULL);
    }

    // Raise flag for initialized client
    SharedData* sharedData = dp -> getSharedData();
    pthread_mutex_lock(&sharedDataLock);
    sharedData->clientReady = 1;
    pthread_mutex_unlock(&sharedDataLock);

    if (dp->client == NULL){
        // Client off
        pthread_detach(pthread_self());
        pthread_exit(NULL);
    }
    
    // Call callback functions
    ros::spin();
    
    pthread_detach(pthread_self());
    pthread_exit(NULL);
}
