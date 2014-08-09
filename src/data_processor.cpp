/**
 * This file (data_processor.cpp) is the main file for the tracking system.
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
#include "drone/data_processor.h"
#include "drone/my_matrix.h"

#include <math.h>

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

namespace drone {
    DataProcessor::DataProcessor(ros::NodeHandle & _n):n (_n){
        // Initialize
        pthread_mutex_init(&sharedDataLock, NULL);
        pthread_mutex_init(&printLock, NULL);
        sharedData = new SharedData;
        sharedData -> id = 0;
        sharedData -> serverReady = 0;
        sharedData -> clientReady = 0;
        sharedData -> done = 0;

        seq = 0;
        currentSharedDataID = 0;
        server = NULL;
        client = NULL;
        pthread_t clientThreadID, serverThreadID;
        
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

        // Read config file for network
        params = readConfig(network_config.c_str());
        if(params == NULL){
            cout << "error reading config file" << endl;
        }

        // Initialize client
        if(!params->clientOff){
            pthread_create(&clientThreadID, NULL, runClient, this);
        } else {
            pthread_mutex_lock(&printLock);
            cout << "Client OFF" << endl;
            pthread_mutex_unlock(&printLock);
            sharedData->clientReady = 1;
        }

        // Wait until client has been initialized
        int clientReady = 0;
        while(!clientReady){
            pthread_mutex_lock(&sharedDataLock);
            clientReady = sharedData -> clientReady;
            pthread_mutex_unlock(&sharedDataLock);
            sleep(1);
        }

        // Initialize server
        if(!params->serverOff){
            pthread_create(&serverThreadID, NULL, runServer, this);
        } else {
            pthread_mutex_lock(&printLock);
            cout << "Server OFF" << endl;
            pthread_mutex_unlock(&printLock);
            sharedData->serverReady = 1;
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

        // Process data of each marker
        int i;
        for (i = 0; i < markerNum; i++){
            int type = (int) (msg -> markers[i].id); // starts from 0

            // Obtain Quaternion info
            Quaternion quat;
            quat.x = msg -> markers[i].pose.pose.orientation.x;
            quat.y = msg -> markers[i].pose.pose.orientation.y;
            quat.z = msg -> markers[i].pose.pose.orientation.z;
            quat.w = msg -> markers[i].pose.pose.orientation.w;

            // Create Homogeneous Matrices
            HomogeneousMatrix h01 = quat; // kinect -> marker
            h01.matrix[0][3] = msg -> markers[i].pose.pose.position.x;
            h01.matrix[1][3] = msg -> markers[i].pose.pose.position.y;
            h01.matrix[2][3] = msg -> markers[i].pose.pose.position.z;

            HomogeneousMatrix h12; // marker -> drone
            h12.matrix[0][0] = 0;
            h12.matrix[0][1] = 0;
            h12.matrix[0][2] = -1;
            h12.matrix[0][3] = -0.04;
            h12.matrix[1][0] = 0;
            h12.matrix[1][1] = -1;
            h12.matrix[1][2] = 0;
            h12.matrix[1][3] = 0;
            h12.matrix[2][0] = -1;
            h12.matrix[2][1] = 0;
            h12.matrix[2][2] = 0;
            h12.matrix[2][3] = -0.23;
            //double h12[4][4] = { { 0, 0, -1, -4 }, { 0, -1, 0, 0}, { -1, 0, 0, -23 }, { 0, 0, 0, 1 }};
            HomogeneousMatrix h02 = h12*h01; // kinect -> drone
            data = h02.toData();
            h02.print();

            /*
            double roll, pitch, yaw;
            roll=atan2(H02[1][0],H02[0][0]);
            pitch=atan2(-1*H02[2][0],sqrt(H02[2][1]*H02[2][1]+H02[2][2]*H02[2][2]));
            yaw=atan2(H02[2][1],H02[2][2]);

            //Print
            printf("\nX,Y,Z: %f,  %f,  %f\n",H02[0][3],H02[1][3],H02[2][3]);
            printf("\nRotation Matrix:\n");
            for(c=0;c<3;c++){
                printf( "\t%f\t%f\t%f\n",H02[0][c],H02[1][c],H02[2][c]);
            }
            printf("Roll: %f\nPitch: %f\nYaw: %f\n",roll*180/3.14159265,pitch*180/3.14159265,yaw*180/3.14159265);
            */

            //printData(data);
        }
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

    // Raise flag for initialized server
    SharedData* sharedData = dp->getSharedData();
    pthread_mutex_lock(&sharedDataLock);
    sharedData->serverReady = 1;
    pthread_mutex_unlock(&sharedDataLock);

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
    status = clientInit(dp->client, dp->getParams()->clientHost, dp->getParams()->clientPort, dp->getParams()->clientType);
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

    // Call callback functions
    ros::spin();
    
    pthread_detach(pthread_self());
    pthread_exit(NULL);
}