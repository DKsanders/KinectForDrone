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
        seqToDrone = 0;
        seqFromDrone = 0;
        pthread_t clientThreadID, serverThreadID;
        NetworkConfigParser networkParser;
        MarkerFileParser markerParser;
        
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
        if (!n_param.getParam ("marker_configuration", path)){
            marker_data = package_path + "/config/marker.config";
        }else{
            marker_data = path;
        }

        // Subscribe to topics
        ar_pose_markers_sub = n.subscribe("ar_pose_markers", SUB_BUFFER_SIZE, &DataProcessor::arPoseMarkersCallback, this);

        // Read config file for network
        status += networkParser.readConfig(server_config.c_str());
        serverParams = networkParser.getConfig();
        status += networkParser.readConfig(client_config.c_str());
        clientParams = networkParser.getConfig();
        status += markerParser.readMarkers(marker_data.c_str());
        markers = markerParser.getMarkers();
        if(status != 0){
            //Error reading config file
            ROS_FATAL("Error reading config file(s)");
            pthread_mutex_lock(&sharedDataLock);
            sharedData -> done = 1;
            pthread_mutex_unlock(&sharedDataLock);
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
        if(markers != NULL) delete markers;
    }

    int DataProcessor::getSeqFromDrone(){
        return seqFromDrone;
    }

    Client* DataProcessor::getClient(){
        return client;
    }

    Server* DataProcessor::getServer(){
        return server;
    }

    ConfigParams* DataProcessor::getClientParams(){
        return clientParams;
    }

    ConfigParams* DataProcessor::getServerParams(){
        return serverParams;
    }

    SharedData* DataProcessor::getSharedData(){
        return sharedData;
    }

    MarkerDataSet* DataProcessor::getMarkerDataSet(){
        return markers;
    }

    void DataProcessor::setClient(Client* _client){
        client = _client;
    }
    void DataProcessor::setServer(Server* _server){
        server = _server;
    }

    void DataProcessor::arPoseMarkersCallback(const drone::ARMarkers::ConstPtr &msg)
    {
        // Handle updated data from drone
        pthread_mutex_lock(&sharedDataLock);
        if (seqFromDrone != sharedData -> id){
            seqFromDrone = sharedData -> id; // update id
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
            int status = 0;
            DroneData data;
            status = processDataToDrone(msg, seqToDrone, data);
            if(status != 0){
                // Error processing data
                return;
            }
            ByteStream stream = data.serialize();
            // Send the message
            if (client->send(stream)) {
                // Couldn't send
                ROS_INFO("failed to send to drone");
            }
            seqToDrone += 1;    
        }
    }

    int DataProcessor::processDataToDrone(const drone::ARMarkers::ConstPtr &msg, const int seq, DroneData& data){
        // Interpret marker data
        int markerNum = msg -> markers.size();
        if(markerNum < 4) { 
            return 1;
        }
        HomogeneousMatrix avg; // average of marker data
        avg.matrix[3][3] = 0;
        HomogeneousMatrix h02offset; // Auxilary matrix, h02 when roll/pitch/yaw = 0
        h02offset.matrix[0][2] = 1;
        h02offset.matrix[1][0] = 1;
        h02offset.matrix[2][1] = 1;

        // Process data of each marker
        int i;
        for (i = 0; i < markerNum; i++){
            int markerType = (int) (msg -> markers[i].id); // starts from 0
            if(markerType >= markers->num) {
                ROS_WARN("Unconfigurated marker");
                return 1;
            }

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
            MarkerData* markerData = markers->getMarker(markerType); // marker -> drone
            HomogeneousMatrix h21 = *markerData;
            HomogeneousMatrix h20 = h10*h21; // kinect -> drone
            avg = avg + h20;
        }
        // Take average of markers
        avg = avg/markerNum;

        // Calculate RPY
        HomogeneousMatrix rpy = h02offset*avg;
        if(avg.matrix[2][0] == 1 || avg.matrix[2][0] == -1 ){
            // Singularity in the homogeneous matrix - unable to calculate roll/pitch/yaw
            return 1;
        } else {
            rpy.hm2rpy();
        }
        data = rpy.toData();
        data.setSeq(seq);
        data.print();
        return 0;
    }
}

void* runServer(void* dataProcessor){
    // Initialize
    drone::DataProcessor* dp = (drone::DataProcessor*) dataProcessor;
    Server* server = dp->getServer();
    ConfigParams* params = dp->getServerParams();
    SharedData* sharedData = dp->getSharedData();
    int status = 0;

    status = serverInit(server, params);
    if (status != 0) {
        cout << "error initializing server" << endl;
        pthread_detach(pthread_self());
        pthread_exit(NULL);
    }

    // Raise flag for initialized server
    pthread_mutex_lock(&sharedDataLock);
    sharedData->serverReady = 1;
    pthread_mutex_unlock(&sharedDataLock);

    if (server == NULL){
        // Server off
        pthread_detach(pthread_self());
        pthread_exit(NULL);
    }
    dp->setServer(server);

    // Accept loop
    while(1){
        status = server->accept();
        if(status != 0){
            pthread_detach(pthread_self());
            pthread_exit(NULL);
        }
        // Process data from drone, store it in sharedData
        while(1){
            status = server->receive();
            if (status != 0){
                // Error receiving
                cout << "recv() failed, exiting processDataFromDrone()" << endl;
                break;
            }
            // Handle dp -> server-> stream

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
    // Initialize
    drone::DataProcessor* dp = (drone::DataProcessor*) dataProcessor;
    Client* client = dp->getClient();
    ConfigParams* params = dp->getClientParams();
    SharedData* sharedData = dp -> getSharedData();
    int status = 0;


    pthread_mutex_lock(&printLock);
    status = clientInit(client, params);
    pthread_mutex_unlock(&printLock);
    if(status != 0) {
        cout << "error initializing client" << endl;
        pthread_detach(pthread_self());
        pthread_exit(NULL);
    }

    // Raise flag for initialized client
    pthread_mutex_lock(&sharedDataLock);
    sharedData->clientReady = 1;
    pthread_mutex_unlock(&sharedDataLock);

    if (client == NULL){
        // Client off
        pthread_detach(pthread_self());
        pthread_exit(NULL);
    }
    
    // Call callback functions
    dp->setClient(client);
    ros::spin();
    
    pthread_detach(pthread_self());
    pthread_exit(NULL);
}
