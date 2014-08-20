/**
 * This file (marker_tracker.cpp) is the main file for the marker tracking system.
 *
 * It tracks markers on the drone using the Microsoft Kinect (c),
 * proccesses the data to estimate the attitude of the drone,
 * and sends the data to the drone via wifi. 
 *
 * This program was run on Ubuntu 12.10 with ROS hydro
 *
 * Author: David Sanders <david.sanders@mail.utoronto.ca>
 *
 * Bugs/Issues
 *  -Program exits with an error (possibly freeing memory twice?)
 *
 * Areas of Improvemnet
 *  - Replace my_matrix functions with proper matrix labraries or tf functions in ROS
 *  - The calibration method, calibrate(), could be improved
 *  - The data from the kinect is precessed in processDataToDrone() on line 224;
 *    If the user wishes to make changes on how to process this data (e.g. replace
 *    the my_matrix functions, use /tf messages instead of ARMarkers, etc), then there
 *    should only be a need to modify this function
 *
 * Other comments
 *  - Currently, data will be processed only if the kinect sees exactly 4 markers,
 *    which should be in the following layout:
 *                         3
 *                2        4        1
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
        clientParams = new ConfigParams;
        serverParams = new ConfigParams;
        markers = new MarkerDataSet;
        calibData = new CalibrationData;
        sampleNum = 0;
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
        *serverParams = networkParser.getConfig();
        status += networkParser.readConfig(client_config.c_str());
        *clientParams = networkParser.getConfig();
        status += markerParser.readMarkers(marker_data.c_str());
        *markers = markerParser.getMarkers();
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
        delete clientParams;
        delete serverParams;
        delete markers;
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
            status = processDataToDrone(msg, data);
            if(status != 0){
                // Error processing data
                return;
            }
            data.print();
            ByteStream stream = data.serialize();
            // Send the message
            if (client->send(stream)) {
                // Couldn't send
                ROS_INFO("failed to send to drone");
            }
        }
        seqToDrone += 1;   
    }

    int DataProcessor::processDataToDrone(const drone::ARMarkers::ConstPtr &msg, DroneData& data){
        // Interpret marker data
        int markerNum = msg -> markers.size();

        // Only work with 4 markers
        if(markerNum != 4) { return 1; }
        
        // Auxilary matrix, h02 when roll/pitch/yaw = 0
        HomogeneousMatrix h02offset; 
        h02offset.matrix[0][2] = 1;
        h02offset.matrix[1][0] = 1;
        h02offset.matrix[2][1] = 1;
        // Vectors from kinect to markers
        Vector vectors[4];
        // For keeping an average
        HomogeneousMatrix avg;

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

            // Create vectors
            vectors[markerType].vector[0] = msg -> markers[i].pose.pose.position.x;
            vectors[markerType].vector[1] = msg -> markers[i].pose.pose.position.y;
            vectors[markerType].vector[2] = msg -> markers[i].pose.pose.position.z;

            // Get marker data and transform it to a homogeneous matrix
            MarkerData markerData = markers->getMarker(markerType); // marker -> drone
            HomogeneousMatrix h21 = markerData;
            HomogeneousMatrix h20 = h10*h21; // kinect -> drone
            
            // Calculate RPY from marker orientation
            HomogeneousMatrix rpy = h02offset*h20;
            if(rpy.matrix[2][0] == 1 || rpy.matrix[2][0] == -1 ){
                // Singularity in the homogeneous matrix - unable to calculate roll/pitch/yaw
                return 1;
            } else {
                rpy.hm2rpy();
            }
            avg = avg + rpy;
        }
        // Find average
        avg = avg/markerNum;
        // Find vectors representing x and y
        Vector vec_x = vectors[0]-vectors[1];
        vec_x.normalize();
        Vector vec_y = vectors[3]-vectors[2];
        vec_y.normalize();
        Vector cross = vec_x * vec_y;
        RotationMatrix rm;
        rm.matrix[0][1] = vec_x.vector[0];
        rm.matrix[1][1] = vec_x.vector[1];
        rm.matrix[2][1] = vec_x.vector[2];
        rm.matrix[0][2] = vec_y.vector[0];
        rm.matrix[1][2] = vec_y.vector[1];
        rm.matrix[2][2] = vec_y.vector[2];
        rm.matrix[0][0] = cross.vector[0];
        rm.matrix[1][0] = cross.vector[1];
        rm.matrix[2][0] = cross.vector[2];
        HomogeneousMatrix hm = rm;
        HomogeneousMatrix h20 = h02offset*hm; // kinect -> drone
        if(h20.matrix[2][0] == 1 || h20.matrix[2][0] == -1 ){
            // Singularity in the homogeneous matrix - unable to calculate roll/pitch/yaw
            return 1;
        } else {
            h20.hm2rpy();
        }

        avg = (avg+h20)/2;

        // Find offset if not done
        if(sampleNum < CALIBRATION_SAMPLE_NUMBER){
            calibData -> calibrate(h20.matrix[0][3], h20.matrix[1][3], h20.matrix[2][3], h20.roll, h20.pitch, h20.yaw);
            sampleNum += 1;
            return 1;
        }

        // Calibrate data
        h20.calibrate(*calibData);

        data = h20.toData();
        data.setSeq(seqToDrone);
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

    /*
    ros::AsyncSpinner spinner(2);// 2 threads
    spinner.start();
    ros::waitForShutdown();
    */
    ros::spin();
    
    pthread_detach(pthread_self());
    pthread_exit(NULL);
}
