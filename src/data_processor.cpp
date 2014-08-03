/**
 * This program processes messages of type ARMarkers
 */

// ROS libs
#include "drone/data_processor.h"
#include <sstream>
#include <pthread.h>
using namespace std;

#define CONFIG_FILE "/home/sander57/catkin_ws/src/drone/run/default.config"

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
        
        params = readConfig(CONFIG_FILE);
        if(params == NULL){
            cout << "error reading config file" << endl;
        }
        if(!params->clientOff){
            if(clientInit(client, params->clientHost, params->clientPort, params->clientType)!=0) {
                cout << "error initializing client" << endl;
            }
            pthread_create(&clientThreadID, NULL, spin, NULL);
        } else {
            cout << "CLIENT: OFF" << endl;
        }
        if(!params->serverOff){
            if(serverInit(server, params->serverHost, params->serverPort, params->serverType)!=0) {
                cout << "error initializing server" << endl;
            }
            pthread_create(&serverThreadID, NULL, processDataFromDrone, (void*)(this));
        } else {
            cout << "SERVER: OFF" << endl;
        }
        // Get parameters
        std::string path;
        //std::string package_path = ros::package::getPath (ROS_PACKAGE_NAME);
        ros::NodeHandle n_param ("~");
        XmlRpc::XmlRpcValue param_center;

        // Subscribe to topics
        ar_pose_markers_sub = n.subscribe("ar_pose_markers", SUB_BUFFER_SIZE, &DataProcessor::arPoseMarkersCallback, this);

        // Request services
  
        // Respond to services
    }

    DataProcessor::~DataProcessor(){
        
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
            delete [] buf;
            seq += 1;    
        }
    }

    DroneData* DataProcessor::processDataToDrone(const drone::ARMarkers::ConstPtr &msg, const int seq){
        // Interpret marker data
        DroneData* data = new DroneData;

        // example
        Quaternion* quat = new Quaternion;
        quat -> w = seq/1000.0;
        quat -> x = seq/1000.0;
        quat -> y = seq/1000.0;
        quat -> z = seq/1000.0;

        data -> seq = seq;
        data -> dist_x = seq/1000.0;
        data -> dist_y = seq/1000.0;
        data -> dist_z = seq/1000.0;
        data -> comment = "comment";

        data -> rm -> matrix[0][0] = seq/1000.0;
        data -> rm -> matrix[0][1] = seq/1000.0;
        data -> rm -> matrix[0][2] = seq/1000.0;
        data -> rm -> matrix[1][0] = seq/1000.0;
        data -> rm -> matrix[1][1] = seq/1000.0;
        data -> rm -> matrix[1][2] = seq/1000.0;
        data -> rm -> matrix[2][0] = seq/1000.0;
        data -> rm -> matrix[2][1] = seq/1000.0;
        data -> rm -> matrix[2][2] = seq/1000.0;

        printData(data);

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

void* processDataFromDrone(void* dataProcessor){
    // Process data from drone, store it in sharedData
    drone::DataProcessor* dp = (drone::DataProcessor*) dataProcessor;
    if(dp->getParams()->serverOff){
        pthread_detach(pthread_self());
        pthread_exit(NULL);
    }
    int status = 0;
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

void* spin(void* obj){
    ros::spin();
    pthread_detach(pthread_self());
    pthread_exit(NULL);
}