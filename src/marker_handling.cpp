#include "drone/marker_handling.h"
#include <iostream>
#include <sstream>
#include <iomanip>
using namespace std;

AllMarkers* msg2marker(const drone::ARMarkers::ConstPtr &msg){
	// Create Struct to hold important info.
	AllMarkers* markers = (AllMarkers*)malloc(sizeof(AllMarkers));
	markers -> seq = msg -> header.seq;
	markers -> markerNum = msg -> markers.size();
	markers -> head = NULL;

	// Process data of each marker
    int i;
    for (i = 0; i < markers->markerNum; i++){
        Marker* marker = (Marker*)malloc(sizeof(Marker));
    	marker -> type = (int) (msg -> markers[i].id); // starts from 0

    	marker -> dist_x = msg -> markers[i].pose.pose.position.x;
    	marker -> dist_y = msg -> markers[i].pose.pose.position.y;
    	marker -> dist_z = msg -> markers[i].pose.pose.position.z;

    	marker -> pose_x = msg -> markers[i].pose.pose.orientation.x;
    	marker -> pose_y = msg -> markers[i].pose.pose.orientation.y;
        marker -> pose_z = msg -> markers[i].pose.pose.orientation.z;
        marker -> pose_w = msg -> markers[i].pose.pose.orientation.w;

    	// Add to head of linked list
    	marker -> link = markers -> head;
    	markers -> head = marker;
    }
    return markers;
}

string marker2str(AllMarkers* markers){
	// Create a string stream
    string temp;
    stringstream markerString;
    markerString << setw(PRECISION); // set precision
    
    // Feed stream with data
    markerString << markers->seq << DELIM;
    markerString << markers->markerNum << DELIM;
    Marker* head = markers -> head;
    while(head != NULL){
		markerString << head->type << DELIM;
		markerString << head->dist_x << DELIM;
		markerString << head->dist_y << DELIM;
		markerString << head->dist_z << DELIM;
		markerString << head->pose_x << DELIM;
		markerString << head->pose_y << DELIM;
        markerString << head->pose_z << DELIM;
        markerString << head->pose_w << DELIM;

    	head = head->link;
    }

    // Output stream to string
    markerString >> temp;
    temp = temp + "\n";
    return temp;
}

AllMarkers* str2marker(char* str){
	stringstream ss(str);
    char delim;
    AllMarkers* markers = (AllMarkers*)malloc(sizeof(AllMarkers));
    // Read in data
    ss >> markers -> seq >> delim;
    ss >> markers -> markerNum >> delim;
    markers -> head = NULL;

    int i;
    for (i = 0; i < markers->markerNum; i++){
        Marker* marker = (Marker*)malloc(sizeof(Marker));

        // Stuff Struct with info
        ss >> marker -> type >> delim;
        ss >> marker -> dist_x >> delim;
        ss >> marker -> dist_y >> delim;
        ss >> marker -> dist_z >> delim;
        ss >> marker -> pose_x >> delim;
        ss >> marker -> pose_y >> delim;
        ss >> marker -> pose_z >> delim;
        ss >> marker -> pose_w >> delim;

        // take care of linked list chaining
        marker -> link = markers -> head;
        markers -> head = marker;
    }
    return markers;
}

void printMarkers(AllMarkers* markers){
    // Print basic info
    cout << "\nSequence Number: " << markers -> seq << endl;
    cout << "Markers Detected: " << markers -> markerNum << endl;
    Marker* marker = markers->head;
    while (marker != NULL){
        // Print marker info
        cout << "\nMarker Type: " << marker -> type << endl;
        cout << "Position: "  << endl;
        cout << "  x "  << marker -> dist_x << endl;
        cout << "  y "  << marker -> dist_y << endl;
        cout << "  z "  << marker -> dist_z << endl;
        cout << "Orientation: "  << endl;
        cout << "  x "  << marker -> pose_x << endl;
        cout << "  y "  << marker -> pose_y << endl;
        cout << "  z "  << marker -> pose_z << endl;
        cout << "  w "  << marker -> pose_w << endl;

        // take care of linked list chaining
        marker = marker->link;
    }
    cout << "---------------------------------------------------------------------"<< endl;
    return;
}

/* Prints the type of marker
0: Black/White
1: Red/Green
2: Blue/Green
*/
void printTypes(int type) {
    cout << "\nMarker Type: ";
    if (type == 0) {
        cout << "Black and White" <<endl;
    } else if (type == 1) {
        cout << "Red and Green" <<endl;
    } else if (type == 2) {
        cout << "Blue and Green" <<endl;
    }
    return;
}