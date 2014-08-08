/**
 * This file (connection.cpp) implements helper functions for
 * initializing servers and clients, declared in connection.h
 * 
 * The servers and clients must be inherited from those
 * declared in network.h
 *
 * Author: David Sanders <david.sanders@mail.utoronto.ca>
 */

#include "network/connection.h"
#include <sstream>
#include <fstream>
using namespace std;

int serverInit(Server*& server){
	// Initialize
	int status;
	string host;
	int port;

	// Read host number
	cout << "Enter host number: ";
	cin >> host;

	// Read port number
	cout << "Enter port number: ";
	cin >> port;

	status = serverInit(server, host.c_str(), port);
	return status;
}

int serverInit(Server*& server, const char* host, const int port){
	// Initialize
	int status;
	int type;
	int validType = 0;

	// Read connection type
	while(!validType){
		cout << "Connection Types:" << endl;
		cout << " [1] TCP" << endl;
		cout << " [2] UDP" << endl;
		cout << "Enter the corresponding Number: ";
		cin >> type;
		if (type != TCP && type != UDP) {
			cout << "Incorrect connection type" << endl;
		}
	}
	status = serverInit(server, host, port, type);
	return status;
}

int serverInit(Server*& server, const char* host, const int port, const string& type){
	return serverInit(server, host, port, str2type(type));
}

int serverInit(Server*& server, const char* host, const int port, const int type){
	// Initialize to appropriate network type
	if(type == TCP){
		cout << "Server is running on " << host << " at port "
			 << port << " using TCP connection" << endl;
		server = new Server_TCP();
	} else if(type == UDP){
		cout << "Server is running on " << host << " at port "
			 << port << " using UDP connection" << endl;
		server = new Server_UDP();
	} else {
		cout << "Incorrect connection type" << endl;
	}
	int status = server->init(host, port);
	return status;
}

int clientInit(Client*& client){
	// Initialize
	int status;
	string host;
	int port;

	// Read host number
	cout << "Enter host number: ";
	cin >> host;

	// Read port number
	cout << "Enter port number: ";
	cin >> port;

	status = clientInit(client, host.c_str(), port);
	return status;
}

int clientInit(Client*& client, const char* host, const int port){
	// Initialize
	int status;
	int type;
	int validType = 0;

	// Read and check connection type
	while(!validType){
		cout << "Connection Types:" << endl;
		cout << " [1] TCP" << endl;
		cout << " [2] UDP" << endl;
		cout << "Enter the corresponding Number: ";
		cin >> type;
		if(type != TCP && type != UDP) {
			cout << "Incorrect connection type" << endl;
		}
	}
	status = clientInit(client, host, port, type);
	return status;
}

int clientInit(Client*& client, const char* host, const int port, const string& type){
	return clientInit(client, host, port, str2type(type));
}

int clientInit(Client*& client, const char* host, const int port, const int type){
	// Initialize to appropriate network type
	if(type == TCP){
		cout << "Client is running on " << host << " at port "
			 << port << " using TCP connection" << endl;
		client = new Client_TCP();
	} else if(type == UDP){
		cout << "Client is running on " << host << " at port "
			 << port << " using UDP connection" << endl;
		client = new Client_UDP();
	} else {
		cout << "Incorrect connection type" << endl;
	}
	int status = client->init(host, port);
	return status;
}

int str2type(const string& str){
	if(str == "TCP") {
		return TCP;
	} else if(str == "UDP") {
		return UDP;
	}
	return -1;
}

string type2str(int type){
	if(type == TCP) { 
		return "TCP";
	} else if(type == UDP) {
		return "UDP";
	}
	return "";
}

ConfigParams* readConfig(const char* config){
	// Check that the file exists
	ifstream file (config);
	if(!file.is_open()) {
		cout << "Unable to find "<< config << endl;
		return NULL;
	}

	// Initialize ConfigParams struct
	ConfigParams* params = new ConfigParams;
	params -> serverOff = 0;
	params -> clientOff = 0;
	
	string line;
	while(getline(file, line)){
		// Process line by line
		parseLine(params, line);
	}
	file.close();
	return params;
}

void parseLine(ConfigParams* params, const string& line){
	// Check if the line is a comment
	if(line[0] == COMMENT_CHAR){
		// Parsing a comment; ignore
		return;
	}
	stringstream entireLine(line);
	string name;
	string value;
	// Read name and value
	entireLine >> name;
	entireLine >> value;
	if(!entireLine.eof()){
		// Does not follow expected format
		//cout << "Too many arguments on line" << endl;
		return;
	}
	if(name == "SERVER_HOST") {
		params -> serverHost = new char[value.length()+1];
		strcpy(params -> serverHost, value.c_str());
	} else if (name == "SERVER_PORT") {
		stringstream ss(value);
		ss >> params -> serverPort;
	} else if (name == "SERVER_TYPE") {
		params -> serverType = str2type(value);
	} else if (name == "SERVER") {
		if(value == "OFF" || value == "0"){
			params -> serverOff = 1;
		}
	} else if (name == "CLIENT_HOST") {
		params -> clientHost = new char[value.length()+1];
		strcpy(params -> clientHost, value.c_str());
	} else if (name == "CLIENT_PORT") {
		stringstream ss(value);
		ss >> params -> clientPort;
	} else if (name == "CLIENT_TYPE") {
		params -> clientType = str2type(value);
	} else if (name == "CLIENT") {
		if(value == "OFF" || value == "0"){
			params -> clientOff = 1;
		}
	} else {
		// Does not follow expected format
		//cout << "Processing a line that does not follow expected format" << endl;
	}
}