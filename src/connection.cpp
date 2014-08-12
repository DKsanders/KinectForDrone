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

ConfigParams::ConfigParams() {
	// Initialize
	state = 0;
	host = NULL;
	port = 0;
	type = 0;
}

ConfigParams::~ConfigParams() {
	if(host != NULL){
		delete host;
	}
}

int extractHost(const string& line, string& host) {
	// Check if localhost
	if(line == "local_host" || line == "localhost" || line == "LOCAL_HOST" || line == "LOCALHOST"){
		host = "127.0.0.1";
		return 0;
	}
	// Initialize
	stringstream ss(line);
	int num;
	char period;
	int i;

	// Check that each number is 0-255 and has periods inbetween
	for(i=0; i<3; i++){
		ss >> num;
		if(num < 0 || num > 255){
			return 1;
		}
		ss >> period;
		if(period != '.'){
			return 1;
		}
	}

	// Make sure there is still the last number to read
	if(ss.eof()) {
		// Error - ran out of data
		return 1;
	}

	// Check last number of host
	ss >> num;
	if(num < 1 || num > 255){
		return 1;
	}

	// Error if it wasn't the last one
	if(!ss.eof()) {
		return 1;
	}

	// Valid host
	host = line;
	return 0;
}

int extractPort(const string& line, int& port) {
	// Check the port number
	stringstream ss(line);
	int temp;
	ss >> temp;
	if(temp < 1 || temp > 65535){
		return 1;
	}

	// Error if stream not empty
	if(!ss.eof()) {
		return 1;
	}

	// Valid port number
	port = temp;
	return 0;
}

int extractType(const string& line, int& type) {
	// Check that the line is valid
	if(line == "TCP" || line == "tcp" || line == "1"){
		type = 1;
		return 0;
	}
	if(line == "UDP" || line == "udp" || line == "2"){
		type = 2;
		return 0;
	}
	return 1;
}

/*
int serverInit(Server*& server){
	ConfigParams dummy;
	dummy.state = 1; // turn on server
	return serverInit(server, &dummy);
}
*/

int serverInit(Server*& server, ConfigParams* params){
	// Initialize
	int status = 0;
	string host;
	int port;
	int type;

	// Check that the server is to be running
	if(params->state == 0){
		cout << "Server OFF" << endl;
		server = NULL;
		return 0;
	}

	// Obtain
	host = params->host;
	port = params->port;
	type = params->type;

	// Initialize server
	if(type == TCP){
		cout << "Server is running on " << host << " at port "
			 << port << " using TCP connection" << endl;
		server = new Server_TCP();
	} else if(type == UDP){
		cout << "Server is running on " << host << " at port "
			 << port << " using UDP connection" << endl;
		server = new Server_UDP();
	}
	status = server->init(host, port);
	return status;
}

int clientInit(Client*& client, ConfigParams* params){
	// Initialize
	int status = 0;
	string host;
	int port;
	int type;

	// Check that the client is to be running
	if(params->state == 0){
		cout << "Client OFF" << endl;
		client = NULL;
		return 0;
	}

	// Obtain
	host = params->host;
	port = params->port;
	type = params->type;

	// Initialize client
	if(type == TCP){
		cout << "Client is running on " << host << " at port "
			 << port << " using TCP connection" << endl;
		client = new Client_TCP();
	} else if(type == UDP){
		cout << "Client is running on " << host << " at port "
			 << port << " using UDP connection" << endl;
		client = new Client_UDP();
	}
	status = client->init(host, port);
	return status;
}

int str2type(const string& str){
	if(str == "TCP" || str == "tcp" ) {
		return TCP;
	} else if(str == "UDP" || str == "dup" ) {
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

	cout << "PROCESSING* "<< config << endl;
	
	// Initialize ConfigParams struct
	ConfigParams* params = new ConfigParams;
	
	string line;
	int lineNum = 1;
	int validConfig = 1;
	while(getline(file, line)){
		// Process line by line
		int status = parseLine(params, line, lineNum);
		lineNum += 1;
		if(status != 0){
			validConfig = 0;
		}
	}
	if(!validConfig){
        cout << "Error reading config file" << endl;
		return NULL;
	}
	cout << "Successfully parsed config file" << endl;
	file.close();
	return params;
}

int parseLine(ConfigParams* params, const string& line, int lineNum){
	// Check if line should be ignored
	if(line.empty() || line[0] == COMMENT_CHAR){
		// Parsing a comment; ignore
		return 0;
	}
	
	// Initialize
	stringstream entireLine(line);
	string name;
	string value;
	int status = 0;

	// Read name and value
	entireLine >> name;
	entireLine >> value;
	if(!entireLine.eof()){
		// Does not follow expected format
		cout << "Error on line " << lineNum << ": too many arguments" << endl;
		return 1;
	}

	if(name == "HOST") {
		string host;
		status = extractHost(value, host);
		if(status == 0){
			params -> host = new char[host.length()+1];
			strcpy(params -> host, host.c_str());
		}
	} else if (name == "PORT") {
		int port;
		status = extractPort(value, port);
		if(status == 0){
			params -> port = port;
		}
	} else if (name == "NETWORK") {
		int type;
		status = extractType(value, type);
		if(status == 0){
			params -> type = type;
		}
	} else if (name == "STATE") {
		if(value == "ON" || value == "on" || value == "On" || value == "1"){
			params -> state = 1;
		} else if (value == "OFF" || value == "off" || value == "Off" || value == "0"){
			params -> state = 0;
		} else {
			status = 1;
		}
	} else {
		status = 1;
	}

	if(status != 0){
		// Does not follow expected format
		cout << "Error on line " << lineNum << ": invalid values" << endl;
		return 1;
	}
	return 0;
}