/**
 * This file (markers.h) provides functions for reading and using 
 * marker data, implemented in markers.cpp
 *
 * Author: David Sanders <david.sanders@mail.utoronto.ca>
 */

#include "drone/parsers.h"
#include "drone/network.h"
#include <iostream>
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

ConfigParams::ConfigParams(const ConfigParams& other) {
	this->state = other.state;
	this->port = other.port;
	this->type = other.type;
	int length = strlen(other.host)+1;
	this->host = new char[length];
	memcpy(this->host, other.host, length);
}

ConfigParams::~ConfigParams() {
	del();
}

void ConfigParams::del() {
	if(host != NULL){
		delete host;
	}
}

ConfigParams& ConfigParams::operator=(const ConfigParams& rhs){
	// Initialize
	del();

	this->state = rhs.state;
	this->port = rhs.port;
	this->type = rhs.type;
	int length = strlen(rhs.host)+1;
	this->host = new char[length];
	memcpy(this->host, rhs.host, length);
	return *this;
}

NetworkConfigParser::NetworkConfigParser(){
	currentLine = 1;
}

NetworkConfigParser::~NetworkConfigParser(){
	;
}

ConfigParams NetworkConfigParser::getConfig(){
	return data;
}

int NetworkConfigParser::readConfig(const char* filePath){
	// Check that the file exists
	ifstream file(filePath);
	if(!file.is_open()) {
		cout << "Unable to find "<< file << endl;
		return 1;
	}

	cout << "PROCESSING: "<< filePath << endl;
	
	// Initialize
	string line;
	int status = 0;
	while(getline(file, line)){
		// Process line by line
		status += parseLine(line);
		currentLine += 1;
	}
	if(data.state == 0) {
		cout << "STATE not specified in file, specify either ON or 'OFF'" << endl;
		status = 1;
	}
	if(data.host == NULL) {
		cout << "HOST not specified in file, specify either the host number 'XXX.XXX.XXX.XXX'" << endl;
		status = 1;
	}
	if(data.port == 0) {
		cout << "PORT not specified in file, specify the a valid port number" << endl;
		status = 1;
	}
	if(data.type == 0) {
		cout << "NETWORK not specified in file, specify either 'UDP' or 'TCP'" << endl;
		status = 1;
	}
	if(status != 0){
        cout << "Error reading config file" << endl;
		return 1;
	}
	cout << "Successfully parsed config file" << endl;
	file.close();
	return 0;
}

int NetworkConfigParser::parseLine(const string& _line){
	stringstream ss(_line);
	string line;
	getline(ss, line);

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
		cout << "Error on line " << currentLine << ": too many arguments" << endl;
		return 1;
	}

	if(name == "HOST") {
		string host;
		status = extractHost(value, host);
		if(status == 0){
			data.host = new char[host.length()+1];
			strcpy(data.host, host.c_str());
		}
	} else if (name == "PORT") {
		int port;
		status = extractPort(value, port);
		if(status == 0){
			data.port = port;
		}
	} else if (name == "NETWORK") {
		int type;
		status = extractType(value, type);
		if(status == 0){
			data.type = type;
		}
	} else if (name == "STATE") {
		if(value == "ON" || value == "on" || value == "On" || value == "1"){
			data.state = ON;
		} else if (value == "OFF" || value == "off" || value == "Off" || value == "0"){
			data.state = OFF;
		} else {
			status = 1;
		}
	} else {
		status = 1;
	}

	if(status != 0){
		// Does not follow expected format
		cout << "Error on line " << currentLine << ": invalid values" << endl;
		return 1;
	}
	return 0;
}

int NetworkConfigParser::extractHost(const string& line, string& host){
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

int NetworkConfigParser::extractPort(const string& line, int& port) {
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

int NetworkConfigParser::extractType(const string& line, int& type) {
	int temp = str2type(line);
	if (temp < 1) {
		return 1;
	}
	type = temp;
	return 0;
}

int NetworkConfigParser::str2type(const string& str){
	if(str == "TCP" || str == "tcp" || str == "1" ) {
		return TCP;
	} else if(str == "UDP" || str == "udp" || str == "2") {
		return UDP;
	}
	return -1;
}

MarkerDataSet::MarkerDataSet(){
	array = NULL;
}

MarkerDataSet::MarkerDataSet(const MarkerDataSet& other){
	// Initialize
	int i;
	this->num = other.num;
	this->array = new MarkerData*[this->num];
	for(i=0; i<num; i++){
    	array[i] = new MarkerData;
    	*(array[i]) = *(other.array[i]);
	}
}

MarkerDataSet::~MarkerDataSet(){
	deleteMarkers();
}

MarkerData MarkerDataSet::getMarker(int id){
    return *(array[id]);
}

void MarkerDataSet::deleteMarkers(){
	// Delete all initialized markers
	if(array != NULL){
    	int i;
    	for(i=0; i<num; i++){
        	delete array[i];
    	}
    	delete [] array;
	}
}

MarkerDataSet& MarkerDataSet::operator=(const MarkerDataSet& rhs){
	// Initialize
	int i;
	deleteMarkers();

	this->num = rhs.num;
	this->array = new MarkerData*[this->num];
	for(i=0; i<num; i++){
    	array[i] = new MarkerData;
    	*(array[i]) = *(rhs.array[i]);
	}
	return *this;
}

MarkerFileParser::MarkerFileParser(){
	currentLine = 1;
	lineCount = 0;
}

MarkerFileParser::~MarkerFileParser(){
	;
}

MarkerDataSet MarkerFileParser::getMarkers(){
	return data;
}

int MarkerFileParser::readMarkers(const char* markers){
	// Check that the file exists
	ifstream file (markers);
	if(!file.is_open()) {
		cout << "UNABLE TO FIND: " << markers << endl;
		return 1;
	}

	// Start processing
	cout << "PROCESSING: " << markers << endl;
	string line;
	int status = 0;
	int validConfig = 1;

	// Parse each line
	while(getline(file, line)){
		status += parseLine(line);
		currentLine += 1;
	}
	int correctCount = data.num * 6 + 1;
	if(lineCount < correctCount){
        cout << "Not enough data for " << data.num << " markers" << endl;
        status = 1;
	}
	if(lineCount > correctCount){
        cout << "Too much data for " << data.num << " markers" << endl;
        status = 1;
	}
	if(status != 0){
        cout << "Error reading marker file" << endl;
		return 1;
	}
	cout << "Successfully parsed marker file" << endl;
	file.close();
	return 0;
}

int MarkerFileParser::parseLine(const string& _line){
	// Check if line should be ignored
	stringstream ss(_line);
	string line;
	getline(ss, line);

	if(line.empty() || line[0] == COMMENT_CHAR){
		// Parsing a comment or whitespace - ignore
		return 0;
	}
	
	// Initialize
	int status = 0;

	// Read name and value
	if(lineCount == 0){
		status = parseMarkerNum(line);
	} else if((lineCount-1)%6 < 3){
		status = parseRow(line);
	} else {
		status = parseDistance(line);
	}

	if(status != 0){
		// Does not follow expected format
		cout << "Error on line " << currentLine << ": invalid values" << endl;
		return 1;
	}

	// Processing line successful
	lineCount += 1;
	currentMarker = (lineCount-1)/6;
	index = (lineCount-1)%6;
	return 0;
}

int MarkerFileParser::parseMarkerNum(const string& line){
	// Initialize
	stringstream ss(line);
	ss >> data.num;
	if(!ss.eof() || data.num < 1){
		// invalid line
		return 1;
	}
	data.array = new MarkerData*[data.num];
	return 0;
}

int MarkerFileParser::parseRow(const string& line){
	// Initialize
	stringstream ss(line);
	if(index == 0){
		data.array[currentMarker] = new MarkerData;
	}
	ss >> data.array[currentMarker]->rm[index][0];
	if(ss.fail()){
		return 1;
	}
	ss >> data.array[currentMarker]->rm[index][1];
	if(ss.fail()){
		return 1;
	}
	ss >> data.array[currentMarker]->rm[index][2];
	if(!ss.eof()){
		return 1;
	}
	return 0;

}

int MarkerFileParser::parseDistance(const string& line){
	// Initialize
	stringstream ss(line);
	if(index == 3){
		ss >> data.array[currentMarker]->dist_x;
	} else if(index == 4){
		ss >> data.array[currentMarker]->dist_y;
	} else if(index == 5){
		ss >> data.array[currentMarker]->dist_z;
	}
	if(!ss.eof()){
		return 1;
	}
	return 0;
}

int serverInit(Server*& server, ConfigParams* params){
	// Initialize
	int status = 0;
	string host;
	int port;
	int type;

	// Check that the server is to be running
	if(params->state == OFF){
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
	if(params->state == OFF){
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