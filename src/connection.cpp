#include "network/connection.h"
#include <sstream>
#include <fstream>
using namespace std;

// Server initialization
int serverInit(Server*& server){
	cout << "Initializing Server" << endl;
	// Identify connection type
	string host;
	int port;
	int type;
	int validType = 0;

	// Read host number
	cout << "Enter host number: ";
	cin >> host;

	// Read port number
	cout << "Enter port number: ";
	cin >> port;

	// Read connection type
	while(!validType){
		cout << "Connection Types:" << endl;
		cout << " [1] TCP" << endl;
		cout << " [2] UDP" << endl;
		cout << "Enter the corresponding Number: ";
		cin >> type;
		if(type == TCP){
			cout << "Server is running on " << host << " at port "
				 << port << " using TCP connection" << endl;
			server = new Server_TCP();
			validType = 1;
		} else if(type == UDP){
			cout << "Server is running on " << host << " at port "
				 << port << " using UDP connection" << endl;
			server = new Server_UDP();
			validType = 1;
		} else {
			cout << "Incorrect connection type" << endl;
		}
	}
	int status = server->init(host, port);
	return status;
}

// Server initialization
int serverInit(Server*& server, const char* host, const int port){
	cout << "Initializing Server" << endl;
	int type;
	int validType = 0;
	// Read connection type
	while(!validType){
		cout << "Connection Types:" << endl;
		cout << " [1] TCP" << endl;
		cout << " [2] UDP" << endl;
		cout << "Enter the corresponding Number: ";
		cin >> type;
		if(type == TCP){
			cout << "Server is running on " << host << " at port "
				 << port << " using TCP connection" << endl;
			server = new Server_TCP();
			validType = 1;
		} else if(type == UDP){
			cout << "Server is running on " << host << " at port "
				 << port << " using UDP connection" << endl;
			server = new Server_UDP();
			validType = 1;
		} else {
			cout << "Incorrect connection type" << endl;
		}
	}
	int status = server->init(host, port);
	return status;
}

// Server initialization
int serverInit(Server*& server, const char* host, const int port, const int type){
	cout << "Initializing Server" << endl;
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

// Client initialization
int clientInit(Client*& client){
	cout << "Initializing Client" << endl;
	// Identify connection type
	string host;
	int port;
	int type;
	int validType = 0;

	// Read host number
	cout << "Enter host number: ";
	cin >> host;

	// Read port number
	cout << "Enter port number: ";
	cin >> port;

	// Read and check connection type
	while(!validType){
		cout << "Connection Types:" << endl;
		cout << " [1] TCP" << endl;
		cout << " [2] UDP" << endl;
		cout << "Enter the corresponding Number: ";
		cin >> type;
		if(type == TCP){
			cout << "Client is running on " << host << " at port "
				 << port << " using TCP connection" << endl;
			client = new Client_TCP();
			validType = 1;
		} else if(type == UDP){
			cout << "Client is running on " << host << " at port "
				 << port << " using UDP connection" << endl;
			client = new Client_UDP();
			validType = 1;
		} else {
			cout << "Incorrect connection type" << endl;
		}
	}
	int status = client->init(host, port);
	return status;
}

// Client initialization
int clientInit(Client*& client, const char* host, const int port){
	cout << "Initializing Client" << endl;
	int type;
	int validType = 0;
	// Read and check connection type
	while(!validType){
		cout << "Connection Types:" << endl;
		cout << " [1] TCP" << endl;
		cout << " [2] UDP" << endl;
		cout << "Enter the corresponding Number: ";
		cin >> type;
		if(type == TCP){
			cout << "Client is running on " << host << " at port "
				 << port << " using TCP connection" << endl;
			client = new Client_TCP();
			validType = 1;
		} else if(type == UDP){
			cout << "Client is running on " << host << " at port "
				 << port << " using UDP connection" << endl;
			client = new Client_UDP();
			validType = 1;
		} else {
			cout << "Incorrect connection type" << endl;
		}
	}
	int status = client->init(host, port);
	return status;
}

// Client initialization
int clientInit(Client*& client, const char* host, const int port, const int type){
	cout << "Initializing Client" << endl;
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

// Map string to index
int str2type(const string& type){
	if(type == "TCP") {
		return TCP;
	} else if(type == "UDP") {
		return UDP;
	}
	return -1;
}

// Map index to string
string type2str(int type){
	if(type == TCP) { 
		return "TCP";
	} else if(type == UDP) {
		return "UDP";
	}
	return "";
}

// Parses a configuration file
// Input: path to file
ConfigParams* readConfig(const char* config){
	ConfigParams* params = new ConfigParams;
	params -> serverOff = 0;
	params -> clientOff = 0;
	ifstream file (config);
	if(!file.is_open()) {
		cout << "Unable to find "<< config << endl;
		return NULL;
	}
	string line;
	while(getline(file, line)){
		// Process line by line
		parseLine(params, line);
	}
	file.close();
	return params;
}

// Parses a line from the configuration file
void parseLine(ConfigParams* params, const string& line){
	if(line[0] == COMMENT_CHAR){
		// Reading a comment; ignore
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