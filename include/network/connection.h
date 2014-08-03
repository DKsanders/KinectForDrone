#ifndef CONNECTION_H
#define CONNECTION_H

#include <unistd.h>
#include <string.h>
#include <iostream>
#include "network/udp.h"
#include "network/tcp.h"

#define TCP 1
#define UDP 2

#define COMMENT_CHAR '#'
#define PRECISION 7 // d.p. of data

using namespace std;

// Structure for holding the configuration parameters
typedef struct ConfigParams{
	// Server configuration
	int serverOff;
	char* serverHost;
	int serverPort;
	int serverType;

	// Client configuration
	char* clientHost;
	int clientPort;
	int clientType;
	int clientOff;

} ConfigParams;

// Initializers
int serverInit(Server*& server);
int serverInit(Server*& server, const char* host, const int port);
int serverInit(Server*& server, const char* host, const int port, const int type);
int clientInit(Client*& client);
int clientInit(Client*& client, const char* host, const int port);
int clientInit(Client*& client, const char* host, const int port, const int type);

// For mapping strings to a number vice versa
int str2type(const string& type);
string type2str(int type);

// Parses a configuration file
// takes in the path to the file as input
ConfigParams* readConfig(const char* config);
// Parses a line of a configuration file
void parseLine(ConfigParams* params, const string& line);

#endif //END_IF_CONNECTION_H