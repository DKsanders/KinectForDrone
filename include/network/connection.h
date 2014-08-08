/**
 * This file (connection.h) provides helper functions for
 * initializing servers and clients, implemented in connection.cpp
 *
 * The servers and clients must be inherited from those
 * declared in network.h
 *
 * Author: David Sanders <david.sanders@mail.utoronto.ca>
 */

#ifndef CONNECTION_H
#define CONNECTION_H

#include "network/udp.h"
#include "network/tcp.h"
#include <unistd.h>
#include <string.h>
#include <iostream>

// Network-type indices
#define TCP 1
#define UDP 2

#define COMMENT_CHAR '#' // for commenting in configuration files

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

/**
 * Initializes server/client
 * Arguments:
 *  server/client(OUTPUT) - server/client to be initialized
 *  host(INPUT) - host adress of the form "XXX.XXX.XXX.XXX"
 *  port(INPUT) - port to open connection
 *  type(INPUT) - index representing network type (e.g.TCP=1)
 * Return:
 *  0 if successful
 */
int serverInit(Server*& server);
int serverInit(Server*& server, const char* host, const int port);
int serverInit(Server*& server, const char* host, const int port, const string& type);
int serverInit(Server*& server, const char* host, const int port, const int type);

int clientInit(Client*& client);
int clientInit(Client*& client, const char* host, const int port);
int clientInit(Client*& client, const char* host, const int port, const string& type);
int clientInit(Client*& client, const char* host, const int port, const int type);

/**
 * Maps a string containing the network type to its index
 * Arguments:
 *  type(INPUT) - string holding network type (e.g. "TCP")
 * Return:
 *  index representing network type; returns -1 if unsuccessful
 */
int str2type(const string& str);

/**
 * Converts an index representing the network type to its name
 * Arguments:
 *  type(INPUT) - index representing network type
 * Return:
 *  string holding name of network; returns empty string if unsuccessful
 */
string type2str(int type);

/**
 * Reads in a configuration file and parses it
 * Arguments:
 *  config(INPUT) - path to configuration file
 * Return:
 *  pointer to ConfigParams struct allocated using new; NULL if unsuccessful
 */
ConfigParams* readConfig(const char* config);

/**
 * Parses a line of a configuration file
 * Arguments:
 *  config(OUTPUT) - updated data from parsing the line
 *  line(INPUT) - a single line from the configuration file
 */
void parseLine(ConfigParams* params, const string& line);

#endif //END_IF_CONNECTION_H