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

#include "udp.h"
#include "tcp.h"
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
	ConfigParams();
	~ConfigParams();
	// Configuration variables
	int state; // 0 is off, 1 is on
	char* host; // "XXX.XXX.XXX.XXX" format
	int port;
	int type;

} ConfigParams;

int getHost(const string& line, string& host);
int getPort(const string& line, int& port);
int getType(const string& line, int& type);

/**
 * Initializes server/client
 * Arguments:
 *  server/client(OUTPUT) - server/client to be initialized; NULL if uninitialized
 *  host(INPUT) - host adress of the form "XXX.XXX.XXX.XXX"
 *  port(INPUT) - port to open connection
 *  type(INPUT) - index representing network type (e.g.TCP=1)
 * Return:
 *  0 if successful, 1 if error occurred
 */
int serverInit(Server*& server);
int serverInit(Server*& server, ConfigParams* params);
int serverInit(Server*& server, const string& host, const int port, const string& type);
int serverInit(Server*& server, const string& host, const int port, const int type);
int serverInit(Server*& server, const char* host, const int port, const string& type);
int serverInit(Server*& server, const char* host, const int port, const int type);

int clientInit(Client*& client);
int clientInit(Client*& client, ConfigParams* params);
int clientInit(Client*& client, const string& host, const int port, const string& type);
int clientInit(Client*& client, const string& host, const int port, const int type);
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
 *  lineNum(INPUT) - line number of config file being processed
 */
int parseLine(ConfigParams* params, const string& line, int lineNum);

#endif //END_IF_CONNECTION_H