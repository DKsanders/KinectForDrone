/**
 * This file (parsers.h) provides functions for parsing 
 * configuration files, implemented in parsers.cpp
 *
 * Author: David Sanders <david.sanders@mail.utoronto.ca>
 *
 /****** NETWORK CONFIG EXAMPLE ******/
 /* # Server Configuration
 *  # COMMENT
 *  STATE ON
 *  HOST 127.0.0.1
 *  PORT 1111
 *  NETWORK TCP
 *
 /****** MARKER CONFIG EXAMPLE ******/
 /* # Marker Data
 *  # COMMENT
 *  # PATTERN 1
 *  # Attitude
 *  1 0 0
 *  0 1 0
 *  0 0 1
 *  # x distance in m
 *  0.1
 *  # y distance in m
 *  0
 *  # z distance in m
 *  -0.2
 */

#ifndef PARSERS_H
#define PARSERS_H

#include "drone/network.h"
#include <string>
using namespace std;

#define COMMENT_CHAR '#'

// Network-type indices
#define TCP 1
#define UDP 2

// State of client/server
#define ON 1
#define OFF 2

using namespace std;

// Structure for holding the configuration parameters
struct ConfigParams{
  ConfigParams();
  ConfigParams(const ConfigParams& other);
  ~ConfigParams();
  // Configuration variables
  int state; // 0 is off, 1 is on
  char* host; // "XXX.XXX.XXX.XXX" format
  int port;
  int type;

  void del();

  ConfigParams& operator=(const ConfigParams& rhs);

};

class NetworkConfigParser{
public:
  NetworkConfigParser();
  ~NetworkConfigParser();

  // Accessors
  ConfigParams getConfig() {return data;};;

  /**
   * Reads in a network config file and parses it
   * Arguments:
   *  filePath(INPUT) - path to file with network configuration
   * Return:
   *  0 if successful
   */
  int readConfig(const char* filePath);

private:
  int currentLine; // line number of config file being processed
  ConfigParams data;

  int parseLine(const string& line);
  int extractHost(const string& line, string& host);
  int extractPort(const string& line, int& port);
  int extractType(const string& line, int& type);

  /**
   * Maps a string containing the network type to its index
   * Arguments:
   *  type(INPUT) - string holding network type (e.g. "TCP")
   * Return:
   *  index representing network type; returns -1 if unsuccessful
   */
  int str2type(const string& str);
};

// Structure for holding a single marker data
struct MarkerData{
  int id;

  // Rotation Matrix
  double rm[3][3];
  double dist_x;
  double dist_y;
  double dist_z;

};

// Structure for holding all marker data
struct MarkerDataSet{
  MarkerDataSet();
  MarkerDataSet(const MarkerDataSet& other);
  ~MarkerDataSet();
  int num;
  MarkerData** array; // array of pointers to markers

  MarkerData getMarker(int id) {return *(array[id]);};;

  void deleteMarkers();

  MarkerDataSet& operator=(const MarkerDataSet& rhs);

};

class MarkerFileParser {
public:
  MarkerFileParser();
  ~MarkerFileParser();

  // Accessors
  MarkerDataSet getMarkers() {return data;};
  
  /**
   * Reads in a marker data file and parses it
   * Arguments:
   *  markers(INPUT) - path to file with marker data
   * Return:
   *  0 if successful
   */
  int readMarkers(const char* markers);

private:
  int currentLine; // line number of marker data file being processed
  int lineCount; // line number of marker data file being processed excluding comments, whitespace and invalid lines
  int currentMarker; // the N-th marker being processed
  int index; // 0~5, 0~2 specify rotation matrix and 3~5 specify distance
  MarkerDataSet data; // parsed data

  /**
   * Parses a single line of from a marker data file
   * Arguments:
   *  line(INPUT) - a line from the marker data file
   * Return:
   *  0 if successful
   */
  int parseLine(const string& line);
  int parseMarkerNum(const string& line);
  int parseRow(const string& line);
  int parseDistance(const string& line);

};

/**
 * Initialization helper for server/client
 * Arguments:
 *  server/client(OUTPUT) - server/client to be initialized; NULL if uninitialized
 *  params(INPUT) - ConfigParams structure containing network configuration
 * Return:
 *  0 if successful, 1 if an error occurred
 */
int serverInit(Server*& server, ConfigParams* params);
int clientInit(Client*& client, ConfigParams* params);

#endif //END_IF_PARSERS_H