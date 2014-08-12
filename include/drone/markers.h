/**
 * This file (markers.h) provides functions for reading and using 
 * marker data, implemented in markers.cpp
 *
 * Author: David Sanders <david.sanders@mail.utoronto.ca>
 *
 /****** MARKER FILE EXAMPLE ******/
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

#ifndef MARKERS_H
#define MARKERS_H

#include <string>
using namespace std;

#define COMMENT_CHAR '#'

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
  ~MarkerDataSet();
  int num;
  struct MarkerData** array; // array of pointers to markers

  struct MarkerData* getMarker(int id);

};

class MarkerFileParser {
public:
  MarkerFileParser();
  ~MarkerFileParser();

  // Accessors
  MarkerDataSet* getMarkers();
  
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
  MarkerDataSet* data; // parsed data

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

#endif //END_IF_MARKERS_H