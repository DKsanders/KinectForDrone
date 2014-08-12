/**
 * This file (markers.h) provides functions for reading and using 
 * marker data, implemented in markers.cpp
 *
 * Author: David Sanders <david.sanders@mail.utoronto.ca>
 */

#include "drone/markers.h"
#include <iostream>
#include <sstream>
#include <fstream>
using namespace std;

MarkerDataSet::~MarkerDataSet(){
    int i;
    for(i=0; i<num; i++){
        delete array[i];
    }
    delete [] array;
}

MarkerData* MarkerDataSet::getMarker(int id){
    return array[id];
}

MarkerFileParser::MarkerFileParser(){
	currentLine = 1;
	lineCount = 0;
	data = NULL;
}

MarkerFileParser::~MarkerFileParser(){
	;
}

MarkerDataSet* MarkerFileParser::getMarkers(){
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
		status = parseLine(line);
		currentLine += 1;
		if(status != 0){
			validConfig = 0;
		}
	}
	if(!validConfig){
        cout << "Error reading marker file" << endl;
		return 1;
	}
	int correctCount = data->num * 6 + 1;
	if(lineCount < correctCount){
        cout << "Not enough data for " << data->num << " markers" << endl;
        return 1;
	}
	if(lineCount > correctCount){
        cout << "Too much data for " << data->num << " markers" << endl;
        return 1;
	}
	cout << "Successfully parsed marker file" << endl;
	file.close();
	return 0;
}

int MarkerFileParser::parseLine(const string& line){
	// Check if line should be ignored
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
	data = new MarkerDataSet;
	stringstream ss(line);
	ss >> data->num;
	if(!ss.eof() || data->num < 1){
		// invalid line
		return 1;
	}
	data->array = new MarkerData*[data->num];
	return 0;
}

int MarkerFileParser::parseRow(const string& line){
	// Initialize
	stringstream ss(line);
	if(index == 0){
		data->array[currentMarker] = new MarkerData;
	}
	ss >> data->array[currentMarker]->rm[index][0];
	if(ss.fail()){
		return 1;
	}
	ss >> data->array[currentMarker]->rm[index][1];
	if(ss.fail()){
		return 1;
	}
	ss >> data->array[currentMarker]->rm[index][2];
	if(!ss.eof()){
		return 1;
	}
	return 0;

}

int MarkerFileParser::parseDistance(const string& line){
	// Initialize
	stringstream ss(line);
	if(index == 3){
		ss >> data->array[currentMarker]->dist_x;
	} else if(index == 4){
		ss >> data->array[currentMarker]->dist_y;
	} else if(index == 5){
		ss >> data->array[currentMarker]->dist_z;
	}
	if(!ss.eof()){
		return 1;
	}
	return 0;
}
