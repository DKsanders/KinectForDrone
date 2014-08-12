/**
 * This file (network.cpp) implements functions for reading and
 * writing from/to byte streams, declared in network.h
 *
 * Author: David Sanders <david.sanders@mail.utoronto.ca>
 */

#include "network/network.h"
#include <endian.h>
#include <stdint.h>

ByteStream::ByteStream() {
	bufSize = DEFAULT_BUF_SIZE;
	buf = new char[bufSize];
	writeIndex = 0;
	readIndex = 0;
}

ByteStream::ByteStream(int _bufSize) {
	bufSize = _bufSize;
	buf = new char[bufSize];
	writeIndex = 0;
	readIndex = 0;
}

ByteStream::ByteStream(const char* data, int dataSize) {
	bufSize = DEFAULT_BUF_SIZE;
	buf = new char[bufSize];
	writeIndex = dataSize;
	readIndex = 0;
	memcpy(buf, data, dataSize);
}

ByteStream::ByteStream(int _bufSize, const char* data, int dataSize) {
	bufSize = _bufSize;
	buf = new char[bufSize];
	writeIndex = dataSize;
	readIndex = 0;
	memcpy(buf, data, dataSize);
}

ByteStream::~ByteStream(){
	delete [] buf;
}

int ByteStream::getWriteIndex(){
	return writeIndex;
}

int ByteStream::getReadIndex(){
	return readIndex;
}

int ByteStream::getBufSize(){
	return bufSize;
}

char* ByteStream::getBuf(){
	return buf;
}

int ByteStream::write(int data) {
	// Check size (should be 4 bytes)
	int size = sizeof(int);
	if((writeIndex + size) > bufSize){
		// Not enough room in buffer
		return 0;
	}
	if(size == 4){
		// As expected
		uint32_t bytes = htole32(*(uint32_t*)(&data));
		memcpy(buf+writeIndex, &bytes, size);
	} else {
		// not standard size, don't write
		return 0;
	}
	writeIndex += size;
	return size;
}

int ByteStream::write(float data) {
	// Check size (should be 4 bytes)
	int size = sizeof(float);
	if((writeIndex + size) > bufSize){
		// Not enough room in buffer
		return 0;
	}
	if(size == 4){
		// As expected
		uint32_t bytes = htole32(*(uint32_t*)(&data));
		memcpy(buf+writeIndex, &bytes, size);
	} else {
		// not standard size, don't write
		return 0;
	}
	writeIndex += size;
	return size;
}

int ByteStream::write(double data) {
	// Check size (should be 8 bytes)
	int size = sizeof(double);
	if((writeIndex + size) > bufSize){
		// Not enough room in buffer
		return 0;
	}
	if(size == 8){
		// As expected
		uint64_t bytes = htole64(*(uint64_t*)(&data));
		memcpy(buf+writeIndex, &bytes, size);
	} else {
		// not standard size, don't write
		return 0;
	}
	writeIndex += size;
	return size;
}

int ByteStream::write(const string& data) {
	return this->write(data.c_str());
}

int ByteStream::write(const char* data) {
	// Write an entire string (including "\0")
	int size = strlen(data)+1;
	if((writeIndex + size) > bufSize){
		// Not enough room in buffer
		return 0;
	}
	return this->write(data, size);
}

int ByteStream::write(const char* data, int length) {
	// Write specified length
	if((writeIndex + length) > bufSize){
		// Not enough room in buffer
		return 0;
	}
	memcpy(buf+writeIndex, data, length);
	writeIndex += length;
	return length;
}

int ByteStream::read(int& data){
	// Check size (should be 4 bytes)
	int size = sizeof(int);
	if((readIndex + size) > bufSize){
		// Not enough room in buffer
		return 0;
	}
	if(size == 4){
		// As expected
		uint32_t temp = le32toh(*(uint32_t*)(buf+readIndex));
		data = *((int*)(&temp));
	} else {
		// not standard size, don't write
		return 0;
	}
	readIndex += size;
	return size;
}

int ByteStream::read(float& data){
	// Check size (should be 4 bytes)
	int size = sizeof(float);
	if((readIndex + size) > bufSize){
		// Not enough room in buffer
		return 0;
	}
	if(size == 4){
		// As expected
		uint32_t temp = le32toh(*(uint32_t*)(buf+readIndex));
		data = *((float*)(&temp));
	} else {
		// not standard size, don't write
		return 0;
	}
	readIndex += size;
	return size;
}

int ByteStream::read(double& data){
	// Check size (should be 4 bytes)
	int size = sizeof(double);
	if((readIndex + size) > bufSize){
		// Not enough room in buffer
		return 0;
	}
	if(size == 8){
		// As expected
		uint64_t temp = le64toh(*(uint64_t*)(buf+readIndex));
		data = *((double*)(&temp));
	} else {
		// not standard size, don't write
		return 0;
	}
	readIndex += size;
	return size;
}

int ByteStream::read(string& data){
	// Read all chars until '\0'
	string temp(buf+readIndex);
	data = temp;
	int size = data.length()+1;
	readIndex += size;
	return size;
}

int ByteStream::read(char* data, int length){
	if((readIndex + length) > bufSize){
		// Not enough room in buffer
		return 0;
	}
	memcpy(data, buf+readIndex, length);
	readIndex += length;
	return length;
}

void ByteStream::clear(){
	this->clear(DEFAULT_BUF_SIZE);
}

void ByteStream::clear(int _bufSize){
	delete [] buf;
	bufSize = _bufSize;
	buf = new char[bufSize];
	writeIndex = 0;
	readIndex = 0;
}

int write2stream(char* buf, int data){
	// Check size (should be 4 bytes)
	int size = sizeof(int);
	if(size == 4){
		// As expected
		uint32_t bytes = htole32(*(uint32_t*)(&data));
		memcpy(buf, &bytes, size);
	} else if (size == 8){
		// int is size 8
    	uint64_t bytes = htole64(*(uint64_t*)(&data));
		memcpy(buf, &bytes, size);
	} else {
		// not standard size, don't write
		return 0;
	}
	return size;
}

int write2stream(char* buf, float data) {
	// Check size (should be 4 bytes)
	int size = sizeof(float);
	if(size == 4){
		// As expected
		uint32_t bytes = htole32(*(uint32_t*)(&data));
		memcpy(buf, &bytes, size);
	} else if (size == 8){
		// float is size 8
    	uint64_t bytes = htole64(*(uint64_t*)(&data));
		memcpy(buf, &bytes, size);
	} else {
		// not standard size, don't write
		return 0;
	}
	return size;
}

int write2stream(char* buf, double data) {
	// Check size (should be 8 bytes)
	int size = sizeof(double);
	if (size == 8){
		// As expected
    	uint64_t bytes = htole64(*(uint64_t*)(&data));
		memcpy(buf, &bytes, size);
	} else if(size == 4){
		// double has size 4...?
		uint32_t bytes = htole32(*(uint32_t*)(&data));
		memcpy(buf, &bytes, size);
	} else {
		// not standard size, don't write
		return 0;
	}
	return size;
}

int write2stream(char* buf, const char* data) {
	// Write an entire string (including "\0")
	return write2stream(buf, data, strlen(data)+1);
}

int write2stream(char* buf, const char* data, int length) {
	// Write specified length
	memcpy(buf, data, length);
	return length;
}

int readFromStream(char* buf, int& data){
	// Check size (should be 4 bytes)
	int size = sizeof(int);
	if(size == 4){
		// As expected
		uint32_t temp = le32toh(*(uint32_t*)buf);
		data = *((int*)(&temp));
	} else if (size == 8){
		// Int is 8 bytes
		uint64_t temp = le64toh(*(uint64_t*)buf);
		data = *((int*)(&temp));
	} else {
		// not standard size, don't write
		return 0;
	}
	return size;
}

int readFromStream(char* buf, float& data){
	// Check size (should be 4 bytes)
	int size = sizeof(float);
	if(size == 4){
		// As expected
		uint32_t temp = le32toh(*(uint32_t*)buf);
		data = *((float*)(&temp));
	} else if (size == 8){
		// Int is 8 bytes
		uint64_t temp = le64toh(*(uint64_t*)buf);
		data = *((float*)(&temp));
	} else {
		// not standard size, don't write
		return 0;
	}
	return size;
}

int readFromStream(char* buf, double& data){
	// Check size (should be 8 bytes)
	int size = sizeof(double);
	if (size == 8){
		// As expected
		uint64_t temp = le64toh(*(uint64_t*)buf);
		data = *((double*)(&temp));
	} else if(size == 4){
		// Double is 4 bytes...?
		uint32_t temp = le32toh(*(uint32_t*)buf);
		data = *((double*)(&temp));
	} else {
		// not standard size, don't write
		return 0;
	}
	return size;
}

int readFromStream(char* buf, string& data){
	// Read all chars until '\0'
	string temp(buf);
	data = temp;
	return (strlen(buf)+1);
}