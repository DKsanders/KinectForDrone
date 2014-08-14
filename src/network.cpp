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
    memset(buf, 0, bufSize);
	writeIndex = 0;
	readIndex = 0;
}

ByteStream::ByteStream(const ByteStream& rhs) {
	writeIndex = rhs.writeIndex;
	readIndex = rhs.readIndex;
	bufSize = rhs.bufSize;
	buf = new char[bufSize];
	memcpy(buf, rhs.buf, bufSize);
}

ByteStream::ByteStream(int _bufSize) {
	bufSize = _bufSize;
	buf = new char[bufSize];
    memset(buf, 0, bufSize);
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

void ByteStream::setWriteIndex(int _writeIndex){
	writeIndex = _writeIndex;
}

void ByteStream::setReadIndex(int _readIndex){
	readIndex = _readIndex;
}

void ByteStream::setBufSize(int _bufSize){
	delete [] buf;
	bufSize = _bufSize;
	buf = new char[bufSize];
    memset(buf, 0, bufSize);
	writeIndex = 0;
	readIndex = 0;
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
	if((readIndex + size) > writeIndex){
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
	if((readIndex + size) > writeIndex){
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
	if((readIndex + size) > writeIndex){
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
    memset(buf, 0, bufSize);
	writeIndex = 0;
	readIndex = 0;
}	

ByteStream& ByteStream::operator=(const ByteStream& rhs) {
	writeIndex = rhs.writeIndex;
	readIndex = rhs.readIndex;
	bufSize = rhs.bufSize;
	buf = new char[bufSize];
	memcpy(buf, rhs.buf, bufSize);
	return *this;
}

ByteStream ByteStream::operator+(const ByteStream& rhs) {
	int total = this->writeIndex - this->readIndex + rhs.writeIndex - rhs.readIndex;
	int bufSize;
	if(total > DEFAULT_BUF_SIZE){
		// Buffer needs to be replaced
		bufSize = total;
	} else {
		bufSize = DEFAULT_BUF_SIZE;
	}
	ByteStream rtn(bufSize);
	rtn.writeIndex = total;
	rtn.readIndex = 0;
	memcpy(rtn.buf, this->buf+this->readIndex, this->writeIndex-this->readIndex);
	memcpy(rtn.buf+this->writeIndex-this->readIndex, rhs.buf+rhs.readIndex, rhs.writeIndex-rhs.readIndex);
	return rtn;
}