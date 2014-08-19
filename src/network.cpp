/**
 * This file (network.cpp) implements functions for reading and
 * writing from/to byte streams, declared in network.h
 *
 * Author: David Sanders <david.sanders@mail.utoronto.ca>
 */

#include "drone/network.h"
#include <arpa/inet.h>
#include <netdb.h>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <endian.h>
#include <stdint.h>
#include <errno.h>

using namespace std;

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

/********************************** TCP *********************************/
// SERVER
Server_TCP::Server_TCP(){

}

Server_TCP::Server_TCP(const char* host, const int port){
    init(host, port);
}

Server_TCP::~Server_TCP(){

}

int Server_TCP::init(const char* host, const int port){
    // Close connection if alread connected
    closeConn();

    // Create a socket
    listeningSock = socket(AF_INET, SOCK_STREAM, 0); // PF_INET instead?
    if(listeningSock < 0){
        perror("Socket creation unsuccessful");
        return 1;
    }
  
    // Allow listening port to be reused if defunct.
    int yes = 1;
    int status = setsockopt(listeningSock, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof yes);
    if (status != 0) {
        // error configuring socket
        perror("Socket configuration fail");
        return 1;
    }

    // Bind it to the listening port.
    memset(&serverAddr, 0, sizeof serverAddr);
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);
    inet_pton(AF_INET, host, &(serverAddr.sin_addr));
    status = ::bind(listeningSock, (struct sockaddr*) &serverAddr, sizeof serverAddr);
    if (status != 0) {
        // error binding socket
        perror("bind() fail");
        return 1;
    }

    // Listen for connections
    status = 0;
    status = ::listen(listeningSock, MAX_LISTENQUEUELEN);
    if (status != 0) {
        // error listening to socket
        perror("listen() fail");
        return 1;
    }
    return 0;
}

int Server_TCP::accept(){

  socklen_t clientAddrLen = sizeof clientAddr;
  clientSock = ::accept(listeningSock, (struct sockaddr*)&clientAddr, &clientAddrLen);
  if (clientSock < 0) {
    // error accepting connection
    perror("accept() fail");
    return 1;
  }
  return 0;
}


// Sends a message
int Server_TCP::send(ByteStream& stream){
    // Add message size to head of stream
    int status = 0;
    ByteStream send;
    send.write(stream.getWriteIndex()); // msg size
    send = send + stream; // actual msg

    size_t tosend = stream.getWriteIndex() + sizeof(int); // bytes needed to be sent

    // Start sending
    char* head = send.getBuf();
    while (tosend > 0){
        ssize_t bytes = ::send(clientSock, head, tosend, 0);
        if (bytes <= 0) {
            // send() was not successful
            perror("Sending unsuccessful");
            return 1;
        }
        tosend -= (size_t) bytes;
        head += bytes;
        stream.setWriteIndex(stream.getWriteIndex()+bytes);
    }
    return 0;
}

int Server_TCP::receive(){
    // Initialze
    ByteStream sizeStream((int)(sizeof(int))); // for storing message size
    int msgSize;
    ssize_t bytes;
    
    // Read message size
    bytes = recv(clientSock, sizeStream.getBuf(), sizeof(int), 0);
    if(bytes != sizeof(int)){
        perror("Unable to read all bytes for message size");
        return 1;
    }
    sizeStream.setWriteIndex(sizeof(int));
    sizeStream.read(msgSize);
    
    // Done reading message size; Read one byte from scoket
    clearBuf(); // clear buffer
    while (msgSize > 0){
        bytes = recv(clientSock, getBuf(), msgSize, 0);
        if (bytes <= 0) {
            // recv() was not successful; so stop
            perror("recv() unsuccessful");
            return 1;
        }
        msgSize = msgSize - bytes;
        stream.setWriteIndex(stream.getWriteIndex()+bytes);
    }
    return 0;
}

// CLIENT
Client_TCP::Client_TCP(){
    
}

Client_TCP::Client_TCP(const char* host, const int port){
    init(host, port);
}

Client_TCP::~Client_TCP(){
    
}

int Client_TCP::init(const char* host, const int port){
    // Close connection if alread connected
    closeConn();

    // Declare vriables
    int status;
    struct addrinfo serverAddr, *res;

    // Create a socket
    sockfd = socket(AF_INET,SOCK_STREAM,0); // PF_INET instead?
    if(sockfd < 0){
        // failed
        perror("Socket creation unsuccessful");
        return 1;
    }

    // Obtain address info
    memset(&serverAddr, 0, sizeof serverAddr);
    serverAddr.ai_family = AF_INET;
    serverAddr.ai_socktype = SOCK_STREAM;
    string temp;
    stringstream ss;
    ss << port;
    ss >> temp;
    const char* portStr = temp.c_str();
    status = getaddrinfo(host, portStr, &serverAddr, &res);
    if (status != 0){
        // unable to get address info
        perror("getaddrinfo() unsuccessful");
        return 1;
    }

    do {
        status = connect(sockfd, res->ai_addr, res->ai_addrlen);
        //cout << res->ai_addr << endl;
        if (status != 0){
            // unable to connect to server
            perror("connect() unsuccessful");
            sleep(CONNECTION_WAIT_TIME);
        }
    } while ( status != 0);
    
    return 0;
}

int Client_TCP::send(ByteStream& stream){
    // Add message size to head of stream
    int status = 0;
    ByteStream send;
    send.write(stream.getWriteIndex()); // msg size
    send = send + stream; // actual msg

    size_t tosend = stream.getWriteIndex() + sizeof(int); // bytes needed to be sent

    // Start sending
    char* head = send.getBuf();
    while (tosend > 0){
        ssize_t bytes = ::send(sockfd, head, tosend, 0);
        if (bytes <= 0) {
            // send() was not successful
            perror("Sending unsuccessful");
            return 1;
        }
        tosend -= (size_t) bytes;
        head += bytes;
    }
    return 0;
}

int Client_TCP::receive(){
    // Initialze
    ByteStream sizeStream((int)(sizeof(int))); // for storing message size
    int msgSize;
    ssize_t bytes;
    
    // Read message size
    bytes = recv(sockfd, sizeStream.getBuf(), sizeof(int), 0);
    if(bytes != sizeof(int)){
        perror("Unable to read all bytes for message size");
        return 1;
    }
    sizeStream.setWriteIndex(sizeof(int));
    sizeStream.read(msgSize);
    
    // Done reading message size; Read one byte from scoket
    clearBuf(); // clear buffer
    while (msgSize > 0){
        bytes = recv(sockfd, getBuf(), msgSize, 0);
        if (bytes <= 0) {
            // recv() was not successful; so stop
            perror("recv() unsuccessful");
            return 1;
        }
        msgSize = msgSize - bytes;
        stream.setWriteIndex(stream.getWriteIndex()+bytes);
    }
    return 0;
}


/********************************** UDP *********************************/

// SERVER
Server_UDP::Server_UDP(){

}

Server_UDP::Server_UDP(const char* host, const int port){
    init(host, port);
}

Server_UDP::~Server_UDP(){

}

int Server_UDP::init(const char* host, const int port){
    // close connection if alread connected
    closeConn();
    
    int status;
    clientSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP); // 0 to IPPROTO_UDP
    if(clientSock < 0){
        perror("Socket creation unsuccessful");
        return 1;
    }
    /*
    if(blocking==0) {
        // Set socket to be non-blocking.  All of the sockets for
        // the incoming connections will also be non-blocking since
        // they will inherit that state from the listening socket.
        int x;
        x = fcntl(listeningSock, F_GETFL, 0);
        fcntl(listeningSock, F_SETFL, x | O_NONBLOCK);
    }
    */
    //Bind the socket
    memset((char *) &listenAddr, 0, sizeof(listenAddr));
    listenAddr.sin_family = AF_INET;
    listenAddr.sin_port = htons(port);
    listenAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    status = ::bind(clientSock, (const sockaddr*)&listenAddr, sizeof(listenAddr));
    if(status != 0){
        perror("bind() unsuccessful");
        return 3;
    }
    return 0;
}

int Server_UDP::accept(){
    // no accepting
    return 0;
}

int Server_UDP::send(ByteStream& stream){
    socklen_t slen = sizeof(clientAddr);
    ssize_t sent = sendto(clientSock, stream.getBuf(), stream.getBufSize(), 0, (const sockaddr*)&clientAddr, slen);
    if (sent < 0){
        perror("Sending unsuccessful");
        return 1;
    }
    return 0;
}

int Server_UDP::receive(){
    // Clear buffer
    clearBuf();
    ssize_t bytes;
    socklen_t slen = sizeof(clientAddr);

    // Read actual msg
    bytes = recvfrom(clientSock, getBuf(), getBufSize(), 0, (sockaddr*)&clientAddr, &slen);
    stream.setWriteIndex(getBufSize());
    if(bytes < 0){
        perror("Receiving unsuccessful");
        return 1;
    }
    return 0;
}


// CLIENT
Client_UDP::Client_UDP(){
    
}

Client_UDP::Client_UDP(const char* host, const int port){
    init(host, port);
}

Client_UDP::~Client_UDP(){
    
}

int Client_UDP::init(const char* host, const int port){
    // close connection if alread connected
    closeConn();
    
    sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if(sockfd < 0){
        // failed
        perror("Socket creation unsuccessful");
        return 1;
    }
    
    memset((char *) &serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);
    if (inet_aton(host, &serverAddr.sin_addr)==0){
        perror("Host decoding unsuccessful");
        return 2;
    }
    return 0;
}

int Client_UDP::send(ByteStream& stream){

    socklen_t slen = sizeof(serverAddr);
    ssize_t sent = sendto(sockfd, stream.getBuf(), stream.getBufSize(), 0, (const sockaddr*)&serverAddr, slen);
    if (sent < 0){
        perror("Sending unsuccessful");
        return 1;
    }
    return 0;
}

int Client_UDP::receive(){
    // Clear buffer
    clearBuf();
    ssize_t bytes;
    socklen_t slen = sizeof(serverAddr);

    // Read actual msg
    bytes = recvfrom(sockfd, getBuf(), getBufSize(), 0, (sockaddr*)&serverAddr, &slen);
    stream.setWriteIndex(getBufSize());
    if(bytes < 0){
        perror("Receiving unsuccessful");
        return 1;
    }
    return 0;
}