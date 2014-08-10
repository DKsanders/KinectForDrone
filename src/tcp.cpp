/**
 * This file (tcp.cpp) implements interfaces for a server and a client
 * that uses TCP, declared in tcp.h
 *
 * Interface is based off of the skeleton declared in network.h
 *
 * Author: David Sanders <david.sanders@mail.utoronto.ca>
 */

#include "network/tcp.h"
#include <arpa/inet.h>
#include <netdb.h>
#include <sstream>
#include <errno.h>
#include <stdio.h>

using namespace std;

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
int Server_TCP::send(const char* msg, const size_t length){
    // Create a message; number of bytes in stream indicated at the head of buffer
    char* totalMsg = new char[(sizeof(char)*(length+sizeof(int)))]; // buffer
    char* current = totalMsg;
    current += write2stream(current, (int)length); // msg size
    current += write2stream(current, msg, (int)length); // actual msg
    size_t tosend = length + sizeof(int); // bytes needed to be sent

    // Start sending
    while (tosend > 0){
        ssize_t bytes = ::send(clientSock, totalMsg, tosend, 0);
        if (bytes <= 0) {
            // send() was not successful
            perror("sending unsuccessful");
            delete [] totalMsg;
            return 1;
        }
        tosend -= (size_t) bytes;
        msg += bytes;
    }
    delete [] totalMsg;
    return 0;
}

int Server_TCP::receive(){
    // For storing msg size
    char* msgSizeBuf = new char[(sizeof(char)*sizeof(int))];
    int msgSize;
    
    // Clear buffer
    memset(buf, 0, sizeof buf);
    char* _buf = (char*) buf;
    ssize_t bytes;
    
    // Read message size
    bytes = recv(clientSock, msgSizeBuf, sizeof(int), 0);
    if(bytes != sizeof(int)){
        perror("Unable to read all bytes for message size");
        return 1;
    }
    readFromStream(msgSizeBuf, msgSize);
    
    // Done reading message size; Read one byte from scoket
    while (msgSize > 0){
        bytes = recv(clientSock, _buf, msgSize, 0);
        if (bytes <= 0) {
            // recv() was not successful; so stop
            perror("recv() unsuccessful");
            return 1;
        }
        msgSize = msgSize - bytes;
    }
    delete [] msgSizeBuf;
    return 0;
}

/***************************************************************************************************/

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

int Client_TCP::send(const char* msg, const size_t length){
    // Create a message; number of bytes in stream indicated at the head of buffer
    char* totalMsg = new char[(sizeof(char)*(length+sizeof(int)))]; // buffer
    char* current = totalMsg;
    current += write2stream(current, (int)length); // msg size
    current += write2stream(current, msg, (int)length); // actual msg
    size_t tosend = length + sizeof(int); // bytes needed to be sent

    // Start sending
    while (tosend > 0){
        ssize_t bytes = ::send(sockfd, totalMsg, tosend, 0);
        if (bytes <= 0) {
            // send() was not successful
            perror("sending unsuccessful");
            delete [] totalMsg;
            return 1;
        }
        tosend -= (size_t) bytes;
        msg += bytes;
    }
    delete [] totalMsg;
    return 0;
}

int Client_TCP::receive(){
    // For storing msg size
    char* msgSizeBuf = new char[(sizeof(char)*sizeof(int))];
    int msgSize;
    
    // Clear buffer
    memset(buf, 0, sizeof buf);
    char* _buf = (char*) buf;
    ssize_t bytes;
    
    // Read message size
    bytes = recv(sockfd, msgSizeBuf, sizeof(int), 0);
    if(bytes != sizeof(int)){
        perror("Unable to read all bytes for message size");
        return 1;
    }
    readFromStream(msgSizeBuf, msgSize);
    
    // Done reading message size; Read one byte from scoket
    while (msgSize > 0){
        bytes = recv(sockfd, _buf, msgSize, 0);
        if (bytes <= 0) {
            // recv() was not successful; so stop
            perror("recv() unsuccessful");
            return 1;
        }
        msgSize = msgSize - bytes;
    }
    delete [] msgSizeBuf;
    return 0;
}