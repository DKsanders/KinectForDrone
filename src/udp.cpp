/**
 * This file (udp.cpp) implements interfaces for a server and a client
 * that uses UDP, declared in udp.h
 *
 * Interface is based off of the skeleton declared in network.h
 *
 * Author: David Sanders <david.sanders@mail.utoronto.ca>
 */

#include "network/udp.h"
#include <arpa/inet.h>
#include <errno.h>
#include <stdio.h>

using namespace std;

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

int Server_UDP::send(const char* msg, const size_t length){
    return 0;
}

int Server_UDP::receive(){
    // Clear buffer
    memset(buf, 0, sizeof buf);
    ssize_t bytes;
    socklen_t slen = sizeof(clientAddr);

    // Read actual msg
    bytes = recvfrom(clientSock, buf, sizeof buf, 0, (sockaddr*)&clientAddr, &slen);
    if(bytes < 0){
        perror("Receiving unsuccessful");
        return 1;
    }
    return 0;
}

/***************************************************************************************************/

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

int Client_UDP::send(const char* msg, const size_t length){

    socklen_t slen = sizeof(serverAddr);
    ssize_t sent = sendto(sockfd, msg, length, 0, (const sockaddr*)&serverAddr, slen);
    if (sent < 0){
        perror("Sending unsuccessful");
        return 1;
    }
    return 0;
}

int Client_UDP::receive(){
    return 0;
}