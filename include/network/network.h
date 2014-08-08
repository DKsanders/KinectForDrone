/**
 * This file (network.h) provides the skeleton structures for
 * implementing servers and clients.
 *
 * Functions for reading and writing from/to byte streams are
 * also declared here, which are implemented in network.cpp
 *
 * Author: David Sanders <david.sanders@mail.utoronto.ca>
 */

#ifndef NETWORK_H
#define NETWORK_H

#include <unistd.h>
#include <string>
#include <netinet/in.h>
#include <cstring>

#define MAX_BUF_SIZE 1024
#define CONNECTION_WAIT_TIME 5 // Time between connection attempts made by client

using namespace std;

// Base of server
typedef class Server{
public:
	Server() {
		listeningSock = 0;
		clientSock = 0;
		memset(buf, 0, sizeof buf);
	}
	virtual ~Server(){
		closeConn();
	};
    
    // Initializers
    int init(string& host, const int port) {return init(host.c_str(), port);};
    virtual int init(const char* host, const int port) = 0;
    
    // For accepting connectinos
    virtual int accept() = 0;
    
    // Sending
    int send(const string& msg) { return send(msg.c_str()); };
    virtual int send(const char* msg) {return send(msg, strlen(msg));};
    virtual int send(const char* msg, const size_t length) = 0;
    
    // Receiving
    virtual int receive() = 0;
    
    // Closing connection
    void closeConn(){
    	if(clientSock != 0){
    		close(clientSock);
    	}
    	if(listeningSock != 0){
    		close(listeningSock);
    	}
    };

    // 
    int getListeningSock(){ return listeningSock; };
    int getClientSock(){ return clientSock; };
    
    // Buffer for receiving data
    char buf[MAX_BUF_SIZE]; // buffer for recieving messages

protected:
    // Variables
	int listeningSock;
    int clientSock;
    struct sockaddr_in clientAddr;
    struct sockaddr_in serverAddr;
    
} Server;

// Base of Client
typedef class Client{
public:
	Client() {
		sockfd = 0;
		memset(buf, 0, sizeof buf);
	}
	virtual ~Client(){
		closeConn();
	};

    // Initializers
    int init(string& host, const int port) {return init(host.c_str(), port);};
    virtual int init(const char* host, const int port) = 0;
    
    // Sending
    int send(const string& msg) { return send(msg.c_str()); };
    virtual int send(const char* msg) {return send(msg, strlen(msg));};
    virtual int send(const char* msg, const size_t length) = 0;
    
    // Receiving
    virtual int receive() = 0;
    
    // Closing Connections
    void closeConn(){
    	if(sockfd != 0){
    		close(sockfd);
    	}
    };

    // 
    int getSockfd(){ return sockfd; };
    
    // Buffer for receiving data
    char buf[MAX_BUF_SIZE]; // buffer for recieving messages
    
protected:
    // Variables
	int sockfd; // socket connection
    
} Client;

/**
 * Writes data to a byte stream in LITTLE ENDIAN
 * Arguments:
 *  buf(OUTPUT) - byte stream
 *  data(INPUT) - data written to byte stream
 * Return:
 *  number of bytes written to byte stream
 */
int write2stream(char* buf, int data);
int write2stream(char* buf, float data);
int write2stream(char* buf, double data);
int write2stream(char* buf, const char* data); // Copies every character until '\0' copied
int write2stream(char* buf, const char* data, int length); // Copies the number of characters specified by length


/**
 * Reads data from a byte stream in LITTLE ENDIAN
 * Arguments:
 *  buf(INPUT) - byte stream
 *  data(OUTPUT) - data written to byte stream
 * Return:
 *  number of bytes read from byte stream
 */
int readFromStream(char* buf, int& data);
int readFromStream(char* buf, float& data);
int readFromStream(char* buf, double& data);
int readFromStream(char* buf, string& data);

#endif //END_IF_NETWORK_H