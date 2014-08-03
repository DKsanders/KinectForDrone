#ifndef NETWORK_H
#define NETWORK_H

#include <unistd.h>
#include <string.h>
#include <iostream>
#include <netinet/in.h>

#define MAX_BUF_SIZE 1024
#define CONNECTION_WAIT_TIME 5 // Time between connection attempts made by client

using namespace std;

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
    
    int init(string& host, const int port) {init(host.c_str(), port);};
    virtual int init(const char* host, const int port) = 0;
    virtual int listen() = 0;
    int send(const string& msg) { return send(msg.c_str()); };
    virtual int send(const char* msg) {send(msg, strlen(msg));};
    virtual int send(const char* msg, const size_t length) = 0;
    virtual int receive() = 0;
    void closeConn(){
    	if(clientSock != 0){
    		close(clientSock);
    	}
    	if(listeningSock != 0){
    		close(listeningSock);
    	}
    };
    int getListeningSock(){ return listeningSock; };
    int getClientSock(){ return clientSock; };
    char buf[MAX_BUF_SIZE]; // buffer for recieving messages

protected:
	int listeningSock;
    int clientSock;
    struct sockaddr_in clientAddr;
    struct sockaddr_in serverAddr;
    
} Server;

typedef class Client{
public:
	Client() {
		sockfd = 0;
		memset(buf, 0, sizeof buf);
	}
	virtual ~Client(){
		closeConn();
	};
    
    int init(string& host, const int port) {init(host.c_str(), port);};
    virtual int init(const char* host, const int port) = 0;
    int send(const string& msg) { return send(msg.c_str()); };
    virtual int send(const char* msg) {send(msg, strlen(msg));};
    virtual int send(const char* msg, const size_t length) = 0;
    virtual int receive() = 0;
    void closeConn(){
    	if(sockfd != 0){
    		close(sockfd);
    	}
    };
    int getSockfd(){ return sockfd; };
    
protected:
	int sockfd; // socket connection
    char buf[MAX_BUF_SIZE]; // buffer for recieving messages
    
} Client;

// Write data to a byte stream in LITTLE ENDIAN; returns the number of bytes written
int write2stream(char* buf, int data);
int write2stream(char* buf, float data);
int write2stream(char* buf, double data);
int write2stream(char* buf, const char* data, int length);
int write2stream(char* buf, const char* data);


// Reads data in LITTLE ENDIAN from a byte stream; returns the number of bytes read
int readFromStream(char* buf, int& data);
int readFromStream(char* buf, float& data);
int readFromStream(char* buf, double& data);
int readFromStream(char* buf, string& data);

#endif //END_IF_NETWORK_H