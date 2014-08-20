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

#define DEFAULT_BUF_SIZE 1024
#define CONNECTION_WAIT_TIME 5 // Time between connection attempts made by client
#define MAX_LISTENQUEUELEN 1

using namespace std;

// Byte Stream for easy reading/writing
class ByteStream {
public:
    ByteStream();
    ByteStream(const ByteStream& other);
    ByteStream(int _bufSize);
    ByteStream(const char* data, int dataSize);
    ByteStream(int _bufSize, const char* data, int dataSize);
    ~ByteStream();

    // Accessors
    int getWriteIndex() {return writeIndex;};
    int getReadIndex() {return readIndex;};
    int getBufSize() {return bufSize;};
    char* getBufHead() {return buf;};

    // Mutators
    void setWriteIndex(int _writeIndex);
    void setReadIndex(int _readIndex);
    void setBufSize(int _bufSize);

    /**
     * Writes data to a byte stream in LITTLE ENDIAN
     * Arguments:
     *  data(INPUT) - data written to byte stream
     * Return:
     *  number of bytes written to byte stream; 0 if failed
     */
    int write(int data);
    int write(float data);
    int write(double data);
    int write(const string& data); // Copies every character until '\0' copied
    int write(const char* data); // Copies every character until '\0' copied
    int write(const char* data, int length); // Copies the number of characters specified by length

    /**
     * Reads data from a byte stream in LITTLE ENDIAN
     * Arguments:
     *  data(OUTPUT) - data read from byte stream
     * Return:
     *  number of bytes read from byte stream; 0 if failed
     */
    int read(int& data);
    int read(float& data);
    int read(double& data);
    int read(string& data);
    int read(char* data, int length);

    void clear(); // clears buffer
    ByteStream& operator=(const ByteStream& rhs);
    ByteStream operator+(const ByteStream& rhs);

private:
    int writeIndex; // Keeps track of where to write to in buffer
    int readIndex; // Keeps track of where to read from in buffer
    int bufSize;
    char* buf;

};

// Base of server
class Server{
public:
    Server() {
        listeningSock = 0;
        clientSock = 0;
    }
    Server(int bufSize):stream(bufSize){
        listeningSock = 0;
        clientSock = 0;
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
    int send(const char* msg) {return send(msg, strlen(msg)+1);};
    int send(const char* msg, const size_t length) {
        ByteStream stream;
        stream.write(msg, length);
        return send(stream);
    };
    virtual int send(ByteStream& stream) = 0;
    
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

    // Reading from buffer
    int read(int& data) {return stream.read(data);};
    int read(float& data) {return stream.read(data);};
    int read(double& data) {return stream.read(data);};
    int read(string& data) {return stream.read(data);};
    int read(char* data, int length) {return stream.read(data,length);};

    // Byte stream handling
    void clearBuf() {stream.clear();};

    // Accessors
    int getListeningSock(){ return listeningSock; };
    int getClientSock(){ return clientSock; };
    char* getBufHead() {return stream.getBufHead();};
    int getBufSize() {return stream.getBufSize();};
    ByteStream getStream() { return stream;};

protected:
    // Variables
	int listeningSock;
    int clientSock;
    struct sockaddr_in clientAddr;
    struct sockaddr_in serverAddr;

    // Buffer
    ByteStream stream;
    
};

// Base of Client
class Client{
public:
    Client() {
        sockfd = 0;
    }
    Client(int bufSize):stream(bufSize){
        sockfd = 0;
    }
	virtual ~Client(){
		closeConn();
	};

    // Initializers
    int init(string& host, const int port) {return init(host.c_str(), port);};
    virtual int init(const char* host, const int port) = 0;
    
    // Sending
    int send(const string& msg) { return send(msg.c_str()); };
    int send(const char* msg) {return send(msg, strlen(msg)+1);};
    int send(const char* msg, const size_t length) {
        ByteStream stream;
        stream.write(msg, length);
        return send(stream);
    };
    virtual int send(ByteStream& stream) = 0;
    
    // Receiving
    virtual int receive() = 0;
    
    // Closing Connections
    void closeConn(){
    	if(sockfd != 0){
    		close(sockfd);
    	}
    };
    
    // Reading from buffer
    int read(int& data) {return stream.read(data);};
    int read(float& data) {return stream.read(data);};
    int read(double& data) {return stream.read(data);};
    int read(string& data) {return stream.read(data);};
    int read(char* data, int length) {return stream.read(data,length);};

    // Byte stream handling
    void clearBuf() {stream.clear();};

    // Accessors
    int getSockfd(){ return sockfd; };
    char* getBufHead() {return stream.getBufHead();};
    int getBufSize() {return stream.getBufSize();};
    ByteStream getStream() {return stream;};
    
protected:
    // Variables
	int sockfd; // socket connection

    // Buffer
    ByteStream stream;
    
};


/********************************** TCP *********************************/
// Server
class Server_TCP : public Server
{
public:
    Server_TCP();
    Server_TCP(const char* host, const int port);
    virtual ~Server_TCP();

    virtual int init(const char* host, const int port);
    virtual int accept();
    virtual int send(ByteStream& stream);
    virtual int receive();

};

// Client
class Client_TCP : public Client
{
public:
    Client_TCP();
    Client_TCP(const char* host, const int port);
    virtual ~Client_TCP();
    
    virtual int init(const char* host, const int port);
    virtual int send(ByteStream& stream);
    virtual int receive();
    
};

/********************************** UDP *********************************/
class Server_UDP : public Server
{
public:
    Server_UDP();
    Server_UDP(const char* host, const int port);
    virtual ~Server_UDP();

    virtual int init(const char* host, const int port);
    virtual int accept();
    virtual int send(ByteStream& stream);
    virtual int receive();
    
protected:
    struct sockaddr_in listenAddr;

};

class Client_UDP : public Client
{
public:
    Client_UDP();
    Client_UDP(const char* host, const int port);
    virtual ~Client_UDP();
    
    virtual int init(const char* host, const int port);
    virtual int send(ByteStream& stream);
    virtual int receive();
    
protected:
    struct sockaddr_in serverAddr;
    
};

#endif //END_IF_NETWORK_H