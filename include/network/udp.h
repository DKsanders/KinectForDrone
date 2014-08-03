// Constants also defined in communication.h

#ifndef _UDP_H
#define _UDP_H

// Libraries
#include <sys/socket.h>
#include <netinet/in.h>
#include "network/network.h"

using namespace std;

typedef class Server_UDP : public Server
{
public:
    Server_UDP();
    Server_UDP(const char* host, const int port);
    virtual ~Server_UDP();

    virtual int init(const char* host, const int port);
    virtual int listen();
    virtual int send(const char* msg, const size_t length);
    virtual int receive();
    
protected:
    struct sockaddr_in listenAddr;

} Server_UDP;

typedef class Client_UDP : public Client
{
public:
    Client_UDP();
    Client_UDP(const char* host, const int port);
    virtual ~Client_UDP();
    
    virtual int init(const char* host, const int port);
    virtual int send(const char* msg, const size_t length);
    virtual int receive();
    
protected:
    struct sockaddr_in serverAddr;
    
} Client_UDP;

#endif // END_IF_UDP_H