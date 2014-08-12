/**
 * This file (udp.h) provides interfaces for a server and a client
 * that uses UDP, implemented in udp.cpp
 *
 * Interface is based off of the skeleton declared in network.h
 *
 * Author: David Sanders <david.sanders@mail.utoronto.ca>
 */

#ifndef _UDP_H
#define _UDP_H

// Libraries
#include "network.h"

using namespace std;

class Server_UDP : public Server
{
public:
    Server_UDP();
    Server_UDP(const char* host, const int port);
    virtual ~Server_UDP();

    virtual int init(const char* host, const int port);
    virtual int accept();
    virtual int send(const char* msg, const size_t length);
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
    virtual int send(const char* msg, const size_t length);
    virtual int receive();
    
protected:
    struct sockaddr_in serverAddr;
    
};

#endif // END_IF_UDP_H