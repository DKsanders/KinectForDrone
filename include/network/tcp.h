/**
 * This file (tcp.h) provides interfaces for a server and a client
 * that uses TCP, implemented in tcp.cpp
 *
 * Interface is based off of the skeleton declared in network.h
 *
 * Author: David Sanders <david.sanders@mail.utoronto.ca>
 */

#ifndef _TCP_H
#define _TCP_H

// Libraries
#include "network.h"

// Constants
#define MAX_LISTENQUEUELEN 1

using namespace std;

// Server
typedef class Server_TCP : public Server
{
public:
	Server_TCP();
	Server_TCP(const char* host, const int port);
	virtual ~Server_TCP();

    virtual int init(const char* host, const int port);
    virtual int accept();
    virtual int send(const char* msg, const size_t length); // NOT IMPLEMENTED
    virtual int receive();

} Server_TCP;

// Client
typedef class Client_TCP : public Client
{
public:
	Client_TCP();
	Client_TCP(const char* host, const int port);
	virtual ~Client_TCP();
    
    virtual int init(const char* host, const int port);
    virtual int send(const char* msg, const size_t length);
    virtual int receive(); // NOT IMPLEMENTED
    
} Client_TCP;

#endif // END_IF_TCP_H