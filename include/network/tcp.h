// Constants also defined in communication.h

#ifndef _TCP_H
#define _TCP_H

// Libraries
#include <sys/socket.h>
#include <netinet/in.h>
#include "network/network.h"

// Constants
#define MAX_LISTENQUEUELEN 1

using namespace std;

typedef class Server_TCP : public Server
{
public:
	Server_TCP();
	Server_TCP(const char* host, const int port);
	virtual ~Server_TCP();

    virtual int init(const char* host, const int port);
    virtual int listen();
    virtual int send(const char* msg, const size_t length);
    virtual int receive();

} Server_TCP;

typedef class Client_TCP : public Client
{
public:
	Client_TCP();
	Client_TCP(const char* host, const int port);
	virtual ~Client_TCP();
    
    virtual int init(const char* host, const int port);
    virtual int send(const char* msg, const size_t length);
    virtual int receive();
    
} Client_TCP;

#endif // END_IF_TCP_H