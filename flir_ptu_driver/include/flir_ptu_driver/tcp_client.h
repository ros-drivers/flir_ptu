

#ifndef TCP_CLIENT_H
#define TCP_CLIENT_H

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h> //inet_addr
#include <netdb.h> //hostent

class TcpClient
{
private:
    int port;
    std::string address;
    struct sockaddr_in server;

public:
    int mysock;
    TcpClient();
    bool conn(std::string ip_address , int port);
    void setTimeout(int secs, int usecs);
    bool send_data(std::string data);
    std::string receive(int size=512);
    size_t readline (std::string &buffer, size_t size = 512, std::string eol = "\n");
};

#endif  // TCP_CLIENT_H
