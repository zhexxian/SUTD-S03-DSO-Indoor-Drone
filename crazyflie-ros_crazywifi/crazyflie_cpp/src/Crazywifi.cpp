#include "Crazywifi.h"

#include <iostream>
#include <sstream>
#include <stdexcept>


#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netdb.h>
#include <arpa/inet.h>

#include <stdlib.h>

void * Crazywifi::get_in_addr(struct sockaddr * sa)
{
    if(sa->sa_family == AF_INET)
    {
        return &(((struct sockaddr_in *)sa)->sin_addr); 
    }
    
    return &(((struct sockaddr_in6 *)sa)->sin6_addr); 
}

Crazywifi::Crazywifi(const char* m_ip_address, const char* m_port)
{
    /*
        1. getaddrinfo
        2. create a new socket
        3. connect to the socket
        4. send data
    */  
    
    //Variables Declaration
    struct addrinfo hints, * res;
    int status;
    
    //clear hints
    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_PASSIVE;

    //status = getaddrinfo("192.168.0.168","23", &hints, &res);

    // sprintf(ip_0_s, "%d", ip_0);
    // sprintf(ip_1_s, "%d", ip_1);
    // sprintf(ip_2_s, "%d", ip_2);
    // sprintf(ip_3_s, "%d", ip_3);
    // strcpy(m_ip_address, ip_0_s);
    // strcat(m_ip_address, ".");
    // strcat(m_ip_address, ip_1_s);
    // strcat(m_ip_address, ".");
    // strcat(m_ip_address, ip_2_s);
    // strcat(m_ip_address, ".");
    // strcat(m_ip_address, ip_3_s);

    //sprintf(m_port, "%d", port);

    // const char* ip_address = m_ip_address;
    // const char* ip_port = m_port;
    // throw std::runtime_error(m_ip_address);

    //status = getaddrinfo(m_ip_address, m_port, &hints, &res);
    status = getaddrinfo("192.168.0.168","23", &hints, &res);
    if(status != 0)
    {
        throw std::runtime_error("Error getaddrinfo\n");
        //exit(1);
    }       
    
    new_conn_fd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if(new_conn_fd < 0)
    {
        throw std::runtime_error("Error socket \n");
        //exit(2);
    }
    
    status = connect(new_conn_fd, res->ai_addr, res->ai_addrlen);
    if(status < 0)
    {
        throw std::runtime_error("Error connect \n");
        //exit(3);
    }
    else {
        fprintf(stderr, "Connected to server \n");
    }
}

uint8_t Crazywifi::chksum8(const unsigned char *buff, size_t len) {
  unsigned int sum;
  for (sum=0; len!=0; len--){
    sum += *(buff++);
  }
  return (uint8_t)sum;
}


void Crazywifi::sendPacket(
    const uint8_t* data,
    uint32_t length,
    Ack& result)
{

        status = write(new_conn_fd,start_byte, 1);
        
        status = write(new_conn_fd, start_byte, 1);
        

        uint8_t Pchannel = data[0];
        const uint8_t *port_channel = &Pchannel;
        status = write(new_conn_fd,port_channel, 1);
        if(status == -1)
        {
            printf("Port and channel *NOT* sent\n");
        }
        

        uint8_t Plength = length - 1;
        const uint8_t *packet_length = &Plength;

        status = write(new_conn_fd,packet_length, 1);
        if(status == -1)
        {
            printf("Packet length *NOT* sent\n");
        }

        const uint8_t *new_data = data + 1;
        status = write(new_conn_fd,new_data, length - 1);
        if(status == -1)
        {
            printf("Data *NOT* sent\n");
        }

        uint8_t Csum = chksum8(data, length)+1;
        const uint8_t *checksum = &Csum; 
        status = write(new_conn_fd, checksum, 1);
        if(status == -1)
        {
            printf("Checksum *NOT* sent\n");
        }

        // Read result
        result.ack = false;
        result.size = 0;
        char recvbuf[512];
        int recvbuflen = 512;
        status = read(new_conn_fd,(unsigned char*)&result,recvbuflen);
        if(status == -1)
        {
            printf("Message not received\n");
        }
        else {
            printf("Message received\n");
        } 

}



// void Crazywifi::setAddress(char* ip_address)
// {
//     m_ip_address = ip_address;
// }

// void Crazywifi::setPort(char* port)
// {
//     m_port = port;
// }