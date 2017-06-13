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

//uint16_t start_byte = 0xAAAA;
//uint8_t checksum;

void * Crazywifi::get_in_addr(struct sockaddr * sa)
{
    if(sa->sa_family == AF_INET)
    {
        return &(((struct sockaddr_in *)sa)->sin_addr); 
    }
    
    return &(((struct sockaddr_in6 *)sa)->sin6_addr); 
}

Crazywifi::Crazywifi()
{
    //int status;
    struct addrinfo hints, * res;
    int listner; 
    
    
    // Before using hint you have to make sure that the data structure is empty 
    memset(& hints, 0, sizeof hints);
    // Set the attribute for hint
    hints.ai_family = AF_UNSPEC; // We don't care V4 AF_INET or 6 AF_INET6
    hints.ai_socktype = SOCK_STREAM; // TCP Socket SOCK_DGRAM 
    hints.ai_flags = AI_PASSIVE; 
    
    // Fill the res data structure and make sure that the results make sense. 
    status = getaddrinfo(NULL, "8888" , &hints, &res);
    if(status != 0)
    {
        //throw std::runtime_error("***************************getaddrinfo error");
        fprintf(stderr,"getaddrinfo error: %s\n",gai_strerror(status));
    }
    
    // Create Socket and check if error occured afterwards
    listner = socket(res->ai_family,res->ai_socktype, res->ai_protocol);
    if(listner < 0 )
    {
        //throw std::runtime_error("***************************socket error");
        fprintf(stderr,"socket error: %s\n",gai_strerror(status));
    }
    
    // Bind the socket to the address of my local machine and port number 
    status = bind(listner, res->ai_addr, res->ai_addrlen); 
    if(status < 0)
    {
        //throw std::runtime_error("***************************bind error");
        fprintf(stderr,"bind: %s\n",gai_strerror(status));
    }

    status = listen(listner, 10); 
    if(status < 0)
    {
        //throw std::runtime_error("***************************listen error");
        fprintf(stderr,"listen: %s\n",gai_strerror(status));
    }
    
    // Free the res linked list after we are done with it   
    freeaddrinfo(res);
    
    
    // We should wait now for a connection to accept
    //int new_conn_fd;
    struct sockaddr_storage client_addr;
    socklen_t addr_size;
    char s[INET6_ADDRSTRLEN]; // an empty string 
        
    // Calculate the size of the data structure 
    addr_size = sizeof client_addr;
    
    printf("I am now accepting connections ...\n");

    //const void *buf = "Welcome";
    //while(1){
    // Accept a new connection and return back the socket desciptor 
    new_conn_fd = accept(listner, (struct sockaddr *) & client_addr, &addr_size);   
    if(new_conn_fd < 0)
    {
        fprintf(stderr,"accept: %s\n",gai_strerror(new_conn_fd));
        //continue;
        _exit(4);
    }

    inet_ntop(client_addr.ss_family, get_in_addr((struct sockaddr *) &client_addr),s ,sizeof s); 
    printf("I am now connected to %s \n",s);
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
        //ssize_t send(int sockfd, const void *buf, size_t nbytes, int flags);
        status = send(new_conn_fd,start_byte, 1,0);
        if(status == -1)
        {
            printf("START_BYTE 1 not sent, closing connection\n");
            //close(new_conn_fd);
            //_exit(4);
        }
        else{
            printf("START_BYTE 1 sent \n");
        }
        status = send(new_conn_fd,start_byte, 1,0);
        if(status == -1)
        {
            printf("START_BYTE 2 not sent, closing connection\n");
            //close(new_conn_fd);
            //_exit(4);
        }
        else{
            printf("START_BYTE 2 sent \n");
        }

        
        status = send(new_conn_fd,data, length, 0);
        if(status == -1)
        {
            printf("Data not sent, losing connection\n");
            //close(new_conn_fd);
            //_exit(4);
        }
        else{
            printf("Data sent \n");
        }

        *checksum = chksum8(data, length);
        status = send(new_conn_fd,checksum, 1, 0);
        if(status == -1)
        {
            printf("Checksum not sent, closing connection\n");
            //close(new_conn_fd);
            //_exit(4);
        }
        else{
            printf("Checksum sent \n");
        }

        // Read result
        result.ack = false;
        result.size = 0;
        char recvbuf[512];
        int recvbuflen = 512;
        //status = recv(new_conn_fd,recvbuf,recvbuflen,0);
        status = recv(new_conn_fd,(unsigned char*)&result,recvbuflen,0);
        if(status == -1)
        {
            printf("Message not received\n");
            //close(new_conn_fd);
            //_exit(4);
        }
        else {
            printf("Message received\n");
        }
    //}
    // Close the socket before we finish 
    //close(new_conn_fd); 

}

// void Crazyradio::sendPacketNoAck(
//     const uint8_t* data,
//     uint32_t length)
// {
//     int status;
//     int transferred;

//     if (!m_handle) {
//         std::cerr << "No valid Crazyradio handle!" << std::endl;
//         return;
//     }

//     // Send data
//     status = libusb_bulk_transfer(
//         m_handle,
//         /* endpoint*/ (0x01 | LIBUSB_ENDPOINT_OUT),
//         (uint8_t*)data,
//         length,
//         &transferred,
//         /*timeout*/ 1000);
//     if (status != LIBUSB_SUCCESS) {
//         std::cerr << "Send " << libusb_error_name(status) << std::endl;
//     }
//     if (length != transferred) {
//         std::cerr << "Did transfer " << transferred << " but " << length << " was requested!" << std::endl;
//     }
// }

