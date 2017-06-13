#pragma once

#include <stddef.h>
#include <stdint.h>
#include <vector>

class Crazywifi
{
public:
	struct Ack
    {
        Ack()
          : ack(0)
          , size(0)
        {}

        uint8_t ack:1;
        uint8_t powerDet:1;
        uint8_t retry:4;
        uint8_t data[32];

        uint8_t size;
    }__attribute__((packed));

    // void setAddress(char* ip_address);
    // char* getAddress() const {
    //     return m_ip_address;
    // }
    // void setPort(char* port);
    // char* getPort() const {
    //     return m_port;
    // }

    int new_conn_fd;
    int status;
    uint8_t Sbyte = 0xAA;
    const uint8_t *start_byte = &Sbyte;
    // char* ip_0_s;
    // char* ip_1_s;
    // char* ip_2_s;
    // char* ip_3_s;
    // char* m_ip_address;
    // char* m_port;
    
public:

    Crazywifi(const char* m_ip_address, const char* m_port );

    void * get_in_addr(struct sockaddr * sa);

    uint8_t chksum8 (const unsigned char *buff, size_t len);

    void sendPacket(
        const uint8_t* data,
        uint32_t length,
    	Ack& result);
};
