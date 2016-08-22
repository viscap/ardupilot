// -*- Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//
//      Copyright (c) 2010 Michael Smith. All rights reserved.
//
#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include <limits.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdarg.h>
#include <AP_Math/AP_Math.h>

#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>

#include "UARTDriver.h"
#include "SITL_State.h"
#include "SITL_UDPDevice.h"

using namespace HALSITL;

bool SITLUARTDriver::_console;

/* UARTDriver method implementations */

void SITLUARTDriver::begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace)
{
    if (txSpace != 0) {
        _txSpace = txSpace;
    }
    if (rxSpace != 0) {
        _rxSpace = rxSpace;
    }

    fprintf(stdout,"----------------%d---------------- console %d\n", _portNumber, _console);

    switch (_portNumber) {
    case 0:
        _udp_start_connection();
        //_tcp_start_connection(true);
        break;

    case 1:
        /* gps */
        _connected = true;
        _fd = _sitlState->gps_pipe();
        break;

    case 2:
        _udp_start_connection();

        if (_sitlState->get_client_address() != NULL) {
    fprintf(stdout,"SITLUARTDriver %d get_client_address\n", _portNumber);
    fflush(stdout);

//            _tcp_start_client(_sitlState->get_client_address());
        } else {
    fprintf(stdout,"SITLUARTDriver %d no get_client_address\n", _portNumber);
    fflush(stdout);
//            _tcp_start_connection(false);
        }

    /*
        if (_sitlState->get_client_address() != NULL) {
            _tcp_start_client(_sitlState->get_client_address());
        } else {
            _tcp_start_connection(false);
        }
        break;
*/
    case 4:
        /* gps2 */
        _connected = true;
        _fd = _sitlState->gps2_pipe();
        break;

    default:
        _udp_start_connection();
        //_tcp_start_connection(false);
        break;
    }
}

void SITLUARTDriver::end()
{
    fprintf(stdout,"SITLUARTDriver %d end\n", _portNumber);
    fflush(stdout);
}

int16_t SITLUARTDriver::available(void)
{
//    fprintf(stdout,"SITLUARTDriver %d available\n", _portNumber);
//    fflush(stdout);

    _check_connection();

    if (!_connected) {
        return 0;
    }
    if(_portNumber==0 || _portNumber==2 || _portNumber==3){
        if(read_bytes.size()==0)
            return 1;
        else
            return read_bytes.size();
    }
    if (_select_check(_fd)) {
#ifdef FIONREAD
        // use FIONREAD to get exact value if possible
        int num_ready;
        if (ioctl(_fd, FIONREAD, &num_ready) == 0) {
            if (num_ready > _rxSpace) {
                return _rxSpace;
            }
            if (num_ready == 0) {
                // EOF is reached
                fprintf(stdout, "Closed connection on serial port %u\n", _portNumber);
                close(_fd);
                _connected = false;
                return 0;
            }
            return num_ready;
        }
#endif
        return 1; // best we can do is say 1 byte available
    }
    return 0;
}



int16_t SITLUARTDriver::txspace(void)
{
    // always claim there is space available
    return _txSpace;
}

int16_t SITLUARTDriver::read(void)
{
    char c;
/*
    if (available() <= 0) {
        return -1;
    }
*/
    if (_portNumber == 1 || _portNumber == 4) {
        if (_sitlState->gps_read(_fd, &c, 1) == 1) {
            return (uint8_t)c;
        }
        return -1;
    }

    if (_console) {
        return ::read(0, &c, 1);
    }
 
    if(_portNumber==0 || _portNumber==2 || _portNumber==3){
        uint8_t buffer[100];
        int read_bu = _device->read(buffer, 100);
        //fprintf(stdout, "readddd: read_bu: %d\n", read_bu);
        //fflush(stdout);
        for(int i=0; i < read_bu; i++){
            read_bytes.push_back(buffer[i]);
        }
        if(read_bu>0){
            fprintf(stdout, "SITLUARTDriver::read  %d available_bytes: %d\n",_portNumber, read_bytes.size());
            fflush(stdout);
        }
        //return read_bu;//_device->read(&buffer, 1);
//        if (read_bu == 1) {
        if(read_bytes.size()>0){
            uint8_t data = read_bytes.front();
            read_bytes.erase(read_bytes.begin());
            return data;
        }
//        }
        return -1;
    }

    int n = recv(_fd, &c, 1, MSG_DONTWAIT);
    if (n <= 0) {
        // the socket has reached EOF
        close(_fd);
        _connected = false;
        fprintf(stdout, "Closed connection on serial port %d\n", _portNumber);
        fflush(stdout);
        return -1;
    }
    if (n == 1) {
        return (uint8_t)c;
    }
    return -1;
}

void SITLUARTDriver::flush(void)
{
}

size_t SITLUARTDriver::write(uint8_t c)
{
    int flags = 0;
    _check_connection();
    if (!_connected) {
        return 0;
    }
    if (_nonblocking_writes) {
        flags |= MSG_DONTWAIT;
    }
    if (_console) {
       if(_portNumber==0 || _portNumber==2 || _portNumber==3){
           size_t size = 1;
           int bytes_write = _device->write(&c, size);
           fprintf(stdout,"SITLUARTDriver %d write %d\n", _portNumber, bytes_write);
           fflush(stdout);
           return bytes_write;
       }
       return ::write(_fd, &c, 1);
    }
    if(_portNumber==0 || _portNumber==2 || _portNumber==3){
       size_t size = 1;
       int bytes_write = _device->write(&c, size);
       fprintf(stdout,"SITLUARTDriver %d write %d\n", _portNumber, bytes_write);
       fflush(stdout);
       return bytes_write;
    }
    return send(_fd, &c, 1, flags);
}

size_t SITLUARTDriver::write(const uint8_t *buffer, size_t size)
{
    if(_portNumber==0 || _portNumber==2 || _portNumber==3){
       int bytes_write = _device->write(buffer, size);
       fprintf(stdout,"SITLUARTDriver %d write2 %d of %d\n", _portNumber, bytes_write, size);
       fflush(stdout);
       return bytes_write;
    }
    size_t n = 0;
    while (size--) {
        n += write(*buffer++);
    }
    return n;
}

/*
  start a UDP connection for the serial port
 */
void SITLUARTDriver::_udp_start_connection(void)
{

    if(_portNumber==0)
       _device = new SITLUDPDevice("127.0.0.1", 5760+_portNumber, false);
    if(_portNumber==2)
       _device = new SITLUDPDevice("127.0.0.1", 5760, false);
    if(_portNumber==3)
       _device = new SITLUDPDevice("127.0.0.1", 5760, false);
   
   _connected = _device->open();
   _device->set_blocking(false);
   fprintf(stdout,"SITLUARTDriver _udp_start_connection %d connected %d\n", _portNumber, _connected);
   fflush(stdout);

/*
    //bool bcast = (_flag && strcmp(_flag, "bcast") == 0);
    _device = new SITLUDPDevice("127.0.0.1", _portNumber, true);
    //(_ip, _base_port, bcast);
    _connected = _device->open();
    _device->set_blocking(false);

    /* try to write on MAVLink packet boundaries if possible */
//    _packetise = true;

}

/*
  start a TCP connection for the serial port. If wait_for_connection
  is true then block until a client connects
 */
void SITLUARTDriver::_tcp_start_connection(bool wait_for_connection)
{
    int one=1;
    struct sockaddr_in sockaddr;
    int ret;

    if (_connected) {
        return;
    }

    if (_console) {
        // hack for console access
        _connected = true;
        _listen_fd = -1;
        _fd = 1;
        return;
    }

    if (_fd != -1) {
        close(_fd);
    }

    if (_listen_fd == -1) {
        memset(&sockaddr,0,sizeof(sockaddr));

#ifdef HAVE_SOCK_SIN_LEN
        sockaddr.sin_len = sizeof(sockaddr);
#endif
        sockaddr.sin_port = htons(_sitlState->base_port() + _portNumber);
        sockaddr.sin_family = AF_INET;

        _listen_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (_listen_fd == -1) {
            fprintf(stderr, "socket failed - %s\n", strerror(errno));
            exit(1);
        }

        /* we want to be able to re-use ports quickly */
        setsockopt(_listen_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

        fprintf(stderr, "bind port %u for %u\n",
                (unsigned)ntohs(sockaddr.sin_port),
                (unsigned)_portNumber);

        ret = bind(_listen_fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr));
        if (ret == -1) {
            fprintf(stderr, "bind failed on port %u - %s\n",
                    (unsigned)ntohs(sockaddr.sin_port),
                    strerror(errno));
            exit(1);
        }

        ret = listen(_listen_fd, 5);
        if (ret == -1) {
            fprintf(stderr, "listen failed - %s\n", strerror(errno));
            exit(1);
        }

        fprintf(stderr, "Serial port %u on TCP port %u\n", _portNumber,
                _sitlState->base_port() + _portNumber);
        fflush(stdout);
    }

    if (wait_for_connection) {
        fprintf(stdout, "Waiting for connection ....\n");
        fflush(stdout);
        _fd = accept(_listen_fd, NULL, NULL);
        if (_fd == -1) {
            fprintf(stderr, "accept() error - %s", strerror(errno));
            exit(1);
        }
        setsockopt(_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
        setsockopt(_fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
        _connected = true;
    }
}


/*
  start a TCP client connection for the serial port. 
 */
void SITLUARTDriver::_tcp_start_client(const char *address)
{
    int one=1;
    struct sockaddr_in sockaddr;
    int ret;

    if (_connected) {
        return;
    }

    if (_fd != -1) {
        close(_fd);
    }

    char *addr2 = strdup(address);
    char *p = strchr(addr2, ':');
    if (p == NULL) {
        fprintf(stderr, "need IP:port\n");
        exit(1);
    }
    *p = 0;
    uint16_t port = htons(atoi(p+1));

    memset(&sockaddr,0,sizeof(sockaddr));

#ifdef HAVE_SOCK_SIN_LEN
    sockaddr.sin_len = sizeof(sockaddr);
#endif
    sockaddr.sin_port = port;
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_addr.s_addr = inet_addr(addr2);

    free(addr2);

    _fd = socket(AF_INET, SOCK_STREAM, 0);
    if (_fd == -1) {
        fprintf(stderr, "socket failed - %s\n", strerror(errno));
        exit(1);
    }

    /* we want to be able to re-use ports quickly */
    setsockopt(_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    ret = connect(_fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr));
    if (ret == -1) {
        fprintf(stderr, "connect failed on port %u - %s\n",
                (unsigned)ntohs(sockaddr.sin_port),
                strerror(errno));
        exit(1);
    }

    setsockopt(_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    setsockopt(_fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    _connected = true;
}

/*
  see if a new connection is coming in
 */
void SITLUARTDriver::_check_connection(void)
{
    if(_portNumber==0 || _portNumber==2 || _portNumber==3){
//        fprintf(stdout, "SITLUARTDriver::_check_connection %d \n",_portNumber);
//        fflush(stdout);
        if (_connected) {
            // we only want 1 connection at a time
            return;
        }
        return;
    }

    if (_connected) {
        // we only want 1 connection at a time
        return;
    }

    if (_select_check(_listen_fd)) {
        _fd = accept(_listen_fd, NULL, NULL);
        if (_fd != -1) {
            int one = 1;
            _connected = true;
            setsockopt(_fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
            setsockopt(_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
            fprintf(stdout, "New connection on serial port %u\n", _portNumber);
        }
    }
}

/*
  use select() to see if something is pending
 */
bool SITLUARTDriver::_select_check(int fd)
{
//    fprintf(stdout, "SITLUARTDriver::_select_check %d \n",_portNumber);
//    fflush(stdout);
    fd_set fds;
    struct timeval tv;

    FD_ZERO(&fds);
    FD_SET(fd, &fds);

    // zero time means immediate return from select()
    tv.tv_sec = 0;
    tv.tv_usec = 0;

    if (select(fd+1, &fds, NULL, NULL, &tv) == 1) {
        return true;
    }
    return false;
}

void SITLUARTDriver::_set_nonblocking(int fd)
{
    unsigned v = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, v | O_NONBLOCK);
}

#endif
