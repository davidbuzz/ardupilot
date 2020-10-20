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
/*
  simple socket handling class for systems with BSD socket API
 */

#include <AP_HAL/AP_HAL.h>
#if HAL_OS_SOCKETS

#include "Socket.h"

#include <unistd.h>

#ifdef _WIN32
#define F_GETFL 0
#define F_SETFL 0
#define F_SETFD 0
#define FD_CLOEXEC 0
#include "../../../lib/nonblocking.h" //its not fcntl but its got a windwos impl
#endif

#ifdef _WIN32
typedef int socklen_t;
#endif


extern const AP_HAL::HAL& hal;

/*
  constructor
 */
SocketAPM::SocketAPM(bool _datagram) :
    SocketAPM(_datagram, 
              socket(AF_INET, _datagram?SOCK_DGRAM:SOCK_STREAM, 0))
{}

SocketAPM::SocketAPM(bool _datagram, int _fd) :
    datagram(_datagram),
    fd(_fd)
{

#ifndef _WIN32  
//FD_CLOEXEC bit set means the file descriptor will automatically be closed during a successful execve(2).
// because ardupilot only does an execv() as part of a reboot() or a watchdog (that actually does a reboot()), simply 
// making sure we call close() prior to the exec() call is pretty easy to ensure, and so this FD_CLOEXEC is unneeded
    fcntl(fd, F_SETFD, FD_CLOEXEC); 
#endif
    if (!datagram) {
        int one = 1;
        #ifndef _WIN32
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
        #else
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, (const char*)&one, sizeof(one));
        #endif
    }
}


SocketAPM::~SocketAPM()
{
    if (fd != -1) {
        hal.console->printf("DSocketAPM desconstructed.\n");
        //::close(fd); //error: ‘::close_used_without_including_unistd_h’ has not been declared
        fd = -1;
    }
}

void SocketAPM::make_sockaddr(const char *address, uint16_t port, struct sockaddr_in &sockaddr)
{
    memset(&sockaddr, 0, sizeof(sockaddr));

#ifdef HAVE_SOCK_SIN_LEN
    sockaddr.sin_len = sizeof(sockaddr);
#endif
    sockaddr.sin_port = htons(port);
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_addr.s_addr = inet_addr(address);
}

/*
  connect the socket
 */
bool SocketAPM::connect(const char *address, uint16_t port)
{
    struct sockaddr_in sockaddr;
    make_sockaddr(address, port, sockaddr);

    if (::connect(fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) != 0) {
        return false;
    }
    return true;
}

/*
  bind the socket
 */
bool SocketAPM::bind(const char *address, uint16_t port)
{
    struct sockaddr_in sockaddr;
    make_sockaddr(address, port, sockaddr);

    if (::bind(fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) != 0) {
        return false;
    }
    return true;
}


/*
  set SO_REUSEADDR
 */
bool SocketAPM::reuseaddress(void)
{

   // return false;// hack to allow it

    int one = 1;
 //       #ifndef _WIN32
    return (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one)) != -1);
 //       #else
 //   return (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, (const char*)&one, sizeof(one)) != -1);
  //      #endif

}


/*
  set blocking state
 */
bool SocketAPM::set_blocking(bool blocking)
{

#ifndef _WIN32
    int fcntl_ret;
    if (blocking) {
        fcntl_ret = fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) & ~O_NONBLOCK);
    } else {
        fcntl_ret = fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
    }
    return fcntl_ret != -1;

#else
    int fcntl_ret = set_nonblocking_flag(fd, blocking);
    return fcntl_ret != -1;
#endif

/*
    // windows todo https://docs.microsoft.com/en-us/windows/win32/api/winsock/nf-winsock-ioctlsocket
    int iResult;
    u_long iMode = 0; // 0  means blocking
    if ( ! blocking ) {
        iMode =  1; // non-zero means non-blocking
    }
    //FIONBIO enables or disables the blocking mode for the  socket based on the numerical value of iMode.
    iResult = ioctlsocket(fd, FIONBIO, &iMode);
    //todo error check iResult
    iResult = iResult; // to stop 'unused warning
    return blocking;
*/
}

/*
  set cloexec state
 */
bool SocketAPM::set_cloexec()
{
#ifndef _WIN32
    return (fcntl(fd, F_SETFD, FD_CLOEXEC) != -1);
#else
    return true;// buzz todo
#endif
}

/*
  send some data
 */
ssize_t SocketAPM::send(const void *buf, size_t size)
{

    return ::send(fd, buf, size, 0);

}

/*
  send some data
 */
ssize_t SocketAPM::sendto(const void *buf, size_t size, const char *address, uint16_t port)
{
    struct sockaddr_in sockaddr;
    make_sockaddr(address, port, sockaddr);

    return ::sendto(fd, buf, size, 0, (struct sockaddr *)&sockaddr, sizeof(sockaddr));

}

/*
  receive some data
 */
ssize_t SocketAPM::recv(void *buf, size_t size, uint32_t timeout_ms)
{
    if (!pollin(timeout_ms)) {
        return -1;
    }
    socklen_t len = sizeof(in_addr);
    #ifndef _WIN32
    return ::recvfrom(fd, buf, size, MSG_DONTWAIT, (sockaddr *)&in_addr, &len);// MSG_DONTWAIT means non-blocking
    #else
        set_blocking(false); //ming doesnt have MSG_DONTWAIT, but set_blocking() uses set_nonblocking_flag() from gnulib which works
        return ::recvfrom(fd, (char*)buf, size, 0, (sockaddr *)&in_addr, &len);
    #endif
}

/*
  return the IP address and port of the last received packet
 */
void SocketAPM::last_recv_address(const char *&ip_addr, uint16_t &port)
{
    ip_addr = inet_ntoa(in_addr.sin_addr);
    port = ntohs(in_addr.sin_port);
}

void SocketAPM::set_broadcast(void)
{
    int one = 1;
    setsockopt(fd,SOL_SOCKET,SO_BROADCAST,(char *)&one,sizeof(one));
}

/*
  return true if there is pending data for input
 */
bool SocketAPM::pollin(uint32_t timeout_ms)
{
    fd_set fds;
    struct timeval tv;

    FD_ZERO(&fds);
    FD_SET(fd, &fds);

    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000UL;

    if (select(fd+1, &fds, nullptr, nullptr, &tv) != 1) {
        return false;
    }
    return true;
}


/*
  return true if there is room for output data
 */
bool SocketAPM::pollout(uint32_t timeout_ms)
{
    fd_set fds;
    struct timeval tv;

    FD_ZERO(&fds);
    FD_SET(fd, &fds);

    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000UL;

    if (select(fd+1, nullptr, &fds, nullptr, &tv) != 1) {
        return false;
    }
    return true;
}

/* 
   start listening for new tcp connections
 */
bool SocketAPM::listen(uint16_t backlog)
{
    return ::listen(fd, (int)backlog) == 0;
}

/*
  accept a new connection. Only valid for TCP connections after
  listen has been used. A new socket is returned
*/
SocketAPM *SocketAPM::accept(uint32_t timeout_ms)
{
    if (!pollin(timeout_ms)) {
        return nullptr;
    }

    int newfd = ::accept(fd, nullptr, nullptr);
    if (newfd == -1) {
        return nullptr;
    }
    // turn off nagle for lower latency
    int one = 1;
    #ifndef _WIN32
    setsockopt(newfd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    #else
    setsockopt(newfd, IPPROTO_TCP, TCP_NODELAY, (const char*)&one, sizeof(one));
    #endif
    return new SocketAPM(false, newfd);
}

#endif // HAL_OS_SOCKETS
