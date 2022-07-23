/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// https://github.com/espressif/esp-idf/blob/v4.4.1/examples/protocols/sockets/udp_server/main/udp_server.c


#include <AP_HAL_ESP32/WiFiUdpDriver.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL_ESP32/Scheduler.h>

#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_netif.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

#include "esp_wifi.h"
#include "esp_event.h"

#include "WiFiSetup.h"


using namespace ESP32;

extern const AP_HAL::HAL& hal;
extern void initialize_wifi(); // see WiFiSetup.cpp

#define UDP_PORT 14550

WiFiUdpDriver::WiFiUdpDriver()
{
    _state = NOT_INITIALIZED;
    accept_socket = -1;
}

void WiFiUdpDriver::begin(uint32_t b)
{
    begin(b, 0, 0);
}

void WiFiUdpDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    if (_state == NOT_INITIALIZED) {
        ::initialize_wifi();
        if (!start_listen()) {
            return;
        }

        if (xTaskCreatePinnedToCore(_wifi_thread2, "APM_WIFI2", Scheduler::WIFI_SS2, this, Scheduler::WIFI_PRIO2, &_wifi_task_handle,1) != pdPASS) {
            hal.console->printf("FAILED to create task _wifi_thread2\n");
        }
        _readbuf.set_size(RX_BUF_SIZE);
        _writebuf.set_size(TX_BUF_SIZE);
        _state = INITIALIZED;
    }
}

void WiFiUdpDriver::end()
{
    //TODO
}

void WiFiUdpDriver::flush()
{
}

bool WiFiUdpDriver::is_initialized()
{
    return true;
}

void WiFiUdpDriver::set_blocking_writes(bool blocking)
{
    //blocking writes do not used anywhere
}

bool WiFiUdpDriver::tx_pending()
{
    return (_writebuf.available() > 0);
}

uint32_t WiFiUdpDriver::available()
{
    return _readbuf.available();
}

uint32_t WiFiUdpDriver::txspace()
{
    int result =  _writebuf.space();
    result -= TX_BUF_SIZE / 4;
    return MAX(result, 0);
}

int16_t WiFiUdpDriver::read()
{
    if (!_read_mutex.take_nonblocking()) {
        return 0;
    }

    uint8_t byte;
    if (!_readbuf.read_byte(&byte)) {
        return -1;
    }

    _read_mutex.give();
    return byte;
}

bool WiFiUdpDriver::start_listen()
{
    accept_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (accept_socket < 0) {
        accept_socket = -1;
        return false;
    }
    int opt;
    setsockopt(accept_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    struct sockaddr_in destAddr;
    destAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    destAddr.sin_family = AF_INET;
    destAddr.sin_port = htons(UDP_PORT);
    int err = bind(accept_socket, (struct sockaddr *)&destAddr, sizeof(destAddr));
    if (err != 0) {
        close(accept_socket);
        accept_socket = 0;
        return false;
    }
    //memset(&client_addr, 0, sizeof(client_addr));
    fcntl(accept_socket, F_SETFL, O_NONBLOCK);

    return true;

}

bool WiFiUdpDriver::read_all()
{
    _read_mutex.take_blocking();
    struct sockaddr_in client_addr;
    socklen_t socklen = sizeof(client_addr);
    int count=0;
    do {
        count = recvfrom(accept_socket, _buffer, sizeof(_buffer) - 1, 0, (struct sockaddr *)&client_addr, &socklen);
        if (count > 0) {
            _readbuf.write(_buffer, count);
            _read_mutex.give();
        } //else {
          //   return false;
        //}
    } while (count > 0);
    _read_mutex.give();
    return true;
}

bool WiFiUdpDriver::write_data()
{

    _write_mutex.take_blocking();
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr("192.168.4.255");
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(UDP_PORT);
    int count = 0;
    do {
        count = _writebuf.peekbytes(_buffer, sizeof(_buffer));
        if (count > 0) {
            count = sendto(accept_socket, _buffer, count, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (count > 0) {
                _writebuf.advance(count);
            } //else {
              //  _write_mutex.give();
              //  return false;
           // }
        }
    } while (count>0);
    _write_mutex.give();
    return true;
}


size_t WiFiUdpDriver::write(uint8_t c)
{
    return write(&c,1);
}

size_t WiFiUdpDriver::write(const uint8_t *buffer, size_t size)
{
    if (!_write_mutex.take_nonblocking()) {
        return 0;
    }
    size_t ret = _writebuf.write(buffer, size);
    _write_mutex.give();
    return ret;
}

void WiFiUdpDriver::_wifi_thread2(void *arg)
{
    WiFiUdpDriver *self = (WiFiUdpDriver *) arg;
    while (true) {
        struct timeval tv = {
            .tv_sec = 0,
            .tv_usec = 100*1000, // 10 times a sec, we try to write-all even if we read nothing , at just 1000, it floggs the APM_WIFI2 task cpu usage unecessarily, slowing APM_WIFI1 response
        };
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(self->accept_socket, &rfds);

        int s = select(self->accept_socket + 1, &rfds, NULL, NULL, &tv);
        if (s > 0 && FD_ISSET(self->accept_socket, &rfds)) {
            self->read_all();
        }
        self->write_data();
    }
}

bool WiFiUdpDriver::discard_input()
{
    return false;
}
