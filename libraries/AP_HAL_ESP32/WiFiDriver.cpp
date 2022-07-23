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

// https://github.com/espressif/esp-idf/blob/v4.4.1/examples/protocols/sockets/tcp_server/main/tcp_server.c


#include <AP_HAL_ESP32/WiFiDriver.h>
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

#define TCP_PORT 5760


WiFiDriver::WiFiDriver()
{
#ifdef WIFIDEBUG
   ////hal.console->printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    _state = NOT_INITIALIZED;
    accept_socket = -1;

    for (unsigned short i = 0; i < WIFI_MAX_CONNECTION; ++i) {
        socket_list[i] = -1;
    }
}

void WiFiDriver::begin(uint32_t b)
{
    begin(b, 0, 0);
}

void WiFiDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
////#ifdef WIFIDEBUG
   ////hal.console->printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
//#endif
    hal.console->printf("%s:%d TCP state:%d\n", __PRETTY_FUNCTION__, __LINE__,_state);

    if (_state == NOT_INITIALIZED) {
        ::initialize_wifi();
        // pin this thread to Core 1
        if (xTaskCreatePinnedToCore(_wifi_thread, "APM_WIFI1", Scheduler::WIFI_SS1, this, Scheduler::WIFI_PRIO1, &_wifi_task_handle,1) != pdPASS) {
        //if (xTaskCreate(_wifi_thread, "APM_WIFI", Scheduler::WIFI_SS, this, Scheduler::WIFI_PRIO1, &_wifi_task_handle) != pdPASS) {
            //hal.console->printf("FAILED to create task _wifi_thread\n");
        }
        _readbuf.set_size(RX_BUF_SIZE);
        _writebuf.set_size(TX_BUF_SIZE);
        _state = INITIALIZED;
    }
}

void WiFiDriver::end()
{
    //TODO
}

void WiFiDriver::flush()
{
}

bool WiFiDriver::is_initialized()
{
    return _state != NOT_INITIALIZED;
}

void WiFiDriver::set_blocking_writes(bool blocking)
{
    //blocking writes do not used anywhere
}

bool WiFiDriver::tx_pending()
{
    return (_writebuf.available() > 0);
}

uint32_t WiFiDriver::available()
{
    if (_state != CONNECTED) {
        return 0;
    }
    //hal.console->printf("WiFiDriver::available ? %d\n",_readbuf.available());
    return _readbuf.available();
}

uint32_t WiFiDriver::txspace()
{
    if (_state != CONNECTED) {
        return 0;
    }
    int result =  _writebuf.space();
    result -= TX_BUF_SIZE / 4;
    return MAX(result, 0);
}

int16_t WiFiDriver::read()
{
    if (_state != CONNECTED) {
        return -1;
    }
    uint8_t byte;
    if (!_readbuf.read_byte(&byte)) {
        return -1;
    }
    return byte;
}

bool WiFiDriver::start_listen()
{
    accept_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (accept_socket < 0) {
        accept_socket = -1;
        return false;
    }
    int opt;
    setsockopt(accept_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    struct sockaddr_in destAddr;
    destAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    destAddr.sin_family = AF_INET;
    destAddr.sin_port = htons(TCP_PORT);
    int err = bind(accept_socket, (struct sockaddr *)&destAddr, sizeof(destAddr));
    if (err != 0) {
        close(accept_socket);
        accept_socket = 0;
        return false;
    }
    err = listen(accept_socket, 5);
    if (err != 0) {
        close(accept_socket);
        accept_socket = -1;
        return false;
    }
    return true;

}

bool WiFiDriver::try_accept()
{
    struct sockaddr_in sourceAddr;
    uint addrLen = sizeof(sourceAddr);
    short i = available_socket();
    if (i != WIFI_MAX_CONNECTION) {
        socket_list[i] = accept(accept_socket, (struct sockaddr *)&sourceAddr, &addrLen);
        if (socket_list[i] >= 0) {
            fcntl(socket_list[i], F_SETFL, O_NONBLOCK);
            return true;
        }
    }
    return false;
}

bool WiFiDriver::read_data()
{
    for (unsigned short i = 0; i < WIFI_MAX_CONNECTION && socket_list[i] > -1; ++i) {
        int count = 0;
        do {
            count = recv(socket_list[i], (void *)_buffer, sizeof(_buffer), 0);
            if (count > 0) {
                _readbuf.write(_buffer, count);
                if (count == sizeof(_buffer)) {
                    _more_data = true;
                }
            } else if (count < 0 && errno != EAGAIN) {
                shutdown(socket_list[i], 0);
                close(socket_list[i]);
                socket_list[i] = -1;
                _state = INITIALIZED;
                return false;
            }
        } while (count > 0);
    }
    return true;
}

bool WiFiDriver::write_data()
{
    for (unsigned short i = 0; i < WIFI_MAX_CONNECTION && socket_list[i] > -1; ++i) {
        int count = 0;
        _write_mutex.take_blocking();
        do {
            count = _writebuf.peekbytes(_buffer, sizeof(_buffer));
            if (count > 0) {
                count = send(socket_list[i], (void*) _buffer, count, 0);
                if (count > 0) {
                    _writebuf.advance(count);
                    if (count == sizeof(_buffer)) {
                        _more_data = true;
                    }
                } else if (count < 0 && errno != EAGAIN) {
                    shutdown(socket_list[i], 0);
                    close(socket_list[i]);
                    socket_list[i] = -1;
                    _state = INITIALIZED;
                    _write_mutex.give();
                    return false;
                }
            }
        } while (count > 0);
    }
    _write_mutex.give();
    return true;
}



size_t WiFiDriver::write(uint8_t c)
{
    return write(&c,1);
}

size_t WiFiDriver::write(const uint8_t *buffer, size_t size)
{
    if (_state != CONNECTED) {
        return 0;
    }
    if (!_write_mutex.take_nonblocking()) {
        return 0;
    }
    size_t ret = _writebuf.write(buffer, size);
    _write_mutex.give();
    return ret;
}

void WiFiDriver::_wifi_thread(void *arg)
{
    WiFiDriver *self = (WiFiDriver *) arg;
    if (!self->start_listen()) {
        vTaskDelete(nullptr);
    }
    while (true) {
        if (self->try_accept()) {
            self->_state = CONNECTED;
            while (true) {
                self->_more_data = false;
                if (!self->read_data()) {
                    self->_state = INITIALIZED;
                    break;
                }
                if (!self->write_data()) {
                    self->_state = INITIALIZED;
                    break;
                }
                if (!self->_more_data) {
                    hal.scheduler->delay_microseconds(1000);
                }
            }
        }
        //hal.console->printf(" ZZZZ 11 \n" );
        hal.scheduler->delay_microseconds(10);

    }
}

bool WiFiDriver::discard_input()
{
    return false;
}

unsigned short WiFiDriver::available_socket()
{
    for (unsigned short i = 0; i < WIFI_MAX_CONNECTION; ++i)
        if (socket_list[i] == -1) {
            return i;
        }

    return WIFI_MAX_CONNECTION;
}
