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

#include <AP_HAL_ESP32/WiFiUdpDriver.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL_ESP32/Scheduler.h>

#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include <esp_wifi.h>
#include "esp_event.h"
//#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_log.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

using namespace ESP32;

extern const AP_HAL::HAL& hal;

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
        initialize_wifi();
        if (!start_listen()) {
            return;
        }

        xTaskCreate(_wifi_thread, "APM_WIFI", Scheduler::WIFI_SS, this, Scheduler::WIFI_PRIO, &_wifi_task_handle);
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
    int count = recvfrom(accept_socket, _buffer, sizeof(_buffer) - 1, 0, (struct sockaddr *)&client_addr, &socklen);
    if (count > 0) {
        _readbuf.write(_buffer, count);
        _read_mutex.give();
    } else {
        return false;
    }
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
    int count = _writebuf.peekbytes(_buffer, sizeof(_buffer));
    if (count > 0) {
        count = sendto(accept_socket, _buffer, count, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (count > 0) {
            _writebuf.advance(count);
        } else {
            _write_mutex.give();
            return false;
        }
    }
    _write_mutex.give();
    return true;
}


static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        printf("station  join, AID=%d", event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        printf("station  leave, AID=%d", event->aid);
    }
}

void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .ap = {
            {.ssid = WIFI_SSID},
            {.password = WIFI_PWD},
            .ssid_len = strlen(WIFI_SSID),
            .channel = 0,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .ssid_hidden =0,
            .max_connection = 4
        },
    };
    if (strlen(WIFI_PWD) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    printf("wifi_init_softap finished. SSID:%s password:%s",
             WIFI_SSID, WIFI_PWD);
}

void WiFiUdpDriver::initialize_wifi()
{

    nvs_flash_init();

    wifi_init_softap();

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

void WiFiUdpDriver::_wifi_thread(void *arg)
{
    WiFiUdpDriver *self = (WiFiUdpDriver *) arg;
    while (true) {
        struct timeval tv = {
            .tv_sec = 0,
            .tv_usec = 1000,
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
