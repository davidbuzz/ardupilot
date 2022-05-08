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
//#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "esp_netif.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

#include "esp_wifi.h"
#include "esp_event.h"


using namespace ESP32;

extern const AP_HAL::HAL& hal;

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
    hal.console->printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);

    if (_state == NOT_INITIALIZED) {
        initialize_wifi();
        if (xTaskCreate(_wifi_thread, "APM_WIFI", Scheduler::WIFI_SS, this, Scheduler::WIFI_PRIO, &_wifi_task_handle) != pdPASS) {
            ////hal.console->printf("FAILED to create task _wifi_thread\n");
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
    //hal.console->printf("ZZZZ read!!\n");
    if (_state != CONNECTED) {
        return -1;
    }
    uint8_t byte;
    if (!_readbuf.read_byte(&byte)) {
        return -1;
            //hal.console->printf("ZZZZ readbyte -1\n");

    }
    //hal.console->printf("ZZZZ read ok\n");

    return byte;
}


bool WiFiDriver::start_listen()
{
#ifdef WIFIDEBUG
////hal.console->printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
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
    destAddr.sin_port = htons(5760);
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
    //hal.console->printf(" ZZZZ 6\n" );

    struct sockaddr_in sourceAddr;
    uint addrLen = sizeof(sourceAddr);
    short i = available_socket();
    if (i != WIFI_MAX_CONNECTION) {
        //hal.console->printf(" ZZZZ 7\n" );
        socket_list[i] = accept(accept_socket, (struct sockaddr *)&sourceAddr, &addrLen);
        //hal.console->printf(" ZZZZ 8\n" );
        if (socket_list[i] >= 0) {
            //hal.console->printf(" ZZZZ 9\n" );
            fcntl(socket_list[i], F_SETFL, O_NONBLOCK);
            return true;
        }
    }
    //hal.console->printf(" ZZZZ 10\n" );
    return false;
}

bool WiFiDriver::read_data()
{
    ////hal.console->printf(" ZZZZ read_data\n" );
    for (unsigned short i = 0; i < WIFI_MAX_CONNECTION && socket_list[i] > -1; ++i) {
        int count = 0;
        do {
            count = recv(socket_list[i], (void *)_buffer, sizeof(_buffer), 0);
            if (count > 0) {
                hal.console->printf(" ZZZZ read_data %d\n",count );
                _readbuf.write(_buffer, count);
                if (count == sizeof(_buffer)) {
                    _more_data = true;
                }
            } else if (count < 0 && errno != EAGAIN) {
                //hal.console->printf(" ZZZZ read_data-shutdown%d\n",count );
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
    //hal.console->printf(" ZZZZ write_data   \n" );
    for (unsigned short i = 0; i < WIFI_MAX_CONNECTION && socket_list[i] > -1; ++i) {
        int count = 0;
        _write_mutex.take_blocking();
        //hal.console->printf(" ZZZZ write_data took sem \n" );

            //static const uint8_t buffer[] = {0xFE ,0x09, 0x4E, 0x01, 0x01, 00, 00,   00,  00,  00,  02, 03, 0x51, 0x04,  03, 0x1C,  0x7F };
            //write(buffer,sizeof(buffer));
        do {
            count = _writebuf.peekbytes(_buffer, sizeof(_buffer));
            //hal.console->printf(" ZZZZ write_data peek-count: %d bufsize:%d\n",count,sizeof(_buffer) );
            if (count > 0) {
                count = send(socket_list[i], (void*) _buffer, count, 0);
                hal.console->printf(" ZZZZ write_data send-count %d\n",count );
                if (count > 0) {
                    _writebuf.advance(count);
                    if (count == sizeof(_buffer)) {
                        _more_data = true;
                        //hal.console->printf(" ZZZZ write_data more data\n" );
                    }
                } else if (count < 0 && errno != EAGAIN) {
                    //hal.console->printf(" ZZZZ write_data shutdown\n" );
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
            //hal.console->printf(" ZZZZ write_data gave sem \n" );
    return true;
}


static void wifi_init_softap(void)
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    //ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));

    // wifi_config_t wifi_config = {
    //     .ap = {
    //         .ssid = WIFI_SSID,
    //         .ssid_len = strlen(WIFI_SSID),
    //         .password = WIFI_PWD,
    //         .max_connection = 4,
    //         .authmode = WIFI_AUTH_WPA_WPA2_PSK
    //     },
    // };
     wifi_config_t wifi_config;
     memset(&wifi_config, 0, sizeof(wifi_config));
     //wifi_config.ap.ssid=(unsigned char)WIFI_SSID;
     wifi_config.ap.ssid_len=strlen(WIFI_SSID);
     //wifi_config.ap.password=WIFI_PWD;
     wifi_config.ap.max_connection=4;
     wifi_config.ap.authmode=WIFI_AUTH_WPA_WPA2_PSK;

    strcpy((char *)wifi_config.ap.ssid, WIFI_SSID);
    strcpy((char *)wifi_config.ap.password, WIFI_PWD);

    // if (strlen(WIFI_PASS) == 0) {
    //     wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    // }

    //wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    //ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    esp_wifi_set_config(WIFI_IF_AP, &wifi_config);

    ESP_ERROR_CHECK(esp_wifi_start());

    esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_AP_DEF"), &ip_info);

    char ip_addr[16];
    inet_ntoa_r(ip_info.ip.addr, ip_addr, 16);
    hal.console->printf("Set up softAP with IP: %s\n", ip_addr);

    hal.console->printf("wifi_init_softap finished. SSID:'%s' password:'%s'\n",  WIFI_SSID, WIFI_PWD);
}


void IRAM_ATTR WiFiDriver::initialize_wifi()
{
#ifdef WIFIDEBUG
   ////hal.console->printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    //hal.console->printf("\n1.WIFI thread has ID %d and %d bytes free stack\n", 42, uxTaskGetStackHighWaterMark(NULL));


    esp_netif_init();
    esp_event_loop_create_default();
    nvs_flash_init();
    esp_netif_create_default_wifi_ap();

    wifi_init_softap();

}

size_t WiFiDriver::write(uint8_t c)
{
    return write(&c,1);
}

size_t WiFiDriver::write(const uint8_t *buffer, size_t size)
{
    ////hal.console->printf(" ZZZZ write? %d\n" , _state );
    if (_state != CONNECTED) {
        return 0;
    }
    //hal.console->printf(" ZZZZ write CONNECTED %d\n" , _state );
    if (!_write_mutex.take_nonblocking()) {
        return 0;
    }
    //hal.console->printf(" ZZZZ write buffsize:%d\n",size );
    size_t ret = _writebuf.write(buffer, size);
    _write_mutex.give();
    return ret;
}

void WiFiDriver::_wifi_thread(void *arg)
{
#ifdef WIFIDEBUG
   //hal.console->printf("%s:%d ZZZZ 0\n", __PRETTY_FUNCTION__, __LINE__);
#endif
    WiFiDriver *self = (WiFiDriver *) arg;
    if (!self->start_listen()) {
        vTaskDelete(nullptr);
    }
    //hal.console->printf(" ZZZZ 1\n" );
    while (true) {
        if (self->try_accept()) {
            //hal.console->printf(" ZZZZ 2 _state = CONNECTED\n" );
            self->_state = CONNECTED;

            while (true) {
                self->_more_data = false;
                if (!self->read_data()) {
                        //hal.console->printf(" ZZZZ 3\n" );

                    self->_state = INITIALIZED;
                    break;
                }
                if (!self->write_data()) {
                        //hal.console->printf(" ZZZZ 4\n" );

                    self->_state = INITIALIZED;
                    break;
                }
                if (!self->_more_data) {
                    hal.scheduler->delay_microseconds(500);
                    //hal.console->printf(" ZZZZ thread delay\n" );
                }
            }
        }
        //hal.console->printf(" ZZZZ 11 \n" );
        hal.scheduler->delay_microseconds(10000);

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
