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
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

using namespace ESP32;

extern const AP_HAL::HAL& hal;

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
    uint8_t byte;
    if (!_readbuf.read_byte(&byte)) {
        return -1;
    }
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
    destAddr.sin_port = htons(14550);
    int err = bind(accept_socket, (struct sockaddr *)&destAddr, sizeof(destAddr));
    if (err != 0) {
        close(accept_socket);
        accept_socket = 0;
        return false;
    }
	memset(&client_addr, 0, sizeof(client_addr));
    	fcntl(accept_socket, F_SETFL, O_NONBLOCK);

    return true;

}

bool WiFiUdpDriver::read_all()
{
	socklen_t socklen = sizeof(client_addr);
	int count = recvfrom(accept_socket , _buffer, sizeof(_buffer) - 1, 0, (struct sockaddr *)&client_addr, &socklen);
	if (count > 0) {
		_readbuf.write(_buffer, count);
		_more_data = true;
	} else {
		return false;
	}
	return true;
}

bool WiFiUdpDriver::write_data()
{
    _write_mutex.take_blocking();
	int count = _writebuf.peekbytes(_buffer, sizeof(_buffer));
	if (count > 0) {
		count = sendto(accept_socket, _buffer, count, 0, (struct sockaddr *)&client_addr, sizeof(client_addr));
		if (count > 0) {
			_writebuf.advance(count);
			_more_data = true;
		} else {
			_write_mutex.give();
			return false;
		}
	}
	_write_mutex.give();
	return true;
}

void WiFiUdpDriver::initialize_wifi()
{
    tcpip_adapter_init();
    nvs_flash_init();
    esp_event_loop_init(nullptr, nullptr);
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_storage(WIFI_STORAGE_FLASH);
    wifi_config_t wifi_config;
    memset(&wifi_config, 0, sizeof(wifi_config));
#ifdef WIFI_SSID
    strcpy((char *)wifi_config.ap.ssid, WIFI_SSID);
#else
    strcpy((char *)wifi_config.ap.ssid, "ardupilot");
#endif
#ifdef WIFI_PWD
    strcpy((char *)wifi_config.ap.password, WIFI_PWD);
#else
    strcpy((char *)wifi_config.ap.password, "ardupilot1");
#endif
    wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    wifi_config.ap.max_connection = 4;
    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config);
    esp_wifi_start();
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
	::printf("Start UDP\n");
    if (!self->start_listen()) {
        vTaskDelete(nullptr);
		::printf("DELETE UDP\n");
    }
	::printf("LISTENING udp\n");
    while (true) {
		self->_more_data = false;
		if (!self->read_all()) {
		}
		if (!self->write_data()) {
		}
		if (!self->_more_data) {
			hal.scheduler->delay_microseconds(1000);
		}
	}
}

bool WiFiUdpDriver::discard_input()
{
	return false;
}
