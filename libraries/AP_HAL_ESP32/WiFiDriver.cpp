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

#include "esp_http_server.h"
#include "esp_spiffs.h"
#include "esp_vfs.h"


using namespace ESP32;

extern const AP_HAL::HAL& hal;

WiFiDriver::WiFiDriver()
{
#ifdef WIFIDEBUG
   ////Scheduler::threadsafe_printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
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
   ////Scheduler::threadsafe_printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
//#endif
    Scheduler::threadsafe_printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);

    if (_state == NOT_INITIALIZED) {
        initialize_wifi();
        if (xTaskCreate(_wifi_thread, "APM_WIFI", Scheduler::WIFI_SS, this, Scheduler::WIFI_PRIO, &_wifi_task_handle) != pdPASS) {
            ////Scheduler::threadsafe_printf("FAILED to create task _wifi_thread\n");
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
    //Scheduler::threadsafe_printf("WiFiDriver::available ? %d\n",_readbuf.available());
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
    //Scheduler::threadsafe_printf("ZZZZ read!!\n");
    if (_state != CONNECTED) {
        return -1;
    }
    uint8_t byte;
    if (!_readbuf.read_byte(&byte)) {
        return -1;
            //Scheduler::threadsafe_printf("ZZZZ readbyte -1\n");

    }
    //Scheduler::threadsafe_printf("ZZZZ read ok\n");

    return byte;
}


bool WiFiDriver::start_listen()
{
#ifdef WIFIDEBUG
////Scheduler::threadsafe_printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
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
    //Scheduler::threadsafe_printf(" ZZZZ 6\n" );

    struct sockaddr_in sourceAddr;
    uint addrLen = sizeof(sourceAddr);
    short i = available_socket();
    if (i != WIFI_MAX_CONNECTION) {
        //Scheduler::threadsafe_printf(" ZZZZ 7\n" );
        socket_list[i] = accept(accept_socket, (struct sockaddr *)&sourceAddr, &addrLen);
        //Scheduler::threadsafe_printf(" ZZZZ 8\n" );
        if (socket_list[i] >= 0) {
            //Scheduler::threadsafe_printf(" ZZZZ 9\n" );
            fcntl(socket_list[i], F_SETFL, O_NONBLOCK);
            return true;
        }
    }
    //Scheduler::threadsafe_printf(" ZZZZ 10\n" );
    return false;
}

bool WiFiDriver::read_data()
{
    ////Scheduler::threadsafe_printf(" ZZZZ read_data\n" );
    for (unsigned short i = 0; i < WIFI_MAX_CONNECTION && socket_list[i] > -1; ++i) {
        int count = 0;
        do {
            count = recv(socket_list[i], (void *)_buffer, sizeof(_buffer), 0);
            if (count > 0) {
                //Scheduler::threadsafe_printf(" ZZZZ read_data %d\n",count );
                _readbuf.write(_buffer, count);
                if (count == sizeof(_buffer)) {
                    _more_data = true;
                }
            } else if (count < 0 && errno != EAGAIN) {
                //Scheduler::threadsafe_printf(" ZZZZ read_data-shutdown%d\n",count );
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
    //Scheduler::threadsafe_printf(" ZZZZ write_data   \n" );
    for (unsigned short i = 0; i < WIFI_MAX_CONNECTION && socket_list[i] > -1; ++i) {
        int count = 0;
        _write_mutex.take_blocking();
        //Scheduler::threadsafe_printf(" ZZZZ write_data took sem \n" );

            //static const uint8_t buffer[] = {0xFE ,0x09, 0x4E, 0x01, 0x01, 00, 00,   00,  00,  00,  02, 03, 0x51, 0x04,  03, 0x1C,  0x7F };
            //write(buffer,sizeof(buffer));
        do {
            count = _writebuf.peekbytes(_buffer, sizeof(_buffer));
            //Scheduler::threadsafe_printf(" ZZZZ write_data peek-count: %d bufsize:%d\n",count,sizeof(_buffer) );
            if (count > 0) {
                count = send(socket_list[i], (void*) _buffer, count, 0);
                //Scheduler::threadsafe_printf(" ZZZZ write_data send-count %d\n",count );
                if (count > 0) {
                    _writebuf.advance(count);
                    if (count == sizeof(_buffer)) {
                        _more_data = true;
                        //Scheduler::threadsafe_printf(" ZZZZ write_data more data\n" );
                    }
                } else if (count < 0 && errno != EAGAIN) {
                    //Scheduler::threadsafe_printf(" ZZZZ write_data shutdown\n" );
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
            //Scheduler::threadsafe_printf(" ZZZZ write_data gave sem \n" );
    return true;
}

void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                  int32_t event_id, void *event_data)
    {
        //Scheduler::threadsafe_printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
        if (WIFI_EVENT == event_base)
        {
            static const wifi_event_t event_type{static_cast<wifi_event_t>(event_id)};

            switch (event_type)
            {
            case WIFI_EVENT_STA_START:
            {
                //std::lock_guard<std::mutex> state_guard(_mutx);
                //_state = READY_TO_CONNECT;
                 //Scheduler::threadsafe_printf("%s:%d WIFI_EVENT_STA_START\n", __PRETTY_FUNCTION__, __LINE__);
                break;
            }

            case WIFI_EVENT_STA_CONNECTED:
            {
                //std::lock_guard<std::mutex> state_guard(_mutx);
                //_state = WAITING_FOR_IP;
                 //Scheduler::threadsafe_printf("%s:%d WIFI_EVENT_STA_CONNECTED\n", __PRETTY_FUNCTION__, __LINE__);
                break;
            }

            case WIFI_EVENT_STA_DISCONNECTED:
            {
               // std::lock_guard<std::mutex> state_guard(_mutx);
               // _state = DISCONNECTED;
                //Scheduler::threadsafe_printf("%s:%d WIFI_EVENT_STA_DISCONNECTED\n", __PRETTY_FUNCTION__, __LINE__);
                break;
            }

            default:
                break;
            }
        }
    }

void ip_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
    {
        //Scheduler::threadsafe_printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
        if (IP_EVENT == event_base)
        {
            const ip_event_t event_type{static_cast<ip_event_t>(event_id)};

            switch (event_type)
            {
            case IP_EVENT_STA_GOT_IP:
            {
                //std::lock_guard<std::mutex> state_guard(_mutx);
                //_state = CONNECTED;
                ////Scheduler::threadsafe_printf("%s:%d IP_EVENT_STA_GOT_IP\n", __PRETTY_FUNCTION__, __LINE__);
                break;
            }

            case IP_EVENT_STA_LOST_IP:
            {
                //std::lock_guard<std::mutex> state_guard(_mutx);
                //if (DISCONNECTED != _state)
                //{
                   // _state = WAITING_FOR_IP;
                //}
                ////Scheduler::threadsafe_printf("%s:%d IP_EVENT_STA_LOST_IP\n", __PRETTY_FUNCTION__, __LINE__);
                break;
            }

            default:
                break;
            }
        }
    }
static void wifi_init_softap(void)
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));

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
    Scheduler::threadsafe_printf("Set up softAP with IP: %s\n", ip_addr);

    Scheduler::threadsafe_printf("wifi_init_softap finished. SSID:'%s' password:'%s'\n",  WIFI_SSID, WIFI_PWD);
}


#define IS_FILE_EXT(filename, ext) \
    (strcasecmp(&filename[strlen(filename) - sizeof(ext) + 1], ext) == 0)

/* Set HTTP response content type according to file extension */
static esp_err_t set_content_type_from_file(httpd_req_t *req, const char *filename)
{
    if (IS_FILE_EXT(filename, ".pdf")) {
        return httpd_resp_set_type(req, "application/pdf");
    } else if (IS_FILE_EXT(filename, ".html")) {
        return httpd_resp_set_type(req, "text/html");
    } else if (IS_FILE_EXT(filename, ".jpeg")) {
        return httpd_resp_set_type(req, "image/jpeg");
    } else if (IS_FILE_EXT(filename, ".ico")) {
        return httpd_resp_set_type(req, "image/x-icon");
    }
    /* This is a limited set only */
    /* For any other type always set as plain text */
    return httpd_resp_set_type(req, "text/plain");
}

/* Scratch buffer size */
#define SCRATCH_BUFSIZE  8192

struct file_server_data {
    /* Base path of file storage */
    char base_path[ESP_VFS_PATH_MAX + 1];

    /* Scratch buffer for temporary storage during file transfer */
    char scratch[SCRATCH_BUFSIZE];
};


/* Copies the full path into destination buffer and returns
 * pointer to path (skipping the preceding base path) */
static const char* get_path_from_uri(char *dest, const char *base_path, const char *uri, size_t destsize)
{
    const size_t base_pathlen = strlen(base_path);
    size_t pathlen = strlen(uri);

    const char *quest = strchr(uri, '?');
    if (quest) {
        pathlen = MIN(pathlen, quest - uri);
    }
    const char *hash = strchr(uri, '#');
    if (hash) {
        pathlen = MIN(pathlen, hash - uri);
    }

    if (base_pathlen + pathlen + 1 > destsize) {
        /* Full path string won't fit into destination buffer */
        return NULL;
    }

    /* Construct full path (base + path) */
    strcpy(dest, base_path);
    strlcpy(dest + base_pathlen, uri, pathlen + 1);

    /* Return pointer to path, skipping the base */
    return dest + base_pathlen;
}

// HTTP GET Handler
static esp_err_t root_get_handler(httpd_req_t *req)
{
    //const uint32_t root_len = root_end - root_start;

    ////Scheduler::threadsafe_printf("Serve root");
    //httpd_resp_set_type(req, "text/html");
    //httpd_resp_send(req, root_start, root_len);
    //const char root_start[] = "_binary_root_html_start");
    //const char root_end[] ="_binary_root_html_end");

    char filepath[255];
    FILE *fd = NULL;
    struct stat file_stat;

    //const char *filename = "index.html";//get_path_from_uri(filepath, ((struct file_server_data *)req->user_ctx)->base_path,  req->uri, sizeof(filepath));
    const char *filename = get_path_from_uri(filepath, ((struct file_server_data *)req->user_ctx)->base_path,  req->uri, sizeof(filepath));

   if (!filename) {
        ////Scheduler::threadsafe_printf("Filename is too long");
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Filename too long");
        return ESP_FAIL;
    }

      if (stat(filepath, &file_stat) == -1) {
        /* If file not present on SPIFFS check if URI
         * corresponds to one of the hardcoded paths */
        if (strcmp(filename, "/index.html") == 0) {
            //return index_html_get_handler(req);
        } else 
        if (strcmp(filename, "/favicon.ico") == 0) {
            //return favicon_get_handler(req);
        }
        ////Scheduler::threadsafe_printf("Failed to stat file : %s", filepath);
        /* Respond with 404 Not Found */
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File does not exist");
        return ESP_FAIL;
    }

    fd = fopen(filepath, "r");
    if (!fd) {
       // //Scheduler::threadsafe_printf("Failed to read existing file : %s", filepath);
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read existing file");
        return ESP_FAIL;
    }

    ////Scheduler::threadsafe_printf("Sending file : %s (%ld bytes)...", filename, file_stat.st_size);
    set_content_type_from_file(req, filename);

    /* Retrieve the pointer to scratch buffer for temporary storage */
    char *chunk = ((struct file_server_data *)req->user_ctx)->scratch;
    size_t chunksize;
    do {
        /* Read file in chunks into the scratch buffer */
        chunksize = fread(chunk, 1, SCRATCH_BUFSIZE, fd);

        if (chunksize > 0) {
            /* Send the buffer contents as HTTP response chunk */
            if (httpd_resp_send_chunk(req, chunk, chunksize) != ESP_OK) {
                fclose(fd);
                ////Scheduler::threadsafe_printf("File sending failed!");
                /* Abort sending file */
                httpd_resp_sendstr_chunk(req, NULL);
                /* Respond with 500 Internal Server Error */
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send file");
               return ESP_FAIL;
           }
        }

        /* Keep looping till the whole file is sent */
    } while (chunksize != 0);

    /* Close file after sending complete */
    fclose(fd);


    return ESP_OK;
}


// HTTP Error (404) Handler - Redirects all requests to the root page
esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err)
{
    // Set status
    httpd_resp_set_status(req, "302 Temporary Redirect");
    // Redirect to the "/" root directory
    httpd_resp_set_hdr(req, "Location", "/");
    // iOS requires content in the response to detect a captive portal, simply redirecting is not sufficient.
    httpd_resp_send(req, "Redirect to the captive portal", HTTPD_RESP_USE_STRLEN);

    ////Scheduler::threadsafe_printf("Redirecting to root");
    return ESP_OK;
}





static const httpd_uri_t show_root_webpage = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = root_get_handler
};

static httpd_handle_t start_webserver(void)
{
    ////Scheduler::threadsafe_printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
    httpd_handle_t server = NULL;
    return server; // hack to disable it.
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_open_sockets = 10; /// must be 3 smaller than  CONFIG_LWIP_MAX_SOCKETS=13 in esp-idf/sdkconfig file, change both together as needed.
    config.lru_purge_enable = true;
    config.server_port = 80;

    // Start the httpd server
    ////Scheduler::threadsafe_printf("Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        ////Scheduler::threadsafe_printf("%s:%d httpd_start\n", __PRETTY_FUNCTION__, __LINE__);
        // Set URI handlers
        ////Scheduler::threadsafe_printf("Registering URI handlers");
        httpd_register_uri_handler(server, &show_root_webpage);
        httpd_register_err_handler(server, HTTPD_404_NOT_FOUND, http_404_error_handler);
    }
    return server;
}

void IRAM_ATTR WiFiDriver::initialize_wifi()
{
#ifdef WIFIDEBUG
   ////Scheduler::threadsafe_printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    //Scheduler::threadsafe_printf("\n1.WIFI thread has ID %d and %d bytes free stack\n", 42, uxTaskGetStackHighWaterMark(NULL));


    esp_netif_init();
    esp_event_loop_create_default();
    nvs_flash_init();
    esp_netif_create_default_wifi_ap();

    wifi_init_softap();

    // Start the server for the first time
    start_webserver();

}

size_t WiFiDriver::write(uint8_t c)
{
    return write(&c,1);
}

size_t WiFiDriver::write(const uint8_t *buffer, size_t size)
{
    ////Scheduler::threadsafe_printf(" ZZZZ write? %d\n" , _state );
    if (_state != CONNECTED) {
        return 0;
    }
    //Scheduler::threadsafe_printf(" ZZZZ write CONNECTED %d\n" , _state );
    if (!_write_mutex.take_nonblocking()) {
        return 0;
    }
    //Scheduler::threadsafe_printf(" ZZZZ write buffsize:%d\n",size );
    size_t ret = _writebuf.write(buffer, size);
    _write_mutex.give();
    return ret;
}

void WiFiDriver::_wifi_thread(void *arg)
{
#ifdef WIFIDEBUG
   //Scheduler::threadsafe_printf("%s:%d ZZZZ 0\n", __PRETTY_FUNCTION__, __LINE__);
#endif
    WiFiDriver *self = (WiFiDriver *) arg;
    if (!self->start_listen()) {
        vTaskDelete(nullptr);
    }
    //Scheduler::threadsafe_printf(" ZZZZ 1\n" );
    while (true) {
        if (self->try_accept()) {
            //Scheduler::threadsafe_printf(" ZZZZ 2 _state = CONNECTED\n" );
            self->_state = CONNECTED;

            //static const uint8_t buffer[] = {0xFE ,0x09, 0x4E, 0x01, 0x01, 00, 00,   00,  00,  00,  02, 03, 0x51, 0x04,  03, 0x1C,  0x7F };
            //self->write(buffer,sizeof(buffer));
            while (true) {
                self->_more_data = false;
                if (!self->read_data()) {
                        //Scheduler::threadsafe_printf(" ZZZZ 3\n" );

                    self->_state = INITIALIZED;
                    break;
                }
                if (!self->write_data()) {
                        //Scheduler::threadsafe_printf(" ZZZZ 4\n" );

                    self->_state = INITIALIZED;
                    break;
                }
                if (!self->_more_data) {
                    hal.scheduler->delay_microseconds(1000);
                    ////Scheduler::threadsafe_printf(" ZZZZ 5\n" );
                }
            }
        }
        //Scheduler::threadsafe_printf(" ZZZZ 11 \n" );
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
