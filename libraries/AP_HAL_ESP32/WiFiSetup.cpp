
#include <AP_HAL_ESP32/AP_HAL_ESP32.h>

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

void wifi_init_softap(void)
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

void initialize_wifi()
{
    static bool is_wifi_setup = false;
    if (is_wifi_setup == false) {
        esp_netif_init();
        esp_event_loop_create_default();
        nvs_flash_init();
        esp_netif_create_default_wifi_ap();

        wifi_init_softap();
        is_wifi_setup = true;
    }
}
