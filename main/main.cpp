/* Hello World Example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stddef.h>
#include <stdio.h>


#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_system.h"
#include "esp_flash.h"

#include "sdkconfig.h"

#define CONFIG_HAL_BOARD HAL_BOARD_ESP32
#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_ESP32_S3EMPTY
#define __AP_LINE__ __LINE__

//#include "AP_HAL_ESP32/boards/esp32empty.h" 

//#include "HAL.h"
#include "AP_HAL/HAL.h"
//#include "AP_HAL/board/esp32.h"
//#include "esp32s3empty.h"

#include "Plane.h"

extern Plane plane;



// so modules/esp_idf/components/freertos/port/port_common.c , a C file, can see it.
 extern "C" {
     //int othermain(int argc, char * argv[]);
     int main(int argc, char * argv[]);
     void app_main();
 }
void app_main()
{
     printf("Hello nerds!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    printf("%dMB %s flash\n", flash_size / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    //for (int i = 10; i >= 0; i--) {
    //    printf("Restarting in %d seconds...\n", i);
       // vTaskDelay(1000 / portTICK_PERIOD_MS);
    //}
    //printf("Restarting now.\n");
    //fflush(stdout);
    //esp_restart();
    char * argv[0];
    main(0,argv);
}

int main(int argc, char * argv[]) {
    //app_main();

    printf("after app main before hal\n");

    hal.run(argc, argv, &plane); 

    printf("HAL has started - Ground Start and INS calibration should begin shortly....\n");

    //othermain(argc, argv); // see ArduPlane/ArduPlane.cpp
    return 0;
}
